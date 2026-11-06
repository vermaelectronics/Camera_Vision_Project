"""
STAP-PI core: Space-Time Adaptive Processing with the Power-Inversion criterion.

Signal model
------------
Each of the ``N`` antenna elements feeds an ``M``-tap delay line.  At sample
``n`` the space-time snapshot is the ``D = N*M`` vector

    x(n) = [x_1(n), x_1(n-1), ..., x_1(n-M+1),
            x_2(n), x_2(n-1), ..., x_2(n-M+1), ... ]^T          (element-major)

and the processor output is

    y(n) = w^H x(n).

Power-inversion criterion
-------------------------
No knowledge of the satellite directions is assumed.  The output power is
minimised subject to a single linear constraint that keeps one tap of one
element (the *reference tap*) at unit gain, which prevents the trivial
``w = 0`` solution:

    min_w  w^H R w     s.t.   w^H s = 1,      s = e_(ref_element, ref_tap)

The closed-form solution (sample-matrix-inversion, SMI) is

    w = R^-1 s / (s^H R^-1 s),        R = E{x x^H} (+ diagonal loading)

Strong interference dominates ``R``, so its subspace is inverted (nulled) by
``R^-1`` while the thermal-noise subspace, which contains the GNSS signals
(they are ~20-30 dB *below* the noise floor), is left essentially untouched.

Why space-time and not space only?
----------------------------------
A 2-element spatial-only PI (``M = 1``) has a single complex weight, so it
can form one null at one frequency.  A wideband barrage jammer occupies the
whole receiver band; the inter-element phase ``2*pi*(f0+f)*tau`` and any
RF/IF channel mismatch (filter ripple, group-delay differences, skew) vary
with frequency ``f``, so a single weight cannot null the jammer over the whole
band.  The ``M``-tap FIR behind each element provides a frequency-dependent
weight ``W_m(f)`` that tracks the null across the band and equalises the
channel mismatch -> deep wideband nulls.

With 2 elements there are ``2M - 1`` adaptive degrees of freedom; one
wideband jammer consumes ~``M + 1`` of them, so a 2-element STAP-PI CRPA can
suppress one wideband (or up to ~``M-1`` narrowband / ``1`` wideband +
some narrowband) jammers.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import cho_factor, cho_solve

__all__ = [
    "StapPIConfig",
    "StapPIResult",
    "StapPIProcessor",
    "space_time_snapshots",
    "constraint_vector",
    "sample_covariance",
    "pi_weights",
    "apply_weight_schedule",
    "space_time_response",
    "element_filter_response",
]


# ---------------------------------------------------------------------------
# Building blocks
# ---------------------------------------------------------------------------
def space_time_snapshots(x: np.ndarray, n_taps: int, history: np.ndarray | None = None) -> np.ndarray:
    """Build the ``(N*M, n_samples)`` space-time snapshot matrix.

    Parameters
    ----------
    x : (n_elements, n_samples) complex array
        Element samples.
    n_taps : int
        Number of taps ``M`` per element.
    history : (n_elements, M-1) complex array, optional
        Last ``M-1`` samples of the previous block (oldest first).  Zeros if
        omitted (filter starts from rest).

    Returns
    -------
    X : (n_elements*n_taps, n_samples) complex array with
        ``X[m*M + k, n] = x[m, n - k]``.
    """
    x = np.atleast_2d(np.asarray(x))
    n_el, n = x.shape
    m_taps = int(n_taps)
    if m_taps < 1:
        raise ValueError("n_taps must be >= 1")
    if history is None:
        history = np.zeros((n_el, m_taps - 1), dtype=complex)
    history = np.asarray(history).reshape(n_el, m_taps - 1)
    xe = np.concatenate([history, x], axis=1)
    X = np.empty((n_el * m_taps, n), dtype=complex)
    for m in range(n_el):
        for k in range(m_taps):
            start = m_taps - 1 - k
            X[m * m_taps + k] = xe[m, start:start + n]
    return X


def constraint_vector(n_elements: int, n_taps: int, ref_element: int = 0, ref_tap: int | None = None) -> np.ndarray:
    """Power-inversion constraint vector ``s`` (a unit vector on the reference tap).

    ``ref_tap=None`` selects the centre tap ``(M-1)//2``.  A centre reference
    tap lets the adaptive filter use both "past" and "future" samples around
    the reference, which gives deeper wideband nulls and less signal
    distortion than a first-tap reference.
    """
    if ref_tap is None:
        ref_tap = (n_taps - 1) // 2
    if not (0 <= ref_element < n_elements and 0 <= ref_tap < n_taps):
        raise ValueError("reference element/tap out of range")
    s = np.zeros(n_elements * n_taps, dtype=complex)
    s[ref_element * n_taps + ref_tap] = 1.0
    return s


def sample_covariance(X: np.ndarray) -> np.ndarray:
    """Hermitian sample covariance ``R = X X^H / K`` of a ``(D, K)`` snapshot matrix."""
    K = X.shape[1]
    R = (X @ X.conj().T) / K
    return 0.5 * (R + R.conj().T)


def pi_weights(R: np.ndarray, s: np.ndarray, loading: float = 0.0) -> np.ndarray:
    """Power-inversion weights ``w = (R + dI)^-1 s / (s^H (R + dI)^-1 s)``.

    Solved with a Cholesky factorisation ``R + dI = L L^H`` followed by a
    forward and a backward substitution (no explicit inverse), which is the
    numerically preferred and hardware-friendly way to do SMI.
    """
    D = R.shape[0]
    Rl = R + float(loading) * np.eye(D)
    factor = cho_factor(Rl, lower=True, check_finite=False)
    z = cho_solve(factor, s, check_finite=False)
    return z / np.vdot(s, z)   # s^H z is real and > 0


def apply_weight_schedule(x: np.ndarray, weights: np.ndarray, block_len: int, n_taps: int) -> np.ndarray:
    """Filter ``x`` with a per-block weight schedule (``weights[b]`` used on block ``b``).

    Because the STAP filter is linear, this lets any *component* of the
    received data (jammer only, noise only, satellite only) be pushed through
    exactly the weights that the adaptive processor applied to the total
    signal - the basis of all the suppression / SINR measurements.
    """
    x = np.atleast_2d(np.asarray(x))
    n_el, n = x.shape
    y = np.empty(n, dtype=complex)
    hist = np.zeros((n_el, n_taps - 1), dtype=complex)
    for b in range(weights.shape[0]):
        lo, hi = b * block_len, min((b + 1) * block_len, n)
        if lo >= n:
            break
        seg = x[:, lo:hi]
        X = space_time_snapshots(seg, n_taps, hist)
        y[lo:hi] = weights[b].conj() @ X
        hist = _next_history(hist, seg, n_taps)
    return y


def space_time_response(w: np.ndarray, n_taps: int, fs: float, freqs: np.ndarray, element_response: np.ndarray) -> np.ndarray:
    """Frequency response of the space-time filter for a given incident wave.

    Parameters
    ----------
    w : (N*M,) weights.
    n_taps : M.
    fs : sample rate [Hz].
    freqs : (F,) baseband frequencies [Hz].
    element_response : (N, F) or (A, N, F) complex response of each element
        (steering phase x channel response) for the wave(s) of interest.

    Returns
    -------
    H : (F,) or (A, F) complex,
        ``H(f) = sum_m sum_k conj(w[m*M+k]) * a_m(f) * exp(-j 2 pi f k / fs)``.
    """
    a = np.asarray(element_response)
    squeeze = a.ndim == 2
    if squeeze:
        a = a[None]
    Wf = element_filter_response(w, n_taps, fs, freqs)                   # (N, F)
    H = np.einsum("nf,anf->af", Wf, a)
    return H[0] if squeeze else H


def element_filter_response(w: np.ndarray, n_taps: int, fs: float, freqs: np.ndarray) -> np.ndarray:
    """``(N, F)`` frequency response ``W_m(f) = sum_k conj(w[m*M+k]) exp(-j 2 pi f k / fs)`` of each element's FIR."""
    w = np.asarray(w)
    Wc = w.conj().reshape(w.size // n_taps, n_taps)
    tap_phasor = np.exp(-2j * np.pi * np.outer(np.arange(n_taps), freqs) / fs)
    return Wc @ tap_phasor


def _next_history(hist: np.ndarray, seg: np.ndarray, n_taps: int) -> np.ndarray:
    if n_taps == 1:
        return hist
    xe = np.concatenate([hist, seg], axis=1)
    return xe[:, -(n_taps - 1):].copy()


# ---------------------------------------------------------------------------
# Block-adaptive processor
# ---------------------------------------------------------------------------
@dataclass
class StapPIConfig:
    """Configuration of the STAP-PI processor.

    Attributes
    ----------
    n_elements : number of antenna elements (2 for this design).
    n_taps : FIR taps per element, ``M``.  ``M = 1`` gives a spatial-only PI.
    ref_element : index of the reference (constrained) element.
    ref_tap : index of the constrained tap, ``None`` -> centre tap.
    block_len : samples per weight-update block.
    cov_decimation : use every ``d``-th snapshot for the covariance estimate
        (an FPGA can estimate ``R`` at a lower rate than the sample rate).
    forgetting : exponential averaging of block covariances,
        ``R_b = f*R_(b-1) + (1-f)*R_block``.  0 -> pure block SMI.
    loading_db : diagonal loading, in dB relative to the noise-floor
        estimate (``noise_power`` if given, otherwise ``lambda_min(R)``).
    noise_power : calibrated per-channel noise power at the processor input
        (optional).
    weight_latency : blocks between the block a weight is computed from and
        the block it is applied to (1 models a causal FPGA/CPU pipeline,
        0 is the non-causal "same block" SMI).
    """

    n_elements: int = 2
    n_taps: int = 5
    ref_element: int = 0
    ref_tap: int | None = None
    block_len: int = 4096
    cov_decimation: int = 1
    forgetting: float = 0.0
    loading_db: float = 3.0
    noise_power: float | None = None
    weight_latency: int = 1

    @property
    def dim(self) -> int:
        return self.n_elements * self.n_taps

    @property
    def ref_tap_index(self) -> int:
        return (self.n_taps - 1) // 2 if self.ref_tap is None else self.ref_tap


@dataclass
class StapPIResult:
    """Output of :meth:`StapPIProcessor.process`."""

    y: np.ndarray                     # (n_samples,) processor output
    weights: np.ndarray               # (n_blocks, D) weights *applied* in each block
    weights_computed: np.ndarray      # (n_blocks, D) weights computed from each block
    loading: np.ndarray               # (n_blocks,) diagonal loading used
    eigenvalues: np.ndarray           # (n_blocks, D) eigenvalues of R (descending)
    config: StapPIConfig = field(repr=False)

    def apply(self, x: np.ndarray) -> np.ndarray:
        """Push another signal (e.g. a single component) through the same weights."""
        return apply_weight_schedule(x, self.weights, self.config.block_len, self.config.n_taps)


class StapPIProcessor:
    """Block-adaptive STAP-PI anti-jam processor (streaming capable)."""

    def __init__(self, config: StapPIConfig | None = None, **kwargs):
        self.cfg = config if config is not None else StapPIConfig(**kwargs)
        if config is not None and kwargs:
            raise TypeError("pass either a StapPIConfig or keyword arguments, not both")
        c = self.cfg
        if c.n_elements < 2:
            raise ValueError("a CRPA needs at least 2 elements")
        if not 0.0 <= c.forgetting < 1.0:
            raise ValueError("forgetting must be in [0, 1)")
        if c.block_len // max(c.cov_decimation, 1) < c.dim:
            raise ValueError("block too short: need at least D snapshots per covariance estimate")
        self.s = constraint_vector(c.n_elements, c.n_taps, c.ref_element, c.ref_tap)
        self.reset()

    # -- state -------------------------------------------------------------
    def reset(self) -> None:
        c = self.cfg
        self._hist = np.zeros((c.n_elements, c.n_taps - 1), dtype=complex)
        self._R = None
        # until the first adaptive weight is available, pass the reference tap through
        self._queue = deque([self.s.copy() for _ in range(c.weight_latency)])

    # -- one weight computation -------------------------------------------
    def loading_for(self, R: np.ndarray, eigvals: np.ndarray | None = None) -> float:
        floor = self.cfg.noise_power
        if floor is None:
            if eigvals is None:
                eigvals = np.linalg.eigvalsh(R)
            floor = max(float(np.min(eigvals)), np.finfo(float).tiny)
        # numerical floor so that R + dI is always positive definite
        floor = max(floor, 1e-14 * float(np.real(np.trace(R))) / R.shape[0])
        return 10.0 ** (self.cfg.loading_db / 10.0) * floor

    def compute_weights(self, R: np.ndarray) -> tuple[np.ndarray, float, np.ndarray]:
        eig = np.linalg.eigvalsh(R)[::-1]
        d = self.loading_for(R, eig)
        return pi_weights(R, self.s, d), d, eig

    # -- streaming processing ---------------------------------------------
    def process(self, x: np.ndarray) -> StapPIResult:
        """Process ``(n_elements, n_samples)`` data block by block.

        For every block ``b``: build snapshots, estimate ``R_b`` (optionally
        exponentially averaged), compute ``w_b``, filter block ``b`` with the
        weight computed ``weight_latency`` blocks earlier.
        """
        c = self.cfg
        x = np.atleast_2d(np.asarray(x, dtype=complex))
        if x.shape[0] != c.n_elements:
            raise ValueError(f"expected {c.n_elements} channels, got {x.shape[0]}")
        n = x.shape[1]
        L = c.block_len
        n_blocks = -(-n // L)
        D = c.dim
        y = np.empty(n, dtype=complex)
        W_app = np.empty((n_blocks, D), dtype=complex)
        W_cmp = np.empty((n_blocks, D), dtype=complex)
        loads = np.empty(n_blocks)
        eigs = np.empty((n_blocks, D))

        for b in range(n_blocks):
            lo, hi = b * L, min((b + 1) * L, n)
            seg = x[:, lo:hi]
            X = space_time_snapshots(seg, c.n_taps, self._hist)
            self._hist = _next_history(self._hist, seg, c.n_taps)

            Xc = X[:, ::c.cov_decimation]
            if Xc.shape[1] >= D:                      # skip covariance update on a too-short tail block
                Rb = sample_covariance(Xc)
                self._R = Rb if self._R is None else c.forgetting * self._R + (1.0 - c.forgetting) * Rb
            if self._R is not None:
                w_new, d, eig = self.compute_weights(self._R)
            else:
                w_new, d, eig = self.s.copy(), 0.0, np.zeros(D)

            if c.weight_latency == 0:
                w_use = w_new
            else:
                w_use = self._queue.popleft()
                self._queue.append(w_new)

            y[lo:hi] = w_use.conj() @ X
            W_app[b], W_cmp[b], loads[b], eigs[b] = w_use, w_new, d, eig

        return StapPIResult(y=y, weights=W_app, weights_computed=W_cmp, loading=loads,
                            eigenvalues=eigs, config=c)
