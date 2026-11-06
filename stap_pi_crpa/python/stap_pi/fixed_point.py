"""
Bit-true model of the RTL space-time filter ``rtl/stap_pi_filter.v``.

Number formats (defaults match the RTL parameters):

* input samples  : ``IW = 16``-bit two's complement per I/Q rail, full scale
  +/-1.0 -> ``x_int = x * 2**(IW-1)`` (a 14-bit ADC left-justified in 16 bits
  maps exactly onto integers),
* weights        : ``WW = 18``-bit, ``WFRAC = 16`` fractional bits (Q1.16,
  range [-2, 2)); the power-inversion reference tap is exactly 1.0 and is the
  largest weight in practice,
* accumulator    : full precision, ``IW + WW + 1 + ceil(log2(D))`` bits,
* output         : ``OW = 16`` bits, ``y_int = sat(round(acc / 2**out_shift))``
  with round-half-up and saturation.

With ``out_shift = WFRAC`` the output has the same scale as the input.
After deep jammer suppression the output is at the thermal-noise level, a few
ADC LSBs, so a smaller ``out_shift`` (more gain) is normally programmed to keep
resolution in the 16-bit output word: ``out_shift = WFRAC - g`` adds ``6.02*g``
dB of gain.
"""

from __future__ import annotations

import numpy as np

__all__ = ["quantize_input", "quantize_weights", "stap_filter_fixed", "to_hex_words"]


def _sat(v: np.ndarray, bits: int) -> np.ndarray:
    lo, hi = -(1 << (bits - 1)), (1 << (bits - 1)) - 1
    return np.clip(v, lo, hi)


def quantize_input(x: np.ndarray, iw: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Complex samples (full scale +/-1) -> integer I/Q of width ``iw`` (round + saturate)."""
    s = float(1 << (iw - 1))
    re = _sat(np.round(np.real(x) * s).astype(np.int64), iw)
    im = _sat(np.round(np.imag(x) * s).astype(np.int64), iw)
    return re, im


def quantize_weights(w: np.ndarray, ww: int = 18, wfrac: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Complex weights -> integer re/im with ``wfrac`` fractional bits in ``ww`` bits (round + saturate)."""
    s = float(1 << wfrac)
    re = _sat(np.round(np.real(w) * s).astype(np.int64), ww)
    im = _sat(np.round(np.imag(w) * s).astype(np.int64), ww)
    return re, im


def stap_filter_fixed(x_re: np.ndarray, x_im: np.ndarray, w_re: np.ndarray, w_im: np.ndarray, block_len: int,
                      n_taps: int, out_shift: int, ow: int = 16) -> tuple[np.ndarray, np.ndarray]:
    """Bit-exact reference of the RTL datapath.

    ``x_re/x_im``: (N, K) integer inputs; ``w_re/w_im``: (n_blocks, N*M) integer
    weights, row ``b`` active for samples ``[b*block_len, (b+1)*block_len)``.
    Computes ``acc = sum conj(w) * x`` (``re = wr*xr + wi*xi``,
    ``im = wr*xi - wi*xr``) exactly, then rounds/saturates like the RTL.
    """
    x_re = np.asarray(x_re, dtype=np.int64)
    x_im = np.asarray(x_im, dtype=np.int64)
    n_el, n = x_re.shape
    pad = np.zeros((n_el, n_taps - 1), dtype=np.int64)
    xr = np.concatenate([pad, x_re], axis=1)
    xi = np.concatenate([pad, x_im], axis=1)
    acc_re = np.zeros(n, dtype=np.int64)
    acc_im = np.zeros(n, dtype=np.int64)
    blk = np.minimum(np.arange(n) // block_len, w_re.shape[0] - 1)
    for m in range(n_el):
        for k in range(n_taps):
            d = m * n_taps + k
            a = xr[m, n_taps - 1 - k:n_taps - 1 - k + n]
            b = xi[m, n_taps - 1 - k:n_taps - 1 - k + n]
            wr = w_re[blk, d]
            wi = w_im[blk, d]
            acc_re += wr * a + wi * b
            acc_im += wr * b - wi * a
    if out_shift > 0:
        half = 1 << (out_shift - 1)
        y_re = (acc_re + half) >> out_shift
        y_im = (acc_im + half) >> out_shift
    else:
        y_re, y_im = acc_re, acc_im
    return _sat(y_re, ow), _sat(y_im, ow)


def to_hex_words(v: np.ndarray, bits: int) -> list[str]:
    """Two's-complement hex strings (for ``$readmemh``)."""
    mask = (1 << bits) - 1
    width = (bits + 3) // 4
    return [format(int(x) & mask, f"0{width}x") for x in np.ravel(v)]
