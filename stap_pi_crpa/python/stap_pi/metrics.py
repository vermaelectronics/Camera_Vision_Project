"""Performance metrics for the STAP-PI anti-jam processor."""

from __future__ import annotations

import numpy as np

from .core import StapPIResult, element_filter_response, space_time_response
from .scenario import CrpaData, gps_ca_code

__all__ = ["power", "db", "evaluate", "effective_cn0", "ca_spectrum", "null_depth_vs_frequency", "snr_gain_vs_frequency",
           "correlate_ca",
           "cn0_from_correlation", "psd"]


def power(x: np.ndarray) -> float:
    return float(np.mean(np.abs(x) ** 2))


def db(v):
    return 10 * np.log10(np.maximum(v, 1e-300))


def _ratio_db(num: float, den: float) -> float:
    if num <= 0:
        return float("-inf")
    return float("inf") if den <= 0 else float(10 * np.log10(num / den))


def evaluate(data: CrpaData, res: StapPIResult, skip_blocks: int | None = None) -> dict:
    """Decompose the output by linearity and compute the standard anti-jam figures.

    Returns (all in dB / dB-Hz):

    ``jnr_in_db``, ``jnr_out_db``
        jammer-to-noise ratio at the reference element and at the output.
    ``suppression_db``
        jammer power at the reference element / jammer power at the output.
    ``noise_gain_db``
        output / input noise power (PI normally leaves it near 0 dB).
    ``cn0_ref_db``
        effective C/N0 of the bare reference element *without* the jammer.
    ``cn0_in_db``
        effective C/N0 of the bare reference element *with* the jammer
        (what a single-antenna receiver would see).
    ``cn0_out_db``
        effective C/N0 at the STAP-PI output with the jammer.
    ``cn0_degradation_db`` = ``cn0_ref_db - cn0_out_db`` (residual jamming loss; negative = array gain)
    ``cn0_improvement_db`` = ``cn0_out_db - cn0_in_db`` (anti-jam gain).

    The effective C/N0 is the post-correlation figure
    ``C_eff / N0_eff`` with ``C_eff = |correlation of the signal component
    with the ideal C/A replica|^2`` and ``N0_eff = integral PSD_(J+N)(f)
    G_CA(f) df`` (spectral-separation form), i.e. exactly what a GNSS
    correlator measures.

    The first ``weight_latency + 1`` blocks (start-up, no adaptive weights yet)
    are excluded unless ``skip_blocks`` says otherwise.
    """
    cfg = res.config
    if skip_blocks is None:
        skip_blocks = cfg.weight_latency + 1
    k0 = skip_blocks * cfg.block_len
    if k0 >= data.x.shape[1]:
        raise ValueError("record too short for the requested skip")
    ref = cfg.ref_element

    yp = {k: res.apply(v) for k, v in data.parts.items()}
    pin = {k: power(v[ref, k0:]) for k, v in data.parts.items()}
    pout = {k: power(v[k0:]) for k, v in yp.items()}
    n_in = pin["noise"] + pin["adc"]
    n_out = pout["noise"] + pout["adc"]

    sc = data.scenario
    ref_parts = {k: v[ref, k0:] for k, v in data.parts.items()}
    out_parts = {k: v[k0:] for k, v in yp.items()}
    cn0_ref = effective_cn0(data, ref_parts["signal"], ref_parts["noise"], k0)
    cn0_in = effective_cn0(data, ref_parts["signal"], ref_parts["jammer"] + ref_parts["noise"] + ref_parts["adc"], k0)
    cn0_out = effective_cn0(data, out_parts["signal"], out_parts["jammer"] + out_parts["noise"] + out_parts["adc"], k0)

    out = {
        "js_db": sc.js_db,
        "jnr_in_db": _ratio_db(pin["jammer"], n_in),
        "jnr_out_db": _ratio_db(pout["jammer"], n_out),
        "suppression_db": _ratio_db(pin["jammer"], pout["jammer"]) if pin["jammer"] > 0 else float("nan"),
        "noise_gain_db": float(db(n_out / n_in)),
        "cn0_ref_db": cn0_ref,
        "cn0_in_db": cn0_in,
        "cn0_out_db": cn0_out,
        "cn0_degradation_db": cn0_ref - cn0_out,
        "cn0_improvement_db": cn0_out - cn0_in,
        "y_parts": yp,
    }
    return out


def ca_spectrum(freqs: np.ndarray, chip_rate: float) -> np.ndarray:
    """Normalised (unit-area) PSD of a BPSK(1) C/A code on the grid ``freqs``."""
    tc = 1.0 / chip_rate
    g = tc * np.sinc(freqs * tc) ** 2
    df = np.abs(freqs[1] - freqs[0])
    return g / (np.sum(g) * df)


def effective_cn0(data: CrpaData, sig: np.ndarray, interf: np.ndarray, start: int = 0, nfft: int = 1024) -> float:
    """Post-correlation C/N0 [dB-Hz] of ``sig`` (satellite component) in ``interf`` (J + N).

    Both inputs are in the same (processor) units, so every gain between the
    antenna and this point cancels.  ``start`` is the sample index of
    ``sig[0]`` in the original record (needed for the replica's time base).
    """
    sc = data.scenario
    # coherent correlation of the signal component with the ideal replica (fine delay search)
    delays = np.arange(-0.30, 0.60001, 0.01)
    corr = correlate_ca(sig, data.fs, sc.prn, sc.doppler_hz, sc.chip_rate, sc.code_phase_chips, delays,
                        t_coh=len(sig) / data.fs, start=0, t0=start / data.fs)
    c_eff = np.max(corr) / len(sig) ** 2
    # effective noise density seen through the C/A spectrum
    f, phi = psd(interf, data.fs, nfft)
    g = ca_spectrum(f, sc.chip_rate)
    n0_eff = np.sum(phi * g) * (f[1] - f[0])
    return float(db(c_eff / n0_eff))


def null_depth_vs_frequency(data: CrpaData, w: np.ndarray, n_taps: int, ref_element: int, freqs: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Cancellation (dB) of a wave from direction ``u`` versus frequency.

    Response of the adaptive space-time filter normalised to the response of
    the bare reference element (what a single-antenna receiver would see).
    """
    a = data.element_response(u, freqs)[0]                    # (n_el, F)
    H = space_time_response(w, n_taps, data.fs, freqs, a)
    return db(np.abs(H) ** 2 / np.abs(a[ref_element]) ** 2)


def snr_gain_vs_frequency(data: CrpaData, w: np.ndarray, n_taps: int, ref_element: int, freqs: np.ndarray,
                          u: np.ndarray) -> np.ndarray:
    """Per-frequency SNR gain (dB) for a wave from ``u`` relative to the bare reference element.

    Thermal noise is independent from element to element and passes through
    each channel's own response, so the output noise PSD is
    ``sum_m |W_m(f)|^2 |H_m(f)|^2`` against ``|H_ref(f)|^2`` at the input.
    For 2 elements the upper bound is +3 dB (coherent combining).
    """
    a = data.element_response(u, freqs)[0]
    Hs = space_time_response(w, n_taps, data.fs, freqs, a)
    ch = data.channel_response(freqs)
    Wf = element_filter_response(w, n_taps, data.fs, freqs)
    sig_gain = np.abs(Hs) ** 2 / np.abs(a[ref_element]) ** 2
    noise_gain = np.sum(np.abs(Wf) ** 2 * np.abs(ch) ** 2, axis=0) / np.abs(ch[ref_element]) ** 2
    return db(sig_gain / noise_gain)


def correlate_ca(y: np.ndarray, fs: float, prn: int, doppler_hz: float, chip_rate: float, code_phase_chips: float,
                 delays_chips: np.ndarray, t_coh: float = 1e-3, start: int = 0, t0: float | None = None) -> np.ndarray:
    """Non-coherently integrated C/A correlation power over code delay (known Doppler).

    ``y[start:]`` is used; ``t0`` is the time of ``y[start]`` (default ``start / fs``).
    """
    n_coh = int(round(t_coh * fs))
    n_int = (len(y) - start) // n_coh
    if n_int < 1:
        raise ValueError("record too short")
    seg = y[start:start + n_int * n_coh]
    t = (start / fs if t0 is None else t0) + np.arange(seg.size) / fs
    base = seg * np.exp(-2j * np.pi * doppler_hz * t)
    code = gps_ca_code(prn).astype(float)
    out = np.empty(len(delays_chips))
    for i, d in enumerate(delays_chips):
        chip = np.floor(code_phase_chips - d + t * chip_rate).astype(np.int64) % 1023
        c = base * code[chip]
        out[i] = np.sum(np.abs(c.reshape(n_int, n_coh).sum(axis=1)) ** 2)
    return out


def cn0_from_correlation(delays_chips: np.ndarray, corr: np.ndarray, t_coh: float = 1e-3, peak_window: float = 1.0, floor_from: float = 3.0) -> float:
    """C/N0 [dB-Hz] estimate from a correlation-vs-delay curve (``SNR_post = C/N0 * T``)."""
    pk = np.max(corr[np.abs(delays_chips) <= peak_window])
    fl = np.mean(corr[np.abs(delays_chips) >= floor_from])
    return float(db(max(pk - fl, 1e-300) / fl / t_coh))


def psd(x: np.ndarray, fs: float, nfft: int = 1024) -> tuple[np.ndarray, np.ndarray]:
    """Welch PSD (Hann, 50 % overlap), returned with fftshifted frequency axis."""
    x = np.asarray(x)
    win = np.hanning(nfft)
    step = nfft // 2
    n_seg = (len(x) - nfft) // step + 1
    acc = np.zeros(nfft)
    for i in range(n_seg):
        seg = x[i * step:i * step + nfft] * win
        acc += np.abs(np.fft.fft(seg)) ** 2
    p = acc / (n_seg * fs * np.sum(win ** 2))
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs))
    return f, np.fft.fftshift(p)
