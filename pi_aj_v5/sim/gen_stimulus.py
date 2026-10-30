#!/usr/bin/env python3
"""
Stimulus generator for the PI-AJ v5 anti-jam core.

Models what the AD9361 (2R2T, LO = 1575.42 MHz, 18 MS/s, 12-bit ADC) delivers
from a 2-element array (d = 9.52 cm = 0.5 lambda at L1):

  * thermal noise, independent per channel
  * one GNSS-like BPSK signal (1.023 Mchip/s PN, below the noise floor)
  * the jammer: CW ("sharp") or band-limited noise ("barrage"), from its own
    direction, plus one ground-reflected copy (multipath)
  * per-channel RF/IF mismatch (gain, phase, 1st-order filter, fractional delay)
  * per-channel I/Q imbalance (creates the jammer IQ image)
  * 12-bit quantisation + hard clipping at the ADC

Writes  <name>.hex  (one line per sample: I0 Q0 I1 Q1, 16-bit two's complement
hex, LSB-aligned 12-bit like axi_ad9361) and <name>.npz (reference code etc.).
"""
import argparse
import numpy as np

FS = 18.0e6
CHIP = 1.023e6
L1 = 1575.42e6
LAMBDA = 299792458.0 / L1
D_ELEM = 0.0952


def steer(theta_deg):
    ph = 2 * np.pi * D_ELEM / LAMBDA * np.sin(np.deg2rad(theta_deg))
    return np.array([1.0, np.exp(1j * ph)])


def frac_delay(x, d):
    """Fractional delay via frequency domain (circular, fine for long runs)."""
    n = len(x)
    f = np.fft.fftfreq(n)
    return np.fft.ifft(np.fft.fft(x) * np.exp(-2j * np.pi * f * d))


def one_pole(x, a):
    """y[n] = (1-a) x[n] + a y[n-1]  (models analog filter mismatch)."""
    y = np.empty_like(x)
    acc = 0j
    b = 1 - a
    for i in range(len(x)):
        acc = b * x[i] + a * acc
        y[i] = acc
    return y


def iq_imbalance(x, g_db, ph_deg):
    g = 10 ** (g_db / 20)
    ph = np.deg2rad(ph_deg)
    i = x.real
    q = g * (np.sin(ph) * x.real + np.cos(ph) * x.imag)
    return i + 1j * q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", required=True)
    ap.add_argument("--n", type=int, default=180000)
    ap.add_argument("--jtype", choices=["cw", "noise", "none"], default="cw")
    ap.add_argument("--jn", type=float, default=61.0, help="J/N in dB (over 18 MHz)")
    ap.add_argument("--jf", type=float, default=-20e3, help="CW offset from LO (Hz)")
    ap.add_argument("--jbw", type=float, default=18e6, help="barrage noise bandwidth (Hz)")
    ap.add_argument("--jaz", type=float, default=55.0, help="jammer angle (deg)")
    ap.add_argument("--mp", type=float, default=-10.0, help="multipath rel. level dB")
    ap.add_argument("--mpaz", type=float, default=-35.0, help="multipath angle (deg)")
    ap.add_argument("--mpdel", type=float, default=0.35, help="multipath delay (samples)")
    ap.add_argument("--cn0", type=float, default=50.0, help="GNSS C/N0 dB-Hz")
    ap.add_argument("--saz", type=float, default=-20.0, help="satellite angle (deg)")
    ap.add_argument("--noise_lsb", type=float, default=0.0,
                    help="thermal noise rms per I/Q (LSB); 0 = auto: RX gain set so the "
                         "jammer just fits the 12-bit ADC (as a real AGC/manual gain must)")
    ap.add_argument("--jpk", type=float, default=1600.0,
                    help="auto mode: CW amplitude / 4x noise-jammer rms target (LSB)")
    ap.add_argument("--jon", type=int, default=9000, help="sample where jammer turns on")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--iq", type=float, default=1.0, help="IQ-imbalance scale (0 = ideal)")
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    n = a.n
    t = np.arange(n)

    # thermal noise: complex, rms per component = noise_lsb
    if a.noise_lsb <= 0:
        if a.jtype == "none":
            a.noise_lsb = 3.0
        else:
            jrms = a.jpk if a.jtype == "cw" else a.jpk / 4.0
            a.noise_lsb = min(3.0, jrms / np.sqrt(2 * 10 ** (a.jn / 10)))
    sig_n = a.noise_lsb
    nvar = 2 * sig_n ** 2                     # complex noise power
    noise = sig_n * (rng.standard_normal((2, n)) + 1j * rng.standard_normal((2, n)))

    # GNSS: BPSK PN, C/N0 -> per-sample SNR = C/N0 - 10log10(FS)
    code = rng.choice([-1.0, 1.0], 1023)
    chip_idx = (np.floor(t * CHIP / FS).astype(int)) % 1023
    s_pow = nvar * 10 ** ((a.cn0 - 10 * np.log10(FS)) / 10)
    dop = 1200.0
    s = np.sqrt(s_pow) * code[chip_idx] * np.exp(2j * np.pi * dop * t / FS)
    sat = np.outer(steer(a.saz), s)

    # jammer
    jp = nvar * 10 ** (a.jn / 10)
    if a.jtype == "cw":
        jb = np.exp(2j * np.pi * a.jf * t / FS + 1j * 0.3)
        # slight phase noise (cheap jammer oscillator)
        jb = jb * np.exp(1j * np.cumsum(rng.standard_normal(n)) * 2e-4)
    elif a.jtype == "noise":
        w = rng.standard_normal(n) + 1j * rng.standard_normal(n)
        W = np.fft.fft(w)
        f = np.fft.fftfreq(n, 1 / FS)
        W[np.abs(f) > a.jbw / 2] = 0
        jb = np.fft.ifft(W)
        jb /= np.sqrt(np.mean(np.abs(jb) ** 2))
    else:
        jb = np.zeros(n, complex)
    jb = np.sqrt(jp) * jb
    jb[: a.jon] = 0
    jam = np.outer(steer(a.jaz), jb)
    if a.jtype != "none" and a.mp > -99:
        jm = frac_delay(jb, a.mpdel) * 10 ** (a.mp / 20) * np.exp(1j * 1.1)
        jm[: a.jon] = 0
        jam = jam + np.outer(steer(a.mpaz), jm)

    x = sat + jam + noise

    # channel mismatch (RX2 relative to RX1)
    x[1] = 1.06 * np.exp(1j * np.deg2rad(37)) * x[1]
    x[0] = one_pole(x[0], 0.05)
    x[1] = one_pole(x[1], 0.12)
    x[1] = frac_delay(x[1], 0.08)
    # IQ imbalance (different per channel)
    x[0] = iq_imbalance(x[0], 0.25 * a.iq, 1.5 * a.iq)
    x[1] = iq_imbalance(x[1], -0.35 * a.iq, -2.2 * a.iq)
    # DC offset residue
    x[0] += 0.7 + 0.4j
    x[1] += -0.5 + 0.9j

    # 12-bit ADC
    q = np.round(np.stack([x.real, x.imag], axis=1))  # shape (2,2,n)
    clipped = np.mean((q > 2047) | (q < -2048))
    q = np.clip(q, -2048, 2047).astype(np.int64)

    with open(a.name + ".hex", "w") as fh:
        for i in range(n):
            vals = [q[0, 0, i], q[0, 1, i], q[1, 0, i], q[1, 1, i]]
            fh.write(" ".join("%04x" % (v & 0xFFFF) for v in vals) + "\n")
    np.savez(a.name + ".npz", code=code, cn0=a.cn0, dop=dop, jon=a.jon,
             noise_lsb=a.noise_lsb, jn=a.jn, jtype=a.jtype)
    peak = np.max(np.abs(q))
    print("%s: n=%d jtype=%s J/N=%.1f dB  noise=%.2f LSB  peak=%d LSB  clipped=%.4f%%" %
          (a.name, n, a.jtype, a.jn, a.noise_lsb, peak, 100 * clipped))


if __name__ == "__main__":
    main()
