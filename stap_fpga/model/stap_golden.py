#!/usr/bin/env python3
"""
stap_golden.py -- bit-true golden model + test-vector generator for the STAP FPGA core.

1. Re-creates the scenario of the original MATLAB code (iniSettings.m):
   GPS L1 C/A (PRN 1) at 10 MHz IF, fs = 60 MHz, 7-element ULA (lambda/2),
   3 wideband jammers at -20/+30/+50 deg, 50 dB JSR, SNR = -31.37 dB,
   true-time-delay wideband array model (GenArraySignal.m).
2. Quantises the antenna samples like an ADC (DW-bit I/Q).
3. Computes the stap_corr results bit-exactly (integer arithmetic).
4. Calls the C weight solver (sw/stap_solve_cli) -> integer weights + output shift.
5. Computes the stap_fir output bit-exactly.
6. Writes the vectors for the Verilog testbench and prints a performance report
   (jammer suppression, null depths, SINR) of the fixed-point implementation
   versus the floating-point MATLAB algorithm, measured OUT-OF-SAMPLE.

Only numpy is required.
"""
import argparse, json, os, subprocess, sys
import numpy as np

# ----------------------------------------------------------------------------- scenario
C, FC, CODE_F = 3e8, 1575.42e6, 1.023e6
LAM = C / FC
IF, FS, NFFT, NSEG = 10e6, 60e6, 1024, 50
TS, SN = 1 / FS, NFFT * NSEG
M, N = 7, 16
T = M * N
NL, NP = 2 * N - 1, M * (M + 1) // 2
D = LAM / 2


def configure(m=7, n=16, fs=60e6, fif=10e6):
    """Set the array size and sampling of the scenario (module-level globals)."""
    global M, N, T, NL, NP, FS, TS, IF
    M, N, FS, IF = m, n, fs, fif
    T, NL, NP, TS = M * N, 2 * N - 1, M * (M + 1) // 2, 1 / FS


def ca_code(prn=1):
    g2s = [5, 6, 7, 8, 17, 18, 139, 140, 141, 251, 252, 254, 255, 256, 257, 258, 469, 470,
           471, 472, 473, 474, 509, 512, 513, 514, 515, 516, 859, 860, 861, 862]
    def lfsr(taps):
        r, out = [-1] * 10, []
        for _ in range(1023):
            out.append(r[9])
            fb = 1
            for t in taps:
                fb *= r[t - 1]
            r = [fb] + r[:9]
        return np.array(out, float)
    g1, g2 = lfsr([3, 10]), lfsr([2, 3, 6, 8, 9, 10])
    sh = g2s[prn - 1]
    g2 = np.concatenate([g2[1023 - sh:], g2[:1023 - sh]])
    return -(g1 * g2)


def array_sig(x, theta_deg):
    """GenArraySignal.m: per-segment FFT, true time delay per bin, overlap/un-window."""
    fgrid = -FS / 2 + np.arange(NFFT) * FS / NFFT
    h = np.hamming(NFFT)
    hinv = 1 / (h + np.roll(h, -NFFT // 2))
    tau = D * np.sin(np.deg2rad(theta_deg)) * np.arange(M) / C
    A = np.exp(-1j * 2 * np.pi * np.outer(tau, fgrid + FC))
    out = np.zeros((M, SN), complex)
    for m, s in enumerate(x.reshape(NSEG, NFFT)):
        sd = np.roll(s, -NFFT // 2)
        F = np.fft.fftshift(np.fft.fft(s * h))
        Fd = np.fft.fftshift(np.fft.fft(sd * h))
        y1 = np.fft.ifft(np.fft.ifftshift(A * F, axes=1), axis=1)
        y2 = np.roll(np.fft.ifft(np.fft.ifftshift(A * Fd, axes=1), axis=1), NFFT // 2, axis=1)
        out[:, m * NFFT:(m + 1) * NFFT] = (y1 + y2) * hinv
    return out


def scenario(seed, jam_db=50.0, snr_db=-31.37, jam_type="rpm",
             jam_theta=(-20.0, 30.0, 50.0), sig_theta=0.0):
    rng = np.random.default_rng(seed)
    t = np.arange(SN) * TS
    xs = ca_code(1)[np.floor(t * CODE_F).astype(int) % 1023] * np.exp(2j * np.pi * IF * t)
    J = len(jam_theta)
    pj = 10 ** (jam_db / 10)
    wbif = IF + np.array([-2e6, 0.0, 2e6, -1e6, 1e6, -3e6])[:J]   # MATLAB: 8/10/12 MHz around a 10 MHz IF
    if jam_type == "rpm":                                   # GenWBInterSig2.m
        un = np.sqrt(0.1) * rng.standard_normal((J, SN))
        jam = np.exp(1j * (2 * np.pi * wbif[:, None] * TS * np.arange(SN) + 100 * un))
    else:                                                   # GenWBInterSig.m (LFM)
        Tt = SN * TS
        tt = t - Tt / 2
        jam = (np.exp(2j * np.pi * wbif[:, None] * tt) * np.exp(2j * np.pi * rng.random((J, 1)))
               * np.exp(1j * np.pi * (20e6 / Tt) * tt ** 2))
    jam *= np.sqrt(pj)
    s_in = array_sig(xs, sig_theta)
    j_in = sum(array_sig(jam[k], jam_theta[k]) for k in range(J))
    pn = 1.0 / 10 ** (snr_db / 10)
    n_in = np.sqrt(pn / 2) * (rng.standard_normal((M, SN)) + 1j * rng.standard_normal((M, SN)))
    return s_in, j_in, n_in


# ----------------------------------------------------------------------------- bit-true models
def adc(x_scaled, dw, adc_bits, align="left"):
    if align == "right":           # AD9361/axi_ad9361 RX format: 12-bit value, sign-extended into 16
        hi = 2 ** (adc_bits - 1)
        q = lambda v: np.clip(np.round(v), -hi, hi - 1).astype(np.int64)
        return q(x_scaled.real), q(x_scaled.imag)
    step = 2 ** (dw - adc_bits)
    lim_hi, lim_lo = 2 ** (dw - 1) - step, -2 ** (dw - 1)
    q = lambda v: np.clip(np.round(v / step) * step, lim_lo, lim_hi).astype(np.int64)
    return q(x_scaled.real), q(x_scaled.imag)


def corr_golden(xr, xi, L):
    """c_ab[q] = sum_l x_a[l+q] * conj(x_b[l+N-1]),  q = 0..2N-2  (tau = q-(N-1))."""
    cre = np.zeros(NP * NL, np.int64)
    cim = np.zeros(NP * NL, np.int64)
    e = 0
    for a in range(M):
        for b in range(a, M):
            br, bi = xr[b, N - 1:N - 1 + L], xi[b, N - 1:N - 1 + L]
            for q in range(NL):
                ar, ai = xr[a, q:q + L], xi[a, q:q + L]
                cre[e] = np.sum(ar * br + ai * bi)
                cim[e] = np.sum(ai * br - ar * bi)
                e += 1
    return cre, cim


def taps(x):
    """(T, K) tap matrix: row m*N+k at column i holds x_m[i-N+1+k] (0 before start)."""
    pad = np.concatenate([np.zeros((M, N - 1), x.dtype), x], axis=1)
    w = np.lib.stride_tricks.sliding_window_view(pad, N, axis=1)  # M, K, N
    return np.transpose(w, (0, 2, 1)).reshape(T, -1)


def fir_golden(xr, xi, wr, wi, shift, ow):
    Xr, Xi = taps(xr), taps(xi)
    yr = wr @ Xr + wi @ Xi                     # Re{conj(w) x}
    yi = wr @ Xi - wi @ Xr                     # Im{conj(w) x}
    lo, hi = -2 ** (ow - 1), 2 ** (ow - 1) - 1
    return np.clip(yr >> shift, lo, hi), np.clip(yi >> shift, lo, hi)


def cov_from_corr(cre, cim, xr, xi, L):
    """Python twin of stap_build_cov(): exact R = Xm Xm^H / L from corr + edge samples."""
    x = xr + 1j * xi
    def idx(a, b, tau):
        return (sum(M - i for i in range(a)) + b - a) * NL + tau + N - 1
    R = np.zeros((T, T), complex)
    for a in range(M):
        for b in range(a, M):
            for k in range(N):
                for kk in range(N):
                    tau = k - kk
                    e = idx(a, b, tau)
                    v = cre[e] + 1j * cim[e]
                    j = np.arange(kk, N - 1)
                    v += np.sum(x[a, j + tau] * np.conj(x[b, j]))
                    v -= np.sum(x[a, j + L + tau] * np.conj(x[b, j + L]))
                    R[a * N + k, b * N + kk] = v / L
                    R[b * N + kk, a * N + k] = np.conj(v) / L
    return R


def steering(theta, f_if=None, frf=None):
    f_if = IF if f_if is None else f_if
    frf = FC + f_if if frf is None else frf
    ss = np.exp(-2j * np.pi * D * frf / C * np.sin(np.deg2rad(theta)) * np.arange(M))
    st = np.exp(2j * np.pi * f_if * TS * np.arange(N))
    return np.kron(ss, st)


# ----------------------------------------------------------------------------- main
def hexs(v, bits):
    return format(int(v) & ((1 << bits) - 1), "0%dx" % ((bits + 3) // 4))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="build")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--L", type=int, default=5000, help="snapshots for covariance")
    ap.add_argument("--K", type=int, default=3000, help="out-of-sample samples checked by the RTL testbench")
    ap.add_argument("--dw", type=int, default=16)
    ap.add_argument("--ww", type=int, default=18)
    ap.add_argument("--ow", type=int, default=16)
    ap.add_argument("--lmax-log2", type=int, default=13)
    ap.add_argument("--adc-bits", type=int, default=14)
    ap.add_argument("--backoff-db", type=float, default=15.0, help="ADC RMS level below full scale")
    ap.add_argument("--headroom", type=int, default=3, help="output RMS headroom bits")
    ap.add_argument("--jam-db", type=float, default=50.0)
    ap.add_argument("--jam", choices=["rpm", "lfm"], default="rpm")
    ap.add_argument("--mode", choices=["mvdr", "pi"], default="mvdr")
    ap.add_argument("--solver", default=None, help="path to stap_solve_cli")
    ap.add_argument("--M", type=int, default=7, help="antenna elements (solver must be built with the same STAP_M)")
    ap.add_argument("--fs", type=float, default=60e6, help="sample rate [Hz]")
    ap.add_argument("--fif", type=float, default=10e6, help="signal IF [Hz] (0 = complex baseband)")
    ap.add_argument("--jam-theta", default="-20,30,50", help="jammer angles [deg], comma separated")
    ap.add_argument("--adc-align", choices=["left", "right"], default="left",
                    help="left: ADC_BITS left-aligned in DW (default); right: AD9361 12-bit right-aligned")
    a = ap.parse_args()
    configure(a.M, 16, a.fs, a.fif)
    jth = tuple(float(v) for v in a.jam_theta.split(","))
    os.makedirs(a.out, exist_ok=True)
    here = os.path.dirname(os.path.abspath(__file__))
    solver = a.solver or os.path.join(a.out, "stap_solve_cli")
    L, SL = a.L, a.L + 2 * N - 2
    CW = 2 * a.dw + 2 + a.lmax_log2

    s_in, j_in, n_in = scenario(a.seed, a.jam_db, jam_type=a.jam, jam_theta=jth)
    tot = s_in + j_in + n_in
    rms = np.sqrt(np.mean(np.abs(tot) ** 2) / 2)               # per I/Q component
    fsc = 2 ** (a.adc_bits - 1) if a.adc_align == "right" else 2 ** (a.dw - 1)
    g = fsc * 10 ** (-a.backoff_db / 20) / rms
    xr, xi = adc(g * tot, a.dw, a.adc_bits, a.adc_align)
    S, J, Nn = g * s_in, g * j_in, g * n_in                     # analysis components (ADC LSB units)
    clip = np.mean((np.abs(xr) >= fsc - 1) | (np.abs(xi) >= fsc - 1))

    # ---- covariance engine (bit exact) + C solver
    cre, cim = corr_golden(xr, xi, L)
    with open(os.path.join(a.out, "corr.txt"), "w") as f:
        for r_, i_ in zip(cre, cim):
            f.write(f"{r_} {i_}\n")
        HT = 2 * N - 2
        for base in (0, L):                                     # head, tail edge samples
            for m in range(M):
                for i in range(HT):
                    f.write(f"{xr[m, base + i]} {xi[m, base + i]}\n")
    # self-check: corr + edge correction == exact Xm*Xm'/L  (what MATLAB computes)
    Xq = taps(xr + 1j * xi)[:, N - 1:N - 1 + L]
    R_exact = Xq @ Xq.conj().T / L
    R_hw = cov_from_corr(cre, cim, xr, xi, L)
    err = np.max(np.abs(R_hw - R_exact)) / np.max(np.abs(R_exact))
    assert err < 1e-12, f"covariance reconstruction error {err}"
    wfile = os.path.join(a.out, "weights.txt")
    subprocess.run([solver, os.path.join(a.out, "corr.txt"), str(L), a.mode, "0", "0.5",
                    repr(IF / FS), str(a.ww), str(a.ow), str(a.headroom), wfile], check=True)
    W = np.loadtxt(wfile, dtype=np.int64)
    wexp, shift = int(W[0, 0]), int(W[0, 1])
    wr, wi = W[1:, 0], W[1:, 1]

    # ---- filter (bit exact)
    yr, yi = fir_golden(xr, xi, wr, wi, shift, a.ow)

    # ---- test vectors
    K = min(a.K, SN - SL)
    with open(os.path.join(a.out, "stim.hex"), "w") as f:
        for i in range(SL + K):
            re_bus = "".join(hexs(xr[m, i], a.dw) for m in reversed(range(M)))
            im_bus = "".join(hexs(xi[m, i], a.dw) for m in reversed(range(M)))
            f.write(re_bus + im_bus + "\n")
    with open(os.path.join(a.out, "corr_exp.hex"), "w") as f:
        for r_, i_ in zip(cre, cim):
            f.write(hexs(r_, 64) + hexs(i_, 64) + "\n")
    with open(os.path.join(a.out, "weights.hex"), "w") as f:
        for r_, i_ in zip(wr, wi):
            f.write(hexs(r_, 32) + hexs(i_, 32) + "\n")
    with open(os.path.join(a.out, "yexp.hex"), "w") as f:
        for i in range(SL, SL + K):
            f.write(hexs(yr[i], a.ow) + hexs(yi[i], a.ow) + "\n")
    with open(os.path.join(a.out, "edges.hex"), "w") as f:      # head then tail, channel-major
        for base in (0, L):
            for m in range(M):
                for i in range(2 * N - 2):
                    f.write(hexs(xr[m, base + i], 16) + hexs(xi[m, base + i], 16) + "\n")
    with open(os.path.join(a.out, "tb_params.vh"), "w") as f:
        f.write(f"`define TB_L {L}\n`define TB_SL {SL}\n`define TB_K {K}\n"
                f"`define TB_SHIFT {shift}\n`define TB_NRES {NP*NL}\n`define TB_T {T}\n"
                f"`define TB_DW {a.dw}\n`define TB_WW {a.ww}\n`define TB_OW {a.ow}\n"
                f"`define TB_LMAX_LOG2 {a.lmax_log2}\n")

    # ---- performance (out of sample: samples after the training block)
    ev = slice(SL + N, SN)
    wq = (wr + 1j * wi) / 2.0 ** wexp                            # effective fixed-point weights
    def through(w, X):                                            # w^H x for every sample
        return np.conj(w) @ taps(X)
    def pwr(v):
        return float(np.mean(np.abs(v) ** 2))
    Pi_in, Pn_in, Ps_in = pwr(J), pwr(Nn), pwr(S)

    # floating-point MATLAB-equivalent reference (float data, full Xm*Xm' covariance)
    Xf = taps(g * tot)[:, N - 1:N - 1 + L]
    Rf = Xf @ Xf.conj().T / L
    Sv = steering(0.0) if a.mode == "mvdr" else np.eye(T)[0]
    u = np.linalg.solve(Rf, Sv)
    wf = u / (Sv.conj() @ u)

    ys_fx = (through(wq, S) * 2.0 ** wexp / 2.0 ** shift)[ev]    # signal part, output scale
    y_fx = (yr + 1j * yi)[ev].astype(complex)
    rep = {
        "adc_bits": a.adc_bits, "adc_clip_fraction": float(clip),
        "SINR_in_dB": 10 * np.log10(Ps_in / (Pi_in + Pn_in)),
        "M": M, "fs_Hz": FS, "IF_Hz": IF, "mode": a.mode, "jammers_deg": list(jth),
        "INR_per_jammer_dB": 10 * np.log10(Pi_in / Pn_in / len(jth)),
        "float_ref": {
            "JRI_dB": 10 * np.log10(Pi_in / pwr(through(wf, J)[ev])),
            "SINR_out_dB": 10 * np.log10(pwr(through(wf, S)[ev]) /
                                         pwr(through(wf, J + Nn)[ev])),
        },
        "fixed_point": {
            "weight_exp": wexp, "out_shift": shift,
            "JRI_dB": 10 * np.log10(Pi_in / pwr(through(wq, J)[ev])),
            "SINR_out_dB_bit_true": 10 * np.log10(pwr(ys_fx) / pwr(y_fx - ys_fx)),
            "output_rms_lsb": float(np.sqrt(pwr(y_fx))),
            "output_clip_fraction": float(np.mean((np.abs(yr[ev]) >= 2 ** (a.ow - 1) - 1) |
                                                  (np.abs(yi[ev]) >= 2 ** (a.ow - 1) - 1))),
        },
        "null_depth_dB": {},
    }
    fband = np.linspace(IF - 10e6, IF + 10e6, 41)          # +-10 MHz around the signal
    look = np.mean([abs(np.conj(wq) @ steering(0.0, f, FC + f)) ** 2 for f in fband])
    for th in jth:
        gq = np.mean([abs(np.conj(wq) @ steering(th, f, FC + f)) ** 2 for f in fband])
        gf = np.mean([abs(np.conj(wf) @ steering(th, f, FC + f)) ** 2 for f in fband])
        lf = np.mean([abs(np.conj(wf) @ steering(0.0, f, FC + f)) ** 2 for f in fband])
        rep["null_depth_dB"][f"{th:+.0f}deg"] = {"fixed": 10 * np.log10(gq / look),
                                               "float": 10 * np.log10(gf / lf)}
    with open(os.path.join(a.out, "report.json"), "w") as f:
        json.dump(rep, f, indent=2)
    print(json.dumps(rep, indent=2))


if __name__ == "__main__":
    main()
