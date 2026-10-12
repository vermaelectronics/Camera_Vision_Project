#!/usr/bin/env python3
"""
Reference model for the I-UESPRIT (improved unitary modified ESPRIT) DOA
estimator from:

    J. Li, G. Yang, X. Ren, Z. Fu, S. Qiu, "Modified ESPRIT and
    Unitary-Transformation Form for Computationally Efficient DOA Estimation
    on FPGA", IEEE Trans. Instrum. Meas., vol. 74, 2025, Art. 2003209.

Two implementations are provided:

  * iuesprit_generic()  - the paper's equations (7), (22)-(37), (38) for an
                          arbitrary M-element ULA, using a full EVD of Psi_r.
  * iuesprit_2ant()     - the closed form used by the RTL for M = 2.

For M = 2, C_r1 = K1*C_r is a 1x2 row vector r, so
Psi_r = (r^T r + mu I)^-1 r^T s is rank one and its only non-zero eigenvalue
equals its trace:   psi = (r . s) / (|r|^2 + mu).
No iterative EVD is required. The test at the bottom checks that both
implementations agree.

Usage:
    python3 i_uesprit_model.py selftest
    python3 i_uesprit_model.py gen  <out_dir> [options]   (stimulus for the RTL testbench)
    python3 i_uesprit_model.py check <out_dir>            (compare RTL results with the model)
"""

import argparse
import json
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Paper equations
# ---------------------------------------------------------------------------

def cumulant_matrix(X):
    """Eq. (7): C(m, n) = cum{x_m, x_0^*, x_0, x_n^*}, X is M x T."""
    M, T = X.shape
    x0 = X[0]
    R = X @ X.conj().T / T                      # R(m,n)  = E[x_m x_n^*]
    P = (X * x0).mean(axis=1)                   # P(m)    = E[x_m x_0]
    M4 = (X * np.abs(x0) ** 2) @ X.conj().T / T  # E[x_m |x0|^2 x_n^*]
    C = (M4
         - np.outer(R[:, 0], R[0, :])
         - np.outer(P, P.conj())
         - R * R[0, 0])
    return C


def covariance_matrix(X):
    return X @ X.conj().T / X.shape[1]


def exchange(n):
    return np.fliplr(np.eye(n))


def unitary_q(n):
    """Eqs. (23)/(24)."""
    k = n // 2
    I = np.eye(k)
    Pi = exchange(k)
    if n % 2 == 0:
        Q = np.block([[I, 1j * I], [Pi, -1j * Pi]])
    else:
        z = np.zeros((k, 1))
        Q = np.block([[I, z, 1j * I],
                      [z.T, np.sqrt(2) * np.ones((1, 1)), z.T],
                      [Pi, z, -1j * Pi]])
    return Q / np.sqrt(2)


def iuesprit_generic(C, p=4, mu=1e-6):
    """Improved UESPRIT (Fig. 3, RMA + PIE) for any M. Returns eigenvalues
    psi of Psi_r sorted by |psi| descending, and the matrix used."""
    M = C.shape[0]
    Cp = np.linalg.matrix_power(C, p)                     # Eq. (38)
    Cp = Cp / np.max(np.abs(Cp))                          # scale-free (ESPRIT is scale invariant)
    Pi = exchange(M)
    Cfb = 0.5 * (Cp + Pi @ Cp.conj() @ Pi)                # Eq. (26)
    QM = unitary_q(M)
    Qm = unitary_q(M - 1)
    Cr = np.real(QM.conj().T @ Cfb @ QM)                  # Eq. (25)
    J1 = np.hstack([np.eye(M - 1), np.zeros((M - 1, 1))])
    J2 = np.hstack([np.zeros((M - 1, 1)), np.eye(M - 1)])
    K1 = 2 * np.real(Qm.conj().T @ J2 @ QM)               # Eq. (30)
    K2 = 2 * np.imag(Qm.conj().T @ J2 @ QM)               # Eq. (31)
    Cr1 = K1 @ Cr                                          # Eq. (33)
    Cr2 = K2 @ Cr                                          # Eq. (34)
    if mu > 0:
        P = np.linalg.solve(Cr1.T @ Cr1 + mu * np.eye(M), Cr1.T)  # Eq. (37)
    else:
        P = np.linalg.pinv(Cr1)                            # mu -> 0 limit
    Psi = P @ Cr2                                          # Eq. (36)
    ev = np.linalg.eigvals(Psi)
    ev = ev[np.argsort(-np.abs(ev))]
    return ev, Psi


def iuesprit_2ant(C, p=4, mu=1e-6):
    """Closed form for M = 2 (what the RTL implements).

    C = [[c00, c01], [c01^*, c11]] (Hermitian).
    mu here is the regulariser of Eq. (37) applied to r = K1*C_r/sqrt(2),
    i.e. mu_rtl = mu_paper / 2 (the sqrt(2) factors of K1, K2 cancel in the
    numerator and denominator)."""
    def bfp(a, d, b):
        # block floating point, as in the RTL: power-of-two scale so that
        # the largest component lies in [0.5, 1)
        m = max(abs(a), abs(d), abs(b.real), abs(b.imag))
        if m == 0:
            return a, d, b
        s = 2.0 ** (np.floor(np.log2(m)) + 1)
        return a / s, d / s, b / s

    a, d, b = bfp(C[0, 0].real, C[1, 1].real, C[0, 1])
    for _ in range(int(np.log2(p))):        # repeated squaring of a 2x2 Hermitian
        a, d, b = bfp(a * a + abs(b) ** 2, d * d + abs(b) ** 2, (a + d) * b)
    A = 0.5 * (a + d)                        # forward-backward average
    cr00, cr01, cr10, cr11 = A + b.real, b.imag, b.imag, A - b.real  # Q^H Cfb Q
    r = np.array([cr00, cr01])               # K1 C_r / sqrt(2)
    s = -np.array([cr10, cr11])              # K2 C_r / sqrt(2)
    num = r @ s
    den = r @ r + mu
    psi = num / den
    phi = 2.0 * np.arctan2(num, den)         # = 2*atan(psi), den > 0
    return psi, phi


def doa_from_phi(phi, k_scale=1.0 / np.pi):
    """Eq. (35): theta = asin(-lambda/(pi d) * atan(psi)) = asin(-k*phi),
    k = lambda / (2 pi d). d = lambda/2 -> k = 1/pi."""
    u = -k_scale * phi
    return np.arcsin(np.clip(u, -1.0, 1.0)), abs(u) > 1.0


# ---------------------------------------------------------------------------
# Stimulus
# ---------------------------------------------------------------------------

def make_snapshots(theta_deg, T, snr_db, amp, d_over_lambda=0.5, rng=None,
                   kind="tone", extra=None):
    """2-element ULA snapshots, phi_m = -2 pi m d sin(theta) / lambda
    (Eq. (2)). Returns 2 x T complex, full scale = 1.0."""
    rng = rng or np.random.default_rng(0)
    th = np.radians(theta_deg)
    a = np.exp(-1j * 2 * np.pi * np.arange(2) * d_over_lambda * np.sin(th))
    n = np.arange(T)
    if kind == "tone":
        s = amp * np.exp(1j * (2 * np.pi * 0.0137 * n + rng.uniform(0, 2 * np.pi)))
    elif kind == "qpsk":
        s = amp * np.exp(1j * (np.pi / 4 + np.pi / 2 * rng.integers(0, 4, T)))
    elif kind == "chirp":
        s = amp * np.exp(1j * np.pi * 1e-4 * n ** 2)
    else:
        raise ValueError(kind)
    X = np.outer(a, s)
    sig_pow = amp ** 2
    npow = sig_pow / 10 ** (snr_db / 10)
    X = X + np.sqrt(npow / 2) * (rng.standard_normal((2, T)) + 1j * rng.standard_normal((2, T)))
    return X


def quantize(X):
    q = lambda v: np.clip(np.round(v * 32768), -32768, 32767).astype(np.int64)
    return q(X.real), q(X.imag)


# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

def selftest():
    rng = np.random.default_rng(1)
    worst = 0.0
    for trial in range(200):
        th = rng.uniform(-80, 80)
        X = make_snapshots(th, 1024, rng.uniform(0, 20), 0.3, rng=rng)
        C = cumulant_matrix(X)
        for p in (1, 2, 4, 8):
            ev, _ = iuesprit_generic(C, p=p, mu=0.0)
            psi, phi = iuesprit_2ant(C, p=p, mu=0.0)
            # one non-zero eigenvalue for M = 2, the other is 0
            assert abs(ev[1]) < 1e-9, ev
            worst = max(worst, abs(ev[0].real - psi) / max(1.0, abs(psi)))
    print("generic EVD vs. closed form: max rel. diff = %.3e" % worst)

    # DOA accuracy sweep (float)
    for snr in (0, 10, 20):
        errs = []
        for th in np.arange(-70, 71, 10):
            X = make_snapshots(th, 1024, snr, 0.3, rng=rng)
            _, phi = iuesprit_2ant(cumulant_matrix(X), p=4)
            est, _ = doa_from_phi(phi)
            errs.append(np.degrees(est) - th)
        print("SNR %2d dB: RMSE = %.3f deg" % (snr, np.sqrt(np.mean(np.square(errs)))))
    print("selftest OK")


def gen(out_dir, args):
    os.makedirs(out_dir, exist_ok=True)
    rng = np.random.default_rng(args.seed)
    T = 1 << args.log2_snapshots
    thetas = [float(t) for t in args.thetas.split(",")]
    i0, q0, i1, q1 = [], [], [], []
    windows = []
    for k, th in enumerate(thetas):
        kind = args.kinds.split(",")[k % len(args.kinds.split(","))]
        X = make_snapshots(th, T, args.snr, args.amp, rng=rng, kind=kind)
        I, Q = quantize(X)
        Xq = (I + 1j * Q) / 32768.0
        Cm = covariance_matrix(Xq) if args.use_cov else cumulant_matrix(Xq)
        psi, phi = iuesprit_2ant(Cm, p=1 << args.p_log2, mu=args.mu_q30 / 2.0 ** 30)
        est, amb = doa_from_phi(phi, args.k_q30 / 2.0 ** 30)
        windows.append(dict(theta_true=th, kind=kind, psi=float(psi), phi=float(phi),
                            theta_model_deg=float(np.degrees(est)), ambiguous=bool(amb)))
        i0 += list(I[0]); q0 += list(Q[0]); i1 += list(I[1]); q1 += list(Q[1])
    with open(os.path.join(out_dir, "stim.hex"), "w") as f:
        for a, b, c, d in zip(i0, q0, i1, q1):
            # tdata = {q1, i1, q0, i0}
            f.write("%04x%04x%04x%04x\n" % (d & 0xFFFF, c & 0xFFFF, b & 0xFFFF, a & 0xFFFF))
    cfg = dict(log2_snapshots=args.log2_snapshots, p_log2=args.p_log2,
               mu_q30=args.mu_q30, k_q30=args.k_q30, use_cov=int(args.use_cov),
               n_samples=len(i0), windows=windows)
    with open(os.path.join(out_dir, "expected.json"), "w") as f:
        json.dump(cfg, f, indent=1)
    with open(os.path.join(out_dir, "cfg.vh"), "w") as f:
        f.write("localparam integer CFG_LOG2_SNAP = %d;\n" % args.log2_snapshots)
        f.write("localparam integer CFG_P_LOG2 = %d;\n" % args.p_log2)
        f.write("localparam integer CFG_MU_Q30 = %d;\n" % args.mu_q30)
        f.write("localparam integer CFG_K_Q30 = %d;\n" % args.k_q30)
        f.write("localparam integer CFG_USE_COV = %d;\n" % int(args.use_cov))
        f.write("localparam integer N_SAMPLES = %d;\n" % len(i0))
        f.write("localparam integer N_WINDOWS = %d;\n" % len(thetas))
    print("wrote %d samples, %d windows to %s" % (len(i0), len(thetas), out_dir))


def check(out_dir, tol_deg):
    exp = json.load(open(os.path.join(out_dir, "expected.json")))
    res = [l.split() for l in open(os.path.join(out_dir, "rtl_results.txt")) if l.strip()]
    s32 = lambda v: v - (1 << 32) if v & 0x80000000 else v
    ok = True
    if len(res) < len(exp["windows"]):
        print("FAIL: RTL produced %d results, expected %d" % (len(res), len(exp["windows"])))
        ok = False
    print(" win  kind   true[deg]  model[deg]   rtl[deg]  |rtl-model|   psi_model   psi_rtl  flags")
    worst = 0.0
    for k, (w, r) in enumerate(zip(exp["windows"], res)):
        phi_q28, psi_q24, th_q28, deg_q16, flags = [int(x, 16) for x in r]
        deg = s32(deg_q16) / 65536.0
        psi = s32(psi_q24) / 2.0 ** 24
        err = abs(deg - w["theta_model_deg"])
        worst = max(worst, err)
        bad = err > tol_deg
        ok &= not bad
        print("%4d  %-5s %10.4f %11.4f %10.4f %12.5f %11.5f %9.5f  %x %s" % (
            k, w["kind"], w["theta_true"], w["theta_model_deg"], deg, err,
            w["psi"], psi, flags, "<-- FAIL" if bad else ""))
    print("worst |RTL - model| = %.5f deg (tolerance %.3f)" % (worst, tol_deg))
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("selftest")
    g = sub.add_parser("gen")
    g.add_argument("out_dir")
    g.add_argument("--thetas", default="-60,-35,-10,0,12.5,30,45,70")
    g.add_argument("--kinds", default="tone,qpsk,chirp")
    g.add_argument("--snr", type=float, default=10.0)
    g.add_argument("--amp", type=float, default=0.25)
    g.add_argument("--log2_snapshots", type=int, default=10)
    g.add_argument("--p_log2", type=int, default=2)
    g.add_argument("--mu_q30", type=int, default=1024)
    g.add_argument("--k_q30", type=int, default=int(round(2 ** 30 / np.pi)))
    g.add_argument("--use_cov", action="store_true")
    g.add_argument("--seed", type=int, default=7)
    c = sub.add_parser("check")
    c.add_argument("out_dir")
    c.add_argument("--tol_deg", type=float, default=0.05)
    a = ap.parse_args()
    if a.cmd == "selftest":
        selftest()
    elif a.cmd == "gen":
        gen(a.out_dir, a)
    else:
        sys.exit(check(a.out_dir, a.tol_deg))


if __name__ == "__main__":
    main()
