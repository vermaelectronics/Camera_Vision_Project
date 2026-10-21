#!/usr/bin/env python3
"""
STAP RTL IP - golden reference model and test vector generator.

  gen   : builds the GNSS anti-jamming scenario of iniSettings.m / STAP_constraint.m
          (GPS L1 C/A signal + 3 wideband phase-modulated jammers + noise on a 7-element ULA,
          true-time-delay wideband array model) and writes the testbench stimulus.
  check : compares the RTL dump against a bit-accurate model of the IP
          (integer covariance, fp32 LDL^H solver with the exact RTL operation
          order, fixed-point weight quantisation and the beamformer datapath),
          reports anti-jamming performance and draws the array response.

Usage:
  python3 -I model/stap_model.py gen   --cfg full --out sim/work_full
  python3 -I model/stap_model.py check --cfg full --out sim/work_full [--plots docs]
"""
import argparse
import json
import os
import sys

import numpy as np

# ----------------------------------------------------------------------------
# IP configurations (must match the parameters passed to the testbench)
# ----------------------------------------------------------------------------
CFGS = {        # min_gain: required SINR improvement [dB] for the regression
    "small": dict(M=4, N=4, LOG2L=10, mode="mvdr", min_gain=20),
    "pi":    dict(M=4, N=8, LOG2L=10, mode="pi", min_gain=20),
    "full":  dict(M=7, N=16, LOG2L=12, mode="mvdr", min_gain=35),
}
DW, WW, WFRAC, OW = 16, 18, 16, 16
NB = 3072                       # samples streamed after the weight update

# ----------------------------------------------------------------------------
# iniSettings.m
# ----------------------------------------------------------------------------
C = 3e8
FC = 1575.42e6
LAMBDA = C / FC
FS = 60e6
TS = 1 / FS
IF = 10e6
CODE_FREQ = 1.023e6
SNR_DB = -31.37
PS = 1.0
PI_J = 100e3
WBIF = [8e6, 10e6, 12e6]
STHETA = 0.0
ITHETA = [-20.0, 30.0, 50.0]
D_EL = LAMBDA / 2


def ca_code(prn=1):
    """GPS C/A code (generateCAcode.m), +/-1 chips."""
    g2s = {1: 5, 2: 6, 3: 7, 4: 8, 5: 17, 6: 18, 7: 139, 8: 140}
    shift = g2s[prn]
    g1 = np.zeros(1023)
    g2 = np.zeros(1023)
    r1 = -np.ones(10)
    r2 = -np.ones(10)
    for i in range(1023):
        g1[i] = r1[9]
        g2[i] = r2[9]
        s1 = r1[2] * r1[9]
        s2 = r2[1] * r2[2] * r2[5] * r2[7] * r2[8] * r2[9]
        r1 = np.concatenate(([s1], r1[:9]))
        r2 = np.concatenate(([s2], r2[:9]))
    g2 = np.concatenate((g2[1023 - shift:], g2[:1023 - shift]))
    return -(g1 * g2)


def array_delay(x, theta_deg, m_el, nfft):
    """True time delay wideband array model (GenArraySignal.m) via FFT."""
    f = np.fft.fftfreq(nfft, TS)
    tau = np.arange(m_el)[:, None] * D_EL * np.sin(np.deg2rad(theta_deg)) / C
    X = np.fft.fft(x, nfft)
    return np.fft.ifft(X[None, :] * np.exp(-1j * 2 * np.pi * (FC + f)[None, :] * tau), axis=1)


def steering(m_el, n_tap, theta_deg=STHETA, f_if=IF):
    s_s = np.exp(-1j * 2 * np.pi * D_EL * np.sin(np.deg2rad(theta_deg)) / LAMBDA * np.arange(m_el))
    s_t = np.exp(1j * 2 * np.pi * f_if * TS * np.arange(n_tap))
    return np.kron(s_s, s_t)


def scenario(cfg, seed=2024):
    rng = np.random.default_rng(seed)
    M, N, L = cfg["M"], cfg["N"], 1 << cfg["LOG2L"]
    na = L + N - 1 + 64
    ns = na + NB
    nfft = 1 << int(np.ceil(np.log2(ns)))
    t = np.arange(nfft) * TS
    # GPS L1 C/A, BPSK at IF (GenGPSL1_t.m)
    code = ca_code(1)
    xs = np.sqrt(PS) * code[(np.floor(t / (1 / CODE_FREQ)).astype(int)) % 1023] \
        * np.exp(1j * 2 * np.pi * IF * t)
    # wideband random phase-modulated jammers (GenWBInterSig2.m, used by
    # STAP_constraint.m): phase = 2*pi*f_IF*n*ts + Kpm*Un, Un ~ N(0, 0.1)
    kpm, sigma2 = 100.0, 0.1
    jam = [np.sqrt(PI_J) * np.exp(1j * (2 * np.pi * fj * t
                                        + kpm * np.sqrt(sigma2) * rng.standard_normal(nfft)))
           for fj in WBIF]
    s_in = array_delay(xs, STHETA, M, nfft)[:, :ns]
    j_in = sum(array_delay(jx, th, M, nfft) for jx, th in zip(jam, ITHETA))[:, :ns]
    pn = PS / 10 ** (SNR_DB / 10)
    nz = np.sqrt(pn / 2) * (rng.standard_normal((M, ns)) + 1j * rng.standard_normal((M, ns)))
    y = s_in + j_in + nz
    scale = 0.9 * (2 ** (DW - 1) - 1) / np.max(np.abs(np.concatenate((y.real, y.imag))))
    lo, hi = -2 ** (DW - 1), 2 ** (DW - 1) - 1
    yq = np.clip(np.round(y.real * scale), lo, hi) + 1j * np.clip(np.round(y.imag * scale), lo, hi)
    return dict(s=s_in * scale, j=j_in * scale, n=nz * scale, yq=yq, scale=scale,
                pn_q=pn * scale ** 2, na=na, ns=ns)


# ----------------------------------------------------------------------------
# fp32 helpers
# ----------------------------------------------------------------------------
f32 = np.float32


def f2u(x):
    return int(np.asarray(x, dtype=np.float32).view(np.uint32))


def u2f(u):
    return np.uint32(u).view(np.float32)


def flz(a):
    """flush denormals to zero like the RTL units."""
    a = np.asarray(a, dtype=np.float32)
    return np.where(np.abs(a) < np.finfo(np.float32).tiny, f32(0), a).astype(np.float32)


def cmac(acc_re, acc_im, ar, ai, br, bi, first):
    """One engine term: acc = (first ? 0 : acc) - a*conj(b), RTL op order."""
    rr = flz(ar * br)
    ii = flz(ai * bi)
    ir = flz(ai * br)
    ri = flz(ar * bi)
    pr = flz(rr + ii)
    pi = flz(ir + (-ri))
    if first:
        acc_re = np.zeros_like(pr)
        acc_im = np.zeros_like(pi)
    return flz(acc_re + (-pr)), flz(acc_im + (-pi))


# ----------------------------------------------------------------------------
# bit-accurate IP model
# ----------------------------------------------------------------------------
def cov_model(yq, M, N, LOG2L, diag_load):
    L = 1 << LOG2L
    nd = M * N
    A = np.zeros((nd, L), dtype=np.int64)
    B = np.zeros((nd, L), dtype=np.int64)
    for m in range(M):
        for t in range(N):
            A[m * N + t] = yq[m, t:t + L].real.astype(np.int64)
            B[m * N + t] = yq[m, t:t + L].imag.astype(np.int64)
    acc_re = A @ A.T + B @ B.T
    acc_im = B @ A.T - A @ B.T
    acc_re = acc_re + np.eye(nd, dtype=np.int64) * (int(diag_load) << LOG2L)
    sc = f32(2.0 ** -LOG2L)
    r_re = (acc_re.astype(np.float32) * sc).astype(np.float32)
    r_im = (acc_im.astype(np.float32) * sc).astype(np.float32)
    return r_re, r_im


def solver_model(r_re, r_im, s_re, s_im):
    """fp32 LDL^H MVDR solver with the exact RTL operation order."""
    n = r_re.shape[0]
    L_re = np.tril(r_re).astype(np.float32)
    L_im = np.tril(r_im).astype(np.float32)
    D = np.zeros(n, np.float32)
    Di = np.zeros(n, np.float32)
    one, mone, nz = f32(1), f32(-1), f32(-0.0)
    for j in range(n):
        v_re = np.zeros(j, np.float32)
        v_im = np.zeros(j, np.float32)
        if j:
            v_re, v_im = cmac(None, None, L_re[j, :j], L_im[j, :j], -D[:j],
                              np.full(j, nz, np.float32), True)
        rows = slice(j, n)
        a_re, a_im = cmac(None, None, L_re[rows, j], L_im[rows, j],
                          np.full(n - j, mone, np.float32), np.zeros(n - j, np.float32), True)
        for k in range(j):
            a_re, a_im = cmac(a_re, a_im, L_re[rows, k], L_im[rows, k], v_re[k], v_im[k], False)
        L_re[rows, j], L_im[rows, j] = a_re, a_im
        D[j] = a_re[0]
        if not D[j] > 0:
            raise RuntimeError(f"non positive pivot at column {j}")
        Di[j] = flz(one / D[j])
        if j < n - 1:
            r2 = slice(j + 1, n)
            L_re[r2, j], L_im[r2, j] = cmac(None, None, L_re[r2, j], L_im[r2, j],
                                            -Di[j], nz, True)
    x_re = np.zeros(n, np.float32)
    x_im = np.zeros(n, np.float32)
    for i in range(n):                                    # forward
        a = cmac(None, None, s_re[i], s_im[i], mone, f32(0), True)
        for k in range(i):
            a = cmac(a[0], a[1], L_re[i, k], L_im[i, k], x_re[k], -x_im[k], False)
        x_re[i], x_im[i] = a
    x_re, x_im = cmac(None, None, x_re, x_im, -Di, np.full(n, nz, np.float32), True)
    for i in range(n - 1, -1, -1):                        # backward
        a = cmac(None, None, x_re[i], x_im[i], mone, f32(0), True)
        for k in range(i + 1, n):
            a = cmac(a[0], a[1], x_re[k], x_im[k], L_re[k, i], L_im[k, i], False)
        x_re[i], x_im[i] = a
    a = (None, None)
    for q in range(n):                                    # alpha = s^H x
        a = cmac(a[0], a[1], x_re[q], x_im[q], -s_re[q], -s_im[q], q == 0)
    alpha = f32(a[0])
    ainv = flz(one / alpha)
    w_re, w_im = cmac(None, None, x_re, x_im, -ainv, nz, True)
    return w_re, w_im, alpha


def fp2fix(f, k):
    u = f2u(f)
    e = (u >> 23) & 0xFF
    if e == 0:
        return 0
    m = (1 << 23) | (u & 0x7FFFFF)
    s = e - 150 + k + WFRAC
    maxmag = (1 << (WW - 1)) - 1
    if s >= 0:
        mag = maxmag if s >= WW - 24 else m << s
    else:
        sh = -s
        mag = 0 if sh > 25 else (m + (1 << (sh - 1))) >> sh
    mag = min(mag, maxmag)
    return -mag if (u >> 31) else mag


def wload_model(w_re, w_im):
    exps = [(f2u(v) >> 23) & 0xFF for v in np.concatenate((w_re, w_im))]
    emax = max(exps)
    k = 0 if emax == 0 else 126 - emax
    wr = np.array([fp2fix(v, k) for v in w_re], dtype=np.int64)
    wi = np.array([fp2fix(v, k) for v in w_im], dtype=np.int64)
    return wr, wi, k


def bf_model(yq, wr, wi, M, N, idx, out_shift):
    """beamformer output for input sample indices idx (newest tap)."""
    out = []
    for kk in idx:
        acc_r = 0
        acc_i = 0
        for m in range(M):
            for t in range(N):
                n = kk - (N - 1) + t
                xr, xi = (int(yq[m, n].real), int(yq[m, n].imag)) if n >= 0 else (0, 0)
                i = m * N + t
                acc_r += int(wr[i]) * xr + int(wi[i]) * xi
                acc_i += int(wr[i]) * xi - int(wi[i]) * xr
        out.append((rnd_sat(acc_i, out_shift) << OW) | (rnd_sat(acc_r, out_shift) & 0xFFFF))
    return out


def rnd_sat(v, sh):
    r = (v + ((1 << (sh - 1)) if sh else 0)) >> sh
    r = max(-(1 << (OW - 1)), min((1 << (OW - 1)) - 1, r))
    return r & ((1 << OW) - 1)


# ----------------------------------------------------------------------------
# helpers for the space-time snapshots / performance
# ----------------------------------------------------------------------------
def snapshots(x, M, N, start, count):
    X = np.zeros((M * N, count), dtype=complex)
    for m in range(M):
        for t in range(N):
            X[m * N + t] = x[m, start + t:start + t + count]
    return X


def out_power(w, x, M, N, start, count):
    X = snapshots(x, M, N, start, count)
    return np.mean(np.abs(np.conj(w) @ X) ** 2)


def to_s16(v):
    v = int(v) & 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


# ----------------------------------------------------------------------------
# gen
# ----------------------------------------------------------------------------
def cmd_gen(cfg, out):
    os.makedirs(out, exist_ok=True)
    M, N = cfg["M"], cfg["N"]
    sc = scenario(cfg)
    yq = sc["yq"]
    with open(os.path.join(out, "stim.hex"), "w") as fh:
        for n in range(sc["ns"]):
            word = 0
            for m in range(M):
                re = int(yq[m, n].real) & 0xFFFF
                im = int(yq[m, n].imag) & 0xFFFF
                word |= ((im << 16) | re) << (32 * m)
            fh.write(f"{word:0{M * 8}x}\n")
    s = steering(M, N) if cfg["mode"] == "mvdr" else np.eye(M * N)[0]
    with open(os.path.join(out, "steer.hex"), "w") as fh:
        for v in s:
            fh.write(f"{f2u(f32(v.real)):08x}\n{f2u(f32(v.imag)):08x}\n")
    diag_load = int(round(0.1 * sc["pn_q"]))           # -10 dB w.r.t. noise
    ctrl = 0x1 | (0x4 if cfg["mode"] == "mvdr" else 0) | 0x100
    with open(os.path.join(out, "cfg.hex"), "w") as fh:
        fh.write(f"{diag_load:08x}\n{WFRAC:08x}\n{ctrl:08x}\n{sc['na']:08x}\n{sc['ns']:08x}\n")
    np.savez(os.path.join(out, "scenario.npz"), **{k: v for k, v in sc.items()})
    json.dump(dict(cfg=cfg, diag_load=diag_load, scale=sc["scale"]),
              open(os.path.join(out, "gen.json"), "w"), indent=1)
    print(f"[gen] M={M} N={N} L={1 << cfg['LOG2L']} mode={cfg['mode']} samples={sc['ns']} "
          f"scale={sc['scale']:.2f} diag_load={diag_load}")


# ----------------------------------------------------------------------------
# check
# ----------------------------------------------------------------------------
def read_hex_words(path):
    return [int(ln.split()[0], 16) for ln in open(path) if ln.strip() and not ln.startswith("//")]


def cmd_check(cfg, out, plots):
    M, N, LOG2L = cfg["M"], cfg["N"], cfg["LOG2L"]
    nd = M * N
    sc = dict(np.load(os.path.join(out, "scenario.npz")))
    meta = json.load(open(os.path.join(out, "gen.json")))
    yq = sc["yq"]
    ok = True

    # 1. covariance (bit exact) ---------------------------------------------
    r_re, r_im = cov_model(yq, M, N, LOG2L, meta["diag_load"])
    words = read_hex_words(os.path.join(out, "rtl_cov.hex"))
    bad = 0
    for i in range(nd):
        for j in range(i + 1):
            w = words[i * (i + 1) // 2 + j]
            if (w & 0xFFFFFFFF) != f2u(r_re[i, j]) or (w >> 32) != f2u(r_im[i, j]):
                bad += 1
    print(f"[check] covariance   : {nd * (nd + 1) // 2} entries, {bad} mismatches "
          f"{'PASS' if bad == 0 else 'FAIL'}")
    ok &= bad == 0

    # 2. fp32 weights (bit exact vs model, accuracy vs float64) -----------
    s = steering(M, N) if cfg["mode"] == "mvdr" else np.eye(nd)[0]
    s_re = s.real.astype(np.float32)
    s_im = s.imag.astype(np.float32)
    w_re, w_im, alpha = solver_model(r_re, r_im, s_re, s_im)
    rtl = read_hex_words(os.path.join(out, "rtl_w.hex"))
    rw_re = np.array([u2f(rtl[2 * i]) for i in range(nd)], np.float32)
    rw_im = np.array([u2f(rtl[2 * i + 1]) for i in range(nd)], np.float32)
    bad = int(np.sum((rw_re != w_re) | (rw_im != w_im)))
    print(f"[check] fp32 weights : {nd} complex, {bad} mismatches vs bit-accurate model "
          f"{'PASS' if bad == 0 else 'FAIL'}")
    ok &= bad == 0
    R64 = r_re.astype(np.float64) + 1j * r_im.astype(np.float64)
    R64 = np.tril(R64) + np.tril(R64, -1).conj().T
    Ri_s = np.linalg.solve(R64, s)
    w64 = Ri_s / (np.conj(s) @ Ri_s)
    w_rtl = rw_re.astype(np.float64) + 1j * rw_im
    rel = np.linalg.norm(w_rtl - w64) / np.linalg.norm(w64)
    print(f"[check] fp32 weights : relative error vs float64 SMI = {rel:.2e} "
          f"(cond(R) = {np.linalg.cond(R64):.2e}) {'PASS' if rel < 1e-2 else 'FAIL'}")
    ok &= rel < 1e-2

    # 3. fixed point weights -------------------------------------------------
    wr, wi, k = wload_model(rw_re, rw_im)
    fx = read_hex_words(os.path.join(out, "rtl_wfix.hex"))
    def sx(v):
        return v - (1 << WW) if v & (1 << (WW - 1)) else v
    fr = np.array([sx(fx[2 * i]) for i in range(nd)])
    fi = np.array([sx(fx[2 * i + 1]) for i in range(nd)])
    stat = dict(ln.split("=") for ln in open(os.path.join(out, "rtl_status.txt")).read().split())
    k_rtl = int(stat["wexp"])
    bad = int(np.sum((fr != wr) | (fi != wi))) + (k_rtl != k)
    print(f"[check] fixed weights: block exponent k={k_rtl} (model {k}), "
          f"{bad} mismatches {'PASS' if bad == 0 else 'FAIL'}")
    ok &= bad == 0

    # 4. beamformer output stream (bit exact) ------------------------------
    rows = [ln.split() for ln in open(os.path.join(out, "rtl_out.txt"))]
    oshift = WFRAC
    e1r = np.zeros(nd, np.int64)
    e1r[0] = 1 << WFRAC
    pre = [(int(r[0]), int(r[1], 16)) for r in rows if r[2] == "0" and int(r[0]) >= N - 1]
    post = [(int(r[0]), int(r[1], 16)) for r in rows if r[2] == "1"]
    pre = pre[:: max(1, len(pre) // 300)]
    ref_pre = bf_model(yq, e1r, np.zeros(nd, np.int64), M, N, [p[0] for p in pre], oshift)
    ref_post = bf_model(yq, wr, wi, M, N, [p[0] for p in post], oshift)
    bad_pre = sum(a[1] != b for a, b in zip(pre, ref_pre))
    bad_post = sum(a[1] != b for a, b in zip(post, ref_post))
    print(f"[check] output stream: {len(pre)} pre-update + {len(post)} post-update samples, "
          f"{bad_pre + bad_post} mismatches {'PASS' if bad_pre + bad_post == 0 else 'FAIL'}")
    ok &= (bad_pre + bad_post) == 0 and len(post) > 0

    # 5. anti-jamming performance -------------------------------------------
    na, ns = int(sc["na"]), int(sc["ns"])
    cnt = ns - na - N
    w_fx = (fr + 1j * fi).astype(complex)
    def sinr(w):
        ps = out_power(w, sc["s"], M, N, na, cnt)
        pj = out_power(w, sc["j"], M, N, na, cnt)
        pn = out_power(w, sc["n"], M, N, na, cnt)
        return ps, pj, pn
    ps_in = np.mean(np.abs(sc["s"]) ** 2)
    pj_in = np.mean(np.abs(sc["j"]) ** 2)
    pn_in = np.mean(np.abs(sc["n"]) ** 2)
    sinr_in = 10 * np.log10(ps_in / (pj_in + pn_in))
    res = {}
    for name, w in (("RTL (fixed point)", w_fx), ("float64 SMI", w64)):
        ps, pj, pn = sinr(w)
        g = np.abs(np.conj(w) @ s) ** 2 if cfg["mode"] == "mvdr" else np.abs(w[0]) ** 2
        res[name] = dict(sinr=10 * np.log10(ps / (pj + pn)),
                         jri=10 * np.log10(pj_in * g / pj),
                         jnr=10 * np.log10(pj / pn))
    yo = np.array([to_s16(d) + 1j * to_s16(d >> 16) for _, d in post])
    print(f"[perf]  input SINR = {sinr_in:6.2f} dB   (JSR = {10*np.log10(pj_in/ps_in):.1f} dB, "
          f"JNR = {10*np.log10(pj_in/pn_in):.1f} dB per element)")
    for name, r in res.items():
        print(f"[perf]  {name:18s}: output SINR = {r['sinr']:6.2f} dB, "
              f"jammer suppression = {r['jri']:5.1f} dB, residual JNR = {r['jnr']:5.1f} dB")
    sinr_gain = res["RTL (fixed point)"]["sinr"] - sinr_in
    loss = res["float64 SMI"]["sinr"] - res["RTL (fixed point)"]["sinr"]
    print(f"[perf]  SINR improvement = {sinr_gain:.1f} dB, implementation loss vs float64 = "
          f"{loss:.2f} dB {'PASS' if sinr_gain > cfg['min_gain'] and abs(loss) < 0.5 else 'FAIL'}")
    ok &= sinr_gain > cfg["min_gain"] and abs(loss) < 0.5

    if plots:
        make_plots(cfg, w_fx, w64, sc, post, yo, plots)
    print("[check] RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def make_plots(cfg, w_fx, w64, sc, post, yo, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(outdir, exist_ok=True)
    M, N = cfg["M"], cfg["N"]
    tag = cfg.get("name", "")
    th = np.arange(-90, 90.01, 0.25)

    def resp(w, f_if):
        rf = FC + f_if
        out = []
        for a in th:
            s_s = np.exp(-1j * 2 * np.pi * D_EL * rf * np.sin(np.deg2rad(a)) / C * np.arange(M))
            s_t = np.exp(1j * 2 * np.pi * f_if * TS * np.arange(N))
            out.append(np.conj(w) @ np.kron(s_s, s_t))
        return np.array(out)

    fig, ax = plt.subplots(figsize=(8, 4.2))
    for w, lab, ls in ((w_fx, "RTL fixed-point weights", "-"), (w64, "float64 SMI", "--")):
        r = resp(w / np.max(np.abs(w)), IF)
        ax.plot(th, 20 * np.log10(np.abs(r) / np.max(np.abs(r)) + 1e-12), ls, label=lab)
    for a in ITHETA:
        ax.axvline(a, color="r", alpha=.4, lw=1)
    ax.axvline(STHETA, color="g", alpha=.4, lw=1)
    ax.set(xlabel="azimuth [deg]", ylabel="normalised gain [dB]", ylim=(-90, 5),
           title=f"STAP array response at IF (M={M}, N={N}); red = jammers, green = GPS")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"beam_pattern_{tag}.png"), dpi=110)
    plt.close(fig)

    f = np.linspace(-FS / 2, FS / 2, 121)
    th2 = np.arange(-90, 90.01, 1.0)
    Z = np.zeros((len(f), len(th2)))
    wn = w_fx / np.max(np.abs(w_fx))
    for fi, ff in enumerate(f):
        rf = FC + ff
        for ti, a in enumerate(th2):
            s_s = np.exp(-1j * 2 * np.pi * D_EL * rf * np.sin(np.deg2rad(a)) / C * np.arange(M))
            s_t = np.exp(1j * 2 * np.pi * ff * TS * np.arange(N))
            Z[fi, ti] = np.abs(np.conj(wn) @ np.kron(s_s, s_t))
    Z = 20 * np.log10(Z / Z.max() + 1e-12)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    im = ax.pcolormesh(th2, f / 1e6, Z, shading="auto", cmap="jet", vmin=-80, vmax=0)
    for a in ITHETA:
        ax.axvline(a, color="w", ls=":", lw=1)
    ax.set(xlabel="azimuth [deg]", ylabel="frequency [MHz]",
           title="space-frequency response of the RTL weights (cf. STAP_constraint.m)")
    fig.colorbar(im, label="gain [dB]")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"space_freq_{tag}.png"), dpi=110)
    plt.close(fig)

    def psd(x):
        nseg = 256
        x = x[: len(x) // nseg * nseg].reshape(-1, nseg) * np.hanning(nseg)
        p = np.mean(np.abs(np.fft.fftshift(np.fft.fft(x, axis=1), axes=1)) ** 2, axis=0)
        return 10 * np.log10(p + 1e-12)
    fax = np.fft.fftshift(np.fft.fftfreq(256, TS)) / 1e6
    yin = sc["yq"][0, [p[0] for p in post]]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(fax, psd(yin), label="input, element 0 (jammed)")
    ax.plot(fax, psd(yo), label="RTL STAP output")
    ax.set(xlabel="frequency [MHz]", ylabel="PSD [dB]",
           title="spectrum before / after STAP (RTL output stream)")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, f"psd_{tag}.png"), dpi=110)
    plt.close(fig)
    print(f"[plots] written to {outdir}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["gen", "check"])
    ap.add_argument("--cfg", default="small", choices=sorted(CFGS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--plots", default=None)
    a = ap.parse_args()
    cfg = dict(CFGS[a.cfg], name=a.cfg)
    if a.cmd == "gen":
        cmd_gen(cfg, a.out)
        return 0
    return cmd_check(cfg, a.out, a.plots)


if __name__ == "__main__":
    sys.exit(main())
