#!/usr/bin/env python3
"""STAP (current SMI-PI, 16 taps/antenna, ARM solves) versus SFAP (FFT, per-bin
2-antenna power inversion, updated every hop in the PL).  Simulation only.

SINR loss = (signal-to-noise ratio of one clean antenna) - (output SINR), in dB,
per 1024-sample window.  The weights are computed from the total input and then
applied to each component (signal, noise, ADC noise, jammer) separately, so the
output power of each component is exact.
"""
import sys, numpy as np
from numpy.lib.stride_tricks import sliding_window_view as swv

FS = 61.44e6
NS = int(6e-3 * FS)            # 6 ms
T_ON = int(1.0e-3 * FS)        # jammer on at 1 ms
SNR_IN_DB = -18.0              # GNSS per-sample SNR on one antenna
WIN = 1024                     # metric window (16.7 us)
rng = np.random.default_rng(1)

def cg(n, p=1.0):
    return np.sqrt(p / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))

def gen_jammer(kind, n):
    t = np.arange(n) / FS
    if kind == "barrage" or kind == "pulse":
        X = np.fft.fft(cg(n))
        f = np.fft.fftfreq(n, 1 / FS)
        X[np.abs(f) > 25e6] = 0
        j = np.fft.ifft(X)
        j /= np.sqrt(np.mean(np.abs(j) ** 2))
        if kind == "pulse":      # 10 us pulses, 100 us period
            m = ((t * 1e6) % 100.0) < 10.0
            j = j * m / np.sqrt(0.1)       # same average power; peak 10x higher
        return j
    if kind == "cw":
        return np.exp(2j * np.pi * 3.1e6 * t)
    if kind.startswith("chirp"):
        per = float(kind[5:]) * 1e-6
        ph = (t % per) / per
        f = -12e6 + 24e6 * ph
        return np.exp(2j * np.pi * np.cumsum(f) / FS)
    raise ValueError(kind)

def delay(x, d):
    return np.concatenate([np.zeros(d, complex), x[:len(x) - d]])

def scene(kind, jn_db, multipath, adc_bits=12):
    n = NS
    # GNSS-like signal: BPSK, 2.046 Mchip/s
    chips = rng.choice([-1.0, 1.0], size=n // int(round(FS / 2.046e6)) + 1)
    s = np.repeat(chips, int(round(FS / 2.046e6)))[:n].astype(complex) * 10 ** (SNR_IN_DB / 20)
    n1, n2 = cg(n), cg(n)
    j = gen_jammer(kind, n)
    j = j * (np.arange(n) >= T_ON) * 10 ** (jn_db / 20)
    h2 = np.array([1.0, 0.12j, -0.05])
    def rx2(x): return np.convolve(x, h2)[:n]
    ps, pj = np.exp(1j * 0.9), np.exp(-1j * 1.7)
    jam1, jam2 = j.copy(), pj * rx2(j)
    if multipath:                       # echo 9 samples later (146 ns), different direction
        e = 0.4 * delay(j, 9)
        jam1 = jam1 + e
        jam2 = jam2 + np.exp(1j * 2.4) * rx2(e)
    sig1, sig2 = s, ps * rx2(s)
    x1 = sig1 + n1 + jam1
    x2 = sig2 + n2 + jam2
    # 12-bit ADC after AGC (jammer/noise rms about -12 dBFS)
    out = {}
    for name, x in (("x1", x1), ("x2", x2)):
        sc = 0.25 * 2047 / np.sqrt(np.mean(np.abs(x[T_ON + 100:]) ** 2) / 2)
        q = np.clip(np.round(x.real * sc), -2048, 2047) + 1j * np.clip(np.round(x.imag * sc), -2048, 2047)
        out[name] = (q / sc) if adc_bits else x
    adc1, adc2 = out["x1"] - x1, out["x2"] - x2
    comps = {"sig": (sig1, sig2), "noise": (n1 + adc1, n2 + adc2), "jam": (jam1, jam2)}
    tot = (out["x1"], out["x2"])
    return comps, tot

# ---------------------------------------------------------------- STAP
NT, REF = 16, 4
def win(x, s, e, L):
    pad = np.concatenate([np.zeros(L - 1, complex), x])
    return swv(pad[s:e + L - 1], L)[:, ::-1]

def stap(comps, tot, solve_every_blocks, lag_blocks, av=0.5):
    B = 4096
    nb = NS // B
    lam = np.concatenate([np.full(NT - 1, 4.0), np.ones(NT)])
    Gavg, w = None, np.zeros(2 * NT - 1, complex)
    outs = {k: np.zeros(NS, complex) for k in comps}
    pending = []            # (available_block, w)
    solve_t = 0
    # first capture after the jammer appears is half an interval later
    first = int(np.ceil(T_ON / B)) + max(1, solve_every_blocks // 2)
    cap_blocks = set()
    b = 0
    while b < nb:
        cap_blocks.add(b)
        b += solve_every_blocks
    cap_blocks = {c for c in cap_blocks if c < int(T_ON / B) - 1} | set(range(first, nb, solve_every_blocks))
    for b in range(nb):
        s, e = b * B, (b + 1) * B
        # weights that become available now
        for a, ww in list(pending):
            if a <= b:
                w = ww; pending.remove((a, ww))
        # apply
        for k in comps:
            w1 = win(comps[k][0], s, e, NT); w2 = win(comps[k][1], s, e, NT)
            d = w1[:, REF]
            U = np.concatenate([np.delete(w1, REF, axis=1), w2], axis=1)
            outs[k][s:e] = d - U @ w
        # capture/solve
        if b in cap_blocks:
            w1 = win(tot[0], s, e, NT); w2 = win(tot[1], s, e, NT)
            d = w1[:, REF]
            U = np.concatenate([np.delete(w1, REF, axis=1), w2], axis=1)
            A = np.concatenate([U, d[:, None]], axis=1)
            G = (A.conj().T @ A) / B
            Gavg = G if Gavg is None else av * Gavg + (1 - av) * G
            Ruu = Gavg[:-1, :-1] + np.diag(lam)
            r = Gavg[:-1, -1]
            wn = np.linalg.solve(Ruu, r)
            pending.append((b + 1 + lag_blocks, wn))
    return outs

# ---------------------------------------------------------------- SFAP
def sfap(comps, tot, K=256, hop=64, lam=0.98, smooth=0, load=1.0, sq=False):
    wa = np.hanning(K + 1)[:-1]
    if sq: wa = wa ** 0.5
    nfr = (NS - K) // hop + 1
    def stft(x):
        fr = swv(x, K)[::hop][:nfr] * wa
        return np.fft.fft(fr, axis=1)
    T = {k: (stft(comps[k][0]), stft(comps[k][1])) for k in comps}
    X1, X2 = stft(tot[0]), stft(tot[1])
    nbv = load * np.sum(wa ** 2)            # noise variance per bin (unit noise)
    R22 = np.zeros(K); R12 = np.zeros(K, complex)
    C = np.zeros((nfr, K), complex)
    for f in range(nfr):
        R22 = lam * R22 + (1 - lam) * np.abs(X2[f]) ** 2
        R12 = lam * R12 + (1 - lam) * X1[f] * np.conj(X2[f])
        a, b = R22, R12
        if smooth:
            kern = np.ones(2 * smooth + 1)
            a = sum(np.roll(R22, i) for i in range(-smooth, smooth + 1))
            b = sum(np.roll(R12, i) for i in range(-smooth, smooth + 1))
        C[f] = b / (a + nbv * (2 * smooth + 1))
    # synthesis with Hann again, normalised
    ws = wa
    norm = np.zeros(NS)
    for f in range(nfr):
        norm[f * hop:f * hop + K] += wa * ws
    outs = {}
    for k in comps:
        Y = T[k][0] - C * T[k][1]
        yf = np.fft.ifft(Y, axis=1) * ws
        y = np.zeros(NS, complex)
        for f in range(nfr):
            y[f * hop:f * hop + K] += yf[f]
        m = norm > 0.5 * norm.max()
        y[m] /= norm[m]; y[~m] = 0
        outs[k] = y
    return outs

def sinr_loss(outs, comps):
    sig = outs["sig"]; no = outs["noise"]; ja = outs["jam"]
    ref = 10 ** (SNR_IN_DB / 10)            # one clean antenna, noise var 1
    nw = NS // WIN
    L = np.zeros(nw)
    for i in range(nw):
        sl = slice(i * WIN, (i + 1) * WIN)
        ps = np.mean(np.abs(sig[sl]) ** 2); pn = np.mean(np.abs(no[sl]) ** 2) + np.mean(np.abs(ja[sl]) ** 2)
        # the reference chain (x1[n-REF]) has unit gain: ref SINR = ref
        L[i] = SNR_IN_DB - 10 * np.log10(ps / pn)
    return L

def summarize(L):
    on = T_ON // WIN
    steady = np.mean(L[-int(1.5e-3 * FS / WIN):])
    avg = np.mean(L[on:])
    t3 = next((((i - on) * WIN / FS * 1e6) for i in range(on, len(L)) if np.all(L[i:i + 3] < 3.0)), np.inf)
    worst = np.max(L[on + 4:])
    return steady, avg, worst, t3

ALGOS = [
    ("STAP-16 ideal (solve every block)",  lambda c, t: stap(c, t, 1, 0)),
    ("STAP-16 as built (200 solves/s)",    lambda c, t: stap(c, t, 75, 15)),
    ("SFAP K256",                          lambda c, t: sfap(c, t, 256, 64, 0.98, 0)),
    ("SFAP K256 smooth+-4",                lambda c, t: sfap(c, t, 256, 64, 0.98, 4)),
    ("SFAP K256 smooth+-16 lam.95",        lambda c, t: sfap(c, t, 256, 64, 0.95, 16)),
]

if __name__ == "__main__":
    cases = []
    for jn in (30, 40, 50, 60):
        cases.append(("barrage", jn, False))
    for jn in (30, 40, 50, 60):
        cases.append(("barrage", jn, True))
    for jn in (40, 60):
        cases.append(("cw", jn, False)); cases.append(("cw", jn, True))
    for per in (100, 1000):
        cases.append((f"chirp{per}", 40, False)); cases.append((f"chirp{per}", 40, True))
    cases.append(("pulse", 40, False)); cases.append(("pulse", 50, True))
    sel = sys.argv[1:] 
    print(f"{'jammer':<12}{'J/N':>4} {'mp':>3}  " + "  ".join(f"{a[0][:22]:<22}" for a in ALGOS))
    print("(each cell: steady loss / mean loss after onset / worst / time to <3 dB in us)")
    for kind, jn, mp in cases:
        if sel and kind not in sel: continue
        comps, tot = scene(kind, jn, mp)
        row = []
        for name, fn in ALGOS:
            L = sinr_loss(fn(comps, tot), comps)
            st, av, wo, t3 = summarize(L)
            row.append(f"{st:5.1f}/{av:5.1f}/{wo:5.1f}/{('inf' if t3==np.inf else '%d' % t3):>4}")
        print(f"{kind:<12}{jn:>4} {'Y' if mp else 'n':>3}  " + "  ".join(f"{r:<22}" for r in row), flush=True)
