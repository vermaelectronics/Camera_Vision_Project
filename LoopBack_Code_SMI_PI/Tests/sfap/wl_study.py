#!/usr/bin/env python3
"""Does SFAP gain from WL-STAP-PI's ideas?  Scene with AD9361-like I/Q imbalance (image rejection 40-45 dB,
different on the two receivers).  Compared: SFAP (linear), SFAP-WL (per-bin widely-linear: X2[k], conj X1[-k], conj X2[-k]),
SFAP + CW line excision."""
import sys, numpy as np
import study as S
from numpy.lib.stride_tricks import sliding_window_view as swv

def scene_iq(kind, jn, mp, irr1=42.0, irr2=46.0):
    rng_state = S.rng
    comps, tot = S.scene(kind, jn, mp)
    # re-create with imbalance: apply to signal+jammer parts, thermal noise untouched, then re-quantise
    k1 = 10 ** (-irr1 / 20) * np.exp(0.7j); k2 = 10 ** (-irr2 / 20) * np.exp(-1.9j)
    (s1, s2), (n1, n2), (j1, j2) = comps["sig"], comps["noise"], comps["jam"]
    def imb(x, k): return x + k * np.conj(x)
    s1, j1 = imb(s1, k1), imb(j1, k1); s2, j2 = imb(s2, k2), imb(j2, k2)
    x1 = s1 + n1 + j1; x2 = s2 + n2 + j2
    return {"sig": (s1, s2), "noise": (n1, n2), "jam": (j1, j2)}, (x1, x2)

def stft(x, K, hop, wa):
    nfr = (S.NS - K) // hop + 1
    return np.fft.fft(swv(x, K)[::hop][:nfr] * wa, axis=1)

def ola(Y, K, hop, wa):
    nfr = Y.shape[0]
    yf = np.fft.ifft(Y, axis=1) * wa
    y = np.zeros(S.NS, complex); norm = np.zeros(S.NS)
    for f in range(nfr):
        y[f*hop:f*hop+K] += yf[f]; norm[f*hop:f*hop+K] += wa * wa
    m = norm > 0.5 * norm.max(); y[m] /= norm[m]; y[~m] = 0
    return y

def sfap_wl(comps, tot, K=1024, hop=512, lam=0.96875, wl=True, load=1.0):
    wa = np.sin(np.pi * np.arange(K) / K)
    X1, X2 = stft(tot[0], K, hop, wa), stft(tot[1], K, hop, wa)
    T = {k: (stft(comps[k][0], K, hop, wa), stft(comps[k][1], K, hop, wa)) for k in comps}
    nfr = X1.shape[0]
    mirror = (-np.arange(K)) % K
    nb = load * np.sum(wa ** 2)
    nd = 3 if wl else 1
    R = np.zeros((K, nd, nd), complex); r = np.zeros((K, nd), complex)
    def reg(A1, A2):       # regressors, shape (K, nd) for one frame
        if wl: return np.stack([A2, np.conj(A1[mirror]), np.conj(A2[mirror])], axis=1)
        return A2[:, None]
    W = np.zeros((nfr, K, nd), complex)
    for f in range(nfr):
        u = reg(X1[f], X2[f]); d = X1[f]
        R = lam * R + (1 - lam) * u[:, :, None] * np.conj(u[:, None, :])
        r = lam * r + (1 - lam) * u * np.conj(d)[:, None]
        A = R + nb * np.eye(nd)[None]
        W[f] = np.linalg.solve(A, r[:, :, None])[:, :, 0]
    out = {}
    for k in comps:
        A1, A2 = T[k]
        Y = np.empty_like(A1)
        for f in range(nfr):
            u = reg(A1[f], A2[f])
            Y[f] = A1[f] - np.sum(np.conj(W[f]) * u, axis=1)
        out[k] = ola(Y, K, hop, wa)
    return out

if __name__ == "__main__":
    S.NS = 60000; S.T_ON = 20000
    cases = [("barrage", 40, True), ("barrage", 50, True), ("barrage", 60, True), ("barrage", 70, True),
             ("cw", 60, True), ("chirp100", 50, True)]
    print("cell: steady SINR loss dB / worst / us to <3 dB   (I/Q image rejection 42 dB on RX1, 46 dB on RX2)")
    print(f"{'jammer':<16}{'J/N':>4}  {'STAP ideal':<22}| {'SFAP':<22}| {'SFAP + WL':<22}", flush=True)
    for kind, jn, mp in cases:
        comps, tot = scene_iq(kind, jn, mp)
        row = []
        for f in (lambda c, t: S.stap(c, t, 1, 0), lambda c, t: S.sfap(c, t, 1024, 512, 0.96875, 0, 1.0, True),
                  lambda c, t: sfap_wl(c, t)):
            L = S.sinr_loss(f(comps, tot), comps)
            st, av, wo, t3 = S.summarize(L)
            row.append(f"{st:6.1f} /{wo:6.1f} /{('never' if t3==np.inf else int(t3)):>5}")
        print(f"{kind+(' +echo' if mp else ''):<16}{jn:>4}  " + " | ".join(r.ljust(22) for r in row), flush=True)
