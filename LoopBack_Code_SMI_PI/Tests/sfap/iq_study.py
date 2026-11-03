#!/usr/bin/env python3
"""Blind I/Q-imbalance corrector per receiver, in front of SFAP:  x' = x - k x*,  k = E[x^2] / (2 E|x|^2)
(circular signals + imbalance:  x = a + k a*  ->  E[x^2] = 2 k P).  Compared with linear SFAP and per-bin widely-linear SFAP."""
import sys, numpy as np
import study as S, wl_study as W
S.NS = 60000; S.T_ON = 20000
def correct(comps, tot, est_from):
    k = []
    for x in tot:
        seg = x[est_from:]
        k.append(np.mean(seg * seg) / (2 * np.mean(np.abs(seg) ** 2)))
    f = lambda a, kk: a - kk * np.conj(a)
    c2 = {n: (f(v[0], k[0]), f(v[1], k[1])) for n, v in comps.items()}
    return c2, (f(tot[0], k[0]), f(tot[1], k[1])), k
if __name__ == "__main__":
    cases = [("barrage", 40, True), ("barrage", 50, True), ("barrage", 60, True), ("barrage", 70, True),
             ("cw", 60, True), ("chirp100", 50, True), ("barrage", 60, False)]
    print("steady SINR loss dB / worst / us to <3 dB.  IRR 42 dB (RX1), 46 dB (RX2)")
    print(f"{'jammer':<16}{'J/N':>4}  {'SFAP':<22}| {'SFAP + WL (3x3/bin)':<22}| {'SFAP + IQ corrector':<22}| k estimate", flush=True)
    sfap = lambda c, t: S.sfap(c, t, 1024, 512, 0.96875, 0, 1.0, True)
    for kind, jn, mp in cases:
        comps, tot = W.scene_iq(kind, jn, mp)
        cc, tt, k = correct(comps, tot, S.T_ON + 2000)
        row = []
        for f, (c, t) in ((sfap, (comps, tot)), (lambda c, t: W.sfap_wl(c, t), (comps, tot)), (sfap, (cc, tt))):
            st, av, wo, t3 = S.summarize(S.sinr_loss(f(c, t), c))
            row.append(f"{st:6.1f} /{wo:6.1f} /{('never' if t3==np.inf else int(t3)):>5}")
        print(f"{kind+(' +echo' if mp else ''):<16}{jn:>4}  " + " | ".join(r.ljust(22) for r in row) +
              f"| |k1|={abs(k[0]):.4f} (true {10**(-42/20):.4f}) |k2|={abs(k[1]):.4f} (true {10**(-46/20):.4f})", flush=True)
