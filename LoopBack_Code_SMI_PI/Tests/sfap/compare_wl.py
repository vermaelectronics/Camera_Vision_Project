#!/usr/bin/env python3
"""Same scenes, three filters, at 30.72 MS/s (the highest rate the WL-STAP-PI hardware can run):
STAP-16 (SMI-PI 4.x, ideal solve every block), SFAP (5.0), WL-STAP-PI (its own float model of the RTL)."""
import sys, numpy as np
sys.path.insert(0, "/tmp/claude-0/-home-user-Camera-Vision-Project/12259fc3-e6c6-58d8-a23e-86d8d89f03a3/scratchpad/wl/LoopBack_Code_WL_STAP_PI/Tests/pi_aj")
import study as S
import model as M
S.FS = 30.72e6
S.NS = 60000; S.T_ON = 20000
W = 1024
def wl(tot):
    x = np.stack([tot[0], tot[1]])
    xn, _ = M.notch(x, rho_shift=6, r_shift=12, a_shift=8, nst=1, image=True)
    y, _ = M.stap_hw(xn, L=5, mu_shift=6, delay=4, wl=True)
    return y
def stap(comps, tot):
    o = S.stap(comps, tot, 1, 0); return o["sig"] + o["noise"] + o["jam"]
def sfap(comps, tot):
    o = S.sfap(comps, tot, 1024, 512, 0.96875, 0, 1.0, True); return o["sig"] + o["noise"] + o["jam"]
ALGOS = [("STAP-16 ideal", stap), ("SFAP", sfap), ("WL-STAP-PI", lambda c, t: wl(t))]
def run(kind, jn, mp):
    comps, tot = S.scene(kind, jn, mp)
    cb, tb = S.scene(kind, -200, mp)
    res = []
    for n, f in ALGOS:
        yj = f(comps, tot); yb = f(cb, tb)
        nb = len(yj) // W
        pb = np.mean([np.mean(abs(yb[i*W:(i+1)*W])**2) for i in range(6, nb-1)])
        ex = np.array([10*np.log10(np.mean(abs(yj[i*W:(i+1)*W])**2) / pb) for i in range(nb-1)])
        i_on = S.T_ON // W
        post = ex[i_on+6:]
        ok = next((((i - i_on) * W / 30.72) for i in range(i_on+1, nb-1) if np.all(ex[i:i+3] < 5.0)), float("inf"))
        res.append((np.median(post), post.max(), ok))
    return res
cases = [("barrage",40,False),("barrage",40,True),("barrage",50,True),("barrage",60,True),("cw",40,True),("cw",60,True),
         ("chirp100",40,True),("chirp1000",40,True),("pulse",40,False)]
sel = sys.argv[1:]
print("cell: steady excess median dB / worst window dB / us until within 5 dB of the no-jammer output")
print(f"{'jammer':<14}{'J/N':>4} " + " | ".join(f"{n:<26}" for n, _ in ALGOS), flush=True)
for kind, jn, mp in cases:
    if sel and kind not in sel: continue
    r = run(kind, jn, mp)
    print(f"{kind+(' +echo' if mp else ''):<14}{jn:>4} " + " | ".join(f"{a:6.1f} /{b:6.1f} /{('never' if c==float('inf') else int(c)):>6}" .ljust(26) for a, b, c in r), flush=True)
