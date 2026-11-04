#!/usr/bin/env python3
"""RTL test of the I/Q-image corrector: scene with image rejection 42 / 46 dB, barrage with echo.
Runs the core with k = 0 and with k estimated the way the firmware does (from the statistics sums), compares the
statistics registers with an exact integer model, and prints the output excess against a jammer-free run."""
import sys, os, subprocess, shutil, re, numpy as np
sys.path.insert(0, os.path.abspath(".."))
import study as S, wl_study as W
kind = sys.argv[1]; jn = float(sys.argv[2])
LN = int(os.environ.get('SFAP_LN', '10')); NN = 1 << LN
wd = f"run_iq_{kind}_{int(jn)}_{LN}"; os.makedirs(wd, exist_ok=True); shutil.copy("tb_core.v", wd); os.chdir(wd)
HDL = "../../../../Source/HDL/"
NS = 60000 if LN == 10 else 130000; S.NS = NS; S.T_ON = 20000
comps, tot = W.scene_iq(kind, jn, True)
x1, x2 = tot
sc = 500.0 / np.sqrt(np.mean(np.abs(x1[S.T_ON+500:])**2)/2)
noise_lsb = np.sqrt(np.mean(np.abs(comps["noise"][0])**2)/2) * sc
def q(x): return np.clip(np.round(x.real*sc), -2048, 2047).astype(int), np.clip(np.round(x.imag*sc), -2048, 2047).astype(int)
def write(x1, x2):
    a, b = q(x1); c, d = q(x2)
    with open("core_in.hex", "w") as fh:
        for i in range(NS):
            fh.write("%012x\n" % (((d[i] & 0xFFF) << 36) | ((c[i] & 0xFFF) << 24) | ((b[i] & 0xFFF) << 12) | (a[i] & 0xFFF)))
    return a + 1j*b, c + 1j*d
def run(k):
    load = int(2 * noise_lsb**2 * 256 * (NN // 2) / 4)
    subprocess.run(["iverilog","-g2005",f"-Ptb_core.NS={NS}",f"-Ptb_core.PAD=14000",f"-Ptb_core.LN={LN}",f"-Ptb_core.SH=5",f"-Ptb_core.LOAD={load}",
        f"-Ptb_core.K1R={k[0]}",f"-Ptb_core.K1I={k[1]}",f"-Ptb_core.K2R={k[2]}",f"-Ptb_core.K2I={k[3]}","-o","tb_core","tb_core.v"] +
        [HDL+f for f in ("sfap_core.v","sfap_iq.v","sfap_fft.v","sfap_bin.v","sfap_twiddle.v","sfap_recip_rom.v","sfap_win_rom.v")], check=True, capture_output=True)
    out = subprocess.run(["vvp","tb_core"], check=True, capture_output=True, text=True).stdout
    rows = np.array([[int(t) for t in l.split()] for l in open("core_out.txt")])
    return rows[:,1] + 1j*rows[:,2], out
cb1, cb2 = comps["sig"][0] + comps["noise"][0], comps["sig"][1] + comps["noise"][1]
write(cb1, cb2); yb, _ = run((0,0,0,0))
q1, q2 = write(x1, x2)
y0, log = run((0,0,0,0))
m = re.search(r"IQSTAT (-?\d+) (-?\d+) (\d+) (-?\d+) (-?\d+) (\d+)", log)
rtl = [int(v) for v in m.groups()]
PAD = 14000
def sums(z):
    # the statistics registers hold the last COMPLETE window of 65536 samples among NS + PAD fed samples
    nwin = (NS + PAD) // 65536
    z = np.concatenate([z, np.zeros(NS + PAD + 65536)])[(nwin - 1) * 65536: nwin * 65536]
    return (int(np.sum(z.real**2 - z.imag**2)) >> 8, int(np.sum(2*z.real*z.imag)) >> 8, int(np.sum(z.real**2 + z.imag**2)) >> 8)
exp = list(sums(q1)) + list(sums(q2))
print("statistics RTL ", rtl); print("statistics model", exp)
kk = []
for z in (q1, q2):
    seg = z[S.T_ON+3000:]
    k = np.mean(seg*seg) / (2*np.mean(abs(seg)**2)); kk.append(k)
kf = [int(round(kk[0].real*32768)), int(round(kk[0].imag*32768)), int(round(kk[1].real*32768)), int(round(kk[1].imag*32768))]
print("k estimate (Q1.15):", kf, " |k1|=%.4f |k2|=%.4f  (true 0.0079 / 0.0050)" % (abs(kk[0]), abs(kk[1])))
y1, _ = run(tuple(kf))
W_ = 1024
def ex(y):
    nb = min(len(yb), len(y)) // W_
    pb = np.mean([np.mean(abs(yb[i*W_:(i+1)*W_])**2) for i in range(4, nb-1)])
    e = np.array([10*np.log10(np.mean(abs(y[i*W_:(i+1)*W_])**2)/pb) for i in range(nb-1)])
    i_on = (S.T_ON - 512)//W_
    post = e[i_on+8:]
    return np.median(post), post.max(), 10*np.log10(np.mean(10**(post/10)))
for name, y in (("k = 0       ", y0), ("k estimated", y1)):
    a, b, c = ex(y); print(f"{kind} J/N {jn:.0f} + echo, IRR 42/46 dB, {name}: steady excess median {a:5.1f} dB, mean-power {c:5.1f} dB, worst window {b:5.1f} dB")
