import sys, subprocess, numpy as np
sys.path.insert(0, "..")
import study as S
S.NS = int(sys.argv[3]) if len(sys.argv) > 3 else 40000
kind = sys.argv[1] if len(sys.argv) > 1 else "none"
jn = float(sys.argv[2]) if len(sys.argv) > 2 else 40
S.T_ON = 6000 if len(sys.argv) <= 4 else int(sys.argv[4])
NS = S.NS
if kind == "none":
    comps, tot = S.scene("cw", -200, False)      # no jammer
else:
    comps, tot = S.scene(kind, jn, True)
x1, x2 = tot
# quantise to 12 bit like the ADC after AGC (scene already did, but the scale is lost): re-quantise with AGC to 25% of full scale rms
ref = np.sqrt(np.mean(np.abs(x1[S.T_ON+500:])**2)/2)
sc = 500.0 / ref if kind != "none" else 40.0
def q(x): return np.clip(np.round(x.real*sc), -2048, 2047).astype(int), np.clip(np.round(x.imag*sc), -2048, 2047).astype(int)
a, b = q(x1); c, d = q(x2)
with open("core_in.hex", "w") as fh:
    for i in range(NS):
        v = ((d[i] & 0xFFF) << 36) | ((c[i] & 0xFFF) << 24) | ((b[i] & 0xFFF) << 12) | (a[i] & 0xFFF)
        fh.write("%012x\n" % v)
subprocess.run(["iverilog", "-g2005", f"-Ptb_core.NS={NS}", "-o", "tb_core", "tb_core.v"] +
    ["../../../Source/HDL/" + f for f in ("sfap_core.v","sfap_fft.v","sfap_bin.v","sfap_twiddle.v","sfap_recip_rom.v","sfap_win_rom.v")], check=True, capture_output=True)
r = subprocess.run(["vvp", "tb_core"], check=True, capture_output=True, text=True)
print(r.stdout.strip().splitlines()[-1])
rows = np.array([[int(t) for t in l.split()] for l in open("core_out.txt")])
y = rows[:,1] + 1j*rows[:,2]
x1c = a + 1j*b
print("output samples:", len(y), "input samples:", NS)
# find delay by cross-correlation on the clean part (before the jammer) or the whole thing
seg = slice(1000, 5000)
best = None
for dly in range(1500, 3200):
    if dly + len(x1c) < len(y): pass
    # y[m] ~ x1[m - dly + shift]; correlate y[dly+n0:dly+n1] with x1c[n0:n1]
    n0, n1 = 1000, 1000 + 2000
    if n1 + dly > len(y): break
    cc = np.abs(np.vdot(x1c[n0:n1], y[n0+dly:n1+dly]))
    if best is None or cc > best[0]: best = (cc, dly)
print("best delay (samples):", best[1])
np.save("core_y.npy", y); np.save("core_x1.npy", x1c)
