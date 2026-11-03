import sys, os, subprocess, numpy as np, shutil
sys.path.insert(0, os.path.abspath(".."))
import study as S
kind = sys.argv[1]; jn = float(sys.argv[2]); mp = (sys.argv[3] == "mp")
wd = f"run_{kind}_{int(jn)}_{sys.argv[3]}"; os.makedirs(wd, exist_ok=True)
shutil.copy("tb_core.v", wd); os.chdir(wd)
HDL = "../../../../Source/HDL/"
SH = sys.argv[4] if len(sys.argv) > 4 else "5"
NS = 60000; S.NS = NS; S.T_ON = 20000
comps, tot = S.scene(kind, jn, mp)
x1, x2 = tot
sc = 500.0 / np.sqrt(np.mean(np.abs(x1[S.T_ON+500:])**2)/2)       # jammer rms 500 LSB (12-bit)
noise_lsb = np.sqrt(np.mean(np.abs(comps["noise"][0])**2)/2) * sc
def q(x): return np.clip(np.round(x.real*sc), -2048, 2047).astype(int), np.clip(np.round(x.imag*sc), -2048, 2047).astype(int)
def run(x1, x2, tag):
    a, b = q(x1); c, d = q(x2)
    with open("core_in.hex", "w") as fh:
        for i in range(NS):
            v = ((d[i] & 0xFFF) << 36) | ((c[i] & 0xFFF) << 24) | ((b[i] & 0xFFF) << 12) | (a[i] & 0xFFF)
            fh.write("%012x\n" % v)
    # diagonal load: noise power per bin ~ (noise rms^2 * 2 per complex) * 16^2 (x16) * sum(w^2)=512 / 4 (halving)
    npow = 2 * noise_lsb**2
    load = int(npow * 256 * 512 / 4)
    subprocess.run(["iverilog","-g2005",f"-Ptb_core.NS={NS}",f"-Ptb_core.SH={SH}",f"-Ptb_core.LOAD={load}","-o","tb_core","tb_core.v"]+
        [HDL+f for f in ("sfap_core.v","sfap_fft.v","sfap_bin.v","sfap_twiddle.v","sfap_recip_rom.v","sfap_win_rom.v")],check=True,capture_output=True)
    subprocess.run(["vvp","tb_core"],check=True,capture_output=True)
    rows = np.array([[int(t) for t in l.split()] for l in open("core_out.txt")])
    return rows[:,1] + 1j*rows[:,2], load
base1 = comps["sig"][0] + comps["noise"][0]; base2 = comps["sig"][1] + comps["noise"][1]
yb, load = run(base1, base2, "base")
yj, _ = run(x1, x2, "jam")
W = 1024
def pw(y, i): return np.mean(np.abs(y[i*W:(i+1)*W])**2)
nb = min(len(yb), len(yj)) // W
pb = np.mean([pw(yb, i) for i in range(4, nb-1)])
print(f"{kind} J/N {jn} {'mp' if mp else 'clean'}  noise {noise_lsb:.2f} LSB, load {load}, baseline output power {10*np.log10(pb):.1f} dB(LSB^2)")
# output index k <-> input time k+512
on_k = S.T_ON - 512
ex = np.array([10*np.log10(pw(yj, i)/pb) for i in range(nb-1)])
i_on = on_k // W
post = ex[i_on+4:]
first_ok = next((((i - i_on) * W / 61.44) for i in range(i_on+1, nb-1) if ex[i] < 5.0 and np.all(ex[i:i+3] < 5.0)), float('inf'))
print(f"  summary: first window within 5 dB of baseline after {first_ok:.0f} us; steady excess median {np.median(post):.1f} dB, mean-power excess {10*np.log10(np.mean(10**(post/10))):.1f} dB, worst window {post.max():.1f} dB, onset window {ex[i_on:i_on+2].max():.1f} dB")
if len(sys.argv) > 5: 
    for i in range(max(0, i_on - 1), min(nb-1, i_on + 12)): print(f"    {i:3d} {(i*W-on_k)/61.44:7.0f} us {ex[i]:6.1f} dB")
