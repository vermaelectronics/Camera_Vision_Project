import numpy as np, subprocess
rng = np.random.default_rng(5)
NFR = 12; N = 1024
SH = 3; LOAD = 2000
# per-bin coupling c0[k] (smooth + random phase), jammer power varies per bin (barrage 40 dB over noise bins ~ 200 LSB)
k = np.arange(N)
c0 = (0.8 + 0.3 * np.cos(2*np.pi*k/N*3)) * np.exp(1j * (1.2 + 0.5*np.sin(2*np.pi*k/N*2)))
noise = 150.0
J = np.where((k % 1024) < 700, 3e5, 0.0)          # jammer in 70% of the bins, amplitude 3e5 (about 66 dB over noise)
X1 = []; X2 = []
for f in range(NFR):
    j = (rng.standard_normal(N) + 1j*rng.standard_normal(N)) / np.sqrt(2) * J
    n1 = (rng.standard_normal(N) + 1j*rng.standard_normal(N)) / np.sqrt(2) * noise
    n2 = (rng.standard_normal(N) + 1j*rng.standard_normal(N)) / np.sqrt(2) * noise
    X1.append(np.round(j + n1)); X2.append(np.round(c0 * j * 0 + (j / c0) + n2))   # x2 = j / c0  so that x1 = c0 * x2
X1 = np.array(X1); X2 = np.array(X2)
with open("bin_in.hex", "w") as fh:
    for f in range(NFR):
        for kk in range(N):
            a, b, c, d = int(X1[f,kk].real), int(X1[f,kk].imag), int(X2[f,kk].real), int(X2[f,kk].imag)
            v = ((d & 0x1FFFFFF) << 75) | ((c & 0x1FFFFFF) << 50) | ((b & 0x1FFFFFF) << 25) | (a & 0x1FFFFFF)
            fh.write("%025x\n" % v)
subprocess.run(["iverilog","-g2005","-o","tb_bin","tb_bin.v","../../../Source/HDL/sfap_bin.v","../../../Source/HDL/sfap_recip_rom.v"],check=True)
subprocess.run(["vvp","tb_bin"],check=True,capture_output=True)
rows = np.array([[int(t) for t in l.split()] for l in open("bin_out.txt")])
assert len(rows) == NFR*N, len(rows)
Y = (rows[:,1] + 1j*rows[:,2]).reshape(NFR, N)
# float reference with the same recursion
R22 = np.zeros(N); R12 = np.zeros(N, complex)
maxrel = 0
for f in range(NFR):
    p22 = np.abs(X2[f])**2; p12 = X1[f]*np.conj(X2[f])
    if f == 0: R22, R12 = p22.copy(), p12.copy()
    else: R22 = R22 + (p22-R22)/2**SH; R12 = R12 + (p12-R12)/2**SH
    c = R12/(R22+LOAD)
    Yref = X1[f] - c*X2[f]
    err = np.abs(Y[f]-Yref)
    jam = J > 0
    print(f"frame {f:2d}: |Y| in jammed bins rms {np.sqrt(np.mean(np.abs(Y[f][jam])**2)):9.1f}  float ref {np.sqrt(np.mean(np.abs(Yref[jam])**2)):9.1f}  "
          f"rms err vs ref {np.sqrt(np.mean(err**2)):7.2f}  (noise {noise:.0f}, input jam {np.sqrt(np.mean(np.abs(X1[f][jam])**2)):.0f})")
