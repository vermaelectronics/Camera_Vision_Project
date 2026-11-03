import numpy as np, subprocess, sys
rng = np.random.default_rng(3)
NF = 3
N = 1024
amp = 1 << 16
frames = []
for f in range(NF):
    x = (rng.integers(-amp, amp, N) + 1j * rng.integers(-amp, amp, N))
    if f == 1:   # tone at -6 dBFS (16-bit scale) + noise of 0.5 LSB(12) = 8 LSB(16): dynamic range test
        t = np.arange(N)
        x = np.round(2**14 * np.exp(2j*np.pi*37.3*t/N) * np.hanning(N+1)[:-1]**0.5 + 8*(rng.standard_normal(N)+1j*rng.standard_normal(N)))
    if f == 2:   # noise only, 8 LSB(16)
        x = np.round(8*(rng.standard_normal(N)+1j*rng.standard_normal(N)))
    frames.append(x)
stream = []
for x in frames:
    stream += [(int(v.real), int(v.imag)) for v in x] + [(0, 0)] * N
with open("fft_in.hex", "w") as fh:
    for re, im in stream:
        fh.write("%013x\n" % (((im & 0x1FFFFFF) << 25) | (re & 0x1FFFFFF)))
subprocess.run(["iverilog", "-g2005", "-Ptb_fft.SC=10'b1000000000", "-o", "tb_fft", "tb_fft.v", "../../../Source/HDL/sfap_fft.v", "../../../Source/HDL/sfap_twiddle.v"], check=True)
subprocess.run(["vvp", "tb_fft"], check=True, capture_output=True)
rows = np.array([[int(t) if t.lstrip("-").isdigit() else 0 for t in ln.split()] for ln in open("fft_out.txt")], dtype=np.int64)
out = rows[:, 1] + 1j * rows[:, 2]
def bitrev(k, b=10):
    r = 0
    for i in range(b): r |= ((k >> i) & 1) << (b - 1 - i)
    return r
br = np.array([bitrev(k) for k in range(N)])
for f, x in enumerate(frames):
    X = np.fft.fft(x) / 2.0                   # one halving
    exp = X[br]
    best = None
    for off in range(0, 2048 * NF):
        seg = out[off:off + N]
        if len(seg) < N: break
        err = np.sqrt(np.mean(np.abs(seg - exp) ** 2))
        if best is None or err < best[0]: best = (err, off)
    ref = np.sqrt(np.mean(np.abs(exp) ** 2))
    print(f"frame {f}: best offset {best[1]} (frame starts at {f*2048}, latency {best[1]-f*2048}), rms error {best[0]:.2f} LSB, signal rms {ref:.0f}, SNR {20*np.log10(ref/max(best[0],1e-9)):.1f} dB")
