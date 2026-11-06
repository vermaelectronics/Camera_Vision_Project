#!/usr/bin/env python3
"""Reference model of gnss_pulse_blank (W=8, D=16, H=16) and the stimulus generator."""
import random, sys
W, D, H = 8, 16, 16

def model(samples, thr, en=1):
    p = [i * i + q * q for i, q in samples]
    out, ex = [], []
    S = 0
    for n in range(len(samples)):
        S += p[n] - (p[n - W] if n >= W else 0)
        ex.append(1 if S > thr else 0)
    nb = bursts = 0
    prev = 0
    res = []
    # output sample j appears when sample j+D has entered; the flags of the samples that
    # have entered so far are known (ages 0..), E[a] = ex[n-a]
    total = len(samples)
    for n in range(total):
        j = n - D
        if j < 0:
            i = q = 0                              # delay line still empty
        else:
            i, q = samples[j]
        blk = en and any(ex[n - a] for a in range(8, D + H + 1) if n - a >= 0)
        if blk:
            i = q = 0
        res.append((i, q))
        if blk:
            nb += 1
            if not prev: bursts += 1
        prev = 1 if blk else 0
    return res, nb, bursts

def gen(ns, seed=1):
    random.seed(seed)
    s = []
    for n in range(ns):
        amp = 4
        i = int(random.gauss(0, amp)); q = int(random.gauss(0, amp))
        # pulses: period 700, 90 on / 5 random bursts
        ph = n % 700
        if 100 <= ph < 190 or 520 <= ph < 524:
            i = int(random.gauss(0, 400)); q = int(random.gauss(0, 400))
        if n == 15000: i = 2047; q = -2048
        s.append((max(-2048, min(2047, i)), max(-2048, min(2047, q))))
    return s

if __name__ == "__main__":
    ns = 20000
    thr = int(sys.argv[1]) if len(sys.argv) > 1 else 20000
    en = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    s = gen(ns)
    with open("blank_in.hex", "w") as f:
        for i, q in s: f.write("%08x\n" % (((q & 0xFFFF) << 16) | (i & 0xFFFF)))
    res, nb, bursts = model(s, thr, en)
    with open("blank_ref.hex", "w") as f:
        for i, q in res: f.write("%08x\n" % (((q & 0xFFFF) << 16) | (i & 0xFFFF)))
    print("MODEL_BLK_CNT %d BURSTS %d" % (nb, bursts))
