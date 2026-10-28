#!/usr/bin/env python3
"""System-level model of a 2-antenna GNSS anti-jam chain (floating point, batch/ideal
weights). Measures per-satellite C/N0 after each algorithm.

  fs 50 MS/s, LO 1570.25 MHz -> L1 at +5.17 MHz, 8 GPS C/A satellites (C/N0 45 dB-Hz
  each, arriving with different inter-element phases psi_s), one jammer from psi_j.
  J/N is over the whole 50 MHz (what the board prints); J/S = J/N + 32 dB.
  Channel mismatch RX2 vs RX1: delay tau samples, gain tilt, gain offset.

usage: array_sim.py [--quick] [--common]   (needs numpy, scipy)
"""
import sys
import numpy as np
from scipy.signal import lfilter

FS = 50e6
FL1 = 5.17e6
RC = 1.023e6
G2 = {1: (2, 6), 2: (3, 7), 3: (4, 8), 4: (5, 9), 5: (1, 9), 6: (2, 10), 7: (1, 8), 8: (2, 9)}


def ca_code(prn):
    a, b = G2[prn]
    g1 = [1] * 10
    g2 = [1] * 10
    code = np.zeros(1023)
    for n in range(1023):
        o = g1[9] ^ g2[a - 1] ^ g2[b - 1]
        code[n] = -1.0 if o else 1.0
        f1 = g1[2] ^ g1[9]
        f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [f1] + g1[:9]
        g2 = [f2] + g2[:9]
    return code


class Scene:
    def __init__(self, nms=20, seed=1, tau=0.15, tilt=0.10, gain_db=1.5, npr=8):
        self.rng = np.random.default_rng(seed)
        self.N = int(nms * 1e-3 * FS)
        self.n = np.arange(self.N)
        self.t = self.n / FS
        self.codes = {p: ca_code(p) for p in range(1, npr + 1)}
        self.chip = (np.floor(self.t * RC).astype(np.int64)) % 1023
        self.tau, self.tilt, self.gain = tau, tilt, 10 ** (gain_db / 20)
        self.psi_s = self.rng.uniform(-np.pi, np.pi, npr)
        self.npr = npr
        self.f = np.fft.fftfreq(self.N, 1 / FS)
        self.noise1 = self.cn()
        self.noise2 = self.cn()
        sat_pow = 10 ** (4.5) / FS          # C/N0 45 dB-Hz relative to unit noise power
        self.sat_amp = np.sqrt(sat_pow)

    def cn(self):
        return (self.rng.standard_normal(self.N) + 1j * self.rng.standard_normal(self.N)) / np.sqrt(2)

    def chan2(self, x, psi):
        """RX2 channel: phase psi, delay tau, gain tilt over +-25 MHz, gain offset."""
        X = np.fft.fft(x)
        H = self.gain * (1 + self.tilt * self.f / 25e6) * np.exp(-2j * np.pi * self.f / FS * self.tau)
        return np.fft.ifft(X * H) * np.exp(1j * psi)

    def sats(self):
        """noiseless sat sum at RX1 and RX2 (each sat with its own psi_s)"""
        s1 = np.zeros(self.N, complex)
        s2 = np.zeros(self.N, complex)
        carrier = np.exp(2j * np.pi * FL1 * self.t)
        for p in range(1, self.npr + 1):
            s = self.sat_amp * self.codes[p][self.chip] * carrier
            s1 += s
            s2 += self.chan2(s, self.psi_s[p - 1])
        return s1, s2

    def jammer(self, kind, jn_db, psi_j, f_cw=FL1 + 2e3, bw=40e6):
        amp = 10 ** (jn_db / 20)
        if kind == 'none':
            return np.zeros(self.N, complex), np.zeros(self.N, complex)
        if kind == 'cw':
            j = amp * np.exp(2j * np.pi * f_cw * self.t + 0.3j)
        elif kind == 'wb':
            w = self.cn()
            W = np.fft.fft(w)
            W[np.abs(self.f) > bw / 2] = 0
            j = np.fft.ifft(W)
            j *= amp / np.sqrt(np.mean(np.abs(j) ** 2))
        elif kind == 'cw2':           # two CWs, 6 dB apart in power
            j = amp * (np.exp(2j * np.pi * f_cw * self.t) + 0.7 * np.exp(2j * np.pi * (f_cw - 7.3e6) * self.t + 1j))
        elif kind == 'cw+wb':         # CW and wideband, each at jn_db - 3 dB
            a = amp * 10 ** (-3 / 20)
            w = self.cn(); W = np.fft.fft(w); W[np.abs(self.f) > bw / 2] = 0
            wb = np.fft.ifft(W); wb *= a / np.sqrt(np.mean(np.abs(wb) ** 2))
            j = a * np.exp(2j * np.pi * f_cw * self.t) + wb
        else:
            raise ValueError(kind)
        return j, self.chan2(j, psi_j)


def lag_matrix(x, offs):
    return np.stack([np.roll(x, -o) for o in offs], axis=1)


def solve_stap(x1, x2, K, reg=1e-6):
    """minimise E|y|^2, y = x1[n] + sum free taps; centre tap of RX1 fixed to 1.
    K temporal taps per antenna (odd), taps at offsets -(K//2)..+K//2."""
    c = K // 2
    offs = list(range(-c, c + 1))
    cols = []
    for o in offs:
        if o != 0:
            cols.append(np.roll(x1, -o))
    for o in offs:
        cols.append(np.roll(x2, -o))
    X = np.stack(cols, axis=1)
    R = X.conj().T @ X / len(x1)
    r = X.conj().T @ x1 / len(x1)
    R += reg * np.trace(R).real / R.shape[0] * np.eye(R.shape[0])
    w = -np.linalg.solve(R, r)
    return x1 + X @ w


def notch(x, f, r=1 - 2 ** -8):
    q = np.exp(2j * np.pi * f / FS)
    return lfilter([1, -q], [1, -r * q], x)


def cn0(sc, y, nblk=18, nlag=25):
    """per-satellite C/N0 [dB-Hz]: 1 ms correlations at the true code phase and at 25
    wrong phases (>= 20 chips away)."""
    L = int(1e-3 * FS)
    nb = min(nblk, sc.N // L - 2)          # skip the first and last ms (filter transients, roll wrap)
    out = []
    ylo = y * np.exp(-2j * np.pi * FL1 * sc.t)
    Y = ylo[L:(nb + 1) * L].reshape(nb, L)
    chip = sc.chip[L:(nb + 1) * L].reshape(nb, L)
    for p in range(1, sc.npr + 1):
        code = sc.codes[p]
        c0 = code[chip]
        r0 = np.sum(Y * c0, axis=1)
        pn = 0.0
        for k in range(nlag):
            ck = code[(chip + 20 + 38 * k) % 1023]
            pn += np.mean(np.abs(np.sum(Y * ck, axis=1)) ** 2)
        pn /= nlag
        ps = np.mean(np.abs(r0) ** 2)
        out.append(10 * np.log10(max(ps - pn, 1e-9) / pn) + 30.0)
    return np.array(out)


ALGS = ['raw RX1', 'PI 1 weight', 'PI+notch', 'STAP3', 'STAP5', 'STAP5+notch']


def run_alg(sc, name, x1, x2, f_cw):
    if name == 'raw RX1':
        return x1
    if name == 'PI 1 weight':
        return solve_stap(x1, x2, 1)
    if name == 'PI+notch':
        a, b = (notch(x1, f_cw), notch(x2, f_cw)) if f_cw is not None else (x1, x2)
        return solve_stap(a, b, 1)
    if name == 'STAP3':
        return solve_stap(x1, x2, 3)
    if name == 'STAP5':
        return solve_stap(x1, x2, 5)
    if name == 'STAP5+notch':
        a, b = (notch(x1, f_cw), notch(x2, f_cw)) if f_cw is not None else (x1, x2)
        return solve_stap(a, b, 5)
    raise ValueError(name)


def main():
    quick = '--quick' in sys.argv
    sc = Scene(nms=12)
    common = '--common' in sys.argv
    if common:                      # both RX ports fed from one source (splitter): sats and jammer arrive identically
        sc.psi_s[:] = 0.0
    s1, s2 = sc.sats()
    ref = cn0(sc, s1 + sc.noise1)
    print(f"no jammer, raw RX1: mean {ref.mean():.1f}  min {ref.min():.1f}  max {ref.max():.1f} dB-Hz (target 45)")
    print("satellite phases psi_s [deg]:", np.round(np.degrees(sc.psi_s)).astype(int))
    cases = []
    psi_j = 0.0 if common else np.radians(70)
    for kind in ['cw', 'wb', 'cw2', 'cw+wb']:
        for jn in ([10, 20, 30] if not quick else [20]):
            cases.append((kind, jn, psi_j, ('common source: sats and jammer identical at both ports' if common else 'jammer at psi_j = 70 deg')))
    for kind, jn, psi, label in cases:
        j1, j2 = sc.jammer(kind, jn, psi)
        x1 = s1 + j1 + sc.noise1
        x2 = s2 + j2 + sc.noise2
        f_cw = FL1 + 2e3 if kind in ('cw', 'cw+wb') else None
        print(f"\n{kind:6s} J/N {jn} dB (J/S {jn + 32}) -- {label}")
        print(f"   {'algorithm':14s} mean   min   per-satellite C/N0 [dB-Hz]")
        for a in ALGS:
            if 'notch' in a and f_cw is None and kind != 'cw2':
                pass
            y = run_alg(sc, a, x1, x2, f_cw)
            c = cn0(sc, y)
            print(f"   {a:14s} {c.mean():5.1f} {c.min():5.1f}   " + " ".join(f"{v:5.1f}" for v in c))


if __name__ == '__main__':
    main()
