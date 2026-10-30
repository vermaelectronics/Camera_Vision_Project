#!/usr/bin/env python3
"""
Floating-point reference model of the PI-AJ v5 algorithm (used for tuning and
as the golden behaviour the RTL is checked against).

  stage 1 : 2 x cascaded adaptive complex notch (ANF), one coefficient shared
            by both channels so the array manifold is preserved
  stage 2 : space-time power inversion (2 ch x L taps), GSC/NLMS form:
            y = x0[n-D] - w^H u ,  w += mu * u * conj(y) / P_u
"""
import sys
import numpy as np
import analyze


def load_hex(fn):
    d = np.loadtxt(fn, dtype=str)
    v = np.vectorize(lambda s: int(s, 16))(d).astype(np.int64)
    v[v >= 32768] -= 65536
    x0 = v[:, 0] + 1j * v[:, 1]
    x1 = v[:, 2] + 1j * v[:, 3]
    return np.stack([x0, x1]).astype(complex)


def notch(x, rho_shift=8, r_shift=12, a_shift=8, nst=2, image=True, apply_main=True):
    """x: (2,n). returns notched (2,n), coefficient of each stage.

    Frequency estimator: lag-1 autocorrelation of the stage input (channel 0),
    r1 = <x[n] x*[n-1]>, whose angle is the dominant tone frequency.  The unit
    coefficient a tracks r1/|r1| (Newton re-normalisation, no divider/sqrt)."""
    n = x.shape[1]
    rho = 1 - 2.0 ** -rho_shift
    out = x.copy()
    hist = []
    stages = []
    for st in range(nst):
        stages.append("est")
        if image and st == 0:
            stages.append("img")
    a_main = None
    for kind in stages:
        a = 1.0 + 0j
        r1 = 0j
        x1 = np.zeros(2, complex)
        y1 = np.zeros(2, complex)
        pin = 1e-9
        pout = 1e-9
        act = False
        inp = out.copy()
        ahist = np.ones(n, complex)
        acth = np.zeros(n, bool)
        for k in range(n):
            xk = inp[:, k]
            if kind == "img":
                a = np.conj(a_main[k])
            y = xk - a * x1 + rho * a * y1
            if kind == "est":
                r1 += (xk[0] * np.conj(x1[0]) - r1) * 2.0 ** -r_shift
                pr = 2.0 ** np.floor(np.log2(max(abs(r1.real), abs(r1.imag), 1e-9)))
                a = a + (r1 / pr) * 2.0 ** -a_shift
                a = a * (1.5 - 0.5 * abs(a) ** 2)
                ahist[k] = a
            x1 = xk
            y1 = y
            pin += (abs(xk[0]) ** 2 - pin) * 2.0 ** -12
            pout += (abs(y[0]) ** 2 - pout) * 2.0 ** -12
            if kind == "img":
                act = act_main[k]
            elif pin > pout + pout / 4:
                act = True
            elif pin < pout + pout / 8:
                act = False
            acth[k] = act
            use = act and (kind == "img" or apply_main)
            out[:, k] = y if use else xk
        if a_main is None:
            a_main = ahist.copy()
            act_main = acth.copy()
        hist.append((kind, a, act))
    return out, hist


def stap(x, L=5, mu_shift=7, freeze=False):
    n = x.shape[1]
    D = (L - 1) // 2
    nw = 2 * L - 1
    w = np.zeros(nw, complex)
    y = np.zeros(n, complex)
    p = 1.0
    buf0 = np.zeros(L, complex)
    buf1 = np.zeros(L, complex)
    mu = 2.0 ** -mu_shift
    for k in range(n):
        buf0 = np.roll(buf0, 1); buf0[0] = x[0, k]
        buf1 = np.roll(buf1, 1); buf1[0] = x[1, k]
        d = buf0[D]
        u = np.concatenate([np.delete(buf0, D), buf1])
        e = d - np.vdot(w, u)
        ip = np.real(np.vdot(u, u))
        p = ip if ip > p else p + (ip - p) * 2.0 ** -6
        pe = 2.0 ** np.floor(np.log2(max(p, 1e-9)))
        if not freeze:
            w = w + mu * u * np.conj(e) / pe
        y[k] = e
    return y, w


def main():
    name = sys.argv[1]
    meta = np.load(name + ".npz")
    x = load_hex(name + ".hex")
    jon = int(meta["jon"])
    settle = jon + 18000
    res = []
    res.append(analyze.report("RX1 in", x[0], 0, meta, settle))
    z, hist = notch(x, nst=int(sys.argv[2]) if len(sys.argv) > 2 else 2)
    res.append(analyze.report("after notch", z[0], 0, meta, settle))
    y, w = stap(z)
    res.append(analyze.report("notch+STAP", y, -2, meta, settle))
    y2, _ = stap(x)
    res.append(analyze.report("STAP only", y2, -2, meta, settle))
    p0 = res[0]["power"]
    for r in res:
        print("%-12s P=%10.3f (%6.1f dB rel in)  C/N0=%6.1f dB-Hz  lag=%d" %
              (r["label"], r["power"], 10 * np.log10(r["power"] / p0), r["cn0"], r["lag"]))
    print("notch:", [("%s %.1f Hz %s" % (k, np.angle(h) * 18e6 / 2 / np.pi, act)) for k, h, act in hist])


if __name__ == "__main__":
    main()


def stap_wl(x, L=5, mu_shift=7):
    """Widely-linear space-time PI: regressor also holds the conjugate taps,
    so the per-channel I/Q imbalance image of a wideband jammer is nullable."""
    n = x.shape[1]
    D = (L - 1) // 2
    w = np.zeros(4 * L - 1, complex)
    y = np.zeros(n, complex)
    p = 1.0
    buf0 = np.zeros(L, complex)
    buf1 = np.zeros(L, complex)
    mu = 2.0 ** -mu_shift
    for k in range(n):
        buf0 = np.roll(buf0, 1); buf0[0] = x[0, k]
        buf1 = np.roll(buf1, 1); buf1[0] = x[1, k]
        d = buf0[D]
        u = np.concatenate([np.delete(buf0, D), buf1, np.conj(buf0), np.conj(buf1)])
        e = d - np.vdot(w, u)
        ip = np.real(np.vdot(u, u))
        p = ip if ip > p else p + (ip - p) * 2.0 ** -6
        pe = 2.0 ** np.floor(np.log2(max(p, 1e-9)))
        w = w + mu * u * np.conj(e) / pe
        y[k] = e
    return y, w


def stap_hw(x, L=5, mu_shift=6, delay=3, wl=True):
    """Exactly the RTL algorithm: widely-linear space-time PI, delayed NLMS
    (weights updated with the regressor/error of `delay` samples ago), power
    normaliser = smoothed |x0|^2+|x1|^2 of the newest sample, power-of-two."""
    n = x.shape[1]
    D = (L - 1) // 2
    nw = (4 * L - 1) if wl else (2 * L - 1)
    w = np.zeros(nw, complex)
    y = np.zeros(n, complex)
    p = 1.0
    LB = L + delay
    b0 = np.zeros(LB, complex)
    b1 = np.zeros(LB, complex)
    eh = np.zeros(delay, complex)
    ph = np.ones(delay)
    mu = 2.0 ** -mu_shift

    def regress(off):
        a0 = b0[off:off + L]
        a1 = b1[off:off + L]
        r = [np.delete(a0, D), a1]
        if wl:
            r += [np.conj(a0), np.conj(a1)]
        return np.concatenate(r)

    for k in range(n):
        b0 = np.roll(b0, 1); b0[0] = x[0, k]
        b1 = np.roll(b1, 1); b1[0] = x[1, k]
        ip = abs(x[0, k]) ** 2 + abs(x[1, k]) ** 2
        p = ip if ip > 8 * p else p + (ip - p) * 2.0 ** -5
        pe = 2.0 ** np.floor(np.log2(max(p, 1e-9)))
        # update with old regressor / old error
        uo = regress(delay)
        w = w + mu * uo * np.conj(eh[-1]) / (ph[-1] * 8 * L)
        u = regress(0)
        e = b0[D] - np.vdot(w, u)
        eh = np.roll(eh, 1); eh[0] = e
        ph = np.roll(ph, 1); ph[0] = pe
        y[k] = e
    return y, w
