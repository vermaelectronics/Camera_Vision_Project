#!/usr/bin/env python3
"""Generate random vectors for tb_fp_units and check RTL results bit-exactly
against numpy float32 (round-to-nearest-even)."""
import sys
import numpy as np

N = 20000


def gen(d):
    rng = np.random.default_rng(1)
    # mixed magnitudes, signs, cancellation cases
    a = (rng.standard_normal(N) * 10.0 ** rng.integers(-8, 8, N)).astype(np.float32)
    b = (rng.standard_normal(N) * 10.0 ** rng.integers(-8, 8, N)).astype(np.float32)
    b[::7] = -a[::7] * np.float32(1 + 2 ** -20)        # near cancellation
    b[::11] = -a[::11]                                 # exact cancellation
    b[::13] = 0.0
    a[::17] = 1.0
    i = rng.integers(-(2 ** 46), 2 ** 46, N)
    i[::5] = rng.integers(-1000, 1000, N // 5)
    np.savetxt(f"{d}/fp_a.hex", a.view(np.uint32), fmt="%08x")
    np.savetxt(f"{d}/fp_b.hex", b.view(np.uint32), fmt="%08x")
    np.savetxt(f"{d}/fp_i.hex", i.astype(np.int64) & ((1 << 48) - 1), fmt="%012x")
    return a, b, i


def ftz(x):
    x = np.asarray(x, dtype=np.float32).copy()
    x[np.abs(x) < np.finfo(np.float32).tiny] = 0.0
    return x


def main(d):
    a, b, i = gen(d) if "--gen" in sys.argv else (None, None, None)
    if "--gen" in sys.argv:
        return 0
    rng_a = np.loadtxt(f"{d}/fp_a.hex", dtype=str)
    a = np.array([int(x, 16) for x in rng_a], dtype=np.uint32).view(np.float32)
    b = np.array([int(x, 16) for x in np.loadtxt(f"{d}/fp_b.hex", dtype=str)],
                 dtype=np.uint32).view(np.float32)
    iv = np.array([int(x, 16) for x in np.loadtxt(f"{d}/fp_i.hex", dtype=str)], dtype=np.int64)
    iv = np.where(iv >= 2 ** 47, iv - 2 ** 48, iv)
    rows = [ln.split() for ln in open(f"{d}/fp_out.txt")]
    h = lambda s: np.uint32(int(s, 16)).view(np.float32)
    errs = {"add": 0, "mul": 0, "i2f": 0, "rcp": 0}
    with np.errstate(all="ignore"):
        for k, r in enumerate(rows):
            ref_add = ftz(np.float32(a[k] + b[k]))
            ref_mul = ftz(np.float32(a[k] * b[k]))
            ref_i2f = np.float32(np.float32(iv[k]) * np.float32(2.0 ** -12))
            for name, ref, got in (("add", ref_add, h(r[0])), ("mul", ref_mul, h(r[1])),
                                   ("i2f", ref_i2f, h(r[2]))):
                if not (ref == got or (np.isinf(ref) and np.isinf(got))):
                    errs[name] += 1
                    if errs[name] < 5:
                        print(f"{name} mismatch #{k}: a={a[k]!r} b={b[k]!r} i={iv[k]} "
                              f"ref={ref!r} got={got!r}")
            if len(r) > 3 and a[k] != 0:
                ref = ftz(np.float32(np.float32(1.0) / a[k]))
                got = h(r[3])
                if not (ref == got or (np.isinf(ref) and np.isinf(got))):
                    errs["rcp"] += 1
                    if errs["rcp"] < 5:
                        print(f"rcp mismatch #{k}: a={a[k]!r} ref={ref!r} got={got!r}")
    print(f"checked {len(rows)} vectors, mismatches: {errs}")
    return 0 if sum(errs.values()) == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else "."))
