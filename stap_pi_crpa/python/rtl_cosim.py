#!/usr/bin/env python3
"""
Co-simulate rtl/stap_pi_filter.v (Icarus Verilog) against the bit-true model.

1. generate the wideband barrage-jammer scenario,
2. run the floating-point STAP-PI processor -> per-block weights,
3. quantise inputs (16 bit) and weights (Q1.16, 18 bit),
4. simulate the RTL with those vectors and compare it *bit-exactly* with
   ``stap_pi.fixed_point.stap_filter_fixed``,
5. report the fixed-point implementation loss against the float processor.

    python3 rtl_cosim.py                      # 2 x 7 taps, 4 blocks of 1024
    python3 rtl_cosim.py --gap 1 --out-shift 12
"""

from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from stap_pi import CrpaScenario, StapPIConfig, StapPIProcessor  # noqa: E402
from stap_pi.fixed_point import quantize_input, quantize_weights, stap_filter_fixed, to_hex_words  # noqa: E402

RTL_DIR = pathlib.Path(__file__).resolve().parents[1] / "rtl"
IW, WW, WFRAC, OW = 16, 18, 16, 16


def run_cosim(n_taps: int = 7, block_len: int = 1024, n_blocks: int = 4, out_shift: int = 10, gap: int = 0,
              js_db: float = 95.0, seed: int = 3, workdir: pathlib.Path | None = None) -> dict:
    if shutil.which("iverilog") is None or shutil.which("vvp") is None:
        raise RuntimeError("Icarus Verilog (iverilog, vvp) not found")
    n = block_len * n_blocks
    sc = CrpaScenario(js_db=js_db, duration_s=n / 30e6, seed=seed)
    data = sc.generate()
    x = data.x[:, :n]
    res = StapPIProcessor(StapPIConfig(n_taps=n_taps, block_len=block_len)).process(x)

    xr, xi = quantize_input(x, IW)
    wr, wi = quantize_weights(res.weights, WW, WFRAC)
    yr_ref, yi_ref = stap_filter_fixed(xr, xi, wr, wi, block_len, n_taps, out_shift, OW)

    tmp = tempfile.TemporaryDirectory() if workdir is None else None
    wd = pathlib.Path(tmp.name if tmp else workdir)
    wd.mkdir(parents=True, exist_ok=True)
    try:
        inter = np.stack([xr[0], xi[0], xr[1], xi[1]], axis=1) if x.shape[0] == 2 else \
            np.stack([v for m in range(x.shape[0]) for v in (xr[m], xi[m])], axis=1)
        (wd / "tb_in.hex").write_text("\n".join(to_hex_words(inter, IW)) + "\n")
        wint = np.stack([wr, wi], axis=2)                     # (blocks, D, 2) -> re, im interleaved
        (wd / "tb_w.hex").write_text("\n".join(to_hex_words(wint, WW)) + "\n")
        params = {"N_EL": x.shape[0], "N_TAPS": n_taps, "IW": IW, "WW": WW, "OW": OW, "NS": n,
                  "NB": n_blocks, "BLOCK": block_len, "OUT_SHIFT": out_shift, "GAP": gap}
        cmd = ["iverilog", "-g2005", "-o", str(wd / "tb.vvp")]
        cmd += [f"-Ptb_stap_pi_filter.{k}={v}" for k, v in params.items()]
        cmd += [str(RTL_DIR / "stap_pi_filter.v"), str(RTL_DIR / "tb" / "tb_stap_pi_filter.v")]
        subprocess.run(cmd, check=True, capture_output=True, text=True)
        sim = subprocess.run(["vvp", "-n", str(wd / "tb.vvp")], cwd=wd, check=True, capture_output=True, text=True)
        out = np.loadtxt(wd / "tb_out.txt", dtype=np.int64, ndmin=2)
    finally:
        if tmp:
            tmp.cleanup()

    mism = int(np.sum((out[:, 0] != yr_ref) | (out[:, 1] != yi_ref)))
    # fixed-point vs floating-point (skip start-up blocks before adaptive weights)
    k0 = 2 * block_len
    scale = 2.0 ** (out_shift - (IW - 1) - WFRAC)
    y_fix = (out[:, 0] + 1j * out[:, 1]) * scale
    err = np.mean(np.abs(y_fix[k0:] - res.y[k0:]) ** 2)
    noise_out = np.mean(np.abs(res.apply(data.parts["noise"][:, :n] + data.parts["adc"][:, :n])[k0:]) ** 2)
    sat = int(np.sum(np.abs(out[k0:]) >= (1 << (OW - 1)) - 1))
    return {
        "samples": n, "mismatches": mism, "sim_log": sim.stdout.strip(),
        "fixed_vs_float_error_db_re_noise": float(10 * np.log10(err / noise_out)),
        "saturated_after_startup": sat,
        "output_rms_lsb": float(np.sqrt(np.mean(np.abs(out[k0:, 0] + 1j * out[k0:, 1]) ** 2) / 2)),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--taps", type=int, default=7)
    ap.add_argument("--block-len", type=int, default=1024)
    ap.add_argument("--blocks", type=int, default=4)
    ap.add_argument("--out-shift", type=int, default=10)
    ap.add_argument("--gap", type=int, default=0)
    ap.add_argument("--js-db", type=float, default=95.0)
    args = ap.parse_args(argv)
    r = run_cosim(args.taps, args.block_len, args.blocks, args.out_shift, args.gap, args.js_db)
    print(r["sim_log"])
    print(f"RTL vs bit-true model : {r['mismatches']} mismatching samples of {r['samples']}")
    print(f"fixed vs float error  : {r['fixed_vs_float_error_db_re_noise']:.1f} dB re. output noise")
    print(f"output rms            : {r['output_rms_lsb']:.0f} LSB (16-bit), saturated samples: {r['saturated_after_startup']}")
    return 0 if r["mismatches"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
