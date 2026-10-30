#!/usr/bin/env python3
"""
Runs the AntSDR integration wrapper (rtl/pi_aj_axis.v: NL=10, DELAY=4, no
notch #2, on a 100 MHz clock) on the jamming scenarios, with the samples
arriving at a fractional rate through a FIFO exactly as in the block design,
the registers set as the firmware sets them (AGC off, make-up shift) and the
12-bit TX1 output measured with the correlator.

    python3 run_axis.py                # all
    python3 run_axis.py cw71 nz50      # selected
"""
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(HERE, "work")
sys.path.insert(0, HERE)
import analyze          # noqa: E402
import run_rtl          # noqa: E402  (scenario list, stimulus generation)

# make-up shift as the firmware chooses it: floor(RX gain cut / 6 dB); the
# stimulus' clean noise is 3 LSB, so the cut is 20 log10(3 / noise)
def makeup(name):
    meta = np.load(os.path.join(WORK, name + ".npz"))
    cut = 20 * np.log10(3.0 / float(meta["noise_lsb"]))
    return max(0, int(cut // 6))


def run(job):
    name, fs = job
    desc, args = run_rtl.SCEN[name]
    if not os.path.exists(os.path.join(WORK, name + ".npz")):
        run_rtl.sh("python3 %s/gen_stimulus.py --name %s --n %d %s" % (HERE, name, run_rtl.N, args))
    sh = makeup(name)
    adapt = 0x0008C600 | ((sh & 0x1F) << 20)
    tag = "ax_%s_%g" % (name, fs)
    log = run_rtl.sh("vvp -n tbx.vvp +IN=%s.hex +OUT=%s +FS_MHZ=%g +CTRL=41B +ADAPT=%08x" %
                     (name, tag, fs, adapt))
    meta = np.load(os.path.join(WORK, name + ".npz"))
    y = np.loadtxt(os.path.join(WORK, tag + "_y.txt"))
    y = (y[:, 0] + 1j * y[:, 1]) / 2.0 ** sh
    r = analyze.report("out", y, 0, meta, int(meta["jon"]) + 18000)
    st = [l for l in log.splitlines() if l.startswith(("STATUS", "samples"))]
    return name, desc, fs, sh, r, st


def main():
    names = sys.argv[1:] or ["none", "cw61", "cw71", "cw76", "cw71c", "cw61g", "nz40", "nz50", "nz50n"]
    root = os.path.dirname(HERE)
    run_rtl.sh("iverilog -g2005 -o tbx.vvp %s/tb/tb_pi_aj_axis.v %s/rtl/*.v" % (root, root))
    jobs = [(n, 30.72) for n in names] + ([("cw71", 18.0)] if not sys.argv[1:] else [])
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
        res = list(ex.map(run, jobs))
    lines = ["| scenario | sample rate | make-up shift | TX1 C/N0 (dB-Hz) |", "|---|---|---|---|"]
    for name, desc, fs, sh, r, st in res:
        lines.append("| %s | %.2f MS/s | %d | %.1f |" % (desc, fs, sh, r["cn0"]))
    print("\n".join(lines))
    for name, desc, fs, sh, r, st in res:
        print(name, fs, " / ".join(st))
    with open(os.path.join(WORK, "axis_results.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
