#!/usr/bin/env python3
"""
Runs the PI-AJ v5 RTL (Icarus Verilog) on a set of jamming scenarios and
measures, from the RTL output, the RMS suppression and the GNSS C/N0 seen by
a correlator (the number that decides if a receiver locks).

    python3 run_rtl.py            # all scenarios
    python3 run_rtl.py cw71 nz50  # selected
"""
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
WORK = os.path.join(HERE, "work")
sys.path.insert(0, HERE)
import analyze  # noqa: E402

N = 180000
# name: (description, generator args)
SCEN = {
    "none":  ("no jammer (reference)",                     "--jtype none"),
    "cw51":  ("CW 1575.40 MHz, J/N 51 dB (~40 dB setting)", "--jn 51"),
    "cw61":  ("CW 1575.40 MHz, J/N 61 dB (~30 dB setting)", "--jn 61"),
    "cw71":  ("CW 1575.40 MHz, J/N 71 dB (~20 dB setting)", "--jn 71"),
    "cw76":  ("CW 1575.40 MHz, J/N 76 dB (~15 dB setting)", "--jn 76"),
    "cw71c": ("CW exactly 1575.42 MHz (DC), J/N 71 dB",     "--jn 71 --jf 0"),
    "cw61g": ("CW 1582.5 MHz (+7.08 MHz), J/N 61 dB",       "--jn 61 --jf 7.08e6"),
    "nz40":  ("barrage noise 18 MHz, J/N 40 dB",            "--jtype noise --jn 40"),
    "nz50":  ("barrage noise 18 MHz, J/N 50 dB",            "--jtype noise --jn 50"),
    "nz50n": ("barrage noise 2 MHz, J/N 50 dB",             "--jtype noise --jn 50 --jbw 2e6"),
}


def sh(cmd):
    return subprocess.run(cmd, shell=True, cwd=WORK, check=True,
                          capture_output=True, text=True).stdout


def build():
    os.makedirs(WORK, exist_ok=True)
    root = os.path.dirname(HERE)
    sh("iverilog -g2005 -o tb.vvp %s/tb/tb_pi_aj.v %s/rtl/*.v" % (root, root))


def run(name):
    desc, args = SCEN[name]
    sh("python3 %s/gen_stimulus.py --name %s --n %d %s" % (HERE, name, N, args))
    log = sh("vvp -n tb.vvp +IN=%s.hex +OUT=rtl_%s" % (name, name))
    with open(os.path.join(WORK, "rtl_%s.log" % name), "w") as fh:
        fh.write(log)
    meta = np.load(os.path.join(WORK, name + ".npz"))
    raw = np.loadtxt(os.path.join(WORK, name + ".hex"), dtype=str)
    v = np.vectorize(lambda s: int(s, 16))(raw[:, :2]).astype(np.int64)
    v[v >= 32768] -= 65536
    x0 = (v[:, 0] + 1j * v[:, 1]) * 16.0          # same scale as RTL internal
    y = np.loadtxt(os.path.join(WORK, "rtl_%s_pi.txt" % name))
    y = y[:, 0] + 1j * y[:, 1]
    settle = int(meta["jon"]) + 18000
    rin = analyze.report("in", x0, 0, meta, settle)
    rout = analyze.report("out", y, 0, meta, settle)
    flags = [l for l in log.splitlines() if l.startswith("STATUS") or "NOTCH1_A" in l]
    return name, desc, rin, rout, flags


def main():
    names = sys.argv[1:] or list(SCEN)
    build()
    with ThreadPoolExecutor(max_workers=os.cpu_count() or 2) as ex:
        res = list(ex.map(run, names))
    lines = ["| scenario | RX1 C/N0 in (dB-Hz) | PI-AJ out C/N0 (dB-Hz) | RMS suppression (dB) |",
             "|---|---|---|---|"]
    for name, desc, rin, rout, flags in res:
        lines.append("| %s | %.1f | %.1f | %.1f |" % (
            desc, rin["cn0"], rout["cn0"], 10 * np.log10(rin["power"] / rout["power"])))
    txt = "\n".join(lines)
    print(txt)
    for name, desc, rin, rout, flags in res:
        print(name, " / ".join(f.strip() for f in flags))
    with open(os.path.join(WORK, "results.md"), "a") as fh:
        fh.write(txt + "\n\n")


if __name__ == "__main__":
    main()
