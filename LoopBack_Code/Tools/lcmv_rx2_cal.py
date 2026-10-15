#!/usr/bin/env python3
"""lcmv_rx2_cal.py -- RX2 channel calibration for the 2-element LCMV CRPA.

The LCMV null lands exactly where LCMV_JAM_ANGLE points ONLY if RX1 and RX2
have the same gain and phase. Any mismatch (cables, AD9361 RX paths,
rx1rx2_phase_inversion_en in main.c) moves the null. LCMV_RX2_CAL (0x4C)
multiplies RX2 by a complex constant c before beamforming; this script finds c.

PROCEDURE
  1. Feed ONE CW tone through a 2-way splitter with EQUAL-LENGTH cables into
     RX1 and RX2 (this is a source at broadside, theta = 0). Keep the level
     well below ADC full scale.
  2. Capture I/Q with the existing gnss_capture path (util_cpack2 order
     I0, Q0, I1, Q1, int16 little-endian, 12-bit right-aligned) and pull the
     buffer off the board as a raw binary file.
  3. Run:   python3 lcmv_rx2_cal.py capture.bin
  4. Type the printed command on the console, e.g.  lcmv_cal=-0.98481 0.17365

With the antennas mounted, also correct for the cable lengths from the
antennas to the splitter point: only the RX paths from that point on are
calibrated by this procedure.

    g = E[x2 conj(x1)] / E[|x1|^2]      (RX2 relative to RX1)
    c = 1 / g                           (so that c * x2 == x1 for theta = 0)
"""
import sys

import numpy as np


def load_capture(path):
    raw = np.fromfile(path, dtype="<i2")
    raw = raw[: (raw.size // 4) * 4].reshape(-1, 4).astype(np.int32)
    # 12-bit right-aligned: sign-extend from bit 11 regardless of [15:12]
    raw = ((raw & 0xFFF) ^ 0x800) - 0x800
    x1 = raw[:, 0] + 1j * raw[:, 1]
    x2 = raw[:, 2] + 1j * raw[:, 3]
    return x1, x2


def rx2_cal(x1, x2):
    g = np.vdot(x1, x2) / np.vdot(x1, x1)          # E[x2 x1*] / E[|x1|^2]
    c = 1.0 / g
    coh = abs(np.vdot(x1, x2)) / np.sqrt(np.vdot(x1, x1).real * np.vdot(x2, x2).real)
    return g, c, coh


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        sys.exit(2)
    x1, x2 = load_capture(sys.argv[1])
    g, c, coh = rx2_cal(x1, x2)
    peak = max(np.abs(x1.real).max(), np.abs(x1.imag).max(),
               np.abs(x2.real).max(), np.abs(x2.imag).max())

    print(f"samples            : {x1.size}")
    print(f"peak |sample|      : {peak:.0f} of 2047" + ("   CLIPPING - reduce level" if peak >= 2040 else ""))
    print(f"RX1/RX2 coherence  : {coh:.4f}" + ("   LOW - check the splitter feed" if coh < 0.95 else ""))
    print(f"RX2 / RX1 (g)      : |g| = {abs(g):.4f} ({20*np.log10(abs(g)):+.2f} dB), "
          f"phase = {np.degrees(np.angle(g)):+.2f} deg")
    if max(abs(c.real), abs(c.imag)) > 1.99993:
        # Q2.14 holds -2 .. +1.99994 per component (RX2 more than 6 dB weaker).
        print(f"note               : |c| = {abs(c):.4f} does not fit Q2.14; limited.")
        print("                     RX2 is > 6 dB weaker than RX1 - fix the RX gains first.")
        c = c / max(abs(c.real), abs(c.imag)) * 1.99993
    print(f"calibration c      : {c.real:+.5f} {c.imag:+.5f}j")
    print()
    print(f"console command    : lcmv_cal={c.real:.5f} {c.imag:.5f}")
    print(f"register 0x4C      : 0x{(int(round(c.imag*16384)) & 0xFFFF):04X}"
          f"{(int(round(c.real*16384)) & 0xFFFF):04X}   (Q2.14)")


if __name__ == "__main__":
    main()
