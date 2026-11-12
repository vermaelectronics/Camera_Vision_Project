# Notes for Claude

## antsdr-fw-patch release zips

Name each new design-A (pass-through + CRPA) package with the next number:
`crpa1`, `crpa2`, `crpa3`, ...

- `crpa1` (2026-11-10): design A modified, SMI-PI v4.9.14 6 + 6 taps, LVDS 2R2T,
  band 7, `axi_tdd` removed (timing fix). Commit 960625c.
- `crpa2` (2026-11-10): crpa1 + design A on bare metal (`./build.sh baremetal`,
  `antsdr-fw-patch/baremetal/`): SMI-PI firmware on the design-A FPGA, GPIO 67 /
  86..93, gnss_passthrough 0x43C10000, VCXO 23000, band 7 saved on SD.
- `crpa3` (2026-11-12): crpa2 with SMI-PI 7 + 7 taps (reference delay 4,
  AJ_INFO 0x07040C07) instead of 6 + 6, Linux and bare metal. Commit 7fb1b60.
- `crpa4` (2026-11-12): crpa3 with SMI-PI 8 + 8 taps (reference delay 4,
  AJ_INFO 0x08040C07), Linux and bare metal. Commit 9917160.
- Next package: `crpa5`.

Files per package: `crpaN-full-src.zip.part1`, `crpaN-full-src.zip.part2`,
`crpaN-kit.zip`, `SHA256SUMS-crpaN.txt`.
