# Notes for Claude

## antsdr-fw-patch release zips

Name each new design-A (pass-through + CRPA) package with the next number:
`crpa1`, `crpa2`, `crpa3`, ...

- `crpa1` (2026-11-10): design A modified, SMI-PI v4.9.14 6 + 6 taps, LVDS 2R2T,
  band 7, `axi_tdd` removed (timing fix). Commit 960625c.
- `crpa2` (2026-11-10): crpa1 + design A on bare metal (`./build.sh baremetal`,
  `antsdr-fw-patch/baremetal/`): SMI-PI firmware on the design-A FPGA, GPIO 67 /
  86..93, gnss_passthrough 0x43C10000, VCXO 23000, band 7 saved on SD.
- Next package: `crpa3`.

Files per package: `crpaN-full-src.zip.part1`, `crpaN-full-src.zip.part2`,
`crpaN-kit.zip`, `SHA256SUMS-crpaN.txt`.
