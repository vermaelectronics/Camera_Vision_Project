# IcePi-Zero bring-up #1: LED blink

The simplest possible "does my toolchain + board + flashing flow actually
work" smoke test — no external components, no wiring, just the board's own
onboard LEDs and buttons.

## What it does

- `led[0]` blinks at 1Hz, driven off the board's 50MHz reference oscillator
  through an exact-cycle-count divider.
- Hold `button[1]` (active-low) and `led[4:1]` becomes a free-running 4-bit
  binary counter (~4Hz), giving a second, visually distinct pattern you can
  trigger — proves button input works too, with zero PC-side tooling
  (no UART, no PC) needed to see *something* moving.
- `button[0]` (active-low) is a synchronous reset for the whole design.

## Why this exists

Every other sub-project in this `icepi_zero_bringup/` collection depends on
the same basic chain working first: real hardware, real oscillator, real
LEDs, a working `yosys` → `nextpnr-ecp5` → `ecppack` → `openFPGALoader`
toolchain. If this one doesn't blink after flashing, nothing else here will
work either, and the problem is almost certainly toolchain/board/wiring, not
RTL logic — this design is deliberately too simple to have a subtle bug.

## Verified

- **Simulation** (`make sim`, Icarus Verilog): checks the blink period is
  *exactly* the expected number of clock cycles (not just "it toggles
  eventually"), and that `button[1]` correctly switches `led[4:1]` between
  held-at-zero and counting. `TB_LED_BLINK: PASS`, 0 errors.
- **Synthesis** (`make synth`, Yosys `synth_ecp5`): 0 CHECK-pass problems,
  58 LUT4 / 53 FF — trivially small.
- **Place & route** (`make pnr`, `nextpnr-ecp5 --25k --package CABGA256
  --speed 6`): closes timing with enormous margin (`clk` domain: 204MHz
  achieved vs. 50MHz target) — unsurprising for a design this small, but
  confirmed against the real toolchain, not assumed.
- **Not yet flashed to real hardware in this session** — the RTL/synthesis/
  P&R chain is verified as above; the actual LED-blinks-on-your-desk step is
  yours to confirm (see Building below).

## Pin/site assignments

All from the IcePi-Zero board's own published LPF
(`gateware/icepi-zero.lpf` at https://github.com/cheyao/icepi-zero) — the
same source `dvp_camera_hdmi_pipeline` (the sibling project in this repo)
uses for its `clk`/`button`/`led` pins. No external wiring — these are all
real, populated components on the board itself.

| Signal | Site |
|---|---|
| `clk` | M1 (50MHz board oscillator) |
| `button[0]` | C4 |
| `button[1]` | C5 |
| `led[0]` | E13 |
| `led[1]` | D14 |
| `led[2]` | E12 |
| `led[3]` | C13 |
| `led[4]` | D13 |

## Building

Same open-source ECP5 flow as every other project in this collection
(`yosys` → `nextpnr-ecp5` → `ecppack` → `openFPGALoader`; see the parent
`icepi_zero_bringup/README.md` for the one-time toolchain install notes):

```bash
cd 01_led_blink
make check     # simulate first — should print TB_LED_BLINK: PASS
make prog      # synth + P&R + pack + flash in one step
```

`led[0]` should start blinking at 1Hz within a second or two of the flash
completing. Hold `button[1]` to see `led[4:1]` start counting.

## Known limitations

- None functionally — this is intentionally as simple as a real FPGA
  design gets. The only "limitation" is that it doesn't do anything useful
  beyond proving the bring-up chain works, which is exactly its job.
