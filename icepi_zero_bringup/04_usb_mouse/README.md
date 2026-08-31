# IcePi-Zero bring-up #4: USB mouse (bit-banged low-speed host)

A low-speed (1.5Mbit/s) USB host, entirely bit-banged over two raw GPIO
pins (D+/D-) with the board's own switchable pull-down resistors — no
external USB PHY chip. Reads a HID boot-protocol mouse's button/X/Y deltas
and reports them over UART.

**This is the highest-risk sub-project in the whole `icepi_zero_bringup`
collection** — comparable to, or exceeding, the difficulty of the real
multi-round hardware debugging the sibling `dvp_camera_hdmi_pipeline`
project needed. Read "Verified" below carefully before trusting this on
real hardware; it is the one sub-project here where the honest answer to
"does the whole thing work" is genuinely "not yet confirmed."

## What it does

1. **`usb_nco.v`** — a fractional-N tick generator (Bresenham/DDA
   accumulator, not a PLL) producing 1.5MHz-average ticks from the 50MHz
   board clock — 50MHz/1.5MHz = 33.33, not an integer ratio, so a fixed
   divider would drift.
2. **`usb_packet_tx.v`** — builds and transmits one complete low-speed USB
   packet (SYNC + PID + payload + CRC5-or-CRC16 + EOP), bit-stuffed and
   NRZI-encoded, straight onto D+/D-.
3. **`usb_packet_rx.v`** — the mirror image: detects start-of-packet,
   recovers bit timing (resynced to every line transition — bit-stuffing
   guarantees one at least every 7 bit periods), NRZI-decodes,
   bit-destuffs, and reports the decoded PID/payload once EOP arrives.
4. **`usb_host_ctrl.v`** — drives a bus reset (SE0 held 15ms), then
   enumerates the attached device with three no-data-stage control
   transfers (SET_ADDRESS → SET_CONFIGURATION → SET_PROTOCOL boot), then
   polls the mouse's interrupt endpoint forever, extracting
   `{buttons, dx, dy}` from each HID boot-protocol report.
5. **`usb_mouse_top.v`** — wires it all to the board's real D+/D- pins and
   pull-down-enable pins, and streams each report as 3 raw bytes over UART.

## Why boot protocol, and why no descriptor parsing

Requesting `SET_PROTOCOL(boot)` during enumeration is what makes a fixed
report layout safe to assume: HID **boot protocol** guarantees a mouse's
interrupt-IN reports are exactly `{buttons[7:0], dx[7:0], dy[7:0], ...}`
— byte 0 a button bitmap, bytes 1–2 signed X/Y deltas — **regardless** of
that specific device's own HID report descriptor. This sidesteps the need
to fetch and parse a report descriptor (itself a meaningfully bigger
undertaking: variable-length, type-tagged binary data describing an
arbitrary bit-level report layout). `usb_host_ctrl.v` also skips
`GET_DESCRIPTOR` entirely for the same reason — this host doesn't need to
identify *which* device is attached, only to talk to *a* low-speed HID
mouse.

## LEDs (all real, onboard, no external wiring beyond the mouse itself)

| LED | Meaning |
|---|---|
| `led[0]` | `device_ready` — enumeration completed successfully (sticky) |
| `led[1]` | Toggles on every mouse report received (movement/button activity) |
| `led[2]` | `error` — gave up during reset/enumeration/polling (sticky) |
| `led[4]` | 1Hz heartbeat |

`button[0]` resets the whole host controller (re-runs bus reset +
enumeration) — use this after plugging in a mouse.

## Verified

- **PHY layer** (`make sim`, `tb_packet_loopback.v`): `usb_packet_tx.v`'s
  D+/D- output wired straight into `usb_packet_rx.v`'s input (a direct
  electrical loopback), round-tripping 10 packets — 2 tokens (different
  CRC5 inputs), 3 handshakes (ACK/NAK/STALL, PID-only with no CRC at all),
  and 5 data packets of varying length including two deliberately crafted
  to force bit-stuffing (`0xFF...FF` and `0x55...55`, each 8 bytes).
  **`TB_PACKET_LOOPBACK: PASS`** — every packet's PID, payload, and (where
  applicable) CRC5/CRC16 round-trip exactly. Both sides implement the real
  USB 2.0 bit-serial NRZI/stuffing/CRC algorithms (not a private
  shortcut), so this demonstrates spec-correct framing, not just
  internal self-consistency.
- **Synthesis** (`make synth`, Yosys `synth_ecp5`): 0 CHECK-pass problems,
  6095 cells.
- **Place & route** (`make pnr`, `nextpnr-ecp5 --25k --package CABGA256
  --speed 6 --seed 1`): closes timing with real margin — **75.87MHz
  achieved vs. 50MHz target**. 0 errors, "Program finished normally."
- **NOT simulated: `usb_host_ctrl.v`'s enumeration + polling sequence.**
  No behavioral USB device model exists in this session to simulate
  against (building an accurate one — SE0/J/K line-state responses,
  correct handshake timing, a believable HID report descriptor and boot
  reports — is its own substantial undertaking, well beyond what the PHY
  loopback test needed). This module is real RTL written directly from
  the USB 2.0 specification's control-transfer and interrupt-transfer
  state diagrams, reusing the now-verified `usb_packet_tx.v`/
  `usb_packet_rx.v` primitives, but its correctness beyond individual
  compilation and synthesis is **unconfirmed**.
- **Not yet flashed to real hardware in this session.**

### Four real bugs found and fixed — two by simulation, two only by real synthesis/P&R

The first two were caught by `tb_packet_loopback.v`, not found by
inspection. The other two are the sharpest illustration in this whole
`icepi_zero_bringup` collection of why "it simulates" and "it's real
hardware-ready" are different claims: **both passed Icarus simulation
outright** (Icarus has no timing model and doesn't care how many gates
a combinational path takes) and were only caught once the design was run
through actual synthesis and place & route — the first time
`usb_packet_tx.v`/`usb_packet_rx.v` were synthesized at all, since the
PHY loopback testbench only ever simulates them.

1. **RX's SYNC/PID boundary was off by one tick, every single time,
   independent of packet content or timing.** SYNC is 8 bits and its
   first bit is free (the idle→K transition *is* that bit's value, no
   sampling needed), so on paper only 7 more ticks should be needed
   before real data starts. Empirically, exactly one more tick than that
   consistently lands inside the sync region — confirmed by directly
   counting decode ticks against the transmitter's actual bit count, and
   by testing PID values on both sides of the SYNC/PID edge boundary
   (ruling out a timing-race explanation, since a deterministic,
   content-independent one-tick discrepancy isn't consistent with
   marginal sampling drift). The synchronizer (2 cycles) and the NCO's
   own resync-application latency (1 more cycle) apparently stack such
   that RX's first post-SOP tick re-samples bit0 rather than landing on
   bit1. Fixed by classifying 8 ticks (not 7) as SYNC, absorbing the
   extra tick harmlessly instead of it corrupting real data — see
   `usb_packet_rx.v`'s comment at the fix for the full account.
2. **The test harness itself had a race**: waiting only on `rx_done`
   before starting the next packet let TX's own EOP sequence (which takes
   several more bit periods after RX has already declared the packet
   done via its SE0 threshold) still be in flight when the next `start`
   pulse arrived — silently dropped, since `usb_packet_tx.v` only samples
   `start` while idle. `usb_host_ctrl.v`'s own control-transfer and
   polling loops have the same structural risk (issuing a new token
   right after a handshake) and were written with an explicit `!tx_busy`
   gate specifically because of what this taught — but that gate itself
   is one of the pieces `usb_host_ctrl.v`'s own lack of simulation
   coverage means is unconfirmed beyond code review.
3. **`usb_packet_tx.v`'s bit-stuffing insertion blew up synthesis
   outright** — thousands of cells and still growing when killed after
   5+ minutes (a comparable-complexity sub-project elsewhere in this
   collection synthesizes in ~10-20 seconds). The original code built the
   entire stuffed bitstream in one combinational sweep per packet, with a
   *variable*, data-dependent write index into a 128-bit register —
   simulates fine (Icarus just executes it procedurally, step by step),
   but Yosys has to model "which of up to ~103 sequential writes to this
   bit wins" as a priority-mux chain, and that blows up combinatorially.
   Fixed by spreading the same computation over one real (or stuff) bit
   per clock cycle instead (a new `ST_BUILD` state) — the same plain,
   cheap "clocked write, counter-indexed" pattern `usb_packet_rx.v`
   already used successfully for its own bit storage, taking at most
   ~103 clock cycles (~2µs) before the tick-paced sending that follows
   even begins.
4. **`usb_packet_rx.v`'s CRC16 recomputation missed 50MHz by more than
   3x** (14.14MHz achieved) — a real place-and-route timing failure, not
   a blow-up. The original code recomputed the CRC16 over up to 64
   payload bits in one combinational loop within a single clock cycle at
   EOP — a 64-deep chain of dependent XOR/shift steps, each one waiting
   on the last, is simply too much logic depth for one 20ns period.
   Fixed the same way as bug 3: spread over one CRC step per clock cycle
   (a new `ST_CRC_LOOP` state) instead of all 64 at once. Real STA
   (`nextpnr-ecp5`), not simulation, is what catches this class of bug —
   see `dvp_camera_hdmi_pipeline`'s and `03_sdcard_hdmi_image`'s own
   READMEs for two earlier, analogous timing-closure findings in this
   repo (both, notably, *display*-pipeline combinational-depth issues —
   this one is the first of that same class found on a *receive* path).

## Building

```bash
cd 04_usb_mouse
make check     # PHY loopback simulation -- should print PASS
make prog      # synth + P&R + pack + flash in one step
```

## Module reference

| File | Role |
|---|---|
| `rtl/usb_nco.v` | Fractional-N 1.5MHz-average bit-rate tick generator |
| `rtl/usb_packet_tx.v` | Packet transmit: SYNC/PID/CRC5-or-16/bit-stuff/NRZI/EOP |
| `rtl/usb_packet_rx.v` | Packet receive: SOP detect, bit recovery, NRZI decode, destuff, CRC check |
| `rtl/usb_host_ctrl.v` | Bus reset, enumeration, HID boot-mouse interrupt polling |
| `rtl/uart_tx.v` | 8N1 UART transmitter (shared design with sibling sub-projects) |
| `rtl/usb_mouse_top.v` | Top-level: pin wiring, report→UART, status LEDs |
| `tb/tb_packet_loopback.v` | The one thing that's actually verified here |

## Known limitations

- **Enumeration/polling logic is unverified beyond synthesis** — see
  "Verified" above. This is the real, primary risk of this sub-project.
- **Pull-down pin polarity is an assumption, not a confirmed fact.**
  `usb_pull_dp`/`usb_pull_dn` are driven high on the assumption that
  they're active-high enables for the board's onboard pull-down
  resistors (the correct role for a USB *host* port). Verify against the
  board schematic before relying on this — driving the wrong polarity
  would leave the bus without proper host-side pull-downs, and device
  attach detection (and everything downstream of it) would never work.
- **No device-detect / re-enumerate-on-hot-plug logic.** `link_state`
  reaching `S_ERROR` (or a mouse being unplugged mid-poll) requires a
  manual `button[0]` reset to recover — a real product would watch for a
  bus-idle-to-attached transition and automatically restart the sequence.
- **Full-speed devices are not supported** — this design only implements
  low-speed (1.5Mbit/s) signaling and timing throughout. A full-speed-only
  mouse (no low-speed fallback) will not enumerate.
- **No SOF (Start-of-Frame) generation.** Real USB hosts send a token
  every 1ms to maintain bus timing/keep-alive; this design relies on
  best-effort continuous IN polling instead, which works with most simple
  low-speed HID devices but is a deviation from the full spec.
- **CRC16 checked on RX, not on TX-generated tokens' CRC5 being verified
  by the device** (that's the device's job, out of this design's
  control) — and RX itself never validates *token* packet CRC5 (see
  `usb_packet_rx.v`'s header comment), since this host never receives
  tokens, only handshakes and data.
