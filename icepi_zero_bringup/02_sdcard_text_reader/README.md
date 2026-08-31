# IcePi-Zero bring-up #2: SD card text file reader

Reads one text file off a FAT16-formatted microSD card (over the board's
onboard SD slot, in SPI mode) and streams its contents out UART, byte by
byte, at boot and again on every button press.

## What it does

1. `sdcard_spi.v` runs the SD card's standard SPI-mode power-up sequence
   (CMD0 → CMD8 → CMD55/ACMD41 loop → CMD58 → CMD16), detects whether the
   card is SDSC (byte-addressed) or SDHC/SDXC (block-addressed), and offers
   a simple "read one 512-byte block" interface to everything above it.
2. `fat16_reader.v` parses the boot sector's BPB (BIOS Parameter Block) at
   runtime, searches the root directory for a file by 8.3 name (`HELLO.TXT`
   by default), and streams that file's bytes out, following the FAT
   cluster chain across as many clusters as the file needs.
3. A 1024-byte `sync_fifo.v` absorbs the rate mismatch between the SD
   card's fast per-block bursts and UART's much slower pace, with
   `fat16_reader` backpressured so it never issues more than one block's
   worth of unread data — no byte is ever dropped, however large the file.
4. `uart_tx.v` (115200 8N1) puts the bytes on the wire, over the board's
   own onboard FTDI USB-UART chip — same USB cable used for flashing, shows
   up as a second COM port.

## Why this order (SD card first, before the HDMI image sub-project)

Sub-project #3 (SD card → HDMI image) reuses this same `sdcard_spi.v` +
a BMP-flavored FAT16 reader wholesale. Proving the SD SPI driver and FAT16
parsing against a simple UART-text-out consumer first — rather than
debugging the storage stack and the display stack simultaneously — is the
same "isolate one unknown at a time" discipline this repo's camera project
needed several real-hardware rounds to (re)learn the hard way.

## Preparing the SD card

Most cards ship formatted FAT32. This reader deliberately only understands
FAT16 (a fixed-size root directory, 16-bit cluster numbers — meaningfully
less logic than FAT32's variable-size root directory and 32-bit chains, and
plenty for a bring-up demo). Reformat a small card (FAT16 needs a card at
most a few GB) as FAT16, e.g.:

```bash
# Linux, replace /dev/sdX1 with your card's actual partition
sudo mkfs.vfat -F 16 /dev/sdX1
```

or use Windows' Format dialog (FAT16 only appears as an option for small
enough cards). Then copy a plain-text file onto it named `HELLO.TXT`
(8.3 name — exactly 5 letters + `.TXT`; change `FILENAME` in
`sdcard_text_reader_top.v`'s parameters to read a different name).

## LEDs (all real, onboard, no external wiring)

| LED | Meaning |
|---|---|
| `led[0]` | SD card detected + initialized (SPI handshake succeeded) |
| `led[1]` | A file read is in progress |
| `led[2]` | Last operation (SD init or FAT16 read) failed — sticky |
| `led[3]` | Last file read completed successfully — sticky |
| `led[4]` | 1Hz heartbeat — proves the clock/bitstream is alive |

Press `button[1]` to re-read the file at any time; `button[0]` resets the
whole design (re-runs SD card init from scratch).

## Verified

- **Simulation** (Icarus Verilog), three separate testbenches:
  - `make sim` (`tb_sdcard_text_reader.v`): `sdcard_spi.v` + `fat16_reader.v`
    directly, against a behavioral SD card SPI slave model
    (`tb/sd_card_model.v`) serving a synthetic-but-spec-correct FAT16 volume
    (`tb/gen_fat16_image.py`) whose `HELLO.TXT` deliberately spans two
    clusters, so the cluster-chain-follow path is actually exercised, not
    just the single-cluster case. **`TB_SDCARD_TEXT_READER: PASS`** — SD
    init reaches `ready` with `card_type` correctly detected as SDHC, the
    boot sector's BPB fields are parsed correctly, the file is found by
    name in the root directory, and all 600 streamed bytes match the
    source file exactly, including the cluster-2 → cluster-3 handoff.
  - `make sim_top` (`tb_sdcard_text_reader_top.v`): the *entire* real
    top-level module (SD SPI + FAT16 + FIFO + UART, exactly as it will be
    on real hardware) against the same card model, decoding the UART
    serial line bit-by-bit externally and checking it against the source
    file. **`TB_SDCARD_TEXT_READER_TOP: PASS`** — this is the one test that
    actually exercises the FIFO backpressure path and the UART handshake
    timing; two real synchronization bugs were caught and fixed here (see
    "Bugs found this way" below), neither of which the lower-level test
    could see.
  - `make uart` (`tb_uart_tx.v`): `uart_tx.v` alone, decoding start/stop
    bits and payload for several bytes. **`TB_UART_TX: PASS`**.
- **Synthesis** (`make synth`, Yosys `synth_ecp5`): 0 CHECK-pass problems.
  2497 cells: 1164 LUT4, 903 FF, 297 CCU2C (carry chains, from the FAT16
  reader's cluster-number arithmetic and byte counters), 1 DP16KD (the
  1024-byte FIFO — fits in a single 16Kbit block RAM), 3 MULT18X18D (the
  `(cluster - 2) * sectors_per_cluster` sector-address multiply, only
  evaluated once per cluster transition, not per byte).
- **Place & route** (`make pnr`, `nextpnr-ecp5 --25k --package CABGA256
  --speed 6 --seed 1`): closes timing with real margin — **70.33MHz achieved
  vs. 50MHz target** on the `clk` domain. 0 errors, "Program finished
  normally."
- **Not yet flashed to real hardware in this session** — `ecppack` and
  `openFPGALoader` aren't installed in this sandbox, so the bitstream-pack
  and flash steps (`make bit` / `make prog`) are untested here; everything
  upstream of them (RTL logic, synthesis, place & route against the real
  board's real LPF) is verified as above. Bring a real FAT16 card and a
  real UART terminal to close that last gap.

### Bugs found this way (real, fixed before reaching hardware)

Two bugs only showed up in the full-top-level test, not the lower-level
one — a good illustration of why both exist:

1. **`sdcard_spi.v`'s `byte_index` output was one position ahead of the
   `byte_data` it labeled.** Both were updated via nonblocking assignment
   on the same edge that also *advanced* `byte_index` for the next byte —
   so a downstream consumer sampling them together one edge later saw the
   already-advanced index paired with the byte it used to label. Fixed by
   separating the internal loop counter (`byte_pos`) from the output
   register (`byte_index`), so `byte_index` reports the pre-increment
   position on the same edge `byte_data` changes.
2. **The FIFO→UART pop gate in `sdcard_text_reader_top.v` allowed a second
   pop to sneak in one cycle before `uart_ready_w` actually dropped.**
   `uart_tx`'s own `ready` signal lags its `busy` state by one cycle (a
   registered ready/valid handshake), and the pop gate wasn't accounting
   for that extra cycle — an early second pop got silently dropped by
   `uart_tx` (already busy) after already being removed from the FIFO,
   shifting every subsequent byte by one position. Fixed by also gating
   new pops on `!uart_valid_r`.

Both were real synchronous-design timing bugs, not simulation artifacts —
both would have corrupted the byte stream on real hardware exactly as they
did in `sim_top`, silently and non-obviously (readable-looking but wrong
text, not an outright crash).

## Building

```bash
cd 02_sdcard_text_reader
make check     # all three simulations — should print PASS x3
make prog      # synth + P&R + pack + flash in one step
```

## Module reference

| File | Role |
|---|---|
| `rtl/spi_master.v` | Generic SPI mode-0 byte shifter, runtime clock divider |
| `rtl/sdcard_spi.v` | SD card SPI-mode init + single-block read state machine |
| `rtl/fat16_reader.v` | FAT16 boot sector parse, root-dir file lookup, cluster-chain stream |
| `rtl/sync_fifo.v` | Single-clock-domain FIFO (SD burst rate → UART pace) |
| `rtl/uart_tx.v` | 8N1 UART transmitter |
| `rtl/sdcard_text_reader_top.v` | Top-level: wiring, debounce, status LEDs |
| `tb/sd_card_model.v` | Simulation-only behavioral SD card SPI slave |
| `tb/gen_fat16_image.py` | Generates the synthetic FAT16 test volume |

## Known limitations

- **FAT16 only**, root directory only, 8.3 short names only (no long
  filenames, no subdirectories) — see "Preparing the SD card" above.
- The behavioral SD card model always emulates an **SDHC/SDXC** card
  (block-addressed CMD17). `sdcard_spi.v`'s SDSC byte-addressing path
  (the `card_type==1` branch, `block_addr << 9`) is implemented from the
  same SD SPI-mode spec but is **not exercised by simulation** — it's a
  single shift-based address computation, low risk, but genuinely
  untested until confirmed against a real SDSC (older, smaller, non-HC)
  card or a model built to emulate one.
- No CRC checking on the SPI link (matches the SD spec's own default —
  SPI-mode CRC is off unless CMD59 turns it on, which this driver never
  sends). A corrupted transfer would currently go undetected rather than
  being retried.
- Button-triggered re-read has hardware debounce (~1ms) but no protection
  against re-reading while the FIFO still holds unsent bytes from a
  previous read that hasn't finished draining to UART — a rapid re-press
  mid-transfer could interleave old and new file bytes. Not a concern for
  the auto-start-once-at-boot use case; worth a `!fat_busy_w` guard before
  relying on rapid manual re-reads.
