# SD card — ANTSDR E310 V1 GNSS loopback + CRPA anti-jam (LoopBack_STAP_CRPA2)

Build it with `./Automation/Linux/build_bin.sh --band N` (the default build is this SD-card image;
see `README_CRPA_PIPELINE.md`).

---

## ⚠️ READ THIS FIRST — THIS BOARD TRANSMITS BY ITSELF

This image starts transmitting on the **power-on band's centre frequency**
(band 1 = GPS L1, 1575.42 MHz) the moment the board is powered, with **no
operator and no confirmation step**.

- **CONDUCTED, ATTENUATED COAX ONLY. NEVER AN ANTENNA.**
- Transmitting GNSS frequencies over the air is illegal in most jurisdictions
  and can disrupt navigation and timing for anything nearby.
- The firmware **cannot** tell what is attached to TX1. The interlock that
  normally forces a deliberate decision has been moved to build time on purpose,
  because an unattended board has nobody to ask.

**Output level:** TX attenuation is driven to the hardware maximum first, then
set to the power-on band's value, **20000 mdB = 20 dB (~ −13 dBm)**, in every band.
That is ~23 dB below the ~+10 dBm where GNSS front ends are damaged, but it is far
hotter than live GNSS (~ −130 dBm): a GNSS receiver connected **directly** to TX1
at this level is driven into saturation and shows no satellites. Put enough
**external attenuation** between TX1 and the receiver (typically 40–60 dB), or
raise `tx1_attenuation` on the console.

To change the level for good: edit the `tx_att_mdb` column of `gnss_bands[]` in
`Source/Firmware/app_gnss_e310/gnss_band.c` (10000..89750) and rebuild with
`--from sw`. Larger = quieter.

---

## What it does

    RX1 ─┐
         ├─► AD9361 ─► axi_ad9361 ─► gnss_stap (CRPA) ─► gnss_passthrough ─► axi_ad9361 ─► TX1
    RX2 ─┘

At power-on, with no console input:

1. the power-on band is applied (centre frequency, rate, bandwidth, RX1/RX2 gain 40 dB, TX 20000 mdB)
2. the TX -> RX leakage check runs and prints its result
3. the CRPA weights are computed and TX1 is switched to the **anti-jam output**
4. the weights keep updating **every 100 ms**

The samples never pass through software; the ARM only computes the weights.
Connect antennas to **both RX1 and RX2**: the anti-jam needs the two elements.

---

## Making the card

1. Format an SD card as **FAT32** (any size; the image is only a few MB).
2. Copy **`BOOT.BIN`** from this folder to the **root** of the card.
   Nothing else is needed — FSBL, bitstream and application are all inside it.
3. Set the board's **BOOT switch to SD**.
4. Insert the card, connect RF (see below), power on.

## Connections

| Port | Connect |
|------|---------|
| RX1 | your GNSS L1 signal source |
| TX1 | GNSS receiver, over **conducted, attenuated** coax |
| USB (PWR & JTAG CONSOLE) | power, and the console at **115200 8N1** |

## What you should see on the console

    ad9361_init : AD936x Rev 2 successfully initialized
    gnss_pt: present at 0x43c00000, version 1.2
    gnss_l1: RF band = LOW (5 MHz - 3 GHz). AD9361 RX port B_BALANCED, TX port TXB
    gnss_l1: RX LO = 1575419998 Hz (-2 Hz from target)
    SELFTEST: PASS - RX samples reach the TX datapath AND the AD9361
    *******************************************************
    LOOPBACK DEPLOYMENT BUILD - TX1 STARTS AUTOMATICALLY
    *******************************************************
    gnss_l1: DAC data source -> DMA; dac_enable i/q reads 1 / 1
    gnss_deploy: RX1 -> PL -> TX1 running continuously at 20000 mdB attenuation
    DEPLOY_RESULT: PASS - transmitting
    gnss_auto: power-on band 1 - GPS L1 / Galileo E1 wide
    gnss_band: band 1 OK
    ---- TX -> RX leakage check (TX1 attenuation 20000 mdB) ----
      ...  OK: no significant TX -> RX leakage   (or a WARNING, see below)
    gnss_deploy: CRPA anti-jam ON (power inversion, adapting every 100 ms)

`dac_enable i/q reads 1 / 1` is the line that matters: it means the AD9361 DAC
is actually accepting our samples. If it reads `0 / 0` nothing is being
transmitted, whatever else the console says.

The console still accepts commands, so you can intervene live:

    gnss_tx=0              stop transmitting, immediately
    gnss_tx=1              start again (then gnss_band=N for the band's level)
    gnss_band=N            another band (1..5); gnss_band? lists them
    tx1_attenuation=15000  TX level (larger number = quieter)
    gnss_loop?             TX -> RX leakage check at the current level
    stap_status?           anti-jam: input powers, power removed, updates
    stap_en=0 / stap_en=1  bypass / anti-jam output
    gnss_status?           AD9361 + passthrough state, read back from hardware
    help?                  everything else

---

## If it does not work

| Symptom | Cause |
|---|---|
| Console shows U-Boot / "Welcome to ANTSDR" / `ant login:` | BOOT switch is on **QSPI**, so it booted the factory Linux. Move it to SD. |
| Console silent, nothing at all | No valid `BOOT.BIN` found. Check FAT32, and that the file is in the card's root and named exactly `BOOT.BIN`. |
| `Tuning RX FAILED!` | The AD9361 interface did not train. See ISSUE-0010 — do not assume a rebuilt bitstream works. |
| `dac_enable i/q reads 0 / 0` | The DAC is discarding our samples; nothing is being transmitted. |
| Receiver sees nothing | 1. Receiver saturated: at 20000 mdB TX1 is ~ −13 dBm; add external attenuation or raise `tx1_attenuation`. 2. `gnss_loop?` shows a WARNING: TX1 is leaking back into RX; raise `tx1_attenuation` until the rise is < 1 dB. 3. No GPS at the antennas (indoors): check with the receiver on the antenna directly. 4. RSSI ~117 dB at high gain means nothing is arriving at RX1. |

---

## Provenance

- `BOOT.BIN` SHA256: `EA82BDAF9535B406603F56ACDDCF01E7509BDABB4716EA58F5CA0D643249A387`
- Bitstream: `antsdr_e310_gnss_2026-09-14_141546.bit`
- Application: `e310_gnss_deploy_2026-09-15_214608.elf` (built with `-DGNSS_DEPLOY_AUTO_TX`)
- FSBL: `zynq_fsbl_2026-09-15_214608.elf`

Full detail, including what has and has not been verified, is in
`../DEPLOYMENT_MANIFEST.json`.
