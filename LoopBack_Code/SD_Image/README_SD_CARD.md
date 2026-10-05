# SD card — ANTSDR E310 V1 GNSS L1 loopback (V08)

Built 2026-09-15_214608 by `Automation/PowerShell/New-Deployment.ps1`.

---

## ⚠️ READ THIS FIRST — THIS BOARD TRANSMITS BY ITSELF

This image starts transmitting on the **GPS L1 centre frequency (1575.42 MHz)**
the moment the board is powered, with **no operator and no confirmation step**.

- **CONDUCTED, ATTENUATED COAX ONLY. NEVER AN ANTENNA.**
- Transmitting GNSS frequencies over the air is illegal in most jurisdictions
  and can disrupt navigation and timing for anything nearby.
- The firmware **cannot** tell what is attached to TX1. The interlock that
  normally forces a deliberate decision has been moved to build time on purpose,
  because an unattended board has nobody to ask.

**Output level:** TX attenuation is driven to the hardware maximum first, then
set to **70 dB (~ −63 dBm)**. That is ~73 dB below the ~+10 dBm where GNSS front
ends are damaged, so it will not harm a receiver — but it is still ~40 dB hotter
than live GNSS, which is why a short conducted path works.

To change it: edit `GNSS_DEPLOY_TX_ATTEN_MDB` in
`Source/Firmware/app_gnss_e310/gnss_l1.h` and rebuild. Larger = quieter.

---

## What it does

    RX1 ──► AD9361 ──► axi_ad9361 ──► gnss_passthrough (PL) ──► axi_ad9361 ──► TX1

Whatever GNSS signal is present on RX1 is carried through programmable logic and
retransmitted on TX1, continuously. No samples pass through software. A GNSS
receiver connected to TX1 sees the satellites present at RX1.

`gnss_passthrough` is currently a transparent pass-through — it is the
insertion point where the CRPA anti-jam algorithm will go.

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
    gnss_pt: present at 0x43c00000, version 1.1
    gnss_l1: RF band = LOW (5 MHz - 3 GHz). AD9361 RX port B_BALANCED, TX port TXB
    gnss_l1: RX LO = 1575419998 Hz (-2 Hz from target)
    SELFTEST: PASS - RX samples reach the TX datapath AND the AD9361
    *******************************************************
    GNSS-CRPA DEPLOYMENT BUILD - TX1 STARTS AUTOMATICALLY
    *******************************************************
    gnss_l1: DAC data source -> DMA; dac_enable i/q reads 1 / 1
    gnss_deploy: RX1 -> PL -> TX1 running continuously at 70000 mdB attenuation
    DEPLOY_RESULT: PASS - transmitting

`dac_enable i/q reads 1 / 1` is the line that matters: it means the AD9361 DAC
is actually accepting our samples. If it reads `0 / 0` nothing is being
transmitted, whatever else the console says.

The console still accepts commands, so you can intervene live:

    gnss_tx=0              stop transmitting, immediately
    gnss_tx=1              start again
    tx1_attenuation=80000  quieter (larger number = quieter)
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
| Receiver sees nothing | Check RX1 actually has signal (console prints RX1 gain and RSSI — a gain pinned at 71 dB with RSSI ~117 dB means nothing is arriving). |

---

## Provenance

- `BOOT.BIN` SHA256: `EA82BDAF9535B406603F56ACDDCF01E7509BDABB4716EA58F5CA0D643249A387`
- Bitstream: `antsdr_e310_gnss_2026-09-14_141546.bit`
- Application: `e310_gnss_deploy_2026-09-15_214608.elf` (built with `-DGNSS_DEPLOY_AUTO_TX`)
- FSBL: `zynq_fsbl_2026-09-15_214608.elf`

Full detail, including what has and has not been verified, is in
`../DEPLOYMENT_MANIFEST.json`.
