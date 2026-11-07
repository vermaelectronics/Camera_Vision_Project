# ANTSDR E310 V1 - CORE LOOPBACK pipeline (no gnss_passthrough, no custom IP)

    RX1/RX2 -> AD9361 -> axi_ad9361 (DAC data select 8 = ADC data) -> AD9361 -> TX1/TX2

This pipeline has **no custom FPGA block**. The block design is the official MicroPhase E310 V1 design
(`Vendor/MicroPhase_E310_V1/hdl/projects/antsdre310/system_bd.tcl`): the same cells and connections, only
reordered and commented. The RX->TX loopback is a feature of the stock ADI `axi_ad9361` core:

    axi_ad9361_tx_channel.v   4'h8: dac_data_out_int <= adc_data;   // raw 12-bit RX sample of the same channel

The firmware writes 8 to the DAC channels' data select (CHAN_CNTRL_7) when TX is switched on: TX1 = RX1, TX2 = RX2,
a few clock cycles of delay, no DMA, no FIFO.

## What changed from LoopBack_Pipeline_noCRPA
| Part | Change |
|---|---|
| Block design | gnss_passthrough instance, its connections and its 0x43C0_0000 aperture removed; the vendor DAC FIFO -> axi_ad9361 data connections restored |
| Build | stage 3 (ip) is a no-op; no `LB_IP_REPO`; no CDC constraint file; `Source/HDL/gnss_passthrough.v` and `Source/IP` removed |
| Firmware | `gnss_passthrough.c` replaced by an emulation layer with the same API that never touches 0x43C0_0000. "Passthrough on" = DAC data select 8, "mute" = 3, DDR round trip (`gnss_ddr_tx=1`) = 2 (real DMA, vendor path). `gnss_l1_set_dac_source()` verifies by reading CHAN_CNTRL_7 back |
| Console | the same commands (`gnss_tx=1`, `gnss_tx?`, `gnss_status?`, ...). Sample counters / snapshots no longer exist (they were inside gnss_passthrough); the data-flow check now reads the AD9361 interface clock and the ADC core status |

Not in this pipeline: anti-jam (STAP/SFAP), band presets, level lock, logger commands. The digital gain is unity: this path adds
no amplification (a passive antenna still needs an LNA or more RX gain).

**Checked here:** the block design differs from the vendor original only in comments and line order (diff without comments); the
firmware compiles (gcc syntax check with stubs). **Not built in Vivado, not run on the board.**

## Build (up to BOOT.BIN)
The ADI library is not in the zip: copy `Vendor/` from one of your earlier project folders.
```bash
cd ~/LoopBack_ADI_CoreLoopback
cp -r ~/LoopBack_Code_SMI_PI_STAP5x5_v4.9.12/Vendor .      # or any earlier folder with Vendor/ADI_hdl_2023_r2
source /tools/Xilinx/Vivado/2023.2/settings64.sh
source /tools/Xilinx/Vitis/2023.2/settings64.sh
chmod +x Automation/Linux/*.sh
./Automation/Linux/build_all.sh --jobs 16 --to boot 2>&1 | tee build_coreloop.log
ls -l Build/Output/BOOT.BIN
```

## Use (PuTTY, 115200)
```
gnss_tx=1        TX1 = RX1 through the ADI core loopback (CONDUCTED, ATTENUATED COAX ONLY)
gnss_tx?         state; "DAC data select 8 (ADC LOOPBACK)" in gnss_status? means the loopback is active
tx1_attenuation=30000    level, smaller = louder
gnss_tx=0        silence
```
SD-card build that starts transmitting by itself: add `--defines GNSS_DEPLOY_AUTO_TX` to the build command.
