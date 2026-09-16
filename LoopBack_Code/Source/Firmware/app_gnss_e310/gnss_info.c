/******************************************************************************
 *  gnss_info.c   --  interactive information menu. GNSS-CRPA MOD-10.
 *  See gnss_info.h for what this is for and the [live] / [cfg] convention.
 *
 *  console_print() LIMITS -- read before editing any format string here:
 *    * supports ONLY %c %s %d %x %f. A %u prints NOTHING and does NOT consume
 *      its va_arg, so it silently shifts every later argument (ISSUE-0012).
 *    * %d takes a `long`. Cast.
 *    * %x with a ZERO value prints nothing unless a width is given, so always
 *      write %08x / %02x for hex, never bare %x.
 *****************************************************************************/

#include "gnss_info.h"
#include "console.h"
#include "gnss_l1.h"
#include "gnss_passthrough.h"
#include "gnss_txdma.h"

#ifdef XILINX_PLATFORM
#include "xil_io.h"
#endif

/* ---- Zynq-7000 PS registers, from UG585 -------------------------------- */
#define SLCR_PSS_IDCODE     0xF8000530U
#define SLCR_BOOT_MODE      0xF800025CU
#define SLCR_PLL_STATUS     0xF800010CU
#define SLCR_ARM_CLK_CTRL   0xF8000120U
#define SLCR_FPGA0_CLK_CTRL 0xF8000170U
#define SLCR_FPGA1_CLK_CTRL 0xF8000180U
#define SLCR_FPGA2_CLK_CTRL 0xF8000190U
#define DEVCFG_INT_STS      0xF800700CU
#define DEVCFG_STATUS       0xF8007014U
#define DEVCFG_MCTRL        0xF8007080U

static uint32_t rd(uint32_t addr)
{
#ifdef XILINX_PLATFORM
    return Xil_In32(addr);
#else
    (void)addr;
    return 0U;
#endif
}

static void rule(void)
{
    console_print("---------------------------------------------------------------\n");
}

static void head(const char *title)
{
    console_print("\n");
    console_print("===============================================================\n");
    console_print(" %s\n", (char *)title);
    console_print("===============================================================\n");
}

/* =========================================================================
 *  1  HARDWARE
 * ====================================================================== */
static void info_hardware(struct ad9361_rf_phy *phy)
{
    uint32_t idcode, boot, pll, mctrl, intsts;
    uint32_t dev;

    head("1  HARDWARE -- what this board actually is");

    console_print(" BOARD                                                   [cfg]\n");
    console_print("   ANTSDR E310, MicroPhase. Revision V1 / Rev 1.x only.\n");
    console_print("   The units in use are ES2 (Type-C), PCB label ANT_E310_ES2,\n");
    console_print("   silkscreen 24-E310B31-R21. E310 V2 / E316 / E200 are OUT OF\n");
    console_print("   SCOPE and nothing here applies to them.\n");
    rule();

    console_print(" ZYNQ-7000 PROCESSING SYSTEM                            [live]\n");
    idcode = rd(SLCR_PSS_IDCODE);
    /* Match the WHOLE IDCODE with the revision nibble [31:28] masked off,
     * rather than picking out a device bitfield.
     *
     * An earlier version of this decoded bits [16:12] against a table written
     * from memory, and got XC7Z020 wrong (0x09 instead of 0x07). It then
     * printed "*** WARNING: this is NOT an XC7Z020 ***" on a board that plainly
     * is one -- a false alarm in the very screen meant to orient a new user.
     * Whole-value comparison against published IDCODEs has less to get wrong,
     * and the raw register is printed either way so the reader can check. */
    dev = idcode & 0x0FFFFFFFU;
    console_print("   PSS_IDCODE  (0xF8000530) : 0x%08x\n", (long)idcode);
    console_print("     silicon revision [31:28]: 0x%02x\n",
                  (long)((idcode >> 28) & 0xFU));
    console_print("     device                 : ");
    switch (dev) {
    case 0x3723093U: console_print("XC7Z010\n");              break;
    case 0x373B093U: console_print("XC7Z015\n");              break;
    case 0x3727093U: console_print("XC7Z020   <- expected\n"); break;
    case 0x372C093U: console_print("XC7Z030\n");              break;
    case 0x3731093U: console_print("XC7Z045\n");              break;
    default:         console_print("not in this decode table\n"); break;
    }
    if (dev != 0x3727093U) {
        console_print("     *** NOTE: this does not match the XC7Z020 IDCODE\n");
        console_print("         (0x_3727093). The design was built for\n");
        console_print("         xc7z020clg400-2. Check the JTAG chain too before\n");
        console_print("         concluding the part is wrong. ***\n");
    }

    mctrl = rd(DEVCFG_MCTRL);
    console_print("   DEVCFG MCTRL(0xF8007080) : 0x%08x\n", (long)mctrl);
    console_print("     PS version [31:28]     : 0x%02x  (the nibble ps7_init\n",
                  (long)((mctrl >> 28) & 0xFU));
    console_print("                               uses to pick its _1_0/_2_0/_3_0\n");
    console_print("                               register sequence)\n");
    console_print("   CPU                      : 2x ARM Cortex-A9 MPCore  [cfg]\n");
    rule();

    console_print(" BOOT SOURCE                                            [live]\n");
    boot = rd(SLCR_BOOT_MODE);
    console_print("   BOOT_MODE   (0xF800025C) : 0x%08x\n", (long)boot);
    console_print("     boot device [3:0]      : 0x%02x  ", (long)(boot & 0xFU));
    switch (boot & 0xFU) {
    case 0x0U: console_print("= JTAG\n");           break;
    case 0x1U: console_print("= QSPI\n");           break;
    case 0x2U: console_print("= NOR\n");            break;
    case 0x4U: console_print("= NAND\n");           break;
    case 0x5U: console_print("= SD card\n");        break;
    default:   console_print("= not decoded here\n"); break;
    }
    console_print("   NOTE: this board has NO JTAG boot position on its switch --\n");
    console_print("   BOOT selects QSPI or SD only. On QSPI it boots the FACTORY\n");
    console_print("   LINUX, which loads its own bitstream and displaces this\n");
    console_print("   firmware. Keep it on SD.\n");
    rule();

    console_print(" MEMORY                                                  [cfg]\n");
    console_print("   DDR3  : MT41J256M16 RE-125, 32-bit bus, 1 GiB total\n");
    console_print("           (corroborated by the factory U-Boot: 'DRAM: 1 GiB')\n");
    console_print("   QSPI  : W25Q256, 32 MiB  [observed from the factory U-Boot\n");
    console_print("           banner, not read by this firmware]\n");
    console_print("   This application runs from DDR at 0x00100000.\n");
    console_print("   DDR buffers used by this project:\n");
    console_print("     0x10000000  8 MiB  I/Q capture A\n");
    console_print("     0x10800000  8 MiB  I/Q capture B\n");
    console_print("     0x11000000         DDR round-trip replay buffer\n");
    rule();

    console_print(" CLOCKS                                                 [live]\n");
    pll = rd(SLCR_PLL_STATUS);
    console_print("   PLL_STATUS  (0xF800010C) : 0x%08x\n", (long)pll);
    console_print("     ARM PLL lock           : %d\n", (long)(pll & 1U));
    console_print("     DDR PLL lock           : %d\n", (long)((pll >> 1) & 1U));
    console_print("     IO  PLL lock           : %d\n", (long)((pll >> 2) & 1U));
    if ((pll & 0x7U) != 0x7U) {
        console_print("     *** WARNING: not all three PLLs are locked. ***\n");
    }
    console_print("   ARM_CLK_CTRL(0xF8000120) : 0x%08x\n", (long)rd(SLCR_ARM_CLK_CTRL));
    console_print("   FPGA0_CLK_CTRL           : 0x%08x   design: 100 MHz [cfg]\n",
                  (long)rd(SLCR_FPGA0_CLK_CTRL));
    console_print("   FPGA1_CLK_CTRL           : 0x%08x   design: 200 MHz [cfg]\n",
                  (long)rd(SLCR_FPGA1_CLK_CTRL));
    console_print("   FPGA2_CLK_CTRL           : 0x%08x   design: 200 MHz [cfg]\n",
                  (long)rd(SLCR_FPGA2_CLK_CTRL));
    console_print("   l_clk (AD9361 interface) : 61.44 MHz at 30.72 MSPS   [cfg]\n");
    console_print("     This is the clock the sample datapath and the custom\n");
    console_print("     block run in. It comes FROM the AD9361, not from the PS.\n");
    rule();

    console_print(" PROGRAMMABLE LOGIC                                     [live]\n");
    console_print("   Part            : xc7z020clg400-2                     [cfg]\n");
    intsts = rd(DEVCFG_INT_STS);
    console_print("   DEVCFG INT_STS  : 0x%08x\n", (long)intsts);
    console_print("     PCFG_DONE [2] : %d   %s\n", (long)((intsts >> 2) & 1U),
                  (char *)(((intsts >> 2) & 1U) ? "PL is configured" : "PL NOT configured"));
    console_print("   DEVCFG STATUS   : 0x%08x\n", (long)rd(DEVCFG_STATUS));
    rule();

    console_print(" CUSTOM IP -- gnss_passthrough                          [live]\n");
    console_print("   base address    : 0x43C00000, 4 kB aperture\n");
    console_print("   ID register     : 0x%08x  (expect 0x47435031, ASCII 'GCP1')\n",
                  (long)gnss_pt_read(GNSS_PT_REG_ID));
    console_print("   VERSION         : 0x%08x  (expect 0x00010001 = v1.1)\n",
                  (long)gnss_pt_read(GNSS_PT_REG_VERSION));
    console_print("     v1.0 is the original identity passthrough and transmits\n");
    console_print("     24 dB LOW. v1.1 adds the RX->TX sample alignment stage.\n");
    rule();

    console_print(" RF FRONT END                                            [cfg]\n");
    console_print("   AD9361, 2R2T, LVDS digital interface, ADC_INIT_DELAY 29.\n");
    console_print("   Controlled over PS7 SPI0 routed through EMIO.\n");
    console_print("   Sample rate, bandwidth, LO and gain are NOT fixed by the\n");
    console_print("   FPGA design -- software sets them at runtime over SPI.\n");
    if (phy == NULL) {
        console_print("   *** ad9361_phy is NULL: the radio did not initialise. ***\n");
    } else {
        console_print("   Radio initialised OK (see the boot banner for its\n");
        console_print("   reported revision).\n");
    }
    console_print("\n Option 2 shows what the radio is CONFIGURED to right now.\n");
}

/* =========================================================================
 *  2  HARDWARE CONFIGURATION (live radio + custom block settings)
 * ====================================================================== */
static void info_config(struct ad9361_rf_phy *phy)
{
    uint32_t ctrl;

    head("2  HARDWARE CONFIGURATION -- what it is set to right now");

    if (phy == NULL) {
        console_print(" ad9361_phy is NULL -- the radio never initialised, so there\n");
        console_print(" is nothing to report. Check the boot log for\n");
        console_print(" 'ad9361_init : AD936x initialization error'.\n");
        return;
    }

    console_print(" AD9361 RUNTIME STATE (every value read back from the chip)\n");
    rule();
    gnss_l1_print_status(phy);

    console_print("\n HOW TO READ THOSE NUMBERS\n");
    rule();
    console_print("   RX1 gain 71 dB with RSSI ~117 dB means the AGC is pinned at\n");
    console_print("   maximum and NOTHING is arriving. A real signal pulls the gain\n");
    console_print("   down: with the bench source connected this reads about\n");
    console_print("   41-43 dB and RSSI 63-66 dB.\n");
    console_print("   TX1 attenuation is in mdB and BIGGER MEANS QUIETER.\n");
    console_print("     89750 = 89.75 dB = quietest the hardware can do (~-83 dBm)\n");
    console_print("     70000 = 70 dB    = the level measured to work (~-63 dBm)\n");
    console_print("   Those dBm figures are COMPUTED from the AD9361 nominal +7 dBm\n");
    console_print("   output minus the commanded attenuation. No power meter has\n");
    console_print("   ever been used on this project.\n");
    rule();

    console_print("\n RF BAND SELECT\n");
    rule();
    console_print("   GPS L1 at 1575.42 MHz is the LOW band (5 MHz - 3 GHz), which\n");
    console_print("   needs TWO things to agree:\n");
    console_print("     1. the AD9361 internal RF ports -> B_BALANCED / TXB\n");
    console_print("        CONFIRMED by read-back at every boot.\n");
    console_print("     2. four external SPDT switches on FPGA pins\n");
    console_print("        G14/C20/B19/B20/E17/A20/D18/D19.\n");
    console_print("   The POLARITY of item 2 is UNRESOLVED: the ES2\n");
    console_print("   schematic's printed truth table contradicts the vendor\n");
    console_print("   firmware, and with a strong signal AND correlation as the\n");
    console_print("   discriminator the two candidates were indistinguishable.\n");
    console_print("   The vendor polarity is used, because it ships -- NOT because\n");
    console_print("   it is confirmed.\n");
    rule();

    console_print("\n CUSTOM BLOCK CONTROL\n");
    rule();
    ctrl = gnss_pt_read(GNSS_PT_REG_CONTROL);
    console_print("   CONTROL (0x0C) : 0x%08x\n", (long)ctrl);
    console_print("     [0] pass_en  : %d   %s\n", (long)(ctrl & 1U),
                  (char *)((ctrl & 1U) ? "live RX->TX passthrough"
                                       : "vendor DMA/DDR drives TX"));
    console_print("     [1] mute     : %d   %s\n", (long)((ctrl >> 1) & 1U),
                  (char *)(((ctrl >> 1) & 1U) ? "output forced to zero" : "output live"));
    console_print("     [2] swap_iq  : %d\n", (long)((ctrl >> 2) & 1U));
    console_print("     [3] ch1_copy : %d\n", (long)((ctrl >> 3) & 1U));
    console_print("   DDR replay     : %s\n",
                  (char *)(gnss_txdma_is_running() ? "RUNNING" : "stopped"));
}

/* =========================================================================
 *  3  PS SIDE
 * ====================================================================== */
static void info_ps(void)
{
    head("3  PS SIDE -- what the ARM half is configured as");

    console_print(" All of this is INHERITED FROM MICROPHASE UNCHANGED. This\n");
    console_print(" project deliberately altered nothing in the PS7 block\n");
    console_print(" by design, so a board that boots the vendor firmware and\n");
    console_print(" a board that boots ours see identical PS hardware.\n");
    rule();

    console_print(" PERIPHERALS AND THEIR PINS                              [cfg]\n");
    console_print("   UART1   MIO 12..13    the console you are reading, 115200 8N1\n");
    console_print("     The vendor header says 921600. That constant belongs to the\n");
    console_print("     IIO-mode driver, NOT this console. 115200 was MEASURED from\n");
    console_print("     the UART registers on a live board, not copied from a doc.\n");
    console_print("   ENET0   MIO 16..27    MDIO on MIO 52..53, PHY reset MIO 46\n");
    console_print("   SD0                   50 MHz -- this is what BOOT.BIN loads from\n");
    console_print("   USB0                  reset on MIO 47\n");
    console_print("   QSPI                  enabled; holds the FACTORY LINUX image\n");
    console_print("   SPI0    over EMIO     control link to the AD9361\n");
    console_print("   EMIO GPIO             64 bits wide\n");
    rule();

    console_print(" CLOCKS TO THE PL                                        [cfg]\n");
    console_print("   FCLK0  100 MHz   AXI-Lite control for every PL peripheral\n");
    console_print("   FCLK1  200 MHz\n");
    console_print("   FCLK2  200 MHz   feeds the IDELAYCTRL the AD9361 RX\n");
    console_print("                    interface tuning depends on\n");
    rule();

    console_print(" MEMORY PORTS -- the PL reaches DDR through these         [cfg]\n");
    console_print("   S_AXI_HP1  <-  axi_ad9361_adc_dma   RX capture WRITES to DDR\n");
    console_print("   S_AXI_HP2  <-  axi_ad9361_dac_dma   TX replay READS from DDR\n");
    console_print("   Two separate ports, so a full-duplex round trip through DDR\n");
    console_print("   does not contend.\n");
    rule();

    console_print(" AXI-LITE ADDRESS MAP (M_AXI_GP0)                        [cfg]\n");
    console_print("   0x41200000  axi_gpreg\n");
    console_print("   0x41600000  axi_iic_main\n");
    console_print("   0x43C00000  gnss_passthrough   <- assigned by THIS project\n");
    console_print("   0x45000000  axi_sysid_0\n");
    console_print("   0x79020000  axi_ad9361\n");
    console_print("   0x7C400000  axi_ad9361_adc_dma\n");
    console_print("   0x7C420000  axi_ad9361_dac_dma\n");
    console_print("   0x43C0_0000 is the conventional free Zynq-7000 GP0 slot and\n");
    console_print("   collides with none of the vendor apertures.\n");
    rule();

    console_print(" INTERRUPTS                                              [cfg]\n");
    console_print("   ps-11  gps_pps        ps-12  dac_dma     ps-13  adc_dma\n");
    console_print("   gnss_passthrough raises NO interrupt. It is polled.\n");
    rule();

    console_print(" BOOT CHAIN ON THIS BOARD                                [cfg]\n");
    console_print("   BootROM -> FSBL -> bitstream into the PL -> this application,\n");
    console_print("   all four packed into ONE BOOT.BIN on a FAT32 SD card.\n");
    console_print("   The FSBL is what brings up the PLLs, MIO, clocks and the DDR\n");
    console_print("   controller. Over JTAG that job is done instead by replaying\n");
    console_print("   ps7_init.tcl, because loadhw() does NOT do it.\n");
}

/* =========================================================================
 *  4  PL SIDE
 * ====================================================================== */
static void info_pl(void)
{
    head("4  PL SIDE -- the IP cores and the order samples flow through them");

    console_print(" RECEIVE CHAIN                                           [cfg]\n");
    rule();
    console_print("   [RX1 SMA]\n");
    console_print("     |  SPDT switch -> balun\n");
    console_print("   AD9361                    external chip, not an IP\n");
    console_print("     |  LVDS, 6 lanes DDR\n");
    console_print("   axi_ad9361                ADI. Deserialises LVDS, applies the\n");
    console_print("     |                       IDELAY tuning, emits adc_data_*\n");
    console_print("     |                       + valid/enable at l_clk 61.44 MHz\n");
    console_print("     +------------------------------+\n");
    console_print("     |                              |\n");
    console_print("   util_ad9361_adc_fifo         gnss_passthrough\n");
    console_print("   (util_wfifo)                 (THE CUSTOM BLOCK)\n");
    console_print("     |  4ch x 16b, elastic           |\n");
    console_print("     |  buffer + clock crossing      |\n");
    console_print("     |  l_clk -> divclk              |\n");
    console_print("   util_ad9361_adc_pack             |\n");
    console_print("   (util_cpack2)                    |\n");
    console_print("     |  packs enabled channels       |\n");
    console_print("     |  into one AXI-Stream          |\n");
    console_print("   axi_ad9361_adc_dma               |\n");
    console_print("   (axi_dmac, CYCLIC=0)             |\n");
    console_print("     |  0x7C400000, IRQ ps-13        |\n");
    console_print("   S_AXI_HP1 -> DDR                 |\n");
    console_print("\n");
    console_print("   The split above is a FAN-OUT, not a rewire. The vendor\n");
    console_print("   capture path still sees exactly the samples the custom block\n");
    console_print("   sees, which makes a DDR capture a genuine cross-check of what\n");
    console_print("   the passthrough is doing rather than a separate mode.\n");
    rule();

    console_print("\n TRANSMIT CHAIN                                         [cfg]\n");
    rule();
    console_print("   DDR -> S_AXI_HP2\n");
    console_print("     |\n");
    console_print("   axi_ad9361_dac_dma        axi_dmac, memory->stream,\n");
    console_print("     |                       CYCLIC=1, 0x7C420000, IRQ ps-12\n");
    console_print("   util_ad9361_dac_upack     util_upack2, inverse of the packer\n");
    console_print("     |\n");
    console_print("   axi_ad9361_dac_fifo       util_rfifo, divclk -> l_clk\n");
    console_print("     |\n");
    console_print("     |   <-- THE FOUR DATA WIRES ARE CUT HERE\n");
    console_print("     v\n");
    console_print("   gnss_passthrough          0x43C00000  <== THE CRPA GOES HERE\n");
    console_print("     |   mux: pass_en=1 -> live RX\n");
    console_print("     |        pass_en=0 -> the DMA/DDR data above\n");
    console_print("     v\n");
    console_print("   axi_ad9361 -> AD9361 -> [TX1 SMA]\n");
    rule();

    console_print("\n THE ONE MODIFICATION THAT MATTERS\n");
    rule();
    console_print("   Upstream wired the DAC FIFO's data pins straight into\n");
    console_print("   axi_ad9361. This project deleted exactly those four\n");
    console_print("   connections and routed them through gnss_passthrough instead.\n");
    console_print("   ONLY THE FOUR DATA BUSES WERE DETACHED. The enable and valid\n");
    console_print("   strobes still go straight from the rfifo to axi_ad9361, so\n");
    console_print("   the vendor DAC timing is untouched and pass_en = 0 reproduces\n");
    console_print("   stock behaviour exactly.\n");
    rule();

    console_print("\n CLOCK DOMAINS                                          [cfg]\n");
    rule();
    console_print("   l_clk        61.44 MHz  AD9361 interface, the custom block\n");
    console_print("                           core, wfifo din, rfifo dout\n");
    console_print("   divclk       divided     cpack2, upack2, both DMA stream sides\n");
    console_print("   sys_cpu_clk  100 MHz     all AXI-Lite control\n");
    console_print("   The two FIFOs exist BECAUSE of that crossing. They are not\n");
    console_print("   there for throughput.\n");
    rule();

    console_print("\n TRAPS IN THIS DATAPATH -- all three cost real days\n");
    rule();
    console_print("   1. axi_dmac has HIDDEN dependencies: util_axis_fifo and\n");
    console_print("      util_cdc. A missing subcore does NOT error -- Vivado\n");
    console_print("      silently LOCKS the cell and later reports a bogus\n");
    console_print("      s_axi.ADDR_WIDTH failure hundreds of lines away.\n");
    console_print("   2. dac_enable is NOT an enable. It is axi_ad9361's read-back\n");
    console_print("      of (dac_data_sel == 4'h2). Unless firmware writes\n");
    console_print("      CHAN_CNTRL_7 = 2 the DAC emits DDS and DISCARDS everything\n");
    console_print("      the custom block produces -- while every counter, the FIFO\n");
    console_print("      level and the sample rate all look perfect.\n");
    console_print("   3. RX and TX sample formats DIFFER. RX is 12-bit RIGHT\n");
    console_print("      aligned; the DAC consumes LEFT aligned dma_data[15:4]. An\n");
    console_print("      identity copy loses 24 dB. A CRPA producing values outside\n");
    console_print("      12-bit signed range MUST saturate before that stage.\n");
}

/* =========================================================================
 *  5  ABOUT THE LOOPBACK TEST
 * ====================================================================== */
static void info_about(void)
{
    head("5  ABOUT THIS LOOPBACK TEST");

    console_print(" WHAT IT DOES\n");
    rule();
    console_print("   Whatever GNSS signal is present on RX1 is received by the\n");
    console_print("   AD9361, carried through programmable logic, and retransmitted\n");
    console_print("   on TX1 -- continuously, and entirely in the PL. NO SAMPLE\n");
    console_print("   PASSES THROUGH SOFTWARE. The ARM core only configures and\n");
    console_print("   monitors.\n");
    console_print("   A GNSS receiver connected to TX1 sees the satellites that are\n");
    console_print("   present at RX1.\n");
    rule();

    console_print("\n WHY IT EXISTS\n");
    rule();
    console_print("   This is the skeleton of a CRPA anti-jam front end. Today\n");
    console_print("   gnss_passthrough is transparent -- Iout = Iin. It is the\n");
    console_print("   insertion point where a Controlled Reception Pattern Antenna\n");
    console_print("   algorithm will replace the identity with adaptive null\n");
    console_print("   steering across two antenna elements.\n");
    console_print("   Proving the loop end to end FIRST means that when the\n");
    console_print("   algorithm goes in, any change it causes is measurable\n");
    console_print("   immediately at a real receiver.\n");
    rule();

    console_print("\n IT IS AN ANTI-JAM FRONT END, NOT A GNSS RECEIVER\n");
    rule();
    console_print("   There is no correlator, no acquisition and no tracking in the\n");
    console_print("   PL. The external receiver does all of that. This board never\n");
    console_print("   needs to 'see' GPS -- the signal stays about 20 dB UNDER the\n");
    console_print("   noise floor throughout, which is normal and correct.\n");
    console_print("   Success is measured at the DOWNSTREAM RECEIVER (C/N0, fix\n");
    console_print("   quality with a jammer present), never on this board.\n");
    rule();

    console_print("\n WHAT HAS BEEN PROVEN ON HARDWARE\n");
    rule();
    console_print("   Phase 1  8 GPS satellites acquired offline from captured I/Q.\n");
    console_print("            Power alone could NOT establish this -- RSSI, AGC\n");
    console_print("            gain and spectrum all say 'strong signal' for any\n");
    console_print("            strong signal. Only despreading identifies GPS.\n");
    console_print("   Phase 2  A real GNSS receiver reached a SUSTAINED 3D FIX on\n");
    console_print("            the retransmitted signal, with transmitter-silent\n");
    console_print("            controls before and after.\n");
    console_print("   DDR      RX -> DMA -> DDR -> DMA -> TX also reaches the\n");
    console_print("            receiver (acquisition; see the limit below).\n");
    console_print("   SD card  All of the above with NO PC and NO JTAG: 17\n");
    console_print("            satellites used, 3D fix, C/N0 up to 45 dB-Hz.\n");
    rule();

    console_print("\n WHAT HAS *NOT* BEEN PROVEN -- do not overclaim\n");
    rule();
    console_print("   * The DDR round trip achieved ACQUISITION but NOT a position\n");
    console_print("     fix, and a cyclic replay never can: it repeats the 50 bps\n");
    console_print("     navigation message, so no consistent time-of-week can be\n");
    console_print("     decoded. That is a property of the test, not a fault.\n");
    console_print("   * NO absolute RF power was ever measured. Every dBm figure is\n");
    console_print("     computed from the AD9361 nominal output minus the commanded\n");
    console_print("     attenuation. Cable loss is uncharacterised.\n");
    console_print("   * The fix is MULTI-CONSTELLATION, not GPS-only. Galileo E1\n");
    console_print("     shares 1575.42 MHz and the 18 MHz retransmit bandwidth\n");
    console_print("     carries GPS, Galileo and SBAS together.\n");
    console_print("   * Position ACCURACY was never checked against a reference.\n");
    console_print("   * No CRPA algorithm exists yet. The block is still identity.\n");
    rule();

    console_print("\n SAFETY -- READ THIS\n");
    rule();
    console_print("   This board TRANSMITS ON THE GPS L1 CENTRE FREQUENCY, and the\n");
    console_print("   SD-card build starts doing so AUTOMATICALLY AT POWER-UP with\n");
    console_print("   no operator present.\n");
    console_print("   CONDUCTED, ATTENUATED COAX ONLY. NEVER AN ANTENNA.\n");
    console_print("   Radiating GNSS frequencies is illegal in most jurisdictions\n");
    console_print("   and can disrupt navigation and timing for anything nearby.\n");
    console_print("   Nothing in software can detect what is attached to TX1.\n");
    console_print("   gnss_tx=0 silences the transmitter in one command.\n");
}

/* =========================================================================
 *  6  REGISTER MAP
 * ====================================================================== */
static void info_registers(void)
{
    head("6  gnss_passthrough REGISTER MAP  (base 0x43C00000, 4 kB)");

    console_print("   0x00  ID              RO  0x47435031, ASCII 'GCP1'\n");
    console_print("   0x04  VERSION         RO  0x00010001 = v1.1\n");
    console_print("   0x08  SCRATCH         RW  read/write test\n");
    console_print("   0x0C  CONTROL         RW  [0] pass_en  [1] mute\n");
    console_print("                             [2] swap_iq  [3] ch1_copy\n");
    console_print("                             [8] cnt_clear\n");
    console_print("   0x10  STATUS          RO  [0] adc_enable_i0  [1] adc_enable_q0\n");
    console_print("                             [2] dac_enable_i0  [3] dac_enable_q0\n");
    console_print("                             [4] fifo0_empty    [5] fifo0_full\n");
    console_print("                             [6] fifo1_empty    [7] fifo1_full\n");
    console_print("                             [8] overflow_sticky\n");
    console_print("                             [9] underflow_sticky\n");
    console_print("                            [16] pass_en in the sample domain\n");
    console_print("   0x14  RX_COUNT_CH0    RO\n");
    console_print("   0x18  TX_COUNT_CH0    RO\n");
    console_print("   0x1C  OVERFLOW_COUNT  RO\n");
    console_print("   0x20  UNDERFLOW_COUNT RO\n");
    console_print("   0x24  RX_SNAPSHOT_CH0 RO  {Q[31:16], I[15:0]}, RIGHT aligned\n");
    console_print("   0x28  TX_SNAPSHOT_CH0 RO  {Q[31:16], I[15:0]}, LEFT aligned\n");
    console_print("   0x2C  FIFO_LEVEL      RO  [5:0] ch0, [13:8] ch1\n");
    console_print("   0x30  RX_COUNT_CH1    RO\n");
    console_print("   0x34  TX_COUNT_CH1    RO\n");
    console_print("   0x38  RX_SNAPSHOT_CH1 RO\n");
    console_print("   0x40  CRPA_COEF0..15  RW  0x40-0x7C, reserved for the CRPA.\n");
    console_print("                             Stored but unused today.\n");
    rule();
    console_print(" STATUS BITS [2] AND [3] ARE THE IMPORTANT ONES.\n");
    console_print("   They are axi_ad9361's read-back of (dac_data_sel == 4'h2),\n");
    console_print("   NOT an enable this block drives. 0 means the DAC core is\n");
    console_print("   sourcing DDS and DISCARDING everything gnss_passthrough\n");
    console_print("   outputs -- while TX_COUNT still advances perfectly.\n");
    rule();
    console_print(" STATUS AND COUNTERS LAG THE SAMPLE DOMAIN.\n");
    console_print("   They are latched every 256 l_clk cycles (~4.2 us) and then\n");
    console_print("   crossed into the AXI domain. Firmware must wait before\n");
    console_print("   reading back a change. That is correct behaviour, not a bug:\n");
    console_print("   it is what makes the whole capture set coherent.\n");
}

/* =========================================================================
 *  7  LIVE HEALTH
 * ====================================================================== */
static void info_health(struct ad9361_rf_phy *phy)
{
    head("7  LIVE HEALTH -- is it working, right now?");

    if (phy == NULL) {
        console_print(" ad9361_phy is NULL -- the radio never initialised.\n");
        return;
    }

    console_print(" Checking the datapath over a short window...\n\n");
    (void)gnss_pt_check_dataflow(200U);

    console_print("\n");
    gnss_pt_print_state();

    console_print("\n WHAT GOOD LOOKS LIKE\n");
    rule();
    console_print("   adc enable i/q : 1 / 1   samples arriving from the AD9361\n");
    console_print("   dac enable i/q : 1 / 1   the DAC is ACCEPTING our samples.\n");
    console_print("                            0 / 0 means nothing is leaving TX1\n");
    console_print("                            from the passthrough, whatever else\n");
    console_print("                            the counters say.\n");
    console_print("   rx / tx counts : both advancing, and close to each other.\n");
    console_print("                    A small difference is EXPECTED -- the two\n");
    console_print("                    are snapshotted up to 256 l_clk apart.\n");
    console_print("   fifo level     : steady near half depth (about 16)\n");
    console_print("   overflow       : 0 / 0    a non-zero sticky flag means the\n");
    console_print("   underflow      : 0 / 0    elastic buffer lost samples\n");
    console_print("   last tx sample : low nibble ZERO -- proves the RX->TX\n");
    console_print("                    alignment stage is present (v1.1). If the\n");
    console_print("                    low nibble is non-zero you are running v1.0\n");
    console_print("                    and transmitting 24 dB low.\n");
}

/* =========================================================================
 *  8  KNOWN ISSUES
 * ====================================================================== */
static void info_issues(void)
{
    head("8  CAUTIONS -- what will bite you");

    console_print(" These are OPERATING cautions: things that will waste your time\n");
    console_print(" or damage something if you do not know them. The project's\n");
    console_print(" defect tracker is deliberately NOT reproduced here -- see\n");
    console_print(" Issues/ISSUE_REGISTER.md in the source package for that.\n");

    console_print("\n 1. IT TRANSMITS ON GPS L1, BY ITSELF.\n");
    rule();
    console_print("   The SD-card build starts transmitting at power-up with no\n");
    console_print("   operator and no confirmation step.\n");
    console_print("   CONDUCTED, ATTENUATED COAX ONLY. NEVER AN ANTENNA.\n");
    console_print("   Radiating GNSS frequencies is illegal in most jurisdictions\n");
    console_print("   and can disrupt navigation and timing nearby.\n");
    console_print("   gnss_tx=0 silences it in one command.\n");
    rule();

    console_print("\n 2. ATTENUATION IS BACKWARDS FROM WHAT YOU EXPECT.\n");
    rule();
    console_print("   tx1_attenuation= takes mdB and BIGGER MEANS QUIETER.\n");
    console_print("     89750 = quietest the hardware can do (~-83 dBm)\n");
    console_print("     70000 = the level measured to work     (~-63 dBm)\n");
    console_print("   Typing a SMALL number makes it LOUDER. Step down slowly and\n");
    console_print("   stop as soon as the receiver tracks.\n");
    rule();

    console_print("\n 3. KEEP THE BOOT SWITCH ON SD.\n");
    rule();
    console_print("   On QSPI the board boots its FACTORY LINUX instead. That image\n");
    console_print("   loads its own bitstream, reconfigures the AD9361 and displaces\n");
    console_print("   this firmware -- while every JTAG marker still reads PASS.\n");
    console_print("   Symptom: the console shows U-Boot and 'Welcome to ANTSDR'\n");
    console_print("   instead of this menu.\n");
    rule();

    console_print("\n 4. TWO DEVICES, TWO SERIAL PORTS -- AND PORTS ARE EXCLUSIVE.\n");
    rule();
    console_print("   This console is the SDR. Your GNSS receiver is a SEPARATE\n");
    console_print("   device on its own port. A GNSS viewer pointed at THIS port\n");
    console_print("   will show nothing and will make this console print\n");
    console_print("   'Invalid command!' as it rejects the viewer's protocol.\n");
    console_print("   Only ONE program can hold a serial port at a time. If a\n");
    console_print("   logging tool has the receiver's port open, your viewer gets\n");
    console_print("   nothing and the board looks broken when it is not.\n");
    rule();

    console_print("\n 5. RX2 HAS NOTHING CONNECTED.\n");
    rule();
    console_print("   Both RX channels are wired in the PL, so a TWO-ELEMENT CRPA\n");
    console_print("   needs no block-design change -- but RX2 currently sees only\n");
    console_print("   its own noise. A second antenna or a splitter is required\n");
    console_print("   before any array work.\n");
    console_print("   One AD9361 gives only TWO coherent channels. More elements\n");
    console_print("   than that needs more hardware.\n");
    rule();

    console_print("\n 6. RX AND TX PHASE IS AN UNKNOWN CONSTANT.\n");
    rule();
    console_print("   The AD9361 LO divider phase can flip 180 degrees per power\n");
    console_print("   cycle, and RX calibration does not imply TX alignment. The\n");
    console_print("   two RX channels share one LO so they are coherent WITHIN a\n");
    console_print("   session, but their relative phase must NOT be assumed zero.\n");
    console_print("   An adaptive weight absorbs it; a calibrated beam pattern or\n");
    console_print("   weights reused across power cycles do NOT.\n");
    rule();

    console_print("\n 7. A GREEN BUILD MARKER IS NOT A WORKING DESIGN.\n");
    rule();
    console_print("   A clean rebuild of this project once passed EVERY marker --\n");
    console_print("   build PASS, timing MET, positive slack -- and then failed\n");
    console_print("   AD9361 RX interface tuning at every delay position on real\n");
    console_print("   hardware. The shipped bitstream is a specific VERIFIED one.\n");
    console_print("   If you rebuild, TEST THE RESULT ON A BOARD before trusting\n");
    console_print("   it. Details are in the source package.\n");
    rule();

    console_print("\n 8. dac_enable IS NOT AN ENABLE.\n");
    rule();
    console_print("   It is axi_ad9361's read-back of (dac_data_sel == 4'h2). If it\n");
    console_print("   reads 0 / 0 the DAC is sourcing its internal DDS and\n");
    console_print("   DISCARDING everything the custom block produces -- while the\n");
    console_print("   counters, the FIFO level and the sample rate all look\n");
    console_print("   perfect. Check it (option 7) before believing TX is working.\n");
}

/* =========================================================================
 *  9  QUICK START
 * ====================================================================== */
static void info_quickstart(void)
{
    head("9  QUICK START -- bringing this up on a new bench");

    console_print(" 1. BOOT SWITCH TO SD. On QSPI you get the factory Linux.\n");
    console_print(" 2. SD card: FAT32, BOOT.BIN in the ROOT. Nothing else needed --\n");
    console_print("    the FSBL, the bitstream and this application are all inside\n");
    console_print("    that one file.\n");
    console_print(" 3. RF, and this is the part that matters:\n");
    console_print("      RX1 <- your GNSS L1 signal source\n");
    console_print("      TX1 -> your GNSS receiver, over CONDUCTED ATTENUATED COAX\n");
    console_print("    NEVER AN ANTENNA ON TX1.\n");
    console_print(" 4. Console: 115200 8N1, no flow control.\n");
    console_print(" 5. Power up. The SD build starts transmitting BY ITSELF.\n");
    rule();

    console_print("\n CONFIRMING IT WORKS, IN ORDER\n");
    rule();
    console_print("   a) Console shows 'SELFTEST: PASS'.\n");
    console_print("   b) Option 7 here: dac enable i/q reads 1 / 1.\n");
    console_print("   c) Option 2 here: RX1 gain is NOT pinned at 71 dB. If it is,\n");
    console_print("      nothing is arriving on RX1 and the rest is meaningless.\n");
    console_print("   d) Your GNSS receiver reports satellites and a fix.\n");
    console_print("   Reference from a working bench: 17 satellites used, 3D fix,\n");
    console_print("   C/N0 up to 45 dB-Hz, at 70000 mdB TX attenuation.\n");
    rule();

    console_print("\n IF THE RECEIVER SEES NOTHING\n");
    rule();
    console_print("   * Is your viewer on the RIGHT SERIAL PORT? This console is\n");
    console_print("     the SDR. The GNSS receiver is a SEPARATE device on its own\n");
    console_print("     port. A viewer pointed here will show nothing and will make\n");
    console_print("     this console print 'Invalid command!'.\n");
    console_print("   * Is another program holding that port open? Serial ports are\n");
    console_print("     EXCLUSIVE. One tool at a time.\n");
    console_print("   * Too quiet? tx1_attenuation=70000. Bigger is quieter.\n");
    console_print("   * Too loud/saturated? Increase toward 89750.\n");
    console_print("   * Check RX1 first (option 2). No input, no output.\n");
    rule();

    console_print("\n COMMANDS\n");
    rule();
    console_print("   gnss_tx=1 / gnss_tx=0        live retransmit / ABORT\n");
    console_print("   gnss_ddr_tx=1 / =0          DDR round trip / stop\n");
    console_print("   tx1_attenuation=N           mdB. BIGGER = QUIETER.\n");
    console_print("   gnss_tx? / gnss_status?     state, read back from hardware\n");
    console_print("   ?                           this menu\n");
    console_print("   help?                       every command (long)\n");
}

/* =========================================================================
 *  Menu
 * ====================================================================== */
void gnss_info_menu(void)
{
    uint32_t status;
    int transmitting = 0;

    status = gnss_pt_read(GNSS_PT_REG_STATUS);
    transmitting = ((status & GNSS_PT_ST_DAC_EN_I0) &&
                    (status & GNSS_PT_ST_DAC_EN_Q0) &&
                    (status & GNSS_PT_ST_PASS_EN_SYNCED)) ? 1 : 0;

    console_print("\n");
    console_print("===============================================================\n");
    console_print("  ANTSDR E310 V1  --  GNSS L1 CRPA loopback\n");
    console_print("  TX1 is %s\n",
                  (char *)(transmitting ? "TRANSMITTING  (GPS L1, 1575.42 MHz)"
                                        : "silent"));
    console_print("===============================================================\n");
    console_print("\n");
    console_print("  INFORMATION -- type the number, then Enter\n");
    console_print("    1   Hardware            silicon, boot source, DDR, flash,\n");
    console_print("                            clocks, PL status, custom IP\n");
    console_print("    2   Configuration       what the radio is set to RIGHT NOW\n");
    console_print("    3   PS side             peripherals, pins, clocks, address\n");
    console_print("                            map, interrupts, boot chain\n");
    console_print("    4   PL side             every IP core and the order samples\n");
    console_print("                            flow through them\n");
    console_print("    5   About this test     what it does, what is proven, what\n");
    console_print("                            is NOT, and the safety rules\n");
    console_print("    6   Register map        gnss_passthrough @ 0x43C00000\n");
    console_print("    7   Live health         is it working right now?\n");
    console_print("    8   Cautions            what will bite you\n");
    console_print("    9   Quick start         bringing this up on a new bench\n");
    console_print("\n");
    console_print("  CONTROL\n");
    console_print("    gnss_tx=1 / gnss_tx=0       live retransmit / ABORT\n");
    console_print("    gnss_ddr_tx=1 / =0         DDR round trip / stop\n");
    console_print("    tx1_attenuation=N          mdB. BIGGER = QUIETER.\n");
    console_print("                               89750 quietest, 70000 known good\n");
    console_print("    gnss_tx? gnss_ddr_tx? gnss_status?   read back from hardware\n");
    console_print("\n");
    console_print("    ?        this menu            help?    every command (long)\n");
    console_print("===============================================================\n");
}

void gnss_info_select(int opt, struct ad9361_rf_phy *phy)
{
    switch (opt) {
    case 1: info_hardware(phy);   break;
    case 2: info_config(phy);     break;
    case 3: info_ps();            break;
    case 4: info_pl();            break;
    case 5: info_about();         break;
    case 6: info_registers();     break;
    case 7: info_health(phy);     break;
    case 8: info_issues();        break;
    case 9: info_quickstart();    break;
    default:
        console_print("\n  No option %d. Valid options are 1 to %d.\n",
                      (long)opt, (long)GNSS_INFO_MAX_OPTION);
        console_print("  Type ? for the menu.\n");
        return;
    }
    console_print("\n  [ ? for the menu ]\n");
}
