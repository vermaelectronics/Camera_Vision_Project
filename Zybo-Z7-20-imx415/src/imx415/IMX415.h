/*
 * IMX415.h
 *
 * Bare-metal (Vitis/standalone, no OS) I2C register-level driver for the
 * Sony IMX415 CMOS image sensor, adapted from Digilent's OV5640.h driver
 * that originally shipped with the Zybo Z7-20 + Pcam 5C reference design.
 *
 * ---------------------------------------------------------------------------
 * WHERE THESE REGISTER VALUES COME FROM
 * ---------------------------------------------------------------------------
 * Register addresses, the sensor's "magic"/undocumented tuning table, the
 * per-lane-rate MIPI D-PHY timing tables, and the per-(lane rate, INCK)
 * clock-configuration tables below were originally ported from the mainline
 * Linux kernel's real, maintained IMX415 driver:
 *
 *     drivers/media/i2c/imx415.c  (Linux kernel, GPL-2.0-only,
 *     Copyright (C) 2023 WolfVision GmbH; upstream at
 *     https://github.com/torvalds/linux/blob/master/drivers/media/i2c/imx415.c)
 *
 * ...and have since been cross-checked byte-for-byte against Sony's own
 * official IMX415-AAQR-C datasheet ("INCK Setting" section, Data rate:
 * 720Mbps/lane and 891Mbps/lane tables) - every register/value pair used
 * below for those two lane rates matches the datasheet exactly. The one
 * discrepancy found: the datasheet's master "Register Map" section (and the
 * Linux driver) place SYS_MODE at 0x3033, while the datasheet's own "INCK
 * Setting" summary tables say 0x3034 for the same register - an apparent
 * internal inconsistency in Sony's document. This driver uses 0x3033 (2
 * independent sources agree). If bring-up gets past the chip-ID check but
 * lane timing looks wrong, try 0x3034 for REG_SYS_MODE as a troubleshooting
 * step.
 *
 * Board-specific facts below (INCK, I2C address, lane wiring, reset
 * signal) come from the schematic for the specific carrier board this was
 * built against - "IMX415 CAM R1", a small 22-pin-FPC breakout board
 * connected to a Zybo Z7-20's Pcam MIPI connector via a Raspberry-Pi-style
 * "Standard-Mini" adapter cable:
 *   - I2C address 0x1A (7-bit): confirmed three independent ways - an
 *     explicit note on the schematic, the sensor's SLAMODE0/SLAMODE1 slave
 *     -address truth table in the datasheet, and the board's own
 *     resistor strapping (both address-select pins pulled low, no pull-ups
 *     populated).
 *   - MIPI lanes: the sensor package is wired for all 4 CSI-2 lanes on this
 *     board, but per the datasheet "In 2 Lane mode, data is output from
 *     Lane1 and Lane2" - and the Raspberry-Pi-style 15-pin "Mini" connector
 *     standard the Zybo's Pcam header uses historically only carries 2 of
 *     those. Hence NUM_DATA_LANES defaults to 2 below.
 *   - INCK: generated on-board by an always-on active oscillator (Kyocera
 *     X1G0048010002, enabled directly from the 1.8V rail, no host control)
 *     feeding the sensor's INCK pin directly - NOT derived from the Zybo.
 *     Its exact frequency could not be confirmed from Kyocera's public part
 *     data in this environment; INCK_HZ defaults to 24MHz (one of the
 *     datasheet's 5 supported frequencies, and the most common choice for
 *     this class of board) as a best-effort default - check the frequency
 *     printed on the oscillator package itself, or Kyocera's site directly,
 *     to confirm.
 *   - Reset wiring: see the "IMPORTANT - sensor reset wiring" note on
 *     reset() below - this is the single most likely hardware gap on this
 *     specific board/cable combination.
 *
 * What's still genuinely unresolved:
 *   - the exact INCK frequency (see above),
 *   - whether the Zybo's single Pcam GPIO pin actually reaches this board's
 *     CAM_RST net through the adapter cable (see reset() below),
 *   - whether this hardware platform's MIPI_D_PHY_RX/MIPI_CSI_2_RX IP cores
 *     were generated for the lane rate you select here (see README.md §3).
 *
 * IMPORTANT: unlike the OV5640, the IMX415 has ONE native readout mode
 * implemented here: a full-pixel-array raw Bayer scan at 3864x2192
 * (RAW10), with WINMODE=0 (all-pixel readout). The sensor hardware itself
 * also supports a window-cropping mode and 2/2-line binning (WINMODE=4h
 * and ADDMODE, per the datasheet) - this driver doesn't implement them
 * (matching the upstream Linux driver's scope), so what you *can* select
 * here is the MIPI lane rate (which changes how fast that same 3864x2192
 * frame can be clocked out, i.e. your achievable frame rate) and the lane
 * count. See README.md for what this means for HDMI preview on the stock
 * hardware platform.
 * ---------------------------------------------------------------------------
 */

#ifndef IMX415_H_
#define IMX415_H_

#include <cstdio>
#include <climits>
#include <stdexcept>
#include <vector>
#include <stdint.h>

#include "I2C_Client.h"
#include "GPIO_Client.h"
#include "../hdmi/VideoOutput.h"

#define SIZEOF_ARRAY(x) sizeof(x)/sizeof(x[0])

namespace digilent {

typedef enum {OK=0, ERR_LOGICAL, ERR_GENERAL} Errc;

namespace IMX415_cfg {
	using config_word_t = struct { uint16_t addr; uint8_t data; };
	// Lane RATE is what's selectable here - there is no per-resolution mode,
	// see the file header comment above.
	using mode_t = enum { MODE_2LANE_720MBPS = 0, MODE_2LANE_891MBPS, MODE_END };

	// =========================================================================
	// Helpers to express a multi-byte Sony CCI register (as used by the
	// mainline driver's CCI_REG16_LE/CCI_REG24_LE) as consecutive single-byte
	// {addr, data} writes, little-endian - i.e. exactly what the real driver's
	// regmap does under the hood. This keeps every table below a literal,
	// auditable transcription of the upstream driver's register/value pairs.
	// =========================================================================
	#define IMX415_REG8(addr, val) \
		{ (uint16_t)(addr), (uint8_t)((val) & 0xFF) }
	#define IMX415_REG16(addr, val) \
		{ (uint16_t)(addr), (uint8_t)((val) & 0xFF) }, \
		{ (uint16_t)((addr)+1), (uint8_t)(((val) >> 8) & 0xFF) }
	#define IMX415_REG24(addr, val) \
		{ (uint16_t)(addr), (uint8_t)((val) & 0xFF) }, \
		{ (uint16_t)((addr)+1), (uint8_t)(((val) >> 8) & 0xFF) }, \
		{ (uint16_t)((addr)+2), (uint8_t)(((val) >> 16) & 0xFF) }

	// =========================================================================
	// Board-specific operating point - CONFIRM against your carrier board.
	// =========================================================================
	// XVCLK/INCK the sensor is actually fed. 24MHz is the most common default
	// on generic IMX415 breakout boards; change to 72000000 (or extend the
	// clk-params table below per the upstream driver) if yours differs.
	uint32_t const INCK_HZ = 24000000;
	// 2-lane matches the 15-pin FPC / Pcam MIPI connector this board uses.
	// Set to 4 only if you've confirmed (schematic/vendor docs) your module
	// wires out 4 data lanes AND rebuilt the D-PHY/CSI-2 RX IP for it.
	uint32_t const NUM_DATA_LANES = 2;

	// Registers (names/addresses match drivers/media/i2c/imx415.c)
	uint16_t const REG_MODE        = 0x3000; // STANDBY: 0=operating, 1=standby
	uint16_t const REG_XMSTA       = 0x3002; // 0=start master streaming, 1=stop
	uint16_t const REG_BCWAIT_TIME = 0x3008; // 16-bit
	uint16_t const REG_CPWAIT_TIME = 0x300A; // 16-bit
	uint16_t const REG_WINMODE     = 0x301C; // 0 = all-pixel readout (only supported mode)
	uint16_t const REG_ADDMODE     = 0x3022; // 0 = no analog binning
	uint16_t const REG_REVERSE     = 0x3030; // h/v flip
	uint16_t const REG_ADBIT       = 0x3031; // 0 = RAW10
	uint16_t const REG_MDBIT       = 0x3032; // 0 = RAW10
	uint16_t const REG_SYS_MODE    = 0x3033; // datasheet's own sections disagree (0x3033 vs 0x3034) - see file header comment
	uint16_t const REG_OUTSEL      = 0x30C0; // 0x22 = VSYNC on XVS, low on XHS
	uint16_t const REG_DRV         = 0x30C1;
	uint16_t const REG_VMAX        = 0x3024; // 24-bit: total frame lines
	uint16_t const REG_HMAX        = 0x3028; // 16-bit: total line length / 12
	uint16_t const REG_SHR0        = 0x3050; // 24-bit: shutter (exposure)
	uint16_t const REG_INCKSEL1    = 0x3115;
	uint16_t const REG_INCKSEL2    = 0x3116;
	uint16_t const REG_INCKSEL3    = 0x3118; // 16-bit
	uint16_t const REG_INCKSEL4    = 0x311A; // 16-bit
	uint16_t const REG_INCKSEL5    = 0x311E;
	uint16_t const REG_SENSOR_INFO = 0x3F12; // 16-bit, chip ID (masked 0xFFF)
	uint16_t const REG_LANEMODE    = 0x4001; // 16-bit: 1=2-lane, 3=4-lane
	uint16_t const REG_TXCLKESC_FREQ = 0x4004; // 16-bit
	uint16_t const REG_INCKSEL6    = 0x400C;
	uint16_t const REG_TCLKPOST    = 0x4018; // 16-bit
	uint16_t const REG_TCLKPREPARE = 0x401A; // 16-bit
	uint16_t const REG_TCLKTRAIL   = 0x401C; // 16-bit
	uint16_t const REG_TCLKZERO    = 0x401E; // 16-bit
	uint16_t const REG_THSPREPARE  = 0x4020; // 16-bit
	uint16_t const REG_THSZERO     = 0x4022; // 16-bit
	uint16_t const REG_THSTRAIL    = 0x4024; // 16-bit
	uint16_t const REG_THSEXIT     = 0x4026; // 16-bit
	uint16_t const REG_TLPX        = 0x4028; // 16-bit
	uint16_t const REG_INCKSEL7    = 0x4074;

	uint16_t const SENSOR_INFO_MASK = 0x0FFF;
	uint16_t const CHIP_ID          = 0x0514;
	uint32_t const LANEMODE_2LANE   = 1;
	uint32_t const LANEMODE_4LANE   = 3;

	// Sensor's raw pixel array (fixed - not a per-mode setting).
	uint32_t const PIXEL_ARRAY_WIDTH  = 3864;
	uint32_t const PIXEL_ARRAY_HEIGHT = 2192;
	uint32_t const PIXEL_ARRAY_VBLANK_MIN = 58;
	uint32_t const HMAX_MULTIPLIER    = 12;

	// -------------------------------------------------------------------------
	// Global init table: documented control registers + Sony's "magic"
	// undocumented analog/timing tuning registers, applied once regardless of
	// lane rate/INCK. Verbatim port of imx415_init_table[] from the upstream
	// driver referenced above.
	// -------------------------------------------------------------------------
	config_word_t const cfg_init_table_[] =
	{
		// all-pixel readout mode, no flip
		IMX415_REG8(REG_WINMODE, 0x00),
		IMX415_REG8(REG_ADDMODE, 0x00),
		IMX415_REG8(REG_REVERSE, 0x00),
		// RAW 10-bit mode
		IMX415_REG8(REG_ADBIT, 0x00),
		IMX415_REG8(REG_MDBIT, 0x00),
		// output VSYNC on XVS and low on XHS
		IMX415_REG8(REG_OUTSEL, 0x22),
		IMX415_REG8(REG_DRV,    0x00),

		// Sony "magic" registers - undocumented, fixed tuning values.
		IMX415_REG8(0x32D4, 0x21), IMX415_REG8(0x32EC, 0xA1),
		IMX415_REG8(0x3452, 0x7F), IMX415_REG8(0x3453, 0x03),
		IMX415_REG8(0x358A, 0x04), IMX415_REG8(0x35A1, 0x02),
		IMX415_REG8(0x36BC, 0x0C), IMX415_REG8(0x36CC, 0x53),
		IMX415_REG8(0x36CD, 0x00), IMX415_REG8(0x36CE, 0x3C),
		IMX415_REG8(0x36D0, 0x8C), IMX415_REG8(0x36D1, 0x00),
		IMX415_REG8(0x36D2, 0x71), IMX415_REG8(0x36D4, 0x3C),
		IMX415_REG8(0x36D6, 0x53), IMX415_REG8(0x36D7, 0x00),
		IMX415_REG8(0x36D8, 0x71), IMX415_REG8(0x36DA, 0x8C),
		IMX415_REG8(0x36DB, 0x00), IMX415_REG8(0x3724, 0x02),
		IMX415_REG8(0x3726, 0x02), IMX415_REG8(0x3732, 0x02),
		IMX415_REG8(0x3734, 0x03), IMX415_REG8(0x3736, 0x03),
		IMX415_REG8(0x3742, 0x03), IMX415_REG8(0x3862, 0xE0),
		IMX415_REG8(0x38CC, 0x30), IMX415_REG8(0x38CD, 0x2F),
		IMX415_REG8(0x395C, 0x0C), IMX415_REG8(0x3A42, 0xD1),
		IMX415_REG8(0x3A4C, 0x77), IMX415_REG8(0x3AE0, 0x02),
		IMX415_REG8(0x3AEC, 0x0C), IMX415_REG8(0x3B00, 0x2E),
		IMX415_REG8(0x3B06, 0x29), IMX415_REG8(0x3B98, 0x25),
		IMX415_REG8(0x3B99, 0x21), IMX415_REG8(0x3B9B, 0x13),
		IMX415_REG8(0x3B9C, 0x13), IMX415_REG8(0x3B9D, 0x13),
		IMX415_REG8(0x3B9E, 0x13), IMX415_REG8(0x3BA1, 0x00),
		IMX415_REG8(0x3BA2, 0x06), IMX415_REG8(0x3BA3, 0x0B),
		IMX415_REG8(0x3BA4, 0x10), IMX415_REG8(0x3BA5, 0x14),
		IMX415_REG8(0x3BA6, 0x18), IMX415_REG8(0x3BA7, 0x1A),
		IMX415_REG8(0x3BA8, 0x1A), IMX415_REG8(0x3BA9, 0x1A),
		IMX415_REG8(0x3BAC, 0xED), IMX415_REG8(0x3BAD, 0x01),
		IMX415_REG8(0x3BAE, 0xF6), IMX415_REG8(0x3BAF, 0x02),
		IMX415_REG8(0x3BB0, 0xA2), IMX415_REG8(0x3BB1, 0x03),
		IMX415_REG8(0x3BB2, 0xE0), IMX415_REG8(0x3BB3, 0x03),
		IMX415_REG8(0x3BB4, 0xE0), IMX415_REG8(0x3BB5, 0x03),
		IMX415_REG8(0x3BB6, 0xE0), IMX415_REG8(0x3BB7, 0x03),
		IMX415_REG8(0x3BB8, 0xE0), IMX415_REG8(0x3BBA, 0xE0),
		IMX415_REG8(0x3BBC, 0xDA), IMX415_REG8(0x3BBE, 0x88),
		IMX415_REG8(0x3BC0, 0x44), IMX415_REG8(0x3BC2, 0x7B),
		IMX415_REG8(0x3BC4, 0xA2), IMX415_REG8(0x3BC8, 0xBD),
		IMX415_REG8(0x3BCA, 0xBD),
	};

	// -------------------------------------------------------------------------
	// MIPI D-PHY timing tables, one per lane rate (verbatim port of
	// imx415_linkrate_720mbps[] / imx415_linkrate_891mbps[]).
	// -------------------------------------------------------------------------
	config_word_t const cfg_dphy_720mbps_[] =
	{
		IMX415_REG16(REG_TCLKPOST,    0x006F),
		IMX415_REG16(REG_TCLKPREPARE, 0x002F),
		IMX415_REG16(REG_TCLKTRAIL,   0x002F),
		IMX415_REG16(REG_TCLKZERO,    0x00BF),
		IMX415_REG16(REG_THSPREPARE,  0x002F),
		IMX415_REG16(REG_THSZERO,     0x0057),
		IMX415_REG16(REG_THSTRAIL,    0x002F),
		IMX415_REG16(REG_THSEXIT,     0x004F),
		IMX415_REG16(REG_TLPX,        0x0027),
	};
	config_word_t const cfg_dphy_891mbps_[] =
	{
		IMX415_REG16(REG_TCLKPOST,    0x007F),
		IMX415_REG16(REG_TCLKPREPARE, 0x0037),
		IMX415_REG16(REG_TCLKTRAIL,   0x0037),
		IMX415_REG16(REG_TCLKZERO,    0x00F7),
		IMX415_REG16(REG_THSPREPARE,  0x003F),
		IMX415_REG16(REG_THSZERO,     0x006F),
		IMX415_REG16(REG_THSTRAIL,    0x003F),
		IMX415_REG16(REG_THSEXIT,     0x005F),
		IMX415_REG16(REG_TLPX,        0x002F),
	};

	// -------------------------------------------------------------------------
	// Clock-configuration tables, one per (lane rate, INCK) pair (verbatim
	// port of the matching entries in imx415_clk_params[]). Add more entries
	// here (from the upstream driver) if your INCK isn't one of these.
	// -------------------------------------------------------------------------
	// 720 Mbps/lane @ INCK = 24MHz
	config_word_t const cfg_clk_720mbps_24mhz_[] =
	{
		IMX415_REG16(REG_BCWAIT_TIME, 0x054),
		IMX415_REG16(REG_CPWAIT_TIME, 0x03B),
		IMX415_REG8 (REG_SYS_MODE,    0x9),
		IMX415_REG8 (REG_INCKSEL1,    0x00),
		IMX415_REG8 (REG_INCKSEL2,    0x23),
		IMX415_REG16(REG_INCKSEL3,    0x0B4),
		IMX415_REG16(REG_INCKSEL4,    0x0FC),
		IMX415_REG8 (REG_INCKSEL5,    0x23),
		IMX415_REG8 (REG_INCKSEL6,    0x0),
		IMX415_REG8 (REG_INCKSEL7,    0x1),
		IMX415_REG16(REG_TXCLKESC_FREQ, 0x0600),
	};
	// 720 Mbps/lane @ INCK = 72MHz
	config_word_t const cfg_clk_720mbps_72mhz_[] =
	{
		IMX415_REG16(REG_BCWAIT_TIME, 0x0F8),
		IMX415_REG16(REG_CPWAIT_TIME, 0x0B0),
		IMX415_REG8 (REG_SYS_MODE,    0x9),
		IMX415_REG8 (REG_INCKSEL1,    0x00),
		IMX415_REG8 (REG_INCKSEL2,    0x28),
		IMX415_REG16(REG_INCKSEL3,    0x0A0),
		IMX415_REG16(REG_INCKSEL4,    0x0E0),
		IMX415_REG8 (REG_INCKSEL5,    0x28),
		IMX415_REG8 (REG_INCKSEL6,    0x0),
		IMX415_REG8 (REG_INCKSEL7,    0x1),
		IMX415_REG16(REG_TXCLKESC_FREQ, 0x1200),
	};
	// 891 Mbps/lane @ INCK = 27MHz
	config_word_t const cfg_clk_891mbps_27mhz_[] =
	{
		IMX415_REG16(REG_BCWAIT_TIME, 0x05D),
		IMX415_REG16(REG_CPWAIT_TIME, 0x042),
		IMX415_REG8 (REG_SYS_MODE,    0x5),
		IMX415_REG8 (REG_INCKSEL1,    0x00),
		IMX415_REG8 (REG_INCKSEL2,    0x23),
		IMX415_REG16(REG_INCKSEL3,    0x0C6),
		IMX415_REG16(REG_INCKSEL4,    0x0E7),
		IMX415_REG8 (REG_INCKSEL5,    0x23),
		IMX415_REG8 (REG_INCKSEL6,    0x0),
		IMX415_REG8 (REG_INCKSEL7,    0x1),
		IMX415_REG16(REG_TXCLKESC_FREQ, 0x06C0),
	};
	// 891 Mbps/lane @ INCK = 37.125MHz
	config_word_t const cfg_clk_891mbps_37_125mhz_[] =
	{
		IMX415_REG16(REG_BCWAIT_TIME, 0x07F),
		IMX415_REG16(REG_CPWAIT_TIME, 0x05B),
		IMX415_REG8 (REG_SYS_MODE,    0x5),
		IMX415_REG8 (REG_INCKSEL1,    0x00),
		IMX415_REG8 (REG_INCKSEL2,    0x24),
		IMX415_REG16(REG_INCKSEL3,    0x0C0),
		IMX415_REG16(REG_INCKSEL4,    0x0E0),
		IMX415_REG8 (REG_INCKSEL5,    0x24),
		IMX415_REG8 (REG_INCKSEL6,    0x0),
		IMX415_REG8 (REG_INCKSEL7,    0x1),
		IMX415_REG16(REG_TXCLKESC_FREQ, 0x0948),
	};
	// 891 Mbps/lane @ INCK = 74.25MHz
	config_word_t const cfg_clk_891mbps_74_25mhz_[] =
	{
		IMX415_REG16(REG_BCWAIT_TIME, 0x0FF),
		IMX415_REG16(REG_CPWAIT_TIME, 0x0B6),
		IMX415_REG8 (REG_SYS_MODE,    0x5),
		IMX415_REG8 (REG_INCKSEL1,    0x00),
		IMX415_REG8 (REG_INCKSEL2,    0x28),
		IMX415_REG16(REG_INCKSEL3,    0x0C0),
		IMX415_REG16(REG_INCKSEL4,    0x0E0),
		IMX415_REG8 (REG_INCKSEL5,    0x28),
		IMX415_REG8 (REG_INCKSEL6,    0x0),
		IMX415_REG8 (REG_INCKSEL7,    0x1),
		IMX415_REG16(REG_TXCLKESC_FREQ, 0x1290),
	};

	// Real per-(lane rate, lane count) minimum HMAX register value (fastest
	// supported line time - "hmax_min" in the upstream driver), indexed
	// [0]=2-lane [1]=4-lane. Used as the streaming HMAX (matches the
	// upstream driver's default/minimum hblank control value).
	uint32_t const HMAX_MIN_720MBPS[2] = { 2032, 1066 };
	uint32_t const HMAX_MIN_891MBPS[2] = { 2200, 1100 };
	// Default streaming VMAX (minimum blanking - matches the upstream
	// driver's default vblank control value).
	uint32_t const VMAX_DEFAULT = PIXEL_ARRAY_HEIGHT + PIXEL_ARRAY_VBLANK_MIN; // 2250

	#undef IMX415_REG8
	#undef IMX415_REG16
	#undef IMX415_REG24
}

class IMX415 {
public:
	class HardwareError;

	IMX415(I2C_Client& iic, GPIO_Client& gpio) :
		iic_(iic), gpio_(gpio)
	{
		reset();
		init();
	}

	void init()
	{
		// Real chip-ID check (mainline driver: imx415_identify_model()).
		// The IMX415's SENSOR_INFO register can only be read while the
		// sensor is OUT of standby, so wake it up first.
		writeReg(IMX415_cfg::REG_MODE, 0x00); // MODE = operating
		usleep(80000); // datasheet: image stabilizes >=24ms after standby-cancel (9 frames); 80ms is a safe margin

		uint8_t info_l = 0, info_h = 0;
		readReg(IMX415_cfg::REG_SENSOR_INFO, info_l);
		readReg(IMX415_cfg::REG_SENSOR_INFO + 1, info_h);
		uint16_t chip_id = (uint16_t)(((uint16_t)info_h << 8) | info_l) & IMX415_cfg::SENSOR_INFO_MASK;

		// Always return to standby afterwards, whether the ID matched or not
		// (mirrors imx415_identify_model()'s `done:` path).
		writeReg(IMX415_cfg::REG_MODE, 0x01); // MODE = standby

		if (chip_id != IMX415_cfg::CHIP_ID)
		{
			char msg[100];
			snprintf(msg, sizeof(msg),
					"IMX415 chip ID mismatch: got 0x%03x, expected 0x%03x\r\n",
					chip_id, IMX415_cfg::CHIP_ID);
			throw HardwareError(HardwareError::WRONG_ID, msg);
		}

		// Global init table (documented control regs + Sony's "magic" analog
		// tuning table). Safe to apply while in standby.
		writeConfig(IMX415_cfg::cfg_init_table_, SIZEOF_ARRAY(IMX415_cfg::cfg_init_table_));

		// Stay in standby until a lane-rate/mode is selected via set_mode(),
		// mirroring the OV5640 driver's "power up, but stay powered down
		// until pipeline_mode_change() picks a mode" behavior in main.cc.
	}

	Errc reset()
	{
		// Power/reset cycle via the shared PS GPIO line, inherited unchanged
		// from the Pcam 5C's single-GPIO OV5640 PWDN/reset pattern.
		//
		// IMPORTANT - sensor reset wiring on the "IMX415 CAM R1" board:
		// this board's 22-pin FPC connector carries CAM_RST (pin 6) and
		// CAM_GPIO (pin 5) as two SEPARATE signals. Per its schematic:
		//   - CAM_RST connects (through a 4.7k series resistor, no pull) to
		//     the sensor's XCLR/reset pin - this is the one that actually
		//     matters, and nothing else on the board drives it.
		//   - CAM_GPIO connects to nothing but a bare test point (TP3) - it
		//     is unused/unconnected on this board.
		//   - Power sequencing (1.1V -> 1.8V -> 2.9V, in the order/timing
		//     Sony's datasheet requires) happens AUTOMATICALLY on this board
		//     via an on-board RC network as soon as 3.3V arrives at the
		//     connector - it does not need a GPIO pulse from the host.
		//   - INCK is generated by an always-on oscillator on the board
		//     (see the file header comment) - also not host-controlled.
		// So the ONLY thing the host genuinely needs to drive here is
		// CAM_RST. The original Pcam 5C's single GPIO conventionally maps
		// to the Raspberry-Pi-standard connector's "CAM_GPIO" position, not
		// "CAM_RST" - if that convention holds through your Standard-Mini
		// adapter cable, this reset() is toggling a pin that goes nowhere on
		// this board, and the sensor's actual reset line is left floating
		// (no pull resistor is populated either way). This is the single
		// most likely reason init() would fail its chip-ID check.
		// To check: probe TP3 (silkscreened on the camera board) while this
		// runs - if it toggles, you've confirmed the gap. Fix: wire a spare
		// Zybo GPIO pin directly to the board's CAM_RST net/pad and extend
		// GPIO_Client (in GPIO_Client.h/PS_GPIO.h) with a second bit for it,
		// driven here instead of (or in addition to) CAM_GPIO0. See
		// README.md, "Wiring & GPIO notes".
		//
		// Timing: Sony's datasheet Power-on Sequence table gives real
		// numbers - XCLR held low >=500ns after power stable (TLOW), then
		// >=1us before INCK needs to be running (T3), then >=20us before the
		// first I2C transaction (T4). The busy-wait below is a much more
		// generous margin than that (it's an uncalibrated instruction-count
		// loop, not a real timer, like the original OV5640 driver's - swap
		// in the Xilinx standalone BSP's calibrated usleep() from sleep.h
		// for accurate, much faster power-up if you want it).
		gpio_.clearBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000);
		gpio_.setBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000);

		return OK;
	}

	Errc set_mode(IMX415_cfg::mode_t mode)
	{
		if (mode >= IMX415_cfg::mode_t::MODE_END)
			return ERR_LOGICAL;

		// Pick the D-PHY timing table + clock-config table + HMAX_MIN entry
		// for the requested lane rate. Only INCK_HZ == 24MHz (720Mbps) /
		// 27MHz (891Mbps) are wired up here, matching this file's
		// INCK_HZ/NUM_DATA_LANES defaults - if you changed those, pick a
		// different cfg_clk_*/HMAX_MIN_* pair from IMX415_cfg above.
		IMX415_cfg::config_word_t const* dphy_cfg = nullptr;
		size_t dphy_size = 0;
		IMX415_cfg::config_word_t const* clk_cfg = nullptr;
		size_t clk_size = 0;
		uint32_t hmax = 0;

		switch (mode)
		{
		case IMX415_cfg::mode_t::MODE_2LANE_720MBPS:
			dphy_cfg = IMX415_cfg::cfg_dphy_720mbps_;
			dphy_size = SIZEOF_ARRAY(IMX415_cfg::cfg_dphy_720mbps_);
			clk_cfg = IMX415_cfg::cfg_clk_720mbps_24mhz_; // INCK_HZ = 24MHz
			clk_size = SIZEOF_ARRAY(IMX415_cfg::cfg_clk_720mbps_24mhz_);
			hmax = IMX415_cfg::HMAX_MIN_720MBPS[0]; // [0] = 2-lane
			break;
		case IMX415_cfg::mode_t::MODE_2LANE_891MBPS:
			dphy_cfg = IMX415_cfg::cfg_dphy_891mbps_;
			dphy_size = SIZEOF_ARRAY(IMX415_cfg::cfg_dphy_891mbps_);
			clk_cfg = IMX415_cfg::cfg_clk_891mbps_27mhz_; // INCK_HZ = 27MHz
			clk_size = SIZEOF_ARRAY(IMX415_cfg::cfg_clk_891mbps_27mhz_);
			hmax = IMX415_cfg::HMAX_MIN_891MBPS[0]; // [0] = 2-lane
			break;
		default:
			return ERR_LOGICAL;
		}

		// Enter standby before changing lane-rate/timing registers, matching
		// the OV5640 driver's software-power-down-around-mode-change pattern
		// (and safe per the upstream driver: these registers are written
		// from imx415_setup(), called before stream-on).
		writeReg(IMX415_cfg::REG_MODE, 0x01); // MODE = standby

		writeConfig(dphy_cfg, dphy_size);
		writeConfig(clk_cfg, clk_size);
		writeReg16(IMX415_cfg::REG_LANEMODE, IMX415_cfg::LANEMODE_2LANE);
		writeReg24(IMX415_cfg::REG_VMAX, IMX415_cfg::VMAX_DEFAULT);
		writeReg16(IMX415_cfg::REG_HMAX, hmax);

		// Leave standby and start master-mode streaming
		// (mirrors imx415_stream_on(): wakeup() then XMSTA=start).
		writeReg(IMX415_cfg::REG_MODE, 0x00); // MODE = operating
		usleep(80000);
		writeReg(IMX415_cfg::REG_XMSTA, 0x00); // XMSTA = start

		return OK;
	}

	// NOTE: intentionally no set_awb()/set_isp_format() here. The IMX415 has
	// no internal ISP - it always outputs raw Bayer data. AWB/gain/format
	// conversion must happen downstream (in the FPGA fabric or in software),
	// not on the sensor itself.

	~IMX415() { }

	void readReg(uint16_t reg_addr, uint8_t& buf)
	{
		for (auto retry_count = retry_count_; retry_count > 0; --retry_count)
		{
			try
			{
				auto buf_addr = std::vector<uint8_t>{(uint8_t)(reg_addr>>8), (uint8_t)reg_addr};
				iic_.write(dev_address_, buf_addr.data(), buf_addr.size());
				iic_.read(dev_address_, &buf, 1);
				break; //If no exceptions, no more retries
			}
			catch (I2C_Client::TransmitError const& e)
			{
				if (retry_count > 0)
				{
					continue;
				}
				else
				{
					throw HardwareError(HardwareError::IIC_NACK, e.what());
				}
			}
		}
	}

	void writeReg(uint16_t reg_addr, uint8_t const reg_data)
	{
		for (auto retry_count = retry_count_; retry_count > 0; --retry_count)
		{
			try
			{
				auto buf = std::vector<uint8_t>{(uint8_t)(reg_addr>>8), (uint8_t)reg_addr, reg_data};
				iic_.write(dev_address_, buf.data(), buf.size());
				break; //If no exceptions, no more retries
			}
			catch (I2C_Client::TransmitError const& e)
			{
				if (retry_count > 0) continue;
				else throw HardwareError(HardwareError::IIC_NACK, e.what());
			}
		}
	}

	// 16/24-bit little-endian Sony CCI register writes (CCI_REG16_LE /
	// CCI_REG24_LE in the mainline driver): consecutive byte addresses,
	// least-significant byte first - same convention as IMX415_REG16/REG24
	// used to build the config_word_t tables above.
	void writeReg16(uint16_t reg_addr, uint32_t val)
	{
		writeReg(reg_addr,     (uint8_t)(val & 0xFF));
		writeReg(reg_addr + 1, (uint8_t)((val >> 8) & 0xFF));
	}
	void writeReg24(uint16_t reg_addr, uint32_t val)
	{
		writeReg(reg_addr,     (uint8_t)(val & 0xFF));
		writeReg(reg_addr + 1, (uint8_t)((val >> 8) & 0xFF));
		writeReg(reg_addr + 2, (uint8_t)((val >> 16) & 0xFF));
	}

	class HardwareError : public std::runtime_error
	{
	public:
		using Errc = enum {WRONG_ID = 1, IIC_NACK};
		HardwareError(Errc errc, char const* msg) : std::runtime_error(msg), errc_(errc) {}
		Errc errc() const { return errc_; }
	private:
		Errc errc_;
	};

private:
	void usleep(uint32_t time)
	{//TODO couldn't think of anything better (kept identical to original OV5640 driver's busy-wait)
		for (uint32_t i=0; i<time; i++) ;
	}
	void writeConfig(IMX415_cfg::config_word_t const* cfg, size_t cfg_size)
	{
		for (size_t i=0; i<cfg_size; ++i)
		{
			writeReg(cfg[i].addr, cfg[i].data);
		}
	}
private:
	I2C_Client& iic_;
	GPIO_Client& gpio_;
	// 7-bit I2C address - confirmed against mainline Linux device-tree
	// examples for the IMX415 (sony,imx415 @ 0x1a). Some vendor breakout
	// boards may strap an alternate address - if init() throws
	// HardwareError::WRONG_ID with an all-zero or all-one readback, suspect
	// an addressing/wiring problem rather than the ID mismatch itself.
	uint8_t dev_address_ = 0x1A;
	unsigned int const retry_count_ = 10;
};

} /* namespace digilent */

#endif /* IMX415_H_ */
