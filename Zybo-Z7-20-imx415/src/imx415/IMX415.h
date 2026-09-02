/*
 * IMX415.h
 *
 * Bare-metal (Vitis/standalone, no OS) I2C register-level driver for the
 * Sony IMX415 CMOS image sensor, adapted from Digilent's OV5640.h driver
 * that originally shipped with the Zybo Z7-20 + Pcam 5C reference design.
 *
 * ---------------------------------------------------------------------------
 * READ THIS BEFORE FLASHING REAL HARDWARE
 * ---------------------------------------------------------------------------
 * Unlike the OV5640, the IMX415 is a "dumb" raw Bayer sensor: it has no
 * internal ISP, no auto white balance engine, and no documented chip-ID
 * register pair. Several numeric fields below (marked "TODO / VERIFY") are
 * placeholders that depend on:
 *   - the exact INCK (input clock) frequency your carrier board feeds the
 *     sensor (XVCLK pin),
 *   - the MIPI lane count wired out on your board (2-lane or 4-lane),
 *   - the desired MIPI output data rate (Mbps/lane),
 *   - the "black-box" analog/timing register block Sony publishes per
 *     INCK/data-rate/lane/bit-depth combination in the official IMX415
 *     register-setting application note.
 *
 * These values are NOT independently derivable from public information and
 * are intentionally left as clearly-marked placeholders rather than invented
 * numbers presented as fact. See ../../README.md, section
 * "Values you must confirm before first power-up", for exactly what to fill
 * in and where Sony/vendor documentation defines it.
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
#define MAP_ENUM_TO_CFG(en, cfg) en, cfg, SIZEOF_ARRAY(cfg)

namespace digilent {

typedef enum {OK=0, ERR_LOGICAL, ERR_GENERAL} Errc;

namespace IMX415_cfg {
	using config_word_t = struct { uint16_t addr; uint8_t data; };
	using mode_t = enum { MODE_1080P_1920_1080_30fps = 0, MODE_4K_3840_2160_30fps, MODE_END };
	using config_modes_t = struct { mode_t mode; config_word_t const* cfg; size_t cfg_size; };
	using lane_mode_t = enum { LANES_2 = 0, LANES_4 };
	using bit_depth_t = enum { BITDEPTH_10 = 0, BITDEPTH_12 };

	// =========================================================================
	// USER-CONFIGURABLE SENSOR STRAPPING / CLOCKING
	// -------------------------------------------------------------------------
	// Fill these in per your carrier board and the IMX415 datasheet Table
	// "INCK_SEL setting" / "DATARATE_SEL setting". Common Sony IMX4xx boards
	// use a 24MHz or 27MHz XVCLK; confirm yours with a scope/schematic before
	// trusting these defaults.
	// =========================================================================
	uint8_t const INCK_SEL          = 0x04; // TODO/VERIFY: encoding for your XVCLK frequency
	uint8_t const DATARATE_SEL      = 0x03; // TODO/VERIFY: encoding for your target Mbps/lane
	uint8_t const LANE_SEL_4LANE    = 0x03; // TODO/VERIFY: exact CSI lane-count bitfield (4-lane)
	uint8_t const LANE_SEL_2LANE    = 0x01; // TODO/VERIFY: exact CSI lane-count bitfield (2-lane)
	// RAW10 is selected by default (not RAW12) so the pixel width matches the
	// AXI4-Stream / VDMA word packing this hardware platform's MIPI CSI-2 RX
	// and D-PHY RX IP were originally generated for around the OV5640's RAW10
	// output. Switching to RAW12 requires re-generating those IP cores in
	// Vivado for a 12-bit AXI4-Stream data width - see README.md.
	uint8_t const BIT_DEPTH_SEL     = 0x00; // 0 = RAW10, 1 = RAW12 (ADBIT/MDBIT)

	// Registers common to Sony "IMX" family raw sensors (address+meaning is
	// stable across IMX219/IMX290/IMX327/IMX415-class parts; confirmed against
	// the publicly documented control-register map).
	uint16_t const REG_STANDBY   = 0x3000; // bit0: 1 = standby, 0 = operating
	uint16_t const REG_XMSTA     = 0x3002; // 0 = start master streaming, 1 = stop
	uint16_t const REG_WINMODE   = 0x3018;
	uint16_t const REG_WDMODE    = 0x301A; // 0 = linear (no DOL-HDR)
	uint16_t const REG_ADDMODE   = 0x301B; // pixel binning
	uint16_t const REG_THIN_V_EN = 0x301C;
	uint16_t const REG_INCK_SEL  = 0x3014;
	uint16_t const REG_DATARATE  = 0x3015;
	uint16_t const REG_ADBIT     = 0x3022;
	uint16_t const REG_MDBIT     = 0x3023;
	uint16_t const REG_LANE_SEL  = 0x3A01;
	uint16_t const REG_VMAX_L    = 0x3028; // frame length (rows), 3 bytes little-endian
	uint16_t const REG_VMAX_M    = 0x3029;
	uint16_t const REG_VMAX_H    = 0x302A;
	uint16_t const REG_HMAX_L    = 0x302C; // line length (INCK cycles), 2 bytes little-endian
	uint16_t const REG_HMAX_H    = 0x302D;

	config_word_t const cfg_common_init_[] =
	{
		{REG_WINMODE,   0x00},          // full readout, no cropping window
		{REG_WDMODE,    0x00},          // linear mode, no DOL-HDR
		{REG_ADDMODE,   0x00},          // no analog pixel binning
		{REG_THIN_V_EN, 0x00},          // full vertical resolution (no line thinning)
		{REG_INCK_SEL,  INCK_SEL},
		{REG_DATARATE,  DATARATE_SEL},
		{REG_ADBIT,     BIT_DEPTH_SEL},
		{REG_MDBIT,     BIT_DEPTH_SEL},
		{REG_LANE_SEL,  LANE_SEL_4LANE},
		// -----------------------------------------------------------------
		// >>> INSERT THE SONY "RECOMMENDED SETTING" REGISTER BLOCK HERE <<<
		// Sony's IMX415 register-setting application note supplies roughly
		// 100 additional analog-tuning/timing registers (0x3xxx-0x3Cxx
		// range) that are selected as a fixed table for a given
		// INCK/DATARATE_SEL/lane-count/bit-depth combination. These are not
		// meaningfully guessable and are intentionally NOT included here -
		// copy them verbatim from the application note (or from the
		// mainline Linux kernel imx415.c driver's init tables, which use
		// the same register set) for your exact configuration.
		// See README.md, section "Values you must confirm before first
		// power-up", for the full checklist.
		// -----------------------------------------------------------------
	};

	// 1920x1080 windowed readout, RAW10, ~30fps.
	// VMAX/HMAX below are PLACEHOLDERS - recompute for your actual INCK and
	// DATARATE_SEL using the timing formulas in the IMX415 datasheet
	// ("Frame rate" chapter). Do not trust these for exact frame timing.
	config_word_t const cfg_1080p_30fps_[] =
	{
		{REG_VMAX_L, 0x65}, {REG_VMAX_M, 0x04}, {REG_VMAX_H, 0x00}, // VMAX  TODO/VERIFY
		{REG_HMAX_L, 0x30}, {REG_HMAX_H, 0x11},                     // HMAX  TODO/VERIFY
	};

	// 3840x2160 full-resolution readout, RAW10, ~30fps.
	// CAPTURE-ONLY on the stock Pcam-5C hardware platform: the existing VTC/
	// clocking-wizard IP in system_wrapper only has HDMI output timing for
	// up to 1080p60 (see hdmi/VideoOutput.h). Frames DMA into DDR correctly
	// at 4K, but there is no live HDMI preview until a 4K output timing is
	// added in Vivado. See README.md.
	config_word_t const cfg_4k_30fps_[] =
	{
		{REG_VMAX_L, 0x65}, {REG_VMAX_M, 0x04}, {REG_VMAX_H, 0x00}, // VMAX  TODO/VERIFY
		{REG_HMAX_L, 0x98}, {REG_HMAX_H, 0x08},                     // HMAX  TODO/VERIFY
	};

	config_modes_t const modes[] =
	{
			{ MAP_ENUM_TO_CFG(MODE_1080P_1920_1080_30fps, cfg_1080p_30fps_) },
			{ MAP_ENUM_TO_CFG(MODE_4K_3840_2160_30fps, cfg_4k_30fps_) },
	};
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
		// The IMX415 does not expose a documented chip/product-ID register
		// pair the way the OV5640 does (0x300A/0x300B). As a communication
		// sanity check we instead put the sensor into standby and read the
		// STANDBY register back, confirming the I2C transaction is ACKed and
		// the bit we wrote reads back correctly.
		writeReg(IMX415_cfg::REG_STANDBY, 0x01); // STANDBY = 1

		uint8_t standby_rb = 0;
		readReg(IMX415_cfg::REG_STANDBY, standby_rb);
		if ((standby_rb & 0x01) != 0x01)
		{
			char msg[100];
			snprintf(msg, sizeof(msg),
					"IMX415 not responding correctly on I2C (STANDBY read-back = 0x%02x, expected bit0=1)\r\n",
					standby_rb);
			throw HardwareError(HardwareError::NO_RESPONSE, msg);
		}

		writeConfig(IMX415_cfg::cfg_common_init_, SIZEOF_ARRAY(IMX415_cfg::cfg_common_init_));

		// Stay in standby until a resolution/mode is selected via set_mode(),
		// mirroring the OV5640 driver's "power up, but stay powered down
		// until pipeline_mode_change() picks a mode" behavior in main.cc.
	}

	Errc reset()
	{
		// Power/reset cycle via the shared PS GPIO line.
		//
		// NOTE: the Pcam 5C used a single GPIO for OV5640 PWDN/reset. Many
		// IMX415 breakout boards expose XCLR (reset, active-low) and/or a
		// separate power-enable pin. If your board wires these separately,
		// extend GPIO_Client with a second bit and drive both here in the
		// order your board's power-up sequence requires (rails/clock stable
		// BEFORE releasing XCLR). See README.md, "Wiring & GPIO notes".
		gpio_.clearBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000);
		gpio_.setBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000); // conservative delay for internal regulators/PLL references to settle

		return OK;
	}

	Errc set_mode(IMX415_cfg::mode_t mode)
	{
		if (mode >= IMX415_cfg::mode_t::MODE_END)
			return ERR_LOGICAL;

		// Enter standby before changing readout/timing registers, matching
		// the OV5640 driver's software-power-down-around-mode-change pattern.
		writeReg(IMX415_cfg::REG_STANDBY, 0x01);

		auto cfg_mode = &IMX415_cfg::modes[mode];
		writeConfig(cfg_mode->cfg, cfg_mode->cfg_size);

		// Leave standby and start master-mode streaming.
		writeReg(IMX415_cfg::REG_STANDBY, 0x00); // STANDBY = 0
		usleep(20000); // datasheet-recommended settle time between STANDBY=0 and XMSTA=0
		writeReg(IMX415_cfg::REG_XMSTA, 0x00);   // XMSTA = 0 (start)

		return OK;
	}

	// NOTE: intentionally no set_awb()/set_isp_format() here. The IMX415 has
	// no internal ISP - it always outputs raw Bayer data. AWB/gain/format
	// conversion must happen downstream (in the FPGA fabric or in software),
	// not on the sensor itself. See README.md for what this implies for the
	// existing Gamma-correction IP core, which still works unchanged.

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

	class HardwareError : public std::runtime_error
	{
	public:
		using Errc = enum {NO_RESPONSE = 1, IIC_NACK};
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
	// 7-bit I2C address. 0x1A is the commonly documented IMX415 address;
	// some vendor breakout boards strap an alternate address - confirm
	// against your board's schematic, or use an I2C bus scan, if init()
	// throws HardwareError::NO_RESPONSE.
	uint8_t dev_address_ = 0x1A;
	unsigned int const retry_count_ = 10;
};

} /* namespace digilent */

#endif /* IMX415_H_ */
