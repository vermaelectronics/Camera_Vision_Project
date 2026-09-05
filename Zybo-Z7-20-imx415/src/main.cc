#include "xparameters.h"

#include "platform/platform.h"
#include "imx415/IMX415.h"
#include "imx415/ScuGicInterruptController.h"
#include "imx415/PS_GPIO.h"
#include "imx415/AXI_VDMA.h"
#include "imx415/PS_IIC.h"

#include "MIPI_D_PHY_RX.h"
#include "MIPI_CSI_2_RX.h"


#define IRPT_CTL_DEVID 		XPAR_PS7_SCUGIC_0_DEVICE_ID
#define GPIO_DEVID			XPAR_PS7_GPIO_0_DEVICE_ID
#define GPIO_IRPT_ID			XPAR_PS7_GPIO_0_INTR
#define CAM_I2C_DEVID		XPAR_PS7_I2C_0_DEVICE_ID
#define CAM_I2C_IRPT_ID		XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID			XPAR_AXIVDMA_0_DEVICE_ID
#define VDMA_MM2S_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_MM2S_INTROUT_INTR
#define VDMA_S2MM_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_S2MM_INTROUT_INTR
#define CAM_I2C_SCLK_RATE	100000

#define DDR_BASE_ADDR		XPAR_DDR_MEM_BASEADDR
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x0A000000)

#define GAMMA_BASE_ADDR     XPAR_AXI_GAMMACORRECTION_0_BASEADDR

using namespace digilent;

// Status, as of this pass (see README.md §3/§4 for the full derivation of
// each):
//
//   1. D-PHY line rate - CONFIRMED and FIXED. This bitstream's timing was
//      originally only closed at 420Mbps/lane (the OV5640's rate);
//      `timing.xdc`'s `dphy_hs_clock_p` constraint has since been retimed
//      to 720Mbps/lane (period 2.778ns, waveform {0.000 1.389}) and
//      re-implemented successfully.
//   2. AXI_BayerToRGB's Bayer/CFA phase - CONFIRMED and FIXED. The VHDL's
//      `case` statement assumed BGGR; IMX415's own "Color Coding of
//      Physical Pixel Array" datasheet diagram places Gb (Green) at
//      (0,0), which is GBRG - a one-column phase shift. Fixed with
//      `case (sCrntPositionIndicatorDly3 xor "01") is` in `AssignOutputs`,
//      resynthesized and re-implemented.
//   3. AXI_BayerToRGB's line-buffer width limit - CONFIRMED, and FIXED
//      here, in software. The block's line buffer (`LineBuffer.vhd`) is
//      hard-limited to 2048px wide; IMX415's native array is 3864px -
//      1816px past that limit, corrupting every captured frame (not just
//      HDMI) regardless of how correct fixes #1 and #2 are. Rather than
//      widening that VHDL (the alternative fix - real RTL change, needs
//      resynthesis), this version crops the sensor itself down to
//      IMX415_cfg::CROP_WIDTH (2040px, the largest multiple-of-24 value
//      that's still <=2048) via IMX415.h's Window Cropping mode
//      registers (`REG_WINMODE=4h`, `REG_PIX_HST`/`REG_PIX_HWIDTH`) - a
//      register-only fix, no Vivado resynthesis needed for this option.
//      The sensor now streams CROP_WIDTH x PIXEL_ARRAY_HEIGHT (2040x2192),
//      not the full PIXEL_ARRAY_WIDTH x PIXEL_ARRAY_HEIGHT (3864x2192) -
//      `configureWrite()` below already reflects this.
//
// What's left, all optional (live HDMI preview only, not needed for
// DDR-only capture):
//   The cropped 2040x2192 frame still doesn't match any entry in
//   hdmi/VideoOutput.h's timing table. But per this project's own
//   `timing.xdc` ("Maximum targeted pixel clock frequency for dynamic
//   video clock generator is 148.5 MHz"), `video_dynclk` is a
//   runtime-reconfigurable clock generator (DRP-driven, AXI-Lite
//   controlled) - so once a target resolution/rate is picked, AXI_VDMA's
//   read side, the VTC, and v_axi4s_vid_out_0 can likely all be
//   reconfigured at runtime (each has its own AXI-Lite config port) with
//   no further Vivado change, as long as the pixel clock stays under that
//   148.5MHz ceiling. None of that is implemented here yet.
//
// This function still only brings up the sensor and the VDMA WRITE side,
// so frames land in DDR at MEM_BASE_ADDR (already demosaiced RGB, per
// AXI_BayerToRGB) where you can inspect them with a debugger/memory
// viewer. Re-enable vid.*/vdma_driver.*Read() here once you've picked a
// live-preview resolution/timing.
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver, IMX415& cam, IMX415_cfg::mode_t mode)
{
	//Bring up input (capture) pipeline back-to-front
	{
		vdma_driver.resetWrite();
		MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
		MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
		cam.reset();
	}

	{
		// CROP_WIDTH (2040), not PIXEL_ARRAY_WIDTH (3864) - the sensor is
		// configured for a horizontal window crop (IMX415.h, REG_WINMODE=4h)
		// specifically so this matches what AXI_BayerToRGB's 2048px-limited
		// line buffer can actually accept. See the header comment above.
		vdma_driver.configureWrite(IMX415_cfg::CROP_WIDTH, IMX415_cfg::PIXEL_ARRAY_HEIGHT);
		Xil_Out32(GAMMA_BASE_ADDR, 3); // Set Gamma correction factor to 1/1.8 (unused without an HDMI/ISP path, harmless to leave configured)
		// TODO CSI-2 / D-PHY config here.
		//
		// IMPORTANT - D-PHY line rate, now CONFIRMED from the real Vivado
		// project's own source (src/constraints/timing.xdc):
		//
		//   # MIPI D-PHY data rate 420Mbps/lane = 210 MHz HS_Clk
		//   create_clock -period 4.761 -name dphy_hs_clock_p ...
		//
		// That's the ONLY rate this bitstream's timing closure was ever
		// verified against - it's what the OV5640's actual default boot
		// mode uses (MODE_1080P_1920_1080_30fps, whose own comment in
		// OV5640.h says "MIPISCLK=420", i.e. the same 420Mbps/lane). A
		// faster OV5640 mode exists in that driver's config table (named
		// "336M_MIPI", ~672Mbps/lane) but is dead code - never wired to
		// main.cc's menu, never re-verified in the XDC after being added.
		// Digilent's own MIPI_D_PHY_RX IP user guide separately states the
		// core "has been tested in dual-lane configuration with 1344 Mbps
		// total data rate" (672Mbps/lane) - a documented upper reference
		// point, but still not what THIS project's constraints reflect.
		//
		// NEITHER of this driver's two IMX415 modes (720/1440 Mbps/lane -
		// the only ones valid at this board's confirmed 24MHz INCK) matches
		// 420Mbps/lane. Selecting either one against the unmodified
		// bitstream means running the D-PHY RX faster than its timing was
		// ever closed for - it may simply not lock, or (worse) may
		// intermittently mis-sample data that looks plausible but is wrong.
		// Before trusting either mode: update the dphy_hs_clock_p
		// create_clock period in timing.xdc for your chosen rate (period_ns
		// = 1000 / (Mbps_per_lane / 2) - 2.778ns for 720Mbps/lane, 1.389ns
		// for 1440Mbps/lane) and re-run implementation to confirm timing
		// closure. 720Mbps/lane is the smaller jump from the 420Mbps
		// validated point (and is within shouting distance of Digilent's
		// own 672Mbps-tested figure above), which is why MODE_2LANE_720MBPS
		// is the default below rather than MODE_2LANE_1440MBPS - "default"
		// here means "less likely to fail timing closure," not "confirmed
		// working." See README.md §3.
		cam.init();
	}

	{
		vdma_driver.enableWrite();
		MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
		MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
		cam.set_mode(mode);
		// NOTE: no cam.set_awb() here - the IMX415 has no internal ISP/AWB
		// engine (see IMX415.h). Any white balance / color processing must
		// happen downstream (FPGA fabric or host-side software).
	}

	// Output (HDMI) pipeline intentionally not brought up here - see the
	// function header comment above.
}

int main()
{
	init_platform();

	ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);
	PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
	PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, 100000);

	IMX415 cam(iic_driver, gpio_driver);
	AXI_VDMA<ScuGicInterruptController> vdma_driver(VDMA_DEVID, MEM_BASE_ADDR, irpt_ctl,
			VDMA_MM2S_IRPT_ID,
			VDMA_S2MM_IRPT_ID);
	// Constructed (brings the HDMI clock domain up) but intentionally not
	// configured/enabled - see the pipeline_mode_change() comment above for
	// why there's no live HDMI preview of the IMX415's raw output yet.
	VideoOutput vid(XPAR_VTC_0_DEVICE_ID, XPAR_VIDEO_DYNCLK_DEVICE_ID);
	(void)vid;

	pipeline_mode_change(vdma_driver, cam, IMX415_cfg::mode_t::MODE_2LANE_720MBPS);


	xil_printf("Video init done. Capturing to DDR at 0x%08x (see README.md - no HDMI preview yet).\r\n", MEM_BASE_ADDR);


	uint8_t read_char0 = 0;
	uint8_t read_char1 = 0;
	uint8_t read_char2 = 0;
	uint8_t read_char4 = 0;
	uint8_t read_char5 = 0;
	uint16_t reg_addr;
	uint8_t reg_value;

	while (1) {
		xil_printf("\r\n\r\n\r\nIMX415 MAIN OPTIONS\r\n");
		xil_printf("\r\nPlease press the key corresponding to the desired option:");
		xil_printf("\r\n  a. Change MIPI Lane Rate (sensor outputs cropped 2040x2192 RAW10 - see IMX415.h CROP_WIDTH)");
		xil_printf("\r\n  b. Write a Register Inside the Image Sensor");
		xil_printf("\r\n  c. Read a Register Inside the Image Sensor");
		xil_printf("\r\n  d. Change Gamma Correction Factor Value\r\n\r\n");

		read_char0 = getchar();
		getchar();
		xil_printf("Read: %d\r\n", read_char0);

		switch(read_char0) {

		case 'a':
			xil_printf("\r\n  Please press the key corresponding to the desired lane rate (both @ INCK=24MHz):");
			xil_printf("\r\n    1. 720 Mbps/lane (2-lane) - closer to what the inherited OV5640-era D-PHY IP was built for");
			xil_printf("\r\n    2. 1440 Mbps/lane (2-lane) - faster, more likely to need the D-PHY IP rebuilt (see README.md)");
			read_char1 = getchar();
			getchar();
			xil_printf("\r\nRead: %d", read_char1);
			switch(read_char1) {
			case '1':
				pipeline_mode_change(vdma_driver, cam, IMX415_cfg::mode_t::MODE_2LANE_720MBPS);
				xil_printf("Lane rate change done.\r\n");
				break;
			case '2':
				pipeline_mode_change(vdma_driver, cam, IMX415_cfg::mode_t::MODE_2LANE_1440MBPS);
				xil_printf("Lane rate change done.\r\n");
				break;
			default:
				xil_printf("\r\n  Selection is outside the available options! Please retry...");
			}
			break;

		case 'b':
			xil_printf("\r\nPlease enter address of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			while (read_char4 < 48) {
				read_char4 = getchar();
			}
			while (read_char5 < 48) {
				read_char5 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char4 <= 57) {
				read_char4 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char4 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char5 <= 57) {
				read_char5 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char5 -= 87;
			}
			reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
			xil_printf("Desired Register Address: %x\r\n", reg_addr);

			read_char1 = 0;
			read_char2 = 0;
			xil_printf("\r\nPlease enter value of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			reg_value = 16*read_char1 + read_char2;
			xil_printf("Desired Register Value: %x\r\n", reg_value);
			cam.writeReg(reg_addr, reg_value);
			xil_printf("Register write done.\r\n");

			break;

		case 'c':
			xil_printf("Please enter address of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			while (read_char4 < 48) {
				read_char4 = getchar();
			}
			while (read_char5 < 48) {
				read_char5 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char4 <= 57) {
				read_char4 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char4 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char5 <= 57) {
				read_char5 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char5 -= 87;
			}
			reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
			xil_printf("Desired Register Address: %x\r\n", reg_addr);

			cam.readReg(reg_addr, reg_value);
			xil_printf("Value of Desired Register: %x\r\n", reg_value);

			break;

		case 'd':
			xil_printf("  Please press the key corresponding to the desired Gamma factor:\r\n");
			xil_printf("    1. Gamma Factor = 1\r\n");
			xil_printf("    2. Gamma Factor = 1/1.2\r\n");
			xil_printf("    3. Gamma Factor = 1/1.5\r\n");
			xil_printf("    4. Gamma Factor = 1/1.8\r\n");
			xil_printf("    5. Gamma Factor = 1/2.2\r\n");
			read_char1 = getchar();
			getchar();
			xil_printf("Read: %d\r\n", read_char1);
			// Convert from ASCII to numeric
			read_char1 = read_char1 - 48;
			if ((read_char1 > 0) && (read_char1 < 6)) {
				Xil_Out32(GAMMA_BASE_ADDR, read_char1-1);
				xil_printf("Gamma value changed to 1.\r\n");
			}
			else {
				xil_printf("  Selection is outside the available options! Please retry...\r\n");
			}
			break;

		default:
			xil_printf("  Selection is outside the available options! Please retry...\r\n");
		}

		read_char1 = 0;
		read_char2 = 0;
		read_char4 = 0;
		read_char5 = 0;
	}


	cleanup_platform();

	return 0;
}
