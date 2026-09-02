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

// The IMX415 has exactly ONE native readout size, IMX415_cfg::PIXEL_ARRAY_
// WIDTH x PIXEL_ARRAY_HEIGHT (3864x2192, RAW10) - there is no sensor-side
// "1080p"/"4K" crop mode like the OV5640 had (see IMX415.h). Only the MIPI
// lane rate is selectable via `mode`.
//
// IMPORTANT - read before expecting a picture on the HDMI output:
// The OV5640/Pcam-5C pipeline this project was adapted from worked over
// direct MIPI->VDMA->HDMI passthrough (no FPGA-side image processing)
// because the OV5640 has an ON-SENSOR ISP that converts Bayer data to
// ready-to-display RGB565 before it ever leaves the sensor (see the OV5640
// driver's ISP_FORMAT_MUX_CONTROL/RGB565 registers). The IMX415 has NO
// on-sensor ISP - it only ever outputs raw, undemosaiced Bayer data. Piping
// that directly to the existing VTC/HDMI-TX path the way this project does
// will NOT produce a viewable color picture, regardless of lane count,
// data rate, or resolution matching - you'll get scrambled/grainy raw
// Bayer noise on screen. A real live preview needs either a Bayer-
// demosaic/ISP IP core added in the FPGA fabric between the CSI-2 RX and
// the VDMA write side, or capturing to DDR and demosaicing in software.
// See README.md §3 and §8.
//
// Because of that (and because 3864x2192 doesn't match any entry in
// hdmi/VideoOutput.h's timing table, and its pixel rate is well beyond what
// this hardware platform's clocking wizard/HDMI TX were built for - see
// README.md), this function deliberately does NOT wire up the VTC/HDMI
// output stage. It brings up the sensor and the VDMA WRITE side only, so
// captured raw Bayer frames land in DDR at MEM_BASE_ADDR where you can
// inspect them with a debugger/memory viewer. Re-enable vid.*/
// vdma_driver.*Read() once you've added real demosaic/output-timing support
// in Vivado.
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
		vdma_driver.configureWrite(IMX415_cfg::PIXEL_ARRAY_WIDTH, IMX415_cfg::PIXEL_ARRAY_HEIGHT);
		Xil_Out32(GAMMA_BASE_ADDR, 3); // Set Gamma correction factor to 1/1.8 (unused without an HDMI/ISP path, harmless to leave configured)
		// TODO CSI-2 / D-PHY config here.
		//
		// IMPORTANT: this project's MIPI_CSI_2_RX / MIPI_D_PHY_RX IP cores
		// (in system_wrapper) were generated for the OV5640's 2-lane, RAW10,
		// up-to-336Mbps/lane MIPI output. Confirm the IP's configured lane
		// count/data rate matches the `mode` you pass below (720 or 891
		// Mbps/lane, 2-lane by default in IMX415.h) - if it doesn't, the
		// CSI-2/D-PHY receiver core will not lock and no image data will
		// arrive, even though the sensor is streaming correctly. See
		// README.md §3.
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
		xil_printf("\r\n  a. Change MIPI Lane Rate (sensor always outputs full 3864x2192 RAW10)");
		xil_printf("\r\n  b. Write a Register Inside the Image Sensor");
		xil_printf("\r\n  c. Read a Register Inside the Image Sensor");
		xil_printf("\r\n  d. Change Gamma Correction Factor Value\r\n\r\n");

		read_char0 = getchar();
		getchar();
		xil_printf("Read: %d\r\n", read_char0);

		switch(read_char0) {

		case 'a':
			xil_printf("\r\n  Please press the key corresponding to the desired lane rate:");
			xil_printf("\r\n    1. 720 Mbps/lane (2-lane) - closer to what the inherited OV5640-era D-PHY IP was built for");
			xil_printf("\r\n    2. 891 Mbps/lane (2-lane) - faster, more likely to need the D-PHY IP rebuilt (see README.md)");
			read_char1 = getchar();
			getchar();
			xil_printf("\r\nRead: %d", read_char1);
			switch(read_char1) {
			case '1':
				pipeline_mode_change(vdma_driver, cam, IMX415_cfg::mode_t::MODE_2LANE_720MBPS);
				xil_printf("Lane rate change done.\r\n");
				break;
			case '2':
				pipeline_mode_change(vdma_driver, cam, IMX415_cfg::mode_t::MODE_2LANE_891MBPS);
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
