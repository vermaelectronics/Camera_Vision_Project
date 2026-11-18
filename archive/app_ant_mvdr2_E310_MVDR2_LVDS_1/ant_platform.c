/***************************************************************************//**
 *   @file   ant_platform.c
 *   @brief  ANTSDR E310 board layer for the antsdr-fw-patch "ant" LVDS HDL.
 *
 *   Values from antsdr-fw-patch: projects/ant/system_top.v (EMIO map),
 *   zynq-ant.dtsi (band switch polarity, VCXO default). Proven on this board
 *   with the Passthrough_LVDS design (all four constellations, 61.44 MSPS).
*******************************************************************************/
#include <stdio.h>
#include <xil_io.h>
#include <xil_printf.h>
#include "app_config.h"
#include "parameters.h"
#include "ant_platform.h"
#include "ad9361.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "gpio_extra.h"

extern struct xil_gpio_init_param xil_gpio_param;	/* main.c */

/* 61.44 MSPS: RX BBPLL, ADC, R2, R1, RF, rate / TX BBPLL, DAC, T2, T1, TF, rate */
static const uint32_t rx_61m44[6] = {983040000, 245760000, 122880000, 61440000, 61440000, 61440000};
static const uint32_t tx_61m44[6] = {983040000, 122880000, 122880000, 61440000, 61440000, 61440000};
#define LO_SPLIT_HZ	3000000000ULL

static struct no_os_gpio_desc *gpio_en_agc;
static struct no_os_gpio_desc *gpio_band[8];
static const int32_t band_pins[8] = {
	GPIO_RX1_BAND_SEL_H, GPIO_RX1_BAND_SEL_L,
	GPIO_TX1_BAND_SEL_H, GPIO_TX1_BAND_SEL_L,
	GPIO_RX2_BAND_SEL_H, GPIO_RX2_BAND_SEL_L,
	GPIO_TX2_BAND_SEL_H, GPIO_TX2_BAND_SEL_L,
};

static int32_t gpio_out(struct no_os_gpio_desc **desc, int32_t pin, uint8_t val)
{
	struct no_os_gpio_init_param p = {
		.number = pin,
		.platform_ops = &xil_gpio_ops,
		.extra = &xil_gpio_param,
	};
	int32_t ret;

	if (!*desc) {
		ret = no_os_gpio_get(desc, &p);
		if (ret)
			return ret;
	}
	return no_os_gpio_direction_output(*desc, val);
}

void ant_platform_pre_init(AD9361_InitParam *ip)
{
	int i;
	uint8_t low = (ANT_INIT_LO_HZ <= LO_SPLIT_HZ);
	uint32_t v;

	/* Zynq DEVCFG INT_STS bit 2 = PCFG_DONE. An AXI access to an
	 * unprogrammed PL hangs the CPU with a silent console. */
	v = Xil_In32(0xF800700C);
	if (!(v & 0x4)) {
		xil_printf("FPGA NOT PROGRAMMED - program the bitstream first. Stop.\r\n");
		while (1);
	}
	ANT_DBG("    axi_ad9361 VERSION = 0x%08x\r\n", (unsigned)Xil_In32(AD9361_RX_0_BASEADDR));

	/* 40 MHz reference must be stable before the BBPLL calibration */
	Xil_Out32(VCXO_CTRL_BASEADDR + ANT_VCXO_REG_MODE, ANT_VCXO_MODE_MANUAL);
	Xil_Out32(VCXO_CTRL_BASEADDR + ANT_VCXO_REG_VALUE, ANT_VCXO_DAC_VALUE);
	no_os_mdelay(10);

	gpio_out(&gpio_en_agc, GPIO_EN_AGC_PIN, 0);	/* SPI gain control */
	for (i = 0; i < 8; i++)				/* <3 GHz: H = 0, L = 1 */
		gpio_out(&gpio_band[i], band_pins[i], (i & 1) ? low : !low);

	ip->gpio_resetb.number = GPIO_RESET_PIN;
	/* the driver must not request the band pins a second time */
	ip->gpio_rx1_ctrl_h.number = -1;  ip->gpio_rx1_ctrl_l.number = -1;
	ip->gpio_tx1_ctrl_h.number = -1;  ip->gpio_tx1_ctrl_l.number = -1;
	ip->gpio_rx2_ctrl_h.number = -1;  ip->gpio_rx2_ctrl_l.number = -1;
	ip->gpio_tx2_ctrl_h.number = -1;  ip->gpio_tx2_ctrl_l.number = -1;

	ip->rx_synthesizer_frequency_hz = ANT_INIT_LO_HZ;
	ip->tx_synthesizer_frequency_hz = ANT_INIT_LO_HZ;
	ip->rf_rx_bandwidth_hz = ANT_INIT_RF_BW_HZ;
	ip->rf_tx_bandwidth_hz = ANT_INIT_RF_BW_HZ;
	ip->rx_rf_port_input_select = low ? B_BALANCED : A_BALANCED;
	ip->tx_rf_port_input_select = low ? TXB : TXA;
	ip->tx_attenuation_mdB = 89750;			/* TX1/TX2 silent at boot */
	ip->gc_rx1_mode = RF_GAIN_MGC;			/* MVDR: manual, equal */
	ip->gc_rx2_mode = RF_GAIN_MGC;
	for (i = 0; i < 6; i++) {			/* tune LVDS at 61.44 MSPS */
		ip->rx_path_clock_frequencies[i] = rx_61m44[i];
		ip->tx_path_clock_frequencies[i] = tx_61m44[i];
	}
}

void ant_platform_boot(struct ad9361_rf_phy *phy)
{
	phy->gpio_desc_rx1_ctrl_h = gpio_band[0];
	phy->gpio_desc_rx1_ctrl_l = gpio_band[1];
	phy->gpio_desc_tx1_ctrl_h = gpio_band[2];
	phy->gpio_desc_tx1_ctrl_l = gpio_band[3];
	phy->gpio_desc_rx2_ctrl_h = gpio_band[4];
	phy->gpio_desc_rx2_ctrl_l = gpio_band[5];
	phy->gpio_desc_tx2_ctrl_h = gpio_band[6];
	phy->gpio_desc_tx2_ctrl_l = gpio_band[7];
}
