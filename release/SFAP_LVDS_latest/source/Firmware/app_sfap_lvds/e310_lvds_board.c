/***************************************************************************//**
 *   @file   e310_lvds_board.c
 *   @brief  ANTSDR E310 board bring-up for the Passthrough_LVDS HDL
 *           (+ SFAP-PI). Same sequence as antsdr_ant_pre_init() in the
 *           working passthrough_lvds firmware.
*******************************************************************************/
#include <stdio.h>
#include <xil_io.h>
#include <xil_printf.h>
#include "app_config.h"
#include "parameters.h"
#include "e310_lvds_board.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "gpio_extra.h"

extern struct xil_gpio_init_param xil_gpio_param;	/* main.c */

/* AD9361 clock chains: BBPLL, ADC/DAC, R2/T2, R1/T1, RF/TF, sample rate */
static const uint32_t rx_61m44[6] = {983040000, 245760000, 122880000, 61440000, 61440000, 61440000};
static const uint32_t tx_61m44[6] = {983040000, 122880000, 122880000, 61440000, 61440000, 61440000};

#define LO_SPLIT_HZ	3000000000ULL	/* RF band switch split */

static struct no_os_gpio_desc *gpio_en_agc;
static struct no_os_gpio_desc *gpio_band[8];

static const int32_t band_pins[8] = {
	GPIO_RX1_BAND_SEL_H, GPIO_RX1_BAND_SEL_L,
	GPIO_TX1_BAND_SEL_H, GPIO_TX1_BAND_SEL_L,
	GPIO_RX2_BAND_SEL_H, GPIO_RX2_BAND_SEL_L,
	GPIO_TX2_BAND_SEL_H, GPIO_TX2_BAND_SEL_L,
};

static int32_t board_gpio_out(struct no_os_gpio_desc **desc, int32_t pin,
			      uint8_t val)
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

void e310_board_pre_init(AD9361_InitParam *ip)
{
	int i;
	uint8_t low = (E310_LO_HZ <= LO_SPLIT_HZ);
	uint32_t v;

	/* Zynq DEVCFG INT_STS bit 2 = PCFG_DONE. Any AXI access to the PL
	 * without a bitstream hangs the CPU without a message. */
	v = Xil_In32(0xF800700C);
	if (!(v & 0x4)) {
		xil_printf("\r\nFPGA NOT PROGRAMMED - use BOOT.BIN with the bitstream, "
			   "or program the FPGA before running the ELF. Stop.\r\n");
		while (1)
			;
	}

	/* 40 MHz VCXO: manual mode, fixed DAC code (no PPS/10 MHz discipline),
	 * settled before the AD9361 BBPLL is calibrated. */
	Xil_Out32(VCXO_CTRL_BASEADDR + E310_VCXO_REG_MODE, E310_VCXO_MODE_MANUAL);
	Xil_Out32(VCXO_CTRL_BASEADDR + E310_VCXO_REG_VALUE, E310_VCXO_DAC_VALUE);
	no_os_mdelay(10);

	/* EN_AGC unused (SPI gain control) -> hold low */
	board_gpio_out(&gpio_en_agc, GPIO_EN_AGC_PIN, 0);

	/* band switches: <3 GHz -> H=0 L=1, else H=1 L=0 (all 4 paths) */
	for (i = 0; i < 8; i++)
		board_gpio_out(&gpio_band[i], band_pins[i], (i & 1) ? low : !low);

	ip->gpio_resetb.number = GPIO_RESET_PIN;

	/* bring the chip up directly at the wanted RF settings */
	ip->rx_synthesizer_frequency_hz = E310_LO_HZ;
	ip->tx_synthesizer_frequency_hz = E310_LO_HZ;
	ip->rf_rx_bandwidth_hz = E310_RF_BW_HZ;
	ip->rf_tx_bandwidth_hz = E310_RF_BW_HZ;
	ip->rx_rf_port_input_select = low ? B_BALANCED : A_BALANCED;
	ip->tx_rf_port_input_select = low ? TXB : TXA;
	ip->tx_attenuation_mdB = E310_TX_ATT_OFF_MDB;

	/* start at 61.44 MS/s so ad9361_init() tunes the LVDS interface at
	 * the final data clock (245.76 MHz) */
	for (i = 0; i < 6; i++) {
		ip->rx_path_clock_frequencies[i] = rx_61m44[i];
		ip->tx_path_clock_frequencies[i] = tx_61m44[i];
	}
}
