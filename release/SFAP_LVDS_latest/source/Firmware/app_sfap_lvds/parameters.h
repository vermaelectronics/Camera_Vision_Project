/***************************************************************************//**
 *   @file   ad9361/src/parameters.h
 *   @brief  Parameters Definitions.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

/* ===========================================================================
 * ANTSDR E310 - Passthrough_LVDS HDL + SFAP-PI.
 * Board map of the passthrough_lvds firmware (system_top.v of that design):
 * EMIO GPIO = 54 + gpio_o index. Fixed AXI addresses from system_bd.tcl.
 * xparam_compat.h makes the classic (XSCT) and the SDT (Vitis Unified) BSP
 * names both work.
 * ======================================================================== */
#ifdef XILINX_PLATFORM
#include "xparam_compat.h"

#define UART_BAUDRATE 115200
#define AD9361_RX_0_BASEADDR		0x79020000
#define AD9361_TX_0_BASEADDR		(0x79020000 + 0x4000)
#define AD9361_RX_1_BASEADDR		AD9361_RX_0_BASEADDR
#define AD9361_TX_1_BASEADDR		AD9361_TX_0_BASEADDR
#define CF_AD9361_RX_DMA_BASEADDR	0x7C400000
#define CF_AD9361_TX_DMA_BASEADDR	0x7C420000
#define VCXO_CTRL_BASEADDR		0x43C00000	/* axi_vcxo_ctrl (AD5660 VCXO DAC) */
/* gnss_passthrough (SFAP-PI control + weights): address assigned by Vivado (0x44A00000 so far), see
 * gnss_passthrough.h (taken from xparameters.h when the BSP defines it). */

#define ADC_DDR_BASEADDR		(XPAR_DDR_MEM_BASEADDR + 0x800000)
#define DAC_DDR_BASEADDR		(XPAR_DDR_MEM_BASEADDR + 0xA000000)

#define GPIO_DEVICE_ID			XPAR_PS7_GPIO_0_DEVICE_ID
#define SPI_DEVICE_ID			XPAR_PS7_SPI_0_DEVICE_ID
#define UART_DEVICE_ID			XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID			XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GIC_DEVICE_ID			XPAR_PS7_SCUGIC_0_DEVICE_ID

/* system_top.v: gpio_o[13]=resetb [12]=en_agc [11:8]=ctl, [15]/[16] =
 * up_enable/up_txnrx, [39:32] = RF band switches  ->  EMIO pin = 54 + n */
#define GPIO_RESET_PIN			67
#define GPIO_EN_AGC_PIN			66
#define GPIO_ENABLE_PIN			69
#define GPIO_TXNRX_PIN			70
#define GPIO_SYNC_PIN			-1
#define GPIO_RESET_PIN_2		-1
#define GPIO_CAL_SW1_PIN		-1
#define GPIO_CAL_SW2_PIN		-1
#define GPIO_CTL0_PIN			62
#define GPIO_CTL1_PIN			63
#define GPIO_CTL2_PIN			64
#define GPIO_CTL3_PIN			65

#define GPIO_RX1_BAND_SEL_H		86
#define GPIO_RX1_BAND_SEL_L		87
#define GPIO_TX1_BAND_SEL_H		88
#define GPIO_TX1_BAND_SEL_L		89
#define GPIO_RX2_BAND_SEL_H		90
#define GPIO_RX2_BAND_SEL_L		91
#define GPIO_TX2_BAND_SEL_H		92
#define GPIO_TX2_BAND_SEL_L		93

/* EMIO 40 (pin 94) was the passthrough_mux enable. Not used any more:
 * RX->TX passthrough is gnss_passthrough CONTROL[0] (gnss_tx=1). */

#define SPI_CS                  0
#define SPI_CS_2                1

#define RX_CORE_BASEADDR	AD9361_RX_0_BASEADDR
#define TX_CORE_BASEADDR	AD9361_TX_0_BASEADDR
#endif

#define DAC_BUFFER_SAMPLES 1024
#define ADC_BUFFER_SAMPLES 16384
#define ADC_CHANNELS 4

#if defined LINUX_PLATFORM || defined GENERIC_PLATFORM
#define RX_CORE_BASEADDR			0
#define TX_CORE_BASEADDR			1
#define CF_AD9361_RX_DMA_BASEADDR	2
#define CF_AD9361_TX_DMA_BASEADDR	3

#define SPI_DEVICE_ID		0
#define SPI_CS			0

#define MAX_SIZE_BASE_ADDR 0x1000

#define DAC_DDR_BASEADDR	((uintptr_t)out_buff)
#define ADC_DDR_BASEADDR	((uintptr_t)in_buff)

#define GPIO_RESET_PIN	1006
#endif

#endif // __PARAMETERS_H__
