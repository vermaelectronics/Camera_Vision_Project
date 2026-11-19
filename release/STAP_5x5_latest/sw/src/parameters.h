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

#ifdef XILINX_PLATFORM
/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <xparameters.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define UART_BAUDRATE 115200
/*
 * ANTSDR E310 - antsdr-fw-patch "ant" HDL (+ Passthrough project)
 * Addresses are fixed by ad_cpu_interconnect in projects/ant/system_bd.tcl,
 * so they are hard-coded here. This works for both the classic Vitis flow
 * and the Vitis Unified 2023.2 SDT flow (where the XPAR_ names changed).
 */
#define AD9361_RX_0_BASEADDR		0x79020000
#define AD9361_TX_0_BASEADDR		(0x79020000 + 0x4000)
#define AD9361_RX_1_BASEADDR		AD9361_RX_0_BASEADDR
#define AD9361_TX_1_BASEADDR		AD9361_TX_0_BASEADDR
#define CF_AD9361_RX_DMA_BASEADDR	0x7C400000
#define CF_AD9361_TX_DMA_BASEADDR	0x7C420000
#define VCXO_CTRL_BASEADDR		0x43C00000	/* axi_vcxo_ctrl (AD5660 VCXO DAC) */

#ifndef XPAR_DDR_MEM_BASEADDR
#define XPAR_DDR_MEM_BASEADDR		0x00000000
#endif
#define ADC_DDR_BASEADDR		(XPAR_DDR_MEM_BASEADDR + 0x800000)
#define DAC_DDR_BASEADDR		(XPAR_DDR_MEM_BASEADDR + 0xA000000)

/* PS peripheral IDs: classic flow uses DEVICE_ID, SDT flow uses BASEADDR */
#ifdef SDT
#define GPIO_DEVICE_ID			XPAR_XGPIOPS_0_BASEADDR
#define SPI_DEVICE_ID			XPAR_XSPIPS_0_BASEADDR
#define UART_DEVICE_ID			XPAR_XUARTPS_0_BASEADDR
#else
#define GPIO_DEVICE_ID			XPAR_PS7_GPIO_0_DEVICE_ID
#define SPI_DEVICE_ID			XPAR_PS7_SPI_0_DEVICE_ID
#define UART_DEVICE_ID			XPAR_XUARTPS_0_DEVICE_ID
#define INTC_DEVICE_ID			XPAR_SCUGIC_SINGLE_DEVICE_ID
#define GIC_DEVICE_ID			XPAR_PS7_SCUGIC_0_DEVICE_ID
#endif

/*
 * EMIO GPIO map of the ant HDL (system_top.v). PS GPIO number = 54 + EMIO bit.
 *   gpio_o[7:0]   AD9361 CTRL_OUT (status, inputs)   -> 54..61
 *   gpio_o[11:8]  AD9361 CTRL_IN                     -> 62..65
 *   gpio_o[12]    EN_AGC                             -> 66
 *   gpio_o[13]    RESETB                             -> 67
 *   gpio_o[15:16] ENABLE / TXNRX (to axi_ad9361 up_) -> 69 / 70
 *   gpio_o[39:32] RF band switches VCRX1_H..VCTX2_L  -> 86..93
 *   gpio_o[40]    passthrough_en (Passthrough proj)  -> 94
 *   gpio_i[40]    passthrough_active readback        -> 94
 */
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

/* RF switch define (H = 3..6 GHz path, L = 4.5 MHz..3 GHz path) */
#define GPIO_RX1_BAND_SEL_H		86
#define GPIO_RX1_BAND_SEL_L		87
#define GPIO_TX1_BAND_SEL_H		88
#define GPIO_TX1_BAND_SEL_L		89
#define GPIO_RX2_BAND_SEL_H		90
#define GPIO_RX2_BAND_SEL_L		91
#define GPIO_TX2_BAND_SEL_H		92
#define GPIO_TX2_BAND_SEL_L		93

/* Passthrough project: RX1->TX1, RX2->TX2 enable (EMIO 40) */
#define GPIO_PASSTHROUGH_PIN		94


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
