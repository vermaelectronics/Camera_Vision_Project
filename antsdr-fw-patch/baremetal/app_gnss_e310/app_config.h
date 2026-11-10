/***************************************************************************//**
 *   @file   app_config.h
 *   @brief  Config file of AD9361/API Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2015(c) Analog Devices, Inc.
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
#ifndef CONFIG_H_
#define CONFIG_H_

/* GNSS-CRPA MOD-8: build-variant defines, injected by Build-Software.ps1.
 * Included FIRST so every variant flag is visible to everything below and to
 * every translation unit that includes app_config.h. See gnss_build_defines.h
 * for why this is a generated header rather than -D on the compiler line. */
#include "gnss_build_defines.h"

#define HAVE_SPLIT_GAIN_TABLE	1 /* only set to 0 in case split_gain_table_mode_enable = 0*/
#define HAVE_TDD_SYNTH_TABLE	1 /* only set to 0 in case split_gain_table_mode_enable = 0*/

#define AD9361_DEVICE			1 /* set it 1 if AD9361 device is used, 0 otherwise */
#define AD9364_DEVICE			0 /* set it 1 if AD9364 device is used, 0 otherwise */
#define AD9363A_DEVICE			0 /* set it 1 if AD9363A device is used, 0 otherwise */
#define CONSOLE_COMMANDS
#define ANTSDR_E310
#define XILINX_PLATFORM
//#define ALTERA_PLATFORM
//#define LINUX_PLATFORM
//#define FMCOMMS5
#define ADI_RF_SOM

/* antsdr-fw-patch crpa2: this firmware runs on the design-A FPGA
 * (plutosdr-fw hdl projects/ant + patches 0001/0002/0003), not on the
 * SMI-PI LoopBack block design. Differences that matter to the firmware:
 *   gnss_passthrough at 0x43C10000 (0x43C00000 is axi_vcxo_ctrl),
 *   AD9361 RESETB on EMIO 13 (GPIO 67), RF band switches on EMIO 32..39
 *   (GPIO 86..93), and the 40 MHz VCXO is trimmed by axi_vcxo_ctrl. */
#define ANTSDR_DESIGN_A
#define GNSS_PT_BASEADDR        0x43C10000U
#define GNSS_VCXO_BASEADDR      0x43C00000U
#define GNSS_VCXO_DAC_DEFAULT   23000U   /* the value the design-A Linux driver (ad5660_mp) sets */
//#define ADI_RF_SOM_CMOS
//#define ADC_DMA_EXAMPLE
//#define ADC_DMA_IRQ_EXAMPLE
//#define DAC_DMA_EXAMPLE
//#define AXI_ADC_NOT_PRESENT
//#define TDD_SWITCH_STATE_EXAMPLE

//#define IIO_SUPPORT

#ifndef IIO_SUPPORT
#define HAVE_VERBOSE_MESSAGES /* Recommended during development prints errors and warnings */
//#define HAVE_DEBUG_MESSAGES /* For Debug purposes only */
#endif // USE_LIBIIO
/*
 * In case memory footprint is a concern these options allow
 * to disable unused functionality which may free up a few kb
 */
#endif
