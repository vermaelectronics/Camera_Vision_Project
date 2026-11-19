/***************************************************************************//**
 *   @file   ant_platform.h
 *   @brief  ANTSDR E310 board layer for the antsdr-fw-patch "ant" LVDS HDL
 *           (Passthrough_MVDR project). Only what differs from the MVDR2
 *           board: VCXO, EMIO pin numbers, band-switch GPIOs, init clocks.
 *           Everything GNSS / anti-jam is the MVDR2 firmware, unchanged.
*******************************************************************************/
#ifndef ANT_PLATFORM_H_
#define ANT_PLATFORM_H_

#include <stdint.h>
#include "ad9361_api.h"

/* Rate and RF plan ad9361_init() comes up at. The boot preset
 * (GNSS_CFG_BOOT_BAND 3: LO 1582.5 MHz, 56 MHz, 61.44 MSPS) is applied right
 * after, so the LVDS interface is tuned at the data clock it then runs at. */
#define ANT_INIT_LO_HZ		1582500000ULL
#define ANT_INIT_RF_BW_HZ	56000000UL
#define ANT_INIT_SAMP_HZ	61440000UL

/* axi_vcxo_ctrl (AD5660 trimming the 40 MHz VCXO): manual, fixed code,
 * as the Linux driver ad5660_mp.c does at probe */
#define ANT_VCXO_MODE_MANUAL	1
#define ANT_VCXO_DAC_VALUE	23000
#define ANT_VCXO_REG_MODE	0x00
#define ANT_VCXO_REG_VALUE	0x04

/* before ad9361_init(): FPGA check, VCXO, EN_AGC low, band switches,
 * init-time RF plan and clock chain (61.44 MSPS) */
void ant_platform_pre_init(AD9361_InitParam *ip);

/* after ad9361_init(): hand the band-switch GPIOs to the phy so the MVDR2
 * code (set_rx_lo_freq / gnss_l1_set_band_polarity) drives the right pins */
void ant_platform_boot(struct ad9361_rf_phy *phy);

#endif
