/***************************************************************************//**
 *   @file   e310_lvds_board.h
 *   @brief  ANTSDR E310 board bring-up for the Passthrough_LVDS HDL
 *           (+ CRPA). Taken from antsdr_ant.c of the working
 *           passthrough_lvds firmware (antsdr_ant_pre_init), without its
 *           console commands, so that the CRPA command set is the only one.
*******************************************************************************/
#ifndef E310_LVDS_BOARD_H_
#define E310_LVDS_BOARD_H_

#include <stdint.h>
#include "ad9361_api.h"

/* Radio state ad9361_init() comes up in. Same as the passthrough firmware,
 * except the RF bandwidth, which is the default RF profile
 * (crpa band 1: 1582.5 MHz, 61.44 MS/s, 56 MHz) so nothing changes when
 * gnss_band_set(GNSS_BAND_DEFAULT) runs after init. The interface is tuned
 * at the final 245.76 MHz data clock. */
#define E310_LO_HZ              1582500000ULL
#define E310_SAMP_HZ            61440000UL
#define E310_RF_BW_HZ           56000000UL
#define E310_TX_ATT_OFF_MDB     89750           /* TX silent at boot */

/* axi_vcxo_ctrl (AD5660 DAC trimming the 40 MHz VCXO) @ VCXO_CTRL_BASEADDR */
#define E310_VCXO_MODE_MANUAL   1
#define E310_VCXO_DAC_VALUE     23000
#define E310_VCXO_REG_MODE      0x00
#define E310_VCXO_REG_VALUE     0x04

/* Call before ad9361_init(): checks the PL is configured, sets the VCXO,
 * EN_AGC low, RF band switches (<3 GHz), and the init parameters above. */
void e310_board_pre_init(AD9361_InitParam *ip);

#endif /* E310_LVDS_BOARD_H_ */
