/******************************************************************************
 *  gnss_info.h
 *  Interactive information menu for the ANTSDR E310 V1 GNSS CRPA loopback.
 *  GNSS-CRPA MOD-10.
 *
 *  WHO THIS IS FOR
 *    Someone who has just been handed this board and a serial terminal, and has
 *    to work out what it is, what it is doing, and whether it is working. It is
 *    a teaching aid as much as a diagnostic.
 *
 *  TWO KINDS OF FACT, ALWAYS DISTINGUISHED
 *    [live] read from a register on this board, right now.
 *    [cfg]  a recorded design or board fact, from
 *           Source/Config/board_e310_v1.json or the pinned vendor sources.
 *
 *    The distinction is not decoration. A [cfg] line says what the design was
 *    built to do; a [live] line says what the silicon is actually doing. When
 *    they disagree, that disagreement IS the bug, and this project has already
 *    lost days to a status bit that was assumed rather than read (ISSUE-0003).
 *
 *  Nothing here is inferred. Where a value could not be read or decoded, it
 *  says so instead of guessing (hard rule 1).
 *****************************************************************************/
#ifndef GNSS_INFO_H_
#define GNSS_INFO_H_

#include "ad9361_api.h"

/* Highest valid menu option. */
#define GNSS_INFO_MAX_OPTION   9

/* Print the numbered main menu. */
void gnss_info_menu(void);

/* Run one menu option. Prints a short complaint if `opt` is out of range.
 * `phy` may be NULL; sections that need the radio say so rather than crash. */
void gnss_info_select(int opt, struct ad9361_rf_phy *phy);

#endif /* GNSS_INFO_H_ */
