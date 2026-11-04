/******************************************************************************
 *  stap_config.h -- array size of the CRPA/STAP core in THIS design.
 *  Picked up automatically by stap_solver.h. Must match the M/N parameters of
 *  gnss_stap (Source/HDL/gnss_stap.v); gnss_stap_probe() checks it against the
 *  CONFIG register at start-up and refuses to run on a mismatch.
 *****************************************************************************/
#ifndef STAP_CONFIG_H_
#define STAP_CONFIG_H_

#define STAP_M  2       /* AD9361 RX1 + RX2                   */
#define STAP_N  16      /* taps per element                   */

#endif
