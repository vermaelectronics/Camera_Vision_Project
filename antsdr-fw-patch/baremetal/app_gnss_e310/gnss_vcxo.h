/******************************************************************************
 * gnss_vcxo.h -- the 40 MHz reference VCXO of the ANTSDR E310 (design A).
 *
 * In the design-A FPGA, axi_vcxo_ctrl (MicroPhase, 0x43C00000) drives the
 * AD5662 DAC that trims the 40 MHz VCXO feeding the AD9361. Under Linux the
 * ad5660_mp driver sets it at boot to a fixed value (mode 1, 23000). This
 * firmware does the same before the AD9361 is initialised, so its clock is the
 * same as on design A with Linux.
 *
 *   vcxo?            mode, DAC value, reference, lock
 *   vcxo_dac=N       fixed DAC value 0..65535 (default 23000)
 *   vcxo_ref=0|1|2   0 = fixed value, 1 = lock to the 10 MHz input,
 *                    2 = lock to the PPS input
 *
 * Registers (axi_vcxo_ctrl_v1_0_S00_AXI): 0x00 mode (1 = fixed value,
 * 0 = lock to the reference), 0x04 user DAC value, 0x08 DAC value in use (RO),
 * 0x0C reference select (0 = 10 MHz, 1 = PPS_IN), 0x10 locked (RO).
 *****************************************************************************/
#ifndef GNSS_VCXO_H_
#define GNSS_VCXO_H_

void gnss_vcxo_init(void);
void get_vcxo(double *param, char param_no);
void set_vcxo_dac(double *param, char param_no);
void set_vcxo_ref(double *param, char param_no);

#endif
