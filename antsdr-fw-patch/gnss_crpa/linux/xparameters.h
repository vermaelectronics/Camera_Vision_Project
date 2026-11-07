#ifndef XPARAMETERS_H_LX
#define XPARAMETERS_H_LX
/* antsdr-fw-patch design A: gnss_passthrough sits at 0x43C10000
 * (0x43C00000 is the AD5660 reference-clock DAC core). */
#define XPAR_GNSS_PASSTHROUGH_0_BASEADDR 0x43C10000U
#endif
