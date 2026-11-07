/* parameters.h for gnss_crpa on Linux (design A address map). */
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__
#include "lx_platform.h"
#define AD9361_RX_0_BASEADDR       LX_ADC_BASE
#define AD9361_TX_0_BASEADDR       LX_DAC_BASE
#define CF_AD9361_RX_DMA_BASEADDR  LX_RX_DMAC_BASE
#define CF_AD9361_TX_DMA_BASEADDR  LX_TX_DMAC_BASE
#endif
