#ifndef PI_NLMS_H
#define PI_NLMS_H

#include <stdint.h>
#include "ap_axi_sdata.h"
#include "hls_stream.h"

// AXI-Stream type: 32-bit packed IQ (bits [15:0]=I, [31:16]=Q)
typedef ap_axiu<32, 0, 0, 0> axis_t;

void pi_nlms(
    hls::stream<axis_t> &in1,
    hls::stream<axis_t> &in2,
    int16_t mu_shift_ctrl,
    int16_t reserved_ctrl,
    hls::stream<axis_t> &out
);

#endif
