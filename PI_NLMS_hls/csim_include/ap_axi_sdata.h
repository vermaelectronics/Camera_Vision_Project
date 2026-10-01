// Minimal ap_axiu/ap_axis model for C simulation with plain g++ (no Vitis install).
// Vitis HLS uses its own ap_axi_sdata.h; this file is only on the csim include path.
#ifndef CSIM_AP_AXI_SDATA_H
#define CSIM_AP_AXI_SDATA_H

#include "ap_int.h"

template <int D, int U, int TI, int TD>
struct ap_axiu {
    ap_uint<D>              data;
    ap_uint<(D + 7) / 8>    keep;
    ap_uint<(D + 7) / 8>    strb;
    ap_uint<U  ? U  : 1>    user;
    ap_uint<1>              last;
    ap_uint<TI ? TI : 1>    id;
    ap_uint<TD ? TD : 1>    dest;
};

template <int D, int U, int TI, int TD>
struct ap_axis {
    ap_int<D>               data;
    ap_uint<(D + 7) / 8>    keep;
    ap_uint<(D + 7) / 8>    strb;
    ap_uint<U  ? U  : 1>    user;
    ap_uint<1>              last;
    ap_uint<TI ? TI : 1>    id;
    ap_uint<TD ? TD : 1>    dest;
};

#endif
