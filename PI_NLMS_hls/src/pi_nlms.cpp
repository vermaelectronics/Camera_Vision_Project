#include "pi_nlms.h"
#include "ap_int.h"

// Parallel priority encoder for MSB detection (100% bit-exact to baseline loop)
static inline int8_t msb_index_fast(uint32_t v) {
#pragma HLS INLINE
    if (v == 0) return -1;
    return (int8_t)(31 - __builtin_clz(v));
}

void pi_nlms(
    hls::stream<axis_t> &in1,
    hls::stream<axis_t> &in2,
    int16_t mu_shift_ctrl,
    int16_t reserved_ctrl,
    hls::stream<axis_t> &out
) {
#pragma HLS INTERFACE axis port=in1
#pragma HLS INTERFACE axis port=in2
#pragma HLS INTERFACE axis port=out
#pragma HLS INTERFACE s_axilite port=mu_shift_ctrl bundle=CTRL_BUS offset=0x10
#pragma HLS INTERFACE s_axilite port=reserved_ctrl bundle=CTRL_BUS offset=0x18
#pragma HLS INTERFACE s_axilite port=return bundle=CTRL_BUS
#pragma HLS INTERFACE ap_ctrl_none port=return

#pragma HLS PIPELINE II=1

    // Persistent state matching baseline initial values
    static int32_t  w_ext_r   = 0;
#pragma HLS RESET variable=w_ext_r
    static int32_t  w_ext_i   = 0;
#pragma HLS RESET variable=w_ext_i
    static uint32_t power_est = 0;
#pragma HLS RESET variable=power_est

    // DSP48E1-aligned 48-bit accumulators (max magnitude over 1024 samples is < 42 bits)
    static ap_int<48> accum_grad_r = 0;
#pragma HLS RESET variable=accum_grad_r
    static ap_int<48> accum_grad_i = 0;
#pragma HLS RESET variable=accum_grad_i
    static uint16_t sample_cnt = 0;
#pragma HLS RESET variable=sample_cnt

    // Pipeline registers for signal path
    static int32_t reg_i1_scaled = 0;
#pragma HLS RESET variable=reg_i1_scaled
    static int32_t reg_q1_scaled = 0;
#pragma HLS RESET variable=reg_q1_scaled
    static int32_t reg_i2_term   = 0;
#pragma HLS RESET variable=reg_i2_term
    static int32_t reg_q2_term   = 0;
#pragma HLS RESET variable=reg_q2_term
    static int16_t reg_i2_filt   = 0;
#pragma HLS RESET variable=reg_i2_filt
    static int16_t reg_q2_filt   = 0;
#pragma HLS RESET variable=reg_q2_filt

    static int16_t i_out_reg = 0;
#pragma HLS RESET variable=i_out_reg
    static int16_t q_out_reg = 0;
#pragma HLS RESET variable=q_out_reg
    static int16_t i2_reg    = 0;
#pragma HLS RESET variable=i2_reg
    static int16_t q2_reg    = 0;
#pragma HLS RESET variable=q2_reg
    static bool sample_valid = false;
#pragma HLS RESET variable=sample_valid

    static ap_int<36> grad_prod_r = 0;
#pragma HLS RESET variable=grad_prod_r
    static ap_int<36> grad_prod_i = 0;
#pragma HLS RESET variable=grad_prod_i
    static bool prod_valid = false;
#pragma HLS RESET variable=prod_valid

    // Weight update state registers
    static ap_uint<3> upd_step       = 0;
#pragma HLS RESET variable=upd_step
    static uint32_t   reg_power_snap = 0;
#pragma HLS RESET variable=reg_power_snap
    static int8_t     reg_msb        = 0;
#pragma HLS RESET variable=reg_msb
    static ap_uint<4> reg_shift_ext  = 0;
#pragma HLS RESET variable=reg_shift_ext
    static ap_int<48> reg_grad_r     = 0;
#pragma HLS RESET variable=reg_grad_r
    static ap_int<48> reg_grad_i     = 0;
#pragma HLS RESET variable=reg_grad_i
    static ap_int<48> reg_delta_r    = 0;
#pragma HLS RESET variable=reg_delta_r
    static ap_int<48> reg_delta_i    = 0;
#pragma HLS RESET variable=reg_delta_i

    const int      POWER_LEAK_SHIFT = 11;
    const int16_t  MU_SHIFT_MIN     = 4;
    const int16_t  MU_SHIFT_MAX     = 31;
    const uint16_t ACCUM_LEN        = 1024;

    if (!in1.empty() && !in2.empty()) {

        axis_t val1 = in1.read();
        axis_t val2 = in2.read();

        int16_t i1 = (int16_t)(uint16_t)(val1.data & 0xFFFFu);
        int16_t q1 = (int16_t)(uint16_t)((val1.data >> 16) & 0xFFFFu);
        int16_t i2 = (int16_t)(uint16_t)(val2.data & 0xFFFFu);
        int16_t q2 = (int16_t)(uint16_t)((val2.data >> 16) & 0xFFFFu);

        // 1. SIGNAL COMBINATION (Exact math matching baseline steps 1 & 2)
        int32_t i_sum = reg_i1_scaled + reg_i2_term;
        int32_t q_sum = reg_q1_scaled + reg_q2_term;

        if (i_sum > 32767)       i_sum = 32767;
        else if (i_sum < -32768) i_sum = -32768;
        if (q_sum > 32767)       q_sum = 32767;
        else if (q_sum < -32768) q_sum = -32768;

        int16_t i_out = (int16_t)i_sum;
        int16_t q_out = (int16_t)q_sum;

        int16_t adapt_w_r = (int16_t)(w_ext_r >> 16);
        int16_t adapt_w_i = (int16_t)(w_ext_i >> 16);

        const int32_t SCALE_707 = 23170;
        reg_i1_scaled = ((int32_t)i1 * SCALE_707) >> 15;
        reg_q1_scaled = ((int32_t)q1 * SCALE_707) >> 15;
        reg_i2_term   = ((int32_t)i2 * adapt_w_r - (int32_t)q2 * adapt_w_i) >> 15;
        reg_q2_term   = ((int32_t)i2 * adapt_w_i + (int32_t)q2 * adapt_w_r) >> 15;
        reg_i2_filt   = i2;
        reg_q2_filt   = q2;

        // 2. POWER ESTIMATION (Exact zero-floor protection)
        uint32_t x2_power = (uint32_t)((int32_t)i2 * i2) + (uint32_t)((int32_t)q2 * q2);
        int32_t  p_delta  = (int32_t)(((int64_t)x2_power - (int64_t)power_est) >> POWER_LEAK_SHIFT);
        int64_t  p_next   = (int64_t)power_est + p_delta;
        power_est = (p_next < 0) ? 0u : (uint32_t)p_next;

        // 3. WEIGHT UPDATE FSM (Deconstructed over 4 clock cycles for timing closure)
        if (upd_step == 4) {
            int64_t next_w_r = (int64_t)w_ext_r - reg_delta_r;
            int64_t next_w_i = (int64_t)w_ext_i - reg_delta_i;

            if (next_w_r > 2147483647LL)       next_w_r = 2147483647LL;
            else if (next_w_r < -2147483648LL) next_w_r = -2147483648LL;
            if (next_w_i > 2147483647LL)       next_w_i = 2147483647LL;
            else if (next_w_i < -2147483648LL) next_w_i = -2147483648LL;

            w_ext_r  = (int32_t)next_w_r;
            w_ext_i  = (int32_t)next_w_i;
            upd_step = 0;
        }
        else if (upd_step == 3) {
            reg_delta_r = reg_grad_r >> reg_shift_ext;
            reg_delta_i = reg_grad_i >> reg_shift_ext;
            upd_step    = 4;
        }
        else if (upd_step == 2) {
            int16_t mu_shift = mu_shift_ctrl + (reg_msb < 0 ? 0 : (int16_t)reg_msb);
            if (mu_shift < MU_SHIFT_MIN) mu_shift = MU_SHIFT_MIN;
            if (mu_shift > MU_SHIFT_MAX) mu_shift = MU_SHIFT_MAX;

            reg_shift_ext = (mu_shift > 16) ? (ap_uint<4>)(mu_shift - 16) : (ap_uint<4>)0;
            upd_step      = 3;
        }
        else if (upd_step == 1) {
            reg_msb  = msb_index_fast(reg_power_snap);
            upd_step = 2;
        }

        // Trigger block accumulation snapshot every 1024 samples
        if (sample_cnt >= ACCUM_LEN) {
            reg_power_snap = power_est;
            reg_grad_r     = accum_grad_r;
            reg_grad_i     = accum_grad_i;
            upd_step       = 1;

            accum_grad_r = 0;
            accum_grad_i = 0;
            sample_cnt   = 0;
        }

        // 4. GRADIENT ACCUMULATION (Exact arithmetic matching baseline step 4)
        if (prod_valid) {
            accum_grad_r += grad_prod_r;
            accum_grad_i += grad_prod_i;
            sample_cnt++;
        }

        if (sample_valid) {
            grad_prod_r = (ap_int<36>)i_out_reg * i2_reg + (ap_int<36>)q_out_reg * q2_reg;
            grad_prod_i = (ap_int<36>)q_out_reg * i2_reg - (ap_int<36>)i_out_reg * q2_reg;
            prod_valid  = true;
        }

        i_out_reg    = i_out;
        q_out_reg    = q_out;
        i2_reg       = reg_i2_filt;
        q2_reg       = reg_q2_filt;
        sample_valid = true;

        (void)reserved_ctrl;

        // 5. PACK AXI OUTPUT
        uint32_t out_word = ((uint32_t)(uint16_t)q_out << 16) | (uint32_t)(uint16_t)i_out;
        axis_t val_out;
        val_out.data = out_word;
        val_out.keep = val1.keep;
        val_out.strb = val1.strb;
        val_out.last = val1.last;
        out.write(val_out);
    }
}
