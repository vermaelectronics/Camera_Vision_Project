// ============================================================================
// pi_aj_frontend.v  --  PI-AJ v5 input conditioning (2 channels)
//
//  * takes the axi_ad9361 style 16-bit words (12-bit ADC, LSB aligned)
//  * scales to the internal format: ADC LSB = 2^IN_SHIFT, i.e. a full-scale
//    12-bit sample becomes +/-32768 (18-bit signed, 1 guard bit)
//  * ADC overload detector (any of I0/Q0/I1/Q1 at/above clip_thr)
//  * optional pulse blanking (zero both channels while clipping + hold)
//  * optional DC blocker, identical on both channels so the array manifold
//    seen by the power-inversion stage is not disturbed
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_frontend #(
    parameter DW       = 18,
    parameter IN_SHIFT = 4,
    parameter DC_SHIFT = 14          // DC-blocker corner ~ fs*2^-14/2pi (175 Hz @18MS/s)
) (
    input  wire                 clk,
    input  wire                 rst,
    // config (quasi static)
    input  wire [15:0]          clip_thr,
    input  wire                 dcblk_en,
    input  wire                 blank_en,
    input  wire [7:0]           blank_hold,
    // raw input
    input  wire                 in_valid,
    input  wire signed [15:0]   in_i0, in_q0, in_i1, in_q1,
    // conditioned output (1 clock latency)
    output reg                  out_valid,
    output reg  signed [DW-1:0] x0_re, x0_im, x1_re, x1_im,
    output reg                  clip
);
    localparam AW = DW + DC_SHIFT + 2;

    function [15:0] absv;
        input signed [15:0] v;
        absv = v[15] ? (~v + 16'd1) : v;
    endfunction

    function signed [DW-1:0] sat;
        input signed [DW+1:0] v;
        if (v > $signed({{3{1'b0}}, {(DW-1){1'b1}}}))
            sat = {1'b0, {(DW-1){1'b1}}};
        else if (v < $signed({{3{1'b1}}, {(DW-1){1'b0}}}))
            sat = {1'b1, {(DW-1){1'b0}}};
        else
            sat = v[DW-1:0];
    endfunction

    wire clip_now = (absv(in_i0) >= clip_thr) | (absv(in_q0) >= clip_thr) |
                    (absv(in_i1) >= clip_thr) | (absv(in_q1) >= clip_thr);

    // scaled inputs
    wire signed [DW+1:0] s_i0 = $signed(in_i0) <<< IN_SHIFT;
    wire signed [DW+1:0] s_q0 = $signed(in_q0) <<< IN_SHIFT;
    wire signed [DW+1:0] s_i1 = $signed(in_i1) <<< IN_SHIFT;
    wire signed [DW+1:0] s_q1 = $signed(in_q1) <<< IN_SHIFT;

    // DC blocker accumulators (dc = acc >>> DC_SHIFT)
    reg  signed [AW-1:0] acc_i0, acc_q0, acc_i1, acc_q1;
    wire signed [DW+1:0] dc_i0 = acc_i0 >>> DC_SHIFT;
    wire signed [DW+1:0] dc_q0 = acc_q0 >>> DC_SHIFT;
    wire signed [DW+1:0] dc_i1 = acc_i1 >>> DC_SHIFT;
    wire signed [DW+1:0] dc_q1 = acc_q1 >>> DC_SHIFT;

    wire signed [DW+1:0] h_i0 = s_i0 - (dcblk_en ? dc_i0 : 0);
    wire signed [DW+1:0] h_q0 = s_q0 - (dcblk_en ? dc_q0 : 0);
    wire signed [DW+1:0] h_i1 = s_i1 - (dcblk_en ? dc_i1 : 0);
    wire signed [DW+1:0] h_q1 = s_q1 - (dcblk_en ? dc_q1 : 0);

    reg [7:0] hold_cnt;
    wire      blank = blank_en & (clip_now | (hold_cnt != 0));

    always @(posedge clk) begin
        if (rst) begin
            acc_i0 <= 0; acc_q0 <= 0; acc_i1 <= 0; acc_q1 <= 0;
            hold_cnt  <= 0;
            out_valid <= 1'b0;
            clip      <= 1'b0;
            x0_re <= 0; x0_im <= 0; x1_re <= 0; x1_im <= 0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                clip <= clip_now;
                if (clip_now)
                    hold_cnt <= blank_hold;
                else if (hold_cnt != 0)
                    hold_cnt <= hold_cnt - 8'd1;
                // the DC estimate is not updated with clipped/blanked samples
                if (!blank) begin
                    acc_i0 <= acc_i0 + h_i0;
                    acc_q0 <= acc_q0 + h_q0;
                    acc_i1 <= acc_i1 + h_i1;
                    acc_q1 <= acc_q1 + h_q1;
                end
                x0_re <= blank ? 0 : sat(h_i0);
                x0_im <= blank ? 0 : sat(h_q0);
                x1_re <= blank ? 0 : sat(h_i1);
                x1_im <= blank ? 0 : sat(h_q1);
            end else begin
                clip <= 1'b0;
            end
        end
    end
endmodule
