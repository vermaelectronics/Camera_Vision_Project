`timescale 1ns/1ps
// ============================================================================
// uesprit_cordic_atan2  --  iterative vectoring CORDIC, z = atan2(y, x)
//
// Inputs : x, y signed 32-bit with |x|, |y| < 2^30.
// Output : angle in Q30 radians, signed 34-bit, range (-pi, pi].
// Latency: ITER + 2 cycles. 30 iterations give ~1e-9 rad resolution.
// ============================================================================
module uesprit_cordic_atan2 #(
    parameter integer ITER = 30
)(
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire signed [31:0] x_in,
    input  wire signed [31:0] y_in,
    output reg  signed [33:0] angle_q30,
    output reg                valid,
    output wire               busy
);

    localparam signed [33:0] PI_Q30 = 34'sd3373259426;

    function signed [33:0] atan_tab;
        input [4:0] i;
        begin
            case (i)
                5'd0:  atan_tab = 34'sd843314857;
                5'd1:  atan_tab = 34'sd497837829;
                5'd2:  atan_tab = 34'sd263043837;
                5'd3:  atan_tab = 34'sd133525159;
                5'd4:  atan_tab = 34'sd67021687;
                5'd5:  atan_tab = 34'sd33543516;
                5'd6:  atan_tab = 34'sd16775851;
                5'd7:  atan_tab = 34'sd8388437;
                5'd8:  atan_tab = 34'sd4194283;
                5'd9:  atan_tab = 34'sd2097149;
                5'd10: atan_tab = 34'sd1048576;
                5'd11: atan_tab = 34'sd524288;
                5'd12: atan_tab = 34'sd262144;
                5'd13: atan_tab = 34'sd131072;
                5'd14: atan_tab = 34'sd65536;
                5'd15: atan_tab = 34'sd32768;
                5'd16: atan_tab = 34'sd16384;
                5'd17: atan_tab = 34'sd8192;
                5'd18: atan_tab = 34'sd4096;
                5'd19: atan_tab = 34'sd2048;
                5'd20: atan_tab = 34'sd1024;
                5'd21: atan_tab = 34'sd512;
                5'd22: atan_tab = 34'sd256;
                5'd23: atan_tab = 34'sd128;
                5'd24: atan_tab = 34'sd64;
                5'd25: atan_tab = 34'sd32;
                5'd26: atan_tab = 34'sd16;
                5'd27: atan_tab = 34'sd8;
                5'd28: atan_tab = 34'sd4;
                5'd29: atan_tab = 34'sd2;
                5'd30: atan_tab = 34'sd1;
                default: atan_tab = 34'sd0;
            endcase
        end
    endfunction

    // |x|,|y| < 2^30; CORDIC gain 1.647 * sqrt(2) < 2.33 -> < 2^32, use 35 b
    reg signed [34:0] x, y;
    reg signed [33:0] z;
    reg [4:0]         i;
    reg               run;

    assign busy = run;

    always @(posedge clk) begin
        if (rst) begin
            x <= 0; y <= 0; z <= 0; i <= 5'd0;
            run <= 1'b0; valid <= 1'b0; angle_q30 <= 0;
        end else begin
            valid <= 1'b0;
            if (start && !run) begin
                // pre-rotate the left half plane by pi
                if (x_in < 0) begin
                    x <= -$signed({{3{x_in[31]}}, x_in});
                    y <= -$signed({{3{y_in[31]}}, y_in});
                    z <= (y_in < 0) ? -PI_Q30 : PI_Q30;
                end else begin
                    x <= {{3{x_in[31]}}, x_in};
                    y <= {{3{y_in[31]}}, y_in};
                    z <= 34'sd0;
                end
                i   <= 5'd0;
                run <= 1'b1;
            end else if (run) begin
                if (i < ITER) begin
                    if (y > 0) begin
                        x <= x + (y >>> i);
                        y <= y - (x >>> i);
                        z <= z + atan_tab(i);
                    end else begin
                        x <= x - (y >>> i);
                        y <= y + (x >>> i);
                        z <= z - atan_tab(i);
                    end
                    i <= i + 1'b1;
                end else begin
                    angle_q30 <= z;
                    valid     <= 1'b1;
                    run       <= 1'b0;
                end
            end
        end
    end

endmodule
