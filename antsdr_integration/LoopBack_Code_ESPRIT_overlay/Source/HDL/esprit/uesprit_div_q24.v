`timescale 1ns/1ps
// ============================================================================
// uesprit_div_q24  --  sequential signed divider, q = sat(n / d) in Q8.24
//
// Inputs : n signed 32-bit (|n| < 2^30), d > 0 signed 32-bit (< 2^30).
// Output : quotient Q8.24, saturated to +/-(2^31 - 1).
// Latency: 56 cycles (restoring radix-2, one quotient bit per clock).
// Used for psi, the eigenvalue of Psi_r (= tan(phi/2)), which the paper's
// PIE stage outputs.
// ============================================================================
module uesprit_div_q24 (
    input  wire               clk,
    input  wire               rst,
    input  wire               start,
    input  wire signed [31:0] n_in,
    input  wire signed [31:0] d_in,
    output reg  signed [31:0] q_out,
    output reg                valid,
    output wire               busy
);

    localparam integer NB = 54;               // |n| << 24 < 2^54

    reg [NB-1:0] dividend;                    // shifts out MSB first
    reg [NB-1:0] quo;
    reg [31:0]   rem;
    reg [31:0]   dv;
    reg          neg, run, dz;
    reg [5:0]    k;

    wire [32:0]  trial = {rem, dividend[NB-1]} - {1'b0, dv};

    assign busy = run;

    always @(posedge clk) begin
        if (rst) begin
            dividend <= 0; quo <= 0; rem <= 0; dv <= 0;
            neg <= 1'b0; run <= 1'b0; dz <= 1'b0; k <= 6'd0;
            q_out <= 0; valid <= 1'b0;
        end else begin
            valid <= 1'b0;
            if (start && !run) begin
                dividend <= {(n_in[31] ? (~n_in[29:0] + 1'b1) : n_in[29:0]), 24'd0};
                dv  <= d_in;
                dz  <= (d_in <= 0);
                neg <= n_in[31];
                rem <= 32'd0;
                quo <= 0;
                k   <= 6'd0;
                run <= 1'b1;
            end else if (run) begin
                if (k < NB) begin
                    if (!trial[32]) begin
                        rem <= trial[31:0];
                        quo <= {quo[NB-2:0], 1'b1};
                    end else begin
                        rem <= {rem[30:0], dividend[NB-1]};
                        quo <= {quo[NB-2:0], 1'b0};
                    end
                    dividend <= {dividend[NB-2:0], 1'b0};
                    k <= k + 1'b1;
                end else begin
                    run   <= 1'b0;
                    valid <= 1'b1;
                    if (dz || quo[NB-1:31] != 0)
                        q_out <= neg ? -32'sd2147483647 : 32'sd2147483647;
                    else
                        q_out <= neg ? -$signed({1'b0, quo[30:0]}) : $signed({1'b0, quo[30:0]});
                end
            end
        end
    end

endmodule
