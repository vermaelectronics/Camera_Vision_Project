`timescale 1ns/1ps

// ============================================================================
// sdiv_q16_serial.v
//
// SERIAL SIGNED DIVIDER WITH Q16.16 OUTPUT
//
//      q = (num << 16) / den          num : signed 64-bit
//                                     den : unsigned 64-bit, > 0
//                                     q   : signed 32-bit Q16.16, saturated
//
// Restoring division, one quotient bit per clock (one 65-bit compare/subtract
// in the critical path). start -> done = 66 clocks.
// ============================================================================

module sdiv_q16_serial (
    input wire clk,
    input wire rst,
    input wire start,

    input wire signed [63:0] num,
    input wire        [63:0] den,

    output reg signed [31:0] q,
    output reg busy,
    output reg done
);

    localparam signed [31:0] SAT_POS =  32'sh7FFF_FFFF;
    localparam signed [31:0] SAT_NEG = -32'sh7FFF_FFFF;

    reg [63:0] dvd;
    reg [63:0] dvs;
    reg [64:0] rem;
    reg [63:0] quo;
    reg        neg;
    reg [6:0]  count;
    reg [1:0]  state;

    localparam S_IDLE = 2'd0;
    localparam S_DIV  = 2'd1;
    localparam S_OUT  = 2'd2;

    wire [64:0] trial = {rem[63:0], dvd[63]};
    wire        ge    = (trial >= {1'b0, dvs});

    always @(posedge clk) begin
        if (rst) begin
            state <= S_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
            dvd   <= 64'd0;
            dvs   <= 64'd1;
            rem   <= 65'd0;
            quo   <= 64'd0;
            neg   <= 1'b0;
            count <= 7'd0;
            q     <= 32'sd0;
        end
        else begin
            done <= 1'b0;

            case (state)

                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        busy  <= 1'b1;
                        neg   <= num[63];
                        dvd   <= (num[63] ? -num : num) << 16;
                        dvs   <= (den == 64'd0) ? 64'd1 : den;
                        rem   <= 65'd0;
                        quo   <= 64'd0;
                        count <= 7'd0;
                        state <= S_DIV;
                    end
                end

                S_DIV: begin
                    if (ge) begin
                        rem <= trial - {1'b0, dvs};
                        quo <= {quo[62:0], 1'b1};
                    end
                    else begin
                        rem <= trial;
                        quo <= {quo[62:0], 1'b0};
                    end
                    dvd   <= dvd << 1;
                    count <= count + 1'b1;
                    if (count == 7'd63)
                        state <= S_OUT;
                end

                S_OUT: begin
                    if (quo > 64'h0000_0000_7FFF_FFFF)
                        q <= neg ? SAT_NEG : SAT_POS;
                    else
                        q <= neg ? -$signed({1'b0, quo[30:0]})
                                 :  $signed({1'b0, quo[30:0]});
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;

            endcase
        end
    end

endmodule
