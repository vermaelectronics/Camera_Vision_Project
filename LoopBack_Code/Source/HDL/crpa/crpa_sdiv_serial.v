// ============================================================================
//  crpa_sdiv_serial.v
//  Serial signed divider with Q16.16 result, for the LCMV weight solver.
//
//      q = (num << 16) / den        num : signed  W-bit
//                                   den : unsigned W-bit, > 0
//                                   q   : signed  QW-bit Q16.16, saturated
//
//  Restoring division, one quotient bit per clock. The critical path is one
//  (W+1)-bit compare/subtract, which closes at 8 ns (rx_clk) for W = 48.
//  start -> done = W + 2 clocks.
//
//  Original work for the ANTSDR E310 V1 GNSS CRPA project.
// ============================================================================

`timescale 1ns/100ps

module crpa_sdiv_serial #(
  parameter W  = 48,     // dividend / divisor width
  parameter QW = 25      // quotient width (Q16.16 saturated to QW bits)
) (
  input                   clk,
  input                   rst,
  input                   start,
  input  signed [W-1:0]   num,
  input         [W-1:0]   den,
  output reg signed [QW-1:0] q,
  output reg              busy,
  output reg              done
);

  localparam signed [QW-1:0] SAT_POS = {1'b0, {(QW-1){1'b1}}};
  localparam signed [QW-1:0] SAT_NEG = -SAT_POS;

  reg [W-1:0] dvd;
  reg [W-1:0] dvs;
  reg [W:0]   rem;
  reg [W-1:0] quo;
  reg         neg;
  reg [6:0]   count;
  reg [1:0]   state;

  localparam S_IDLE = 2'd0;
  localparam S_DIV  = 2'd1;
  localparam S_OUT  = 2'd2;

  wire [W:0] trial = {rem[W-1:0], dvd[W-1]};
  wire       ge    = (trial >= {1'b0, dvs});

  // |num| << 16
  wire [W-1:0] num_abs = num[W-1] ? -num : num;

  always @(posedge clk) begin
    if (rst) begin
      state <= S_IDLE;
      busy  <= 1'b0;
      done  <= 1'b0;
      dvd   <= {W{1'b0}};
      dvs   <= {{(W-1){1'b0}}, 1'b1};
      rem   <= {(W+1){1'b0}};
      quo   <= {W{1'b0}};
      neg   <= 1'b0;
      count <= 7'd0;
      q     <= {QW{1'b0}};
    end else begin
      done <= 1'b0;
      case (state)
        S_IDLE: begin
          busy <= 1'b0;
          if (start) begin
            busy  <= 1'b1;
            neg   <= num[W-1];
            dvd   <= num_abs << 16;
            dvs   <= (den == {W{1'b0}}) ? {{(W-1){1'b0}}, 1'b1} : den;
            rem   <= {(W+1){1'b0}};
            quo   <= {W{1'b0}};
            count <= 7'd0;
            state <= S_DIV;
          end
        end

        S_DIV: begin
          if (ge) begin
            rem <= trial - {1'b0, dvs};
            quo <= {quo[W-2:0], 1'b1};
          end else begin
            rem <= trial;
            quo <= {quo[W-2:0], 1'b0};
          end
          dvd   <= dvd << 1;
          count <= count + 1'b1;
          if (count == W-1)
            state <= S_OUT;
        end

        S_OUT: begin
          if (quo > {{(W-QW+1){1'b0}}, {(QW-1){1'b1}}})
            q <= neg ? SAT_NEG : SAT_POS;
          else
            q <= neg ? -$signed({1'b0, quo[QW-2:0]})
                     :  $signed({1'b0, quo[QW-2:0]});
          busy  <= 1'b0;
          done  <= 1'b1;
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
