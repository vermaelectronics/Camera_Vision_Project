`timescale 1ns/1ps
// tmds_encoder : DVI/HDMI TMDS 8b/10b-style encoder for one color
// channel, implementing the published DVI 1.0 transition-minimized,
// DC-balanced encoding algorithm (the same algorithm every TMDS/DVI
// encoder uses - it is a fixed bit-manipulation defined by the DVI
// spec, not board- or vendor-specific).
//
// *** This implementation was written from memory of the widely
// published algorithm, without access to the authoritative DVI 1.0
// spec text or an existing reference implementation to check it
// against bit-for-bit (no internet access in this environment). The
// disparity bookkeeping here is made self-provably-correct (it counts
// the actual bits transmitted, so it cannot silently drift), and the
// four fixed control-period tokens are widely and consistently cited
// constants I'm confident in - but the invert/no-invert DECISION RULE
// itself must match the fixed algorithm every real HDMI/DVI receiver
// expects, which I cannot independently verify here. Cross-check this
// against a canonical reference implementation, or test it against a
// real monitor/capture, before trusting it for a real display. ***
//
// Stage 1 picks XOR or XNOR encoding of data[7:0] (whichever gives
// fewer bit transitions), recorded in q_m[8] (1=XOR, 0=XNOR).
// Stage 2 chooses to send q_m[7:0] or its inverse (recorded in the
// output's bit 9) so the long-run count of 1s and 0s stays balanced -
// required for the receiver's AC-coupled input and clock recovery.
// During blanking (de=0) the four fixed DVI control tokens are sent
// instead of encoded pixel data.
//
// One cycle of register latency: tmds_q is valid one pix_clk cycle
// after data/ctrl/de are presented, so any signal that must stay
// aligned with tmds_q needs a matching register stage at the call site.
module tmds_encoder (
    input  wire       pix_clk,
    input  wire       rst,
    input  wire [7:0] data,   // pixel data for this channel, valid when de=1
    input  wire [1:0] ctrl,   // control pair sent during blanking (blue channel carries {vsync,hsync}; red/green tie this to 2'b00)
    input  wire       de,     // 1 = active video (encode data), 0 = blanking (encode ctrl)
    output reg  [9:0] tmds_q
);
    // ---- Stage 1: minimize transitions -------------------------------
    reg [3:0] n1_data;
    integer   i;
    always @(*) begin
        n1_data = 4'd0;
        for (i = 0; i < 8; i = i + 1)
            n1_data = n1_data + {3'd0, data[i]};
    end

    wire use_xnor = (n1_data > 4'd4) || ((n1_data == 4'd4) && (data[0] == 1'b0));

    reg [8:0] q_m;
    always @(*) begin
        q_m[0] = data[0];
        q_m[1] = use_xnor ? ~(q_m[0] ^ data[1]) : (q_m[0] ^ data[1]);
        q_m[2] = use_xnor ? ~(q_m[1] ^ data[2]) : (q_m[1] ^ data[2]);
        q_m[3] = use_xnor ? ~(q_m[2] ^ data[3]) : (q_m[2] ^ data[3]);
        q_m[4] = use_xnor ? ~(q_m[3] ^ data[4]) : (q_m[3] ^ data[4]);
        q_m[5] = use_xnor ? ~(q_m[4] ^ data[5]) : (q_m[4] ^ data[5]);
        q_m[6] = use_xnor ? ~(q_m[5] ^ data[6]) : (q_m[5] ^ data[6]);
        q_m[7] = use_xnor ? ~(q_m[6] ^ data[7]) : (q_m[6] ^ data[7]);
        q_m[8] = use_xnor ? 1'b0 : 1'b1; // 1 = XOR used, 0 = XNOR used
    end

    reg [3:0] n1_qm;
    always @(*) begin
        n1_qm = q_m[0]+q_m[1]+q_m[2]+q_m[3]+q_m[4]+q_m[5]+q_m[6]+q_m[7];
    end
    wire [3:0] n0_qm = 4'd8 - n1_qm;

    // ---- Stage 2: running disparity control --------------------------
    // disparity tracks (ones - zeros) actually transmitted in bits[7:0]
    // of every active-video symbol sent so far - computed directly from
    // the bits this module actually outputs each cycle, so it cannot
    // drift out of sync with reality regardless of the decision logic
    // above it.
    reg signed [4:0] disparity;

    wire invert_tiebreak = (q_m[8] == 1'b0);
    wire invert_bias      = (disparity[4] == 1'b0) ? (n1_qm > n0_qm) : (n0_qm > n1_qm);
    wire invert = ((disparity == 5'sd0) || (n1_qm == n0_qm)) ? invert_tiebreak : invert_bias;

    wire [7:0] out_bits = invert ? ~q_m[7:0] : q_m[7:0];
    wire [3:0] n1_out    = out_bits[0]+out_bits[1]+out_bits[2]+out_bits[3]+
                           out_bits[4]+out_bits[5]+out_bits[6]+out_bits[7];
    wire [3:0] n0_out    = 4'd8 - n1_out;

    function [9:0] ctrl_token;
        input [1:0] c;
        begin
            case (c)
                2'b00: ctrl_token = 10'b1101010100;
                2'b01: ctrl_token = 10'b0010101011;
                2'b10: ctrl_token = 10'b0101010100;
                default: ctrl_token = 10'b1010101011;
            endcase
        end
    endfunction

    always @(posedge pix_clk) begin
        if (rst) begin
            disparity <= 5'sd0;
            tmds_q    <= 10'b1101010100;
        end else if (!de) begin
            disparity <= 5'sd0; // reset at every blanking boundary
            tmds_q    <= ctrl_token(ctrl);
        end else begin
            tmds_q    <= {invert, q_m[8], out_bits};
            disparity <= disparity + $signed({1'b0, n1_out}) - $signed({1'b0, n0_out});
        end
    end

endmodule
