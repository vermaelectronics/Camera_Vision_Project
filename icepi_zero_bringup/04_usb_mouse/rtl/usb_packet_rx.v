// ============================================================================
// usb_packet_rx.v -- receives one low-speed USB packet off D+/D-: detects
// start-of-packet (the idle bus's first J->K transition), recovers bit
// timing via usb_nco (re-synced to every line transition seen, since
// bit-stuffing guarantees one at least every 7 bit periods), NRZI-decodes
// and bit-destuffs the result, and reports the decoded PID/payload once
// EOP (SE0) is detected.
// ----------------------------------------------------------------------------
// Scope: this host controller only ever needs to *receive* handshakes
// (ACK/NAK/STALL) and DATA0/DATA1 packets -- it generates tokens itself,
// never receives them -- so token-packet CRC5 decoding is intentionally
// not implemented here (an unrecognized/unexpected PID is just reported
// not-ok). Max payload is capped at 8 bytes, matching low-speed USB's
// fixed 8-byte max packet size for every endpoint type.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module usb_packet_rx #(
    parameter integer CLK_FREQ_HZ = 50_000_000
) (
    input  wire clk,
    input  wire rst,

    input  wire dp_i, dn_i,  // synchronized (2FF, by the caller) line-state inputs

    output reg          busy,
    output reg          done,               // one-cycle pulse
    output reg          ok,                 // framing + (if applicable) CRC all checked out
    output reg  [3:0]   pid,
    output reg  [63:0]  payload,
    output reg  [3:0]   payload_len_bytes
);

    wire cur_j   = (dp_i == 1'b0) && (dn_i == 1'b1);
    wire cur_k   = (dp_i == 1'b1) && (dn_i == 1'b0);
    wire cur_se0 = (dp_i == 1'b0) && (dn_i == 1'b0);

    reg prev_dp, prev_dn;
    always @(posedge clk) begin
        prev_dp <= dp_i;
        prev_dn <= dn_i;
    end
    wire prev_j        = (prev_dp == 1'b0) && (prev_dn == 1'b1);
    wire line_changed  = (dp_i != prev_dp) || (dn_i != prev_dn);

    reg  nco_resync;
    wire nco_tick;

    usb_nco #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_nco (
        .clk(clk), .rst(rst), .resync(nco_resync), .tick(nco_tick)
    );

    localparam ST_IDLE = 2'd0, ST_RECV = 2'd1, ST_EOP = 2'd2, ST_CRC_LOOP = 2'd3;
    reg [1:0] state;

    reg        prev_nrzi_k;    // last sampled line state (K=1/J=0), for NRZI decode
    reg [6:0]  bit_count;      // bits sampled since SOP, including the 8 SYNC bits
    reg [2:0]  ones_run;
    reg [87:0] rx_bits;        // PID + payload + CRC, post-destuff, arrival order
    reg [6:0]  real_bit_count; // count of rx_bits actually filled (post-destuff)
    reg [9:0]  se0_count;

    integer i;
    reg [15:0] crc16_calc;
    reg        fb16;
    reg [4:0]  crc5_calc;
    reg        fb5;
    reg        sample_k;
    reg        bit_val;
    reg [6:0]  pbits;
    reg [6:0]  crc_i;

    localparam integer SE0_EOP_THRESHOLD = 20; // system clocks (~400ns, < 1 bit period)

    always @(posedge clk) begin
        done       <= 1'b0;
        nco_resync <= 1'b0;

        if (rst) begin
            state <= ST_IDLE;
            busy  <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    busy <= 1'b0;
                    if (prev_j && cur_k) begin
                        // start of packet: this K *is* SYNC's first bit (a 0,
                        // since idle-J -> K is a transition = NRZI '0').
                        busy           <= 1'b1;
                        nco_resync     <= 1'b1;
                        prev_nrzi_k    <= 1'b1; // we're sitting on K right now
                        bit_count      <= 7'd1; // that first bit is already "sampled"
                        ones_run       <= 3'd0;
                        real_bit_count <= 7'd0;
                        se0_count      <= 10'd0;
                        state          <= ST_RECV;
                    end
                end

                ST_RECV: begin
                    if (line_changed) nco_resync <= 1'b1;

                    if (cur_se0) begin
                        se0_count <= se0_count + 1'b1;
                        if (se0_count + 1'b1 == SE0_EOP_THRESHOLD) begin
                            state <= ST_EOP;
                        end
                    end else begin
                        se0_count <= 10'd0;

                        if (nco_tick) begin
                            // decode this bit via NRZI: same-as-prev = 1, changed = 0
                            sample_k = cur_k;
                            bit_val  = (sample_k == prev_nrzi_k);
                            prev_nrzi_k <= sample_k;

                            // SYNC is 8 bits and bit0 is free (the J->K SOP edge itself
                            // *is* bit0's value, no sampling needed) -- so on paper only
                            // 7 more ticks (bits 1..7) should be needed before real data
                            // starts. Empirically, one more tick than that lands inside
                            // the sync region every time (verified via a direct decode
                            // tick count against the transmitter's real total bit count,
                            // and confirmed by testing PID values on both sides of the
                            // SYNC/PID edge boundary) -- the synchronizer (2 cycles) and
                            // the NCO's own resync-application latency (1 more cycle;
                            // see usb_nco.v) apparently stack such that RX's first
                            // post-SOP tick re-samples bit0 rather than landing on bit1.
                            // Treating bit_count 1..8 (not 1..7) as SYNC absorbs that
                            // extra tick harmlessly instead of it corrupting real data.
                            if (bit_count < 7'd9) begin
                                // SYNC field bit -- not destuffed, not stored
                                bit_count <= bit_count + 1'b1;
                            end else begin
                                bit_count <= bit_count + 1'b1;
                                if (ones_run == 3'd6) begin
                                    // expected stuff bit -- discard, don't store
                                    ones_run <= 3'd0;
                                end else begin
                                    if (real_bit_count < 7'd88)
                                        rx_bits[real_bit_count] <= bit_val;
                                    real_bit_count <= real_bit_count + 1'b1;
                                    ones_run <= bit_val ? (ones_run + 1'b1) : 3'd0;
                                end
                            end
                        end
                    end
                end

                ST_EOP: begin
                    // finalize: extract PID, check complement, and (for DATA
                    // packets) kick off CRC16 recomputation -- spread over
                    // ST_CRC_LOOP below, one bit per clock cycle, rather than
                    // all 64 XOR/shift steps chained combinationally in this
                    // one cycle (which is exactly the class of bug
                    // usb_packet_tx.v's own header comment documents: real
                    // STA (nextpnr-ecp5) missed 50MHz by more than 3x with
                    // the CRC done this way in a single cycle -- Icarus
                    // simulation, with no timing model, never caught it).
                    pid <= rx_bits[3:0];

                    if (real_bit_count < 7'd8 ||
                        rx_bits[4] != ~rx_bits[0] || rx_bits[5] != ~rx_bits[1] ||
                        rx_bits[6] != ~rx_bits[2] || rx_bits[7] != ~rx_bits[3]) begin
                        ok    <= 1'b0;
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        payload_len_bytes <= 4'd0;
                        state <= ST_IDLE;
                    end else if (rx_bits[3:0] == 4'b0010 || rx_bits[3:0] == 4'b1010 ||
                                 rx_bits[3:0] == 4'b1110) begin
                        // ACK / NAK / STALL: PID only, no payload
                        ok    <= (real_bit_count == 7'd8);
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        payload_len_bytes <= 4'd0;
                        state <= ST_IDLE;
                    end else if (rx_bits[3:0] == 4'b0011 || rx_bits[3:0] == 4'b1011) begin
                        // DATA0 / DATA1
                        if (real_bit_count >= 7'd24 &&                  // >=8 PID +0 payload +16 CRC
                            ((real_bit_count - 7'd8 - 7'd16) % 7'd8) == 7'd0 &&
                            (real_bit_count - 7'd8 - 7'd16) <= 7'd64) begin
                            pbits      <= real_bit_count - 7'd8 - 7'd16;
                            crc16_calc <= 16'hFFFF;
                            crc_i      <= 7'd0;
                            state      <= ST_CRC_LOOP;
                        end else begin
                            ok    <= 1'b0;
                            done  <= 1'b1;
                            busy  <= 1'b0;
                            payload_len_bytes <= 4'd0;
                            state <= ST_IDLE;
                        end
                    end else begin
                        // token PIDs / anything else: not expected as RX input here
                        ok    <= 1'b0;
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        payload_len_bytes <= 4'd0;
                        state <= ST_IDLE;
                    end
                end

                ST_CRC_LOOP: begin
                    if (crc_i == pbits) begin
                        ok                <= (rx_bits[8 + pbits +: 16] == ~crc16_calc);
                        payload           <= rx_bits[8 +: 64];
                        payload_len_bytes <= pbits[6:3];
                        done  <= 1'b1;
                        busy  <= 1'b0;
                        state <= ST_IDLE;
                    end else begin
                        fb16       = crc16_calc[15] ^ rx_bits[8 + crc_i];
                        crc16_calc <= {crc16_calc[14:0], 1'b0} ^ (fb16 ? 16'h8005 : 16'h0000);
                        crc_i      <= crc_i + 1'b1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
