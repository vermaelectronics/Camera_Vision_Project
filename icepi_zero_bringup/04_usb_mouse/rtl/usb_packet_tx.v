// ============================================================================
// usb_packet_tx.v -- builds and transmits one complete low-speed USB packet
// (SYNC + PID + payload + CRC + EOP), bit-stuffed and NRZI-encoded, driving
// D+/D- directly.
// ----------------------------------------------------------------------------
// The unstuffed logical bitstream (PID + payload) and its CRC5/CRC16 are
// computed in one combinational/sequential sweep the cycle `start` is
// asserted (a small, statically-bounded (<=88 real bits) unrolled loop --
// fine as plain combinational logic, since each real_bits[i] is written
// at most once). Bit-stuffing them into `stuffed[]`, though, is NOT done
// that way: an earlier version built the whole stuffed stream the same
// way, in one combinational sweep with a *variable* write index (si) --
// simulated fine (Icarus just executes it procedurally), but real
// synthesis (Yosys) had never actually been run against it until this
// module was integrated into a full top-level design, where it blew up
// into a five-thousand-plus-cell priority-mux chain (still growing when
// killed) -- every one of up to ~103 sequential variable-indexed writes to
// the same 128-bit register within a single clock edge needs its own
// "does this write win" mux over every prior write. ST_BUILD below spreads
// that same computation over one real (or stuff) bit per clock cycle
// instead -- exactly the plain, cheap "clocked write, counter-indexed"
// pattern usb_packet_rx.v already uses successfully for its own bit
// storage -- taking at most ~103 systemclock cycles (~2us), utterly
// negligible next to the many *microseconds* of bit-tick-paced sending
// that follows it.
//
// CRC5 (11-bit token payloads) and CRC16 (data payloads) both use the
// standard USB bit-serial algorithm: process bits LSB-first, feedback from
// the shift register's own MSB, generator polynomials 0x05 and 0x8005
// (the non-reflected forms -- correct for this LSB-first, left-shifting
// style), init all-ones, final result bit-inverted before transmission.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module usb_packet_tx (
    input  wire        clk,
    input  wire        rst,

    input  wire        tick,           // from usb_nco, ~1.5MHz average

    input  wire        start,          // pulse to begin
    input  wire [3:0]  pid,
    input  wire        is_token,       // 1 = token (11-bit payload, CRC5)
    input  wire        is_handshake,   // 1 = PID only, no payload, no CRC at all
                                        // (ACK/NAK/STALL -- takes priority over is_token)
    input  wire [10:0] token_payload,  // {endp[3:0], addr[6:0]}, bit0 sent first
    input  wire [63:0] data_payload,   // up to 8 bytes, byte0 = bits[7:0]
    input  wire [3:0]  data_len_bytes, // 0..8

    output reg          busy,
    output reg          done,           // one-cycle pulse

    output reg          dp_oe, output reg dp_o,
    output reg          dn_oe, output reg dn_o
);

    // ---- built once at `start` ----
    reg [87:0]  real_bits;       // PID + payload + CRC, unstuffed, bit0 sent first
    reg [6:0]   real_total;
    reg [127:0] stuffed;         // SYNC (unstuffed) ++ stuffed(real_bits)
    reg [7:0]   stuffed_total;

    integer i;
    reg [4:0] crc5;
    reg       fb5;
    reg [15:0] crc16;
    reg        fb16;

    // ---- ST_BUILD's own one-bit-per-cycle state ----
    reg [6:0] build_i;
    reg [7:0] build_si;
    reg [2:0] build_ones_run;
    reg       build_stuff_pending;

    localparam ST_IDLE  = 3'd0, ST_BUILD = 3'd1, ST_SEND = 3'd2, ST_EOP0 = 3'd3,
               ST_EOP1  = 3'd4, ST_EOPJ  = 3'd5, ST_DONE = 3'd6;
    reg [2:0] state;

    reg [7:0] ptr;
    reg       cur_line;  // 0 = J, 1 = K (NRZI logical line state)
    reg       next_line;

    always @(posedge clk) begin
        done <= 1'b0;

        if (rst) begin
            state    <= ST_IDLE;
            busy     <= 1'b0;
            cur_line <= 1'b0; // idle = J
            dp_oe    <= 1'b0;
            dn_oe    <= 1'b0;
        end else if (state == ST_IDLE) begin
            dp_oe <= 1'b0;
            dn_oe <= 1'b0;
            if (start) begin
                busy <= 1'b1;

                // PID: pid[0..3] then ~pid[0..3], send order
                real_bits[0] <= pid[0]; real_bits[1] <= pid[1];
                real_bits[2] <= pid[2]; real_bits[3] <= pid[3];
                real_bits[4] <= ~pid[0]; real_bits[5] <= ~pid[1];
                real_bits[6] <= ~pid[2]; real_bits[7] <= ~pid[3];

                if (is_handshake) begin
                    real_total <= 7'd8; // PID only -- no payload, no CRC at all
                end else if (is_token) begin
                    for (i = 0; i < 11; i = i + 1)
                        real_bits[8 + i] <= token_payload[i];

                    crc5 = 5'b11111;
                    for (i = 0; i < 11; i = i + 1) begin
                        fb5 = crc5[4] ^ token_payload[i];
                        crc5 = {crc5[3:0], 1'b0};
                        if (fb5) crc5 = crc5 ^ 5'b00101;
                    end
                    for (i = 0; i < 5; i = i + 1)
                        real_bits[19 + i] <= ~crc5[i];

                    real_total <= 7'd24; // 8(PID) + 11(payload) + 5(CRC5)
                end else begin
                    for (i = 0; i < 64; i = i + 1)
                        real_bits[8 + i] <= (i < data_len_bytes * 8) ? data_payload[i] : 1'b0;

                    crc16 = 16'hFFFF;
                    for (i = 0; i < 64; i = i + 1) begin
                        if (i < data_len_bytes * 8) begin
                            fb16 = crc16[15] ^ data_payload[i];
                            crc16 = {crc16[14:0], 1'b0};
                            if (fb16) crc16 = crc16 ^ 16'h8005;
                        end
                    end
                    for (i = 0; i < 16; i = i + 1)
                        real_bits[8 + data_len_bytes * 8 + i] <= ~crc16[i];

                    real_total <= 8 + data_len_bytes * 8 + 16;
                end

                // SYNC: 0,0,0,0,0,0,0,1 in send order -- a fixed constant,
                // written directly into stuffed[] (never stuffed itself).
                stuffed[7:0]        <= 8'b1000_0000;
                build_i             <= 7'd0;
                build_si            <= 8'd8;
                build_ones_run      <= 3'd0;
                build_stuff_pending <= 1'b0;
                state               <= ST_BUILD;
            end
        end else if (state == ST_BUILD) begin
            if (build_stuff_pending) begin
                stuffed[build_si]   <= 1'b0; // forced stuff bit
                build_si            <= build_si + 1'b1;
                build_stuff_pending <= 1'b0;
            end else if (build_i == real_total) begin
                stuffed_total <= build_si;
                ptr           <= 8'd0;
                cur_line      <= 1'b0; // idle/J until the first stuffed bit (a 0) toggles it
                dp_oe         <= 1'b1;
                dn_oe         <= 1'b1;
                dp_o          <= 1'b0; // J
                dn_o          <= 1'b1;
                state         <= ST_SEND;
            end else begin
                stuffed[build_si] <= real_bits[build_i];
                build_si          <= build_si + 1'b1;
                build_i           <= build_i + 1'b1;
                if (real_bits[build_i]) begin
                    if (build_ones_run == 3'd5) begin
                        build_ones_run      <= 3'd0;
                        build_stuff_pending <= 1'b1;
                    end else begin
                        build_ones_run <= build_ones_run + 1'b1;
                    end
                end else begin
                    build_ones_run <= 3'd0;
                end
            end
        end else if (state == ST_SEND) begin
            if (tick) begin
                // NRZI: 0 = toggle the line, 1 = hold it. next_line: 0=J, 1=K.
                next_line = stuffed[ptr] ? cur_line : ~cur_line;
                cur_line  <= next_line;
                dp_o      <= next_line;   // K: dp=1,dn=0 -- J: dp=0,dn=1 (low-speed polarity)
                dn_o      <= ~next_line;
                if (ptr + 1'b1 == stuffed_total) begin
                    state <= ST_EOP0;
                end else begin
                    ptr <= ptr + 1'b1;
                end
            end
        end else if (state == ST_EOP0) begin
            if (tick) begin
                dp_o <= 1'b0; dn_o <= 1'b0; // SE0, bit period 1
                state <= ST_EOP1;
            end
        end else if (state == ST_EOP1) begin
            if (tick) begin
                dp_o <= 1'b0; dn_o <= 1'b0; // SE0, bit period 2
                state <= ST_EOPJ;
            end
        end else if (state == ST_EOPJ) begin
            if (tick) begin
                dp_o  <= 1'b0; dn_o <= 1'b1; // J, bit period 3
                state <= ST_DONE;
            end
        end else if (state == ST_DONE) begin
            dp_oe <= 1'b0;
            dn_oe <= 1'b0;
            busy  <= 1'b0;
            done  <= 1'b1;
            state <= ST_IDLE;
        end
    end

endmodule

`default_nettype wire
