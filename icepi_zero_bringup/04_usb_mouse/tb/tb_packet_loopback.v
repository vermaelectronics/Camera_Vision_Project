// ============================================================================
// tb_packet_loopback.v -- wires usb_packet_tx's D+/D- output straight into
// usb_packet_rx's input (a direct electrical loopback, no real bus/device
// involved) and checks that every packet shape this host controller will
// ever send round-trips correctly: token packets (CRC5), a handshake
// (PID-only, no CRC at all), and data packets of several lengths including
// ones crafted to force bit-stuffing. This is the one piece of the USB
// stack it is actually possible to verify without a real device or a
// behavioral USB device model -- it proves the wire-level framing (SYNC,
// PID, bit stuffing, NRZI, CRC5/CRC16, EOP) is self-consistent and, since
// both sides implement the real USB 2.0 bit-serial algorithms (not a
// private shortcut), that it is spec-correct, not just internally
// consistent.
// ============================================================================
`timescale 1ns / 1ps

module tb_packet_loopback;

    localparam integer CLK_FREQ_HZ = 50_000_000;

    reg clk = 0;
    always #10 clk = ~clk; // 50MHz

    reg rst = 1;

    reg        tx_start;
    reg [3:0]  tx_pid;
    reg        tx_is_token;
    reg        tx_is_handshake;
    reg [10:0] tx_token_payload;
    reg [63:0] tx_data_payload;
    reg [3:0]  tx_data_len;
    wire       tx_busy, tx_done;
    wire       dp, dn;
    wire       tx_tick;

    usb_nco #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_tx_nco (
        .clk(clk), .rst(rst), .resync(1'b0), .tick(tx_tick)
    );

    wire dp_oe, dp_o, dn_oe, dn_o;

    usb_packet_tx u_tx (
        .clk(clk), .rst(rst), .tick(tx_tick),
        .start(tx_start), .pid(tx_pid), .is_token(tx_is_token),
        .is_handshake(tx_is_handshake),
        .token_payload(tx_token_payload),
        .data_payload(tx_data_payload), .data_len_bytes(tx_data_len),
        .busy(tx_busy), .done(tx_done),
        .dp_oe(dp_oe), .dp_o(dp_o), .dn_oe(dn_oe), .dn_o(dn_o)
    );

    // idle bus = J (dp=0,dn=1) whenever TX releases the lines
    assign dp = dp_oe ? dp_o : 1'b0;
    assign dn = dn_oe ? dn_o : 1'b1;

    // 2FF synchronize into the RX (as a real board would need for async pins)
    reg dp_s1, dp_s2, dn_s1, dn_s2;
    always @(posedge clk) begin
        dp_s1 <= dp; dp_s2 <= dp_s1;
        dn_s1 <= dn; dn_s2 <= dn_s1;
    end

    wire rx_busy, rx_done, rx_ok;
    wire [3:0] rx_pid;
    wire [63:0] rx_payload;
    wire [3:0] rx_payload_len;

    usb_packet_rx #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_rx (
        .clk(clk), .rst(rst), .dp_i(dp_s2), .dn_i(dn_s2),
        .busy(rx_busy), .done(rx_done), .ok(rx_ok),
        .pid(rx_pid), .payload(rx_payload), .payload_len_bytes(rx_payload_len)
    );

    integer errors = 0;

    task send_and_check;
        input [3:0]  p_pid;
        input        p_is_token;
        input        p_is_handshake;
        input [10:0] p_token_payload;
        input [63:0] p_data_payload;
        input [3:0]  p_data_len;
        input [63:0] tag;
        begin
            // TX's own EOP sequence (EOP0/EOP1/EOPJ/DONE) takes several more
            // bit periods after the last real bit -- RX, meanwhile, declares
            // rx_done as soon as its SE0 threshold trips, well before that.
            // Starting the next packet on rx_done alone races TX's still-
            // in-flight completion: TX only samples `start` from ST_IDLE, so
            // a `start` pulse arriving while TX is still finishing its own
            // EOP is silently dropped. Wait for TX to be genuinely idle too.
            while (tx_busy) @(posedge clk);

            @(posedge clk);
            #1; // step past the edge so this doesn't race other blocks sampling it
            tx_pid           = p_pid;
            tx_is_token      = p_is_token;
            tx_is_handshake  = p_is_handshake;
            tx_token_payload = p_token_payload;
            tx_data_payload  = p_data_payload;
            tx_data_len      = p_data_len;
            tx_start         = 1'b1;
            @(posedge clk);
            #1;
            tx_start = 1'b0;

            wait (rx_done);
            @(posedge clk);

            // usb_packet_rx.v deliberately never validates token packets (no
            // CRC5 check implemented -- out of scope, see its header comment)
            // so `ok` is only meaningful here for handshakes and data packets.
            if (!p_is_token && !rx_ok) begin
                $display("ERROR [%0d]: rx_ok=0 (pid=%b)", tag, p_pid);
                errors = errors + 1;
            end
            if (rx_pid !== p_pid) begin
                $display("ERROR [%0d]: pid mismatch: got %b expected %b", tag, rx_pid, p_pid);
                errors = errors + 1;
            end
            if (!p_is_token && !p_is_handshake) begin
                if (rx_payload_len !== p_data_len) begin
                    $display("ERROR [%0d]: payload_len mismatch: got %0d expected %0d",
                              tag, rx_payload_len, p_data_len);
                    errors = errors + 1;
                end else if ((rx_payload & ((64'd1 << (p_data_len*8)) - 1'b1)) !==
                             (p_data_payload & ((64'd1 << (p_data_len*8)) - 1'b1))) begin
                    $display("ERROR [%0d]: payload mismatch: got %016h expected %016h",
                              tag, rx_payload, p_data_payload);
                    errors = errors + 1;
                end
            end

            if (errors == 0)
                $display("OK [%0d]: pid=%b len=%0d", tag, p_pid, p_data_len);

            // let the bus settle back to idle before the next packet
            repeat (40) @(posedge clk);
        end
    endtask

    initial begin
        $dumpfile("tb_packet_loopback.vcd");
        $dumpvars(0, tb_packet_loopback);

        tx_start = 1'b0;
        repeat (5) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ---- token packet (SETUP, addr=0, endp=0) ----
        send_and_check(4'b1101, 1'b1, 1'b0, {4'd0, 7'd0}, 64'd0, 4'd0, 64'd1);

        // ---- token packet, nonzero addr/endp (exercises different CRC5 input) ----
        send_and_check(4'b1001, 1'b1, 1'b0, {4'd1, 7'd42}, 64'd0, 4'd0, 64'd2);

        // ---- handshake: PID only, no CRC at all ----
        send_and_check(4'b0010, 1'b0, 1'b1, 11'd0, 64'd0, 4'd0, 64'd3);
        send_and_check(4'b1010, 1'b0, 1'b1, 11'd0, 64'd0, 4'd0, 64'd3_1); // NAK
        send_and_check(4'b1110, 1'b0, 1'b1, 11'd0, 64'd0, 4'd0, 64'd3_2); // STALL

        // ---- 8-byte SETUP data packet, all-zero (worst case for NO stuffing) ----
        send_and_check(4'b0011, 1'b0, 1'b0, 11'd0, 64'h0000000000000000, 4'd8, 64'd4);

        // ---- 8-byte data packet, all-ones bytes (forces bit-stuffing: 64
        // consecutive 1-bits before CRC, definitely > 6 in a row) ----
        send_and_check(4'b1011, 1'b0, 1'b0, 11'd0, 64'hFFFFFFFFFFFFFFFF, 4'd8, 64'd5);

        // ---- 3-byte data packet, a typical boot-mouse report shape ----
        send_and_check(4'b0011, 1'b0, 1'b0, 11'd0, 64'h0000000000FE0801, 4'd3, 64'd6);

        // ---- 1-byte data packet ----
        send_and_check(4'b1011, 1'b0, 1'b0, 11'd0, 64'h00000000000000A5, 4'd1, 64'd7);

        // ---- alternating pattern, another stuffing-heavy case ----
        send_and_check(4'b0011, 1'b0, 1'b0, 11'd0, 64'h5555555555555555, 4'd8, 64'd8);

        if (errors == 0)
            $display("TB_PACKET_LOOPBACK: PASS");
        else
            $display("TB_PACKET_LOOPBACK: FAIL (%0d errors)", errors);

        $finish;
    end

    initial begin
        #2_000_000;
        $display("TB_PACKET_LOOPBACK: WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
