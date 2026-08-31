// ============================================================================
// usb_host_ctrl.v -- minimal low-speed USB host controller: bus reset,
// enumeration (SET_ADDRESS -> SET_CONFIGURATION -> SET_PROTOCOL boot), then
// continuous IN polling of a HID boot-protocol mouse's interrupt endpoint.
// ----------------------------------------------------------------------------
// Deliberately skips GET_DESCRIPTOR entirely -- this host doesn't need to
// identify *which* device is attached, only to talk to *a* low-speed HID
// mouse, so descriptor parsing (a meaningfully bigger undertaking: multi-
// packet reassembly, descriptor-type dispatch) is out of scope. All three
// enumeration requests used (SET_ADDRESS, SET_CONFIGURATION, SET_PROTOCOL)
// are no-data-stage control transfers (wLength=0): SETUP+DATA0(8 bytes),
// wait ACK, then a status-stage IN expecting a zero-length DATA1, ACked in
// turn -- the same two-stage shape reused three times with different SETUP
// payloads, via CTL_* states shared by all three requests.
//
// Requesting SET_PROTOCOL(boot) is what makes the fixed-format assumption
// below safe: HID boot protocol guarantees a mouse's IN reports are
// exactly {buttons[7:0], dx[7:0], dy[7:0], ...} -- byte 0 the button
// bitmap, bytes 1-2 signed X/Y deltas -- regardless of that device's own
// (unparsed, in this design) HID report descriptor.
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module usb_host_ctrl #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter [3:0]   DEV_ENDPOINT = 4'd1
) (
    input  wire        clk,
    input  wire        rst,

    output wire         dp_oe, output wire dp_o,
    output wire         dn_oe, output wire dn_o,
    input  wire          dp_i, dn_i,   // synchronized (2FF) by the caller

    output reg  [2:0]  link_state,   // 0=reset 1=enum_addr 2=enum_cfg 3=enum_proto 4=polling 5=error
    output reg          device_ready, // sticky: enumeration completed
    output reg          error,        // sticky: gave up (see link_state for which stage)

    output reg  [7:0]  buttons,
    output reg  [7:0]  dx,           // two's-complement
    output reg  [7:0]  dy,           // two's-complement
    output reg          report_valid  // one-cycle pulse: buttons/dx/dy just updated
);

    // ---- PHY: packet TX/RX + bit-rate tick ----
    wire tick;
    usb_nco #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_nco (.clk(clk), .rst(rst), .resync(1'b0), .tick(tick));

    reg        tx_start, tx_is_token, tx_is_handshake;
    reg [3:0]  tx_pid;
    reg [10:0] tx_token_payload;
    reg [63:0] tx_data_payload;
    reg [3:0]  tx_data_len;
    wire       tx_busy, tx_done;
    wire       phy_dp_oe, phy_dp_o, phy_dn_oe, phy_dn_o;

    usb_packet_tx u_tx (
        .clk(clk), .rst(rst), .tick(tick),
        .start(tx_start), .pid(tx_pid), .is_token(tx_is_token), .is_handshake(tx_is_handshake),
        .token_payload(tx_token_payload), .data_payload(tx_data_payload), .data_len_bytes(tx_data_len),
        .busy(tx_busy), .done(tx_done),
        .dp_oe(phy_dp_oe), .dp_o(phy_dp_o), .dn_oe(phy_dn_oe), .dn_o(phy_dn_o)
    );

    wire        rx_busy, rx_done, rx_ok;
    wire [3:0]  rx_pid;
    wire [63:0] rx_payload;
    wire [3:0]  rx_payload_len;

    usb_packet_rx #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_rx (
        .clk(clk), .rst(rst), .dp_i(dp_i), .dn_i(dn_i),
        .busy(rx_busy), .done(rx_done), .ok(rx_ok),
        .pid(rx_pid), .payload(rx_payload), .payload_len_bytes(rx_payload_len)
    );

    localparam [3:0] PID_OUT=4'b0001, PID_IN=4'b1001, PID_SOF=4'b0101, PID_SETUP=4'b1101,
                     PID_DATA0=4'b0011, PID_DATA1=4'b1011,
                     PID_ACK=4'b0010, PID_NAK=4'b1010, PID_STALL=4'b1110;

    // ---- reset drive: SE0 for >=10ms, then release and let the device settle ----
    reg        reset_drive;
    reg [26:0] reset_cnt;
    localparam integer RESET_HOLD_CYCLES    = (CLK_FREQ_HZ / 1000) * 15; // 15ms
    localparam integer RESET_RECOVER_CYCLES = (CLK_FREQ_HZ / 1000) * 10; // 10ms

    assign dp_oe = reset_drive ? 1'b1 : phy_dp_oe;
    assign dp_o  = reset_drive ? 1'b0 : phy_dp_o;
    assign dn_oe = reset_drive ? 1'b1 : phy_dn_oe;
    assign dn_o  = reset_drive ? 1'b0 : phy_dn_o;

    // ---- SETUP payloads for the three enumeration requests (LE bytes) ----
    localparam [63:0] SETUP_ADDR  = 64'h0000000000010500; // SET_ADDRESS(1)
    localparam [63:0] SETUP_CFG   = 64'h0000000000010900; // SET_CONFIGURATION(1)
    localparam [63:0] SETUP_PROTO = 64'h0000000000000B21; // SET_PROTOCOL(boot=0), interface class request

    reg [6:0] dev_addr;      // 0 until SET_ADDRESS's status stage completes, then 1
    reg [1:0] req_sel;       // 0=SET_ADDRESS 1=SET_CONFIGURATION 2=SET_PROTOCOL
    reg [1:0] data_toggle;   // DATA0/1 toggle for endpoint0 control transfers (unused meaningfully here: every SETUP starts DATA0, status IN always expects DATA1)
    reg [1:0] ep1_toggle;    // DATA0/1 toggle tracking for endpoint1 IN polling

    localparam [4:0]
        S_RESET_DRIVE   = 5'd0,
        S_RESET_RECOVER = 5'd1,
        S_CTL_SETUP_TOK = 5'd2,
        S_CTL_SETUP_DAT = 5'd3,
        S_CTL_SETUP_ACK = 5'd4,
        S_CTL_STAT_TOK  = 5'd5,
        S_CTL_STAT_WAIT = 5'd6,
        S_CTL_STAT_ACK  = 5'd7,
        S_CTL_NEXT      = 5'd8,
        S_POLL_TOK      = 5'd9,
        S_POLL_WAIT      = 5'd10,
        S_POLL_ACK       = 5'd11,
        S_ERROR          = 5'd12;

    reg [4:0]  state;
    reg [15:0] retry_cnt;
    localparam integer MAX_RETRY = 16'd20000; // generous: real devices can NAK for a while

    reg [63:0] cur_setup;

    always @(*) begin
        case (req_sel)
            2'd0: cur_setup = SETUP_ADDR;
            2'd1: cur_setup = SETUP_CFG;
            default: cur_setup = SETUP_PROTO;
        endcase
    end

    always @(posedge clk) begin
        tx_start     <= 1'b0;
        report_valid <= 1'b0;

        if (rst) begin
            state        <= S_RESET_DRIVE;
            reset_drive  <= 1'b1;
            reset_cnt    <= 27'd0;
            dev_addr     <= 7'd0;
            req_sel      <= 2'd0;
            device_ready <= 1'b0;
            error        <= 1'b0;
            link_state   <= 3'd0;
            retry_cnt    <= 16'd0;
            ep1_toggle   <= 2'd0;
        end else begin
            case (state)
                S_RESET_DRIVE: begin
                    link_state <= 3'd0;
                    if (reset_cnt == RESET_HOLD_CYCLES - 1) begin
                        reset_drive <= 1'b0;
                        reset_cnt   <= 27'd0;
                        state       <= S_RESET_RECOVER;
                    end else begin
                        reset_cnt <= reset_cnt + 1'b1;
                    end
                end

                S_RESET_RECOVER: begin
                    if (reset_cnt == RESET_RECOVER_CYCLES - 1) begin
                        req_sel   <= 2'd0;
                        retry_cnt <= 16'd0;
                        state     <= S_CTL_SETUP_TOK;
                        link_state<= 3'd1;
                    end else begin
                        reset_cnt <= reset_cnt + 1'b1;
                    end
                end

                // ---- control transfer: SETUP token ----
                S_CTL_SETUP_TOK: begin
                    if (!tx_busy) begin
                        tx_pid           <= PID_SETUP;
                        tx_is_token      <= 1'b1;
                        tx_is_handshake  <= 1'b0;
                        tx_token_payload <= {4'd0, dev_addr}; // endp0
                        tx_start         <= 1'b1;
                        state            <= S_CTL_SETUP_DAT;
                    end
                end

                S_CTL_SETUP_DAT: begin
                    if (tx_done) begin
                        tx_pid          <= PID_DATA0;
                        tx_is_token     <= 1'b0;
                        tx_is_handshake <= 1'b0;
                        tx_data_payload <= cur_setup;
                        tx_data_len     <= 4'd8;
                        tx_start        <= 1'b1;
                        state           <= S_CTL_SETUP_ACK;
                    end
                end

                S_CTL_SETUP_ACK: begin
                    if (tx_done && !rx_busy) begin
                        // wait for the device's handshake to the DATA0 we just sent
                    end
                    if (rx_done) begin
                        if (rx_ok && rx_pid == PID_ACK) begin
                            state <= S_CTL_STAT_TOK;
                        end else if (retry_cnt == MAX_RETRY) begin
                            state <= S_ERROR;
                        end else begin
                            retry_cnt <= retry_cnt + 1'b1;
                            state     <= S_CTL_SETUP_TOK; // retry the whole SETUP stage
                        end
                    end
                end

                // ---- control transfer: status stage (IN, expect a ZLP) ----
                S_CTL_STAT_TOK: begin
                    if (!tx_busy && !rx_busy) begin
                        tx_pid           <= PID_IN;
                        tx_is_token      <= 1'b1;
                        tx_is_handshake  <= 1'b0;
                        tx_token_payload <= {4'd0, dev_addr};
                        tx_start         <= 1'b1;
                        state            <= S_CTL_STAT_WAIT;
                    end
                end

                S_CTL_STAT_WAIT: begin
                    if (rx_done) begin
                        if (rx_ok && (rx_pid == PID_DATA0 || rx_pid == PID_DATA1)) begin
                            state <= S_CTL_STAT_ACK;
                        end else if (retry_cnt == MAX_RETRY) begin
                            state <= S_ERROR;
                        end else begin
                            retry_cnt <= retry_cnt + 1'b1;
                            state     <= S_CTL_STAT_TOK; // NAK or garbled -- retry the IN
                        end
                    end
                end

                S_CTL_STAT_ACK: begin
                    if (!tx_busy) begin
                        tx_pid          <= PID_ACK;
                        tx_is_handshake <= 1'b1;
                        tx_is_token     <= 1'b0;
                        tx_start        <= 1'b1;
                        state           <= S_CTL_NEXT;
                    end
                end

                S_CTL_NEXT: begin
                    if (tx_done) begin
                        retry_cnt <= 16'd0;
                        if (req_sel == 2'd0) begin
                            dev_addr   <= 7'd1; // SET_ADDRESS's status stage just completed
                            req_sel    <= 2'd1;
                            link_state <= 3'd2;
                            state      <= S_CTL_SETUP_TOK;
                        end else if (req_sel == 2'd1) begin
                            req_sel    <= 2'd2;
                            link_state <= 3'd3;
                            state      <= S_CTL_SETUP_TOK;
                        end else begin
                            device_ready <= 1'b1;
                            link_state   <= 3'd4;
                            ep1_toggle   <= 2'd0;
                            state        <= S_POLL_TOK;
                        end
                    end
                end

                // ---- steady state: poll the mouse's interrupt endpoint ----
                S_POLL_TOK: begin
                    if (!tx_busy && !rx_busy) begin
                        tx_pid           <= PID_IN;
                        tx_is_token      <= 1'b1;
                        tx_is_handshake  <= 1'b0;
                        tx_token_payload <= {DEV_ENDPOINT, dev_addr};
                        tx_start         <= 1'b1;
                        state            <= S_POLL_WAIT;
                    end
                end

                S_POLL_WAIT: begin
                    if (rx_done) begin
                        if (rx_ok && (rx_pid == PID_DATA0 || rx_pid == PID_DATA1) &&
                            rx_payload_len >= 4'd3) begin
                            buttons      <= rx_payload[7:0];
                            dx           <= rx_payload[15:8];
                            dy           <= rx_payload[23:16];
                            report_valid <= 1'b1;
                            state        <= S_POLL_ACK;
                        end else begin
                            // NAK (nothing new) or a malformed/short packet -- just
                            // poll again; a mouse legitimately NAKs constantly
                            // between real movement/button events.
                            state <= S_POLL_TOK;
                        end
                    end
                end

                S_POLL_ACK: begin
                    if (!tx_busy) begin
                        tx_pid          <= PID_ACK;
                        tx_is_handshake <= 1'b1;
                        tx_is_token     <= 1'b0;
                        tx_start        <= 1'b1;
                        state           <= S_POLL_TOK;
                    end
                end

                S_ERROR: begin
                    error      <= 1'b1;
                    link_state <= 3'd5;
                    // stays here until rst -- a real product would retry the
                    // whole reset/enumeration sequence on a fresh attach
                    // (device-detect logic), out of scope for this bring-up demo
                end

                default: state <= S_ERROR;
            endcase
        end
    end

endmodule

`default_nettype wire
