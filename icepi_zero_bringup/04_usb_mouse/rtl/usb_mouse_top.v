// ============================================================================
// usb_mouse_top.v -- IcePi-Zero bring-up #4: bit-banged low-speed USB host,
// reads a HID boot-protocol mouse's button/X/Y deltas and reports them over
// UART (and a rough-activity LED), with no external USB PHY chip -- the
// board's own raw D+/D- pins plus its switchable pull-down resistors are
// enough for low-speed (1.5Mbit/s) host operation.
// ----------------------------------------------------------------------------
// Highest-risk sub-project in this collection: the wire-level packet
// framing (SYNC/NRZI/bit-stuffing/CRC5/CRC16/EOP, in usb_packet_tx.v /
// usb_packet_rx.v) is verified by a real electrical TX->RX loopback
// testbench (tb_packet_loopback.v). The enumeration + HID-polling sequence
// on top of that (usb_host_ctrl.v) is real RTL written to the USB 2.0 spec
// but is NOT simulated against a behavioral USB device model -- no such
// model exists in this session, and building an accurate one is its own
// substantial undertaking. See the sub-project README's "Verified" section
// for the exact boundary of what is and isn't confirmed here.
//
// LEDs: [0]=device_ready (sticky, enumeration completed) [1]=activity
// (toggles on every report_valid pulse) [2]=error (sticky) [3]=unused
// [4]=1Hz heartbeat. button[0]=reset (re-run the whole reset+enumeration
// sequence -- use this after plugging in a mouse).
// ============================================================================
`timescale 1ns / 1ps
`default_nettype none

module usb_mouse_top #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer BAUD        = 115200
) (
    input  wire        clk,        // 50MHz board oscillator (site M1)
    input  wire  [1:0] button,     // active-low; [0]=reset
    output wire  [4:0] led,
    output wire         uart_txd,

    inout  wire         usb_dp,     // site F15
    inout  wire         usb_dn,     // site E16
    output wire         usb_pull_dp, // site G15 -- board's switchable D+ pull-down enable
    output wire         usb_pull_dn  // site H14 -- board's switchable D- pull-down enable
);

    wire rst = ~button[0];

    // Host role: pull DOWN both D+ and D- (marks this end as a host port,
    // per the USB spec's asymmetric host/device pull resistor convention --
    // it's the attached device that pulls one line up to signal its speed).
    // ASSUMPTION, not yet hardware-confirmed: these two board pins are
    // active-high enables for onboard pull-down resistors -- verify against
    // the board schematic before relying on this; see README.
    assign usb_pull_dp = 1'b1;
    assign usb_pull_dn = 1'b1;

    // ---- tri-state D+/D- ----
    wire dp_oe, dp_o, dn_oe, dn_o;
    assign usb_dp = dp_oe ? dp_o : 1'bz;
    assign usb_dn = dn_oe ? dn_o : 1'bz;

    reg dp_s1, dp_s2, dn_s1, dn_s2;
    always @(posedge clk) begin
        dp_s1 <= usb_dp; dp_s2 <= dp_s1;
        dn_s1 <= usb_dn; dn_s2 <= dn_s1;
    end

    wire [2:0] link_state;
    wire       device_ready, error;
    wire [7:0] buttons, dx, dy;
    wire       report_valid;

    usb_host_ctrl #(.CLK_FREQ_HZ(CLK_FREQ_HZ)) u_host (
        .clk(clk), .rst(rst),
        .dp_oe(dp_oe), .dp_o(dp_o), .dn_oe(dn_oe), .dn_o(dn_o),
        .dp_i(dp_s2), .dn_i(dn_s2),
        .link_state(link_state), .device_ready(device_ready), .error(error),
        .buttons(buttons), .dx(dx), .dy(dy), .report_valid(report_valid)
    );

    // ---- report -> UART: 3 raw bytes per report (buttons, dx, dy) ----
    reg [1:0] report_byte_idx;
    reg       report_sending;
    reg [7:0] report_buf [0:2];
    wire      uart_ready;
    reg       uart_valid_r;
    reg [7:0] uart_data_r;

    always @(posedge clk) begin
        uart_valid_r <= 1'b0;
        if (rst) begin
            report_sending <= 1'b0;
        end else begin
            if (report_valid && !report_sending) begin
                report_buf[0]   <= buttons;
                report_buf[1]   <= dx;
                report_buf[2]   <= dy;
                report_byte_idx <= 2'd0;
                report_sending  <= 1'b1;
            end else if (report_sending && uart_ready && !uart_valid_r) begin
                uart_data_r  <= report_buf[report_byte_idx];
                uart_valid_r <= 1'b1;
                if (report_byte_idx == 2'd2) begin
                    report_sending <= 1'b0;
                end else begin
                    report_byte_idx <= report_byte_idx + 1'b1;
                end
            end
        end
    end

    uart_tx #(.CLK_FREQ_HZ(CLK_FREQ_HZ), .BAUD(BAUD)) u_uart (
        .clk(clk), .rst(rst),
        .data(uart_data_r), .valid(uart_valid_r), .ready(uart_ready),
        .tx(uart_txd)
    );

    // ---- status LEDs ----
    reg act_led;
    always @(posedge clk) begin
        if (rst) act_led <= 1'b0;
        else if (report_valid) act_led <= ~act_led;
    end

    localparam integer HALF_PERIOD = CLK_FREQ_HZ / 2;
    reg [$clog2(HALF_PERIOD)-1:0] hb_cnt;
    reg hb_r;
    always @(posedge clk) begin
        if (rst) begin
            hb_cnt <= 0; hb_r <= 1'b0;
        end else if (hb_cnt == HALF_PERIOD - 1) begin
            hb_cnt <= 0; hb_r <= ~hb_r;
        end else begin
            hb_cnt <= hb_cnt + 1'b1;
        end
    end

    assign led[0] = device_ready;
    assign led[1] = act_led;
    assign led[2] = error;
    assign led[3] = 1'b0;
    assign led[4] = hb_r;

endmodule

`default_nettype wire
