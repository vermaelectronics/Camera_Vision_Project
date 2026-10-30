// ============================================================================
// pi_aj_cdc_bus.v  --  continuous multi-bit clock-domain crossing
//
// Four-phase-free toggle handshake: the source holds a snapshot of src_data
// while 'req' toggles; the destination captures it after a 2-FF synchronised
// toggle and acknowledges. A new snapshot is taken as soon as the ack returns,
// so dst_data follows src_data with a latency of a few clocks of each domain.
// Every bit of dst_data comes from one coherent snapshot.
// ============================================================================
`timescale 1ns / 1ps
module pi_aj_cdc_bus #(
    parameter W = 32,
    parameter [W-1:0] INIT = {W{1'b0}}
) (
    input  wire         src_clk,
    input  wire         src_rst,
    input  wire [W-1:0] src_data,
    input  wire         dst_clk,
    input  wire         dst_rst,
    output reg  [W-1:0] dst_data
);
    reg [W-1:0] hold;
    reg         req;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_s;
    reg         ack;
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_s;
    reg         req_d;

    always @(posedge src_clk) begin
        if (src_rst) begin
            hold  <= INIT;
            req   <= 1'b0;
            ack_s <= 2'b00;
        end else begin
            ack_s <= {ack_s[0], ack};
            if (ack_s[1] == req) begin
                hold <= src_data;
                req  <= ~req;
            end
        end
    end

    always @(posedge dst_clk) begin
        if (dst_rst) begin
            req_s    <= 2'b00;
            req_d    <= 1'b0;
            ack      <= 1'b0;
            dst_data <= INIT;
        end else begin
            req_s <= {req_s[0], req};
            req_d <= req_s[1];
            if (req_s[1] != req_d) begin
                dst_data <= hold;
                ack      <= req_s[1];
            end
        end
    end
endmodule
