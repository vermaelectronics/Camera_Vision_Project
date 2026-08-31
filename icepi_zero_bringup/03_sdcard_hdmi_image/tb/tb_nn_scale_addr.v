// ============================================================================
// tb_nn_scale_addr.v -- checks small_x combinationally across every column
// of a 1280-wide row, and small_y across every row-boundary event of one
// full 720-row frame (cheap: only 720 clocked events, not 921,600 pixels).
// ============================================================================
`timescale 1ns / 1ps

module tb_nn_scale_addr;

    localparam integer IMG_W = 160;
    localparam integer IMG_H = 120;

    reg clk = 0;
    always #5 clk = ~clk;

    reg rst = 1;
    reg de;
    reg [15:0] x, y;
    wire [7:0] small_x, small_y;

    nn_scale_addr #(.X_SHIFT(3), .Y_SCALE(720 / IMG_H)) dut (
        .clk(clk), .rst(rst), .de(de), .x(x), .y(y),
        .small_x(small_x), .small_y(small_y)
    );

    integer errors = 0;
    integer xi, yi;

    initial begin
        de = 1'b0; x = 16'd0; y = 16'd0;
        repeat (3) @(posedge clk);
        rst = 0;
        @(posedge clk);

        // ---- small_x: purely combinational, check every column ----
        de = 1'b1;
        for (xi = 0; xi < 1280; xi = xi + 1) begin
            x = xi[15:0];
            #1;
            if (small_x !== (xi / 8)) begin
                $display("ERROR: small_x wrong at x=%0d: got %0d expected %0d", xi, small_x, xi/8);
                errors = errors + 1;
            end
        end

        // ---- small_y: clock through one full frame's row boundaries ----
        for (yi = 0; yi < 720; yi = yi + 1) begin
            y = yi[15:0];
            x = 16'd0;
            @(posedge clk);
            #1;
            if (small_y !== (yi / 6)) begin
                $display("ERROR: small_y wrong at y=%0d: got %0d expected %0d", yi, small_y, yi/6);
                errors = errors + 1;
            end
        end

        // ---- frame wrap: next frame's first row should reset small_y to 0 ----
        y = 16'd0;
        x = 16'd0;
        @(posedge clk);
        #1;
        if (small_y !== 8'd0) begin
            $display("ERROR: small_y did not reset at start of next frame (got %0d)", small_y);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_NN_SCALE_ADDR: PASS");
        else
            $display("TB_NN_SCALE_ADDR: FAIL (%0d errors)", errors);

        $finish;
    end

endmodule
