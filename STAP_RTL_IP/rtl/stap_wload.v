//-----------------------------------------------------------------------------
// stap_wload : fp32 weight vector -> fixed point beamformer weights
//
//   Pass 1 : scan all weights, find the largest exponent Emax
//   Pass 2 : convert every component with the common block exponent
//            k = 126 - Emax, so max|Re|,|Im| lands in [0.5, 1.0)
//   Then pulse w_swap to activate the new set in the beamformer.
//   The applied exponent k is reported (output gain = 2^k).
//-----------------------------------------------------------------------------
module stap_wload #(
    parameter integer NDIM  = 112,
    parameter integer WW    = 18,
    parameter integer WFRAC = 16,
    // derived - do not override
    parameter integer IW    = $clog2(NDIM + 1)
) (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              start,
    output reg               busy,
    output reg               done,
    // weight vector source (1-cycle read latency)
    output reg  [IW-1:0]     x_raddr,
    input  wire [63:0]       x_rdata,
    // beamformer shadow bank
    output reg               w_we,
    output reg  [IW-1:0]     w_addr,
    output wire [WW-1:0]     w_re,
    output wire [WW-1:0]     w_im,
    output reg               w_swap,
    output reg  signed [9:0] wexp
);

    localparam [1:0] S_IDLE = 2'd0, S_SCAN = 2'd1, S_CONV = 2'd2, S_SWAP = 2'd3;

    reg [1:0]    state;
    reg [IW:0]   cnt;
    reg          rv, dv;            // address presented / data valid
    reg [IW-1:0] da;                // address belonging to x_rdata
    reg [7:0]    emax;
    wire [7:0]   e_hi;

    wire [7:0] e_re = x_rdata[30:23];
    wire [7:0] e_im = x_rdata[62:55];
    assign e_hi = (e_im > e_re) ? e_im : e_re;

    stap_fp2fix #(.W(WW), .FRAC(WFRAC)) u_cre (.clk(clk), .f(x_rdata[31:0]),  .k(wexp), .y(w_re));
    stap_fp2fix #(.W(WW), .FRAC(WFRAC)) u_cim (.clk(clk), .f(x_rdata[63:32]), .k(wexp), .y(w_im));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            busy    <= 1'b0;
            done    <= 1'b0;
            cnt     <= {(IW+1){1'b0}};
            x_raddr <= {IW{1'b0}};
            rv      <= 1'b0;
            dv      <= 1'b0;
            da      <= {IW{1'b0}};
            emax    <= 8'd0;
            w_we    <= 1'b0;
            w_addr  <= {IW{1'b0}};
            w_swap  <= 1'b0;
            wexp    <= 10'sd0;
        end else begin
            done   <= 1'b0;
            w_swap <= 1'b0;
            rv     <= 1'b0;
            dv     <= rv;
            da     <= x_raddr;
            w_we   <= dv && (state == S_CONV);
            w_addr <= da;

            // pass 1: running maximum of the exponent fields
            if (dv && state == S_SCAN && e_hi > emax)
                emax <= e_hi;

            case (state)
                S_IDLE: if (start) begin
                    busy    <= 1'b1;
                    emax    <= 8'd0;
                    cnt     <= {(IW+1){1'b0}};
                    x_raddr <= {IW{1'b0}};
                    state   <= S_SCAN;
                end

                S_SCAN: begin
                    if (cnt < NDIM) begin
                        rv      <= 1'b1;
                        x_raddr <= cnt[IW-1:0];
                        cnt     <= cnt + 1'b1;
                    end else if (!rv && !dv) begin
                        wexp  <= (emax == 8'd0) ? 10'sd0 : (10'sd126 - $signed({2'b00, emax}));
                        cnt   <= {(IW+1){1'b0}};
                        state <= S_CONV;
                    end
                end

                S_CONV: begin
                    if (cnt < NDIM) begin
                        rv      <= 1'b1;
                        x_raddr <= cnt[IW-1:0];
                        cnt     <= cnt + 1'b1;
                    end else if (!rv && !dv && !w_we) begin
                        state <= S_SWAP;
                    end
                end

                S_SWAP: begin
                    w_swap <= 1'b1;
                    busy   <= 1'b0;
                    done   <= 1'b1;
                    state  <= S_IDLE;
                end
            endcase
        end
    end

endmodule
