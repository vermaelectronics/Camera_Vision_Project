//-----------------------------------------------------------------------------
// stap_beamformer : space-time adaptive filter  y(n) = w^H x(n)
//
//   x(n) : M x N tapped delay lines, element i = m*N + t holds
//          y_m(n - (N-1) + t)  (t = N-1 is the newest sample)
//   w    : fixed point weights (WW bits, WFRAC fraction bits), double
//          buffered - a full new set is written to the shadow bank and made
//          active atomically with w_swap, so the output never sees a
//          partially updated weight vector.
//
//   Fully parallel: M*N complex multipliers + pipelined adder tree, one
//   output per input sample (latency 4 + log2(M*N) cycles).
//-----------------------------------------------------------------------------
module stap_beamformer #(
    parameter integer M     = 7,
    parameter integer N     = 16,
    parameter integer DW    = 16,
    parameter integer WW    = 18,
    parameter integer WFRAC = 16,
    parameter integer OW    = 16,
    // derived - do not override
    parameter integer IW    = $clog2(M * N + 1)
) (
    input  wire                 clk,
    input  wire                 rst_n,
    // input stream, channel m = s_data[m*2*DW +: 2*DW] = {Q, I}
    input  wire                 s_valid,
    input  wire [M*2*DW-1:0]    s_data,
    // output stream {Q, I}
    output reg                  m_valid,
    output reg  [2*OW-1:0]      m_data,
    // configuration
    input  wire [5:0]           out_shift,
    // shadow weight write + swap
    input  wire                 w_we,
    input  wire [IW-1:0]        w_addr,
    input  wire [WW-1:0]        w_re,
    input  wire [WW-1:0]        w_im,
    input  wire                 w_swap
);

    localparam integer NDIM = M * N;
    localparam integer LV   = $clog2(NDIM);          // adder tree depth
    localparam integer NP   = 1 << LV;               // padded leaf count
    localparam integer PW   = DW + WW + 1;           // combined product width
    localparam integer AW   = PW + LV;               // accumulator width

    integer i;

    //------------------------------------------------------- weights
    reg signed [WW-1:0] wa_re [0:NDIM-1];
    reg signed [WW-1:0] wa_im [0:NDIM-1];
    reg signed [WW-1:0] ws_re [0:NDIM-1];
    reg signed [WW-1:0] ws_im [0:NDIM-1];

    always @(posedge clk) begin
        if (w_we) begin
            ws_re[w_addr] <= w_re;
            ws_im[w_addr] <= w_im;
        end
    end

    // active bank: reset to w = e1 (reference element pass-through)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NDIM; i = i + 1) begin
                wa_re[i] <= (i == 0) ? (1 << WFRAC) : 0;
                wa_im[i] <= 0;
            end
        end else if (w_swap) begin
            for (i = 0; i < NDIM; i = i + 1) begin
                wa_re[i] <= ws_re[i];
                wa_im[i] <= ws_im[i];
            end
        end
    end

    //------------------------------------------------------- tapped delay lines
    reg signed [DW-1:0] x_re [0:NDIM-1];
    reg signed [DW-1:0] x_im [0:NDIM-1];
    integer m, t;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < NDIM; i = i + 1) begin
                x_re[i] <= {DW{1'b0}};
                x_im[i] <= {DW{1'b0}};
            end
        end else if (s_valid) begin
            for (m = 0; m < M; m = m + 1) begin
                for (t = 0; t < N - 1; t = t + 1) begin
                    x_re[m*N + t] <= x_re[m*N + t + 1];
                    x_im[m*N + t] <= x_im[m*N + t + 1];
                end
                x_re[m*N + N - 1] <= s_data[m*2*DW +: DW];
                x_im[m*N + N - 1] <= s_data[m*2*DW + DW +: DW];
            end
        end
    end

    //------------------------------------------------------- products
    // conj(w) * x = (wr*xr + wi*xi) + j(wr*xi - wi*xr)
    reg signed [DW+WW-1:0] p_rr [0:NDIM-1];
    reg signed [DW+WW-1:0] p_ii [0:NDIM-1];
    reg signed [DW+WW-1:0] p_ri [0:NDIM-1];
    reg signed [DW+WW-1:0] p_ir [0:NDIM-1];
    reg signed [AW-1:0]    nr   [0:2*NP-2];          // adder tree (heap order)
    reg signed [AW-1:0]    ni   [0:2*NP-2];

    always @(posedge clk) begin
        for (i = 0; i < NDIM; i = i + 1) begin
            p_rr[i] <= wa_re[i] * x_re[i];
            p_ii[i] <= wa_im[i] * x_im[i];
            p_ri[i] <= wa_re[i] * x_im[i];
            p_ir[i] <= wa_im[i] * x_re[i];
        end
        // leaves
        for (i = 0; i < NP; i = i + 1) begin
            if (i < NDIM) begin
                nr[NP-1+i] <= p_rr[i] + p_ii[i];
                ni[NP-1+i] <= p_ri[i] - p_ir[i];
            end else begin
                nr[NP-1+i] <= 0;
                ni[NP-1+i] <= 0;
            end
        end
        // internal nodes, one register per level
        for (i = 0; i < NP - 1; i = i + 1) begin
            nr[i] <= nr[2*i+1] + nr[2*i+2];
            ni[i] <= ni[2*i+1] + ni[2*i+2];
        end
    end

    //------------------------------------------------------- output scaling
    localparam integer LAT = 3 + LV;                 // TDL, prod, leaf, tree
    reg [LAT-1:0] vpipe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vpipe <= {LAT{1'b0}};
        else        vpipe <= {vpipe[LAT-2:0], s_valid};
    end

    localparam signed [AW:0] OMAX = (1 <<< (OW - 1)) - 1;
    localparam signed [AW:0] OMIN = -(1 <<< (OW - 1));

    // round half up, arithmetic shift right, saturate to OW bits
    function [OW-1:0] rnd_sat;
        input signed [AW-1:0] v;
        input [5:0]           sh;
        reg signed [AW:0]     vv, half, r;
        begin
            vv   = v;
            half = (sh == 6'd0) ? {(AW+1){1'b0}} : ({{AW{1'b0}}, 1'b1} << (sh - 1));
            r    = (vv + half) >>> sh;
            if (r > OMAX)
                rnd_sat = OMAX[OW-1:0];
            else if (r < OMIN)
                rnd_sat = OMIN[OW-1:0];
            else
                rnd_sat = r[OW-1:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_valid <= 1'b0;
            m_data  <= {2*OW{1'b0}};
        end else begin
            m_valid <= vpipe[LAT-1];
            m_data  <= {rnd_sat(ni[0], out_shift), rnd_sat(nr[0], out_shift)};
        end
    end

endmodule
