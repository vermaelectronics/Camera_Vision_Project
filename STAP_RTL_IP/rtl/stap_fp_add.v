//-----------------------------------------------------------------------------
// stap_fp_add : IEEE-754 single precision adder, 2-cycle pipelined
//   y = a + b
//   - round to nearest, ties to even
//   - denormals flushed to zero (inputs and outputs)
//   - overflow saturates to +/-Inf, NaN not supported
//-----------------------------------------------------------------------------
module stap_fp_add (
    input  wire        clk,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] y
);

    function [4:0] lzc27;
        input [26:0] v;
        integer k;
        reg found;
        begin
            lzc27 = 5'd27;
            found = 1'b0;
            for (k = 26; k >= 0; k = k - 1) begin
                if (!found && v[k]) begin
                    lzc27 = 26 - k;
                    found = 1'b1;
                end
            end
        end
    endfunction

    //------------------------------------------------------------- stage 1
    wire [30:0] mag_a = (a[30:23] == 8'd0) ? 31'd0 : a[30:0];
    wire [30:0] mag_b = (b[30:23] == 8'd0) ? 31'd0 : b[30:0];
    wire        swap  = (mag_b > mag_a);
    wire [30:0] big   = swap ? mag_b : mag_a;
    wire [30:0] sml   = swap ? mag_a : mag_b;
    wire        s_big = swap ? b[31] : a[31];
    wire        s_sml = swap ? a[31] : b[31];

    wire [26:0] m_big = (big[30:23] == 8'd0) ? 27'd0 : {1'b1, big[22:0], 3'b000};
    wire [26:0] m_sml = (sml[30:23] == 8'd0) ? 27'd0 : {1'b1, sml[22:0], 3'b000};
    wire [7:0]  d     = big[30:23] - sml[30:23];

    reg  [26:0] m_al;
    reg         stk;
    always @(*) begin
        if (d >= 8'd27) begin
            m_al = 27'd0;
            stk  = |m_sml;
        end else begin
            m_al = m_sml >> d;
            stk  = |(m_sml & ((27'd1 << d) - 27'd1));
        end
    end

    reg        s1_sign, s1_sub;
    reg [7:0]  s1_e;
    reg [26:0] s1_mb, s1_ms;
    always @(posedge clk) begin
        s1_sign <= s_big;
        s1_sub  <= s_big ^ s_sml;
        s1_e    <= big[30:23];
        s1_mb   <= m_big;
        s1_ms   <= {m_al[26:1], m_al[0] | stk};
    end

    //------------------------------------------------------------- stage 2
    wire [27:0] sum = s1_sub ? ({1'b0, s1_mb} - {1'b0, s1_ms})
                             : ({1'b0, s1_mb} + {1'b0, s1_ms});
    wire [4:0]  lz  = lzc27(sum[26:0]);

    reg  [26:0] mn;
    reg  signed [9:0] en;
    reg  [24:0] mr;
    reg         rup;
    reg  [31:0] res;
    always @(*) begin
        if (sum[27]) begin
            mn = {sum[27:2], sum[1] | sum[0]};
            en = $signed({2'b00, s1_e}) + 10'sd1;
        end else begin
            mn = sum[26:0] << lz;
            en = $signed({2'b00, s1_e}) - $signed({5'd0, lz});
        end
        rup = mn[2] & (mn[1] | mn[0] | mn[3]);
        mr  = {1'b0, mn[26:3]} + {24'd0, rup};
        if (mr[24]) begin
            mr = mr >> 1;
            en = en + 10'sd1;
        end
        if (sum == 28'd0 || s1_mb == 27'd0 && s1_ms == 27'd0 || en <= 0)
            res = 32'd0;
        else if (en >= 10'sd255)
            res = {s1_sign, 8'hFF, 23'd0};
        else
            res = {s1_sign, en[7:0], mr[22:0]};
    end

    always @(posedge clk)
        y <= res;

endmodule
