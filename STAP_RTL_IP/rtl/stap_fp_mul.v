//-----------------------------------------------------------------------------
// stap_fp_mul : IEEE-754 single precision multiplier, 2-cycle pipelined
//   y = a * b
//   - round to nearest, ties to even
//   - denormals flushed to zero, overflow saturates to +/-Inf
//-----------------------------------------------------------------------------
module stap_fp_mul (
    input  wire        clk,
    input  wire [31:0] a,
    input  wire [31:0] b,
    output reg  [31:0] y
);

    //------------------------------------------------------------- stage 1
    reg               s1_sign, s1_zero;
    reg signed [9:0]  s1_e;
    reg [47:0]        s1_p;
    always @(posedge clk) begin
        s1_sign <= a[31] ^ b[31];
        s1_zero <= (a[30:23] == 8'd0) || (b[30:23] == 8'd0);
        s1_e    <= $signed({2'b00, a[30:23]}) + $signed({2'b00, b[30:23]}) - 10'sd127;
        s1_p    <= {1'b1, a[22:0]} * {1'b1, b[22:0]};
    end

    //------------------------------------------------------------- stage 2
    reg  [23:0]       m;
    reg               g, rs, rup;
    reg  [24:0]       mr;
    reg  signed [9:0] e;
    reg  [31:0]       res;
    always @(*) begin
        if (s1_p[47]) begin
            m  = s1_p[47:24];
            g  = s1_p[23];
            rs = |s1_p[22:0];
            e  = s1_e + 10'sd1;
        end else begin
            m  = s1_p[46:23];
            g  = s1_p[22];
            rs = |s1_p[21:0];
            e  = s1_e;
        end
        rup = g & (rs | m[0]);
        mr  = {1'b0, m} + {24'd0, rup};
        if (mr[24]) begin
            mr = mr >> 1;
            e  = e + 10'sd1;
        end
        if (s1_zero || e <= 0)
            res = 32'd0;
        else if (e >= 10'sd255)
            res = {s1_sign, 8'hFF, 23'd0};
        else
            res = {s1_sign, e[7:0], mr[22:0]};
    end

    always @(posedge clk)
        y <= res;

endmodule
