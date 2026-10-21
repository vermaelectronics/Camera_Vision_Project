//-----------------------------------------------------------------------------
// stap_fp_recip : IEEE-754 single precision reciprocal y = 1/x
//   Sequential restoring divider (51 cycles from start to done).
//   - round to nearest, ties to even
//   - x == 0 gives +/-Inf, results outside the normal range saturate/flush
//-----------------------------------------------------------------------------
module stap_fp_recip (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire [31:0] x,
    output reg         done,
    output wire [31:0] y        // valid while done is high
);

    reg        busy;
    reg [5:0]  cnt;
    reg [25:0] rem;
    reg [25:0] q;
    reg [23:0] m;
    reg [7:0]  ex;
    reg        sx;
    reg        fz;              // fraction field of x is zero -> 1/x exact

    // trial subtraction of one long-division step
    wire [25:0] rem_sh = {rem[24:0], (cnt == 6'd49)};
    wire        ge     = (rem_sh >= {2'b00, m});

    // final normalisation / rounding
    reg  [24:0]       mr;
    reg  signed [9:0] e;
    reg               rup;
    reg  [31:0]       res;
    always @(*) begin
        rup = 1'b0;
        // 2^49 / m lies in (2^25, 2^26]
        if (fz) begin
            mr  = 25'h0800000;                         // exactly 1.0
            e   = 10'sd254 - $signed({2'b00, ex});
        end else begin
            rup = q[1] & (q[0] | (rem != 26'd0) | q[2]);
            mr  = {1'b0, q[25:2]} + {24'd0, rup};
            e   = 10'sd253 - $signed({2'b00, ex});
            if (mr[24]) begin
                mr = mr >> 1;
                e  = e + 10'sd1;
            end
        end
        if (ex == 8'd0 || e >= 10'sd255)
            res = {sx, 8'hFF, 23'd0};
        else if (e <= 0)
            res = 32'd0;
        else
            res = {sx, e[7:0], mr[22:0]};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 1'b0;
            done <= 1'b0;
            cnt  <= 6'd0;
            rem  <= 26'd0;
            q    <= 26'd0;
            m    <= 24'd0;
            ex   <= 8'd0;
            sx   <= 1'b0;
            fz   <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1;
                cnt  <= 6'd49;
                rem  <= 26'd0;
                q    <= 26'd0;
                m    <= {1'b1, x[22:0]};
                ex   <= x[30:23];
                sx   <= x[31];
                fz   <= (x[22:0] == 23'd0);
            end else if (busy) begin
                rem <= ge ? (rem_sh - {2'b00, m}) : rem_sh;
                q   <= {q[24:0], ge};
                if (cnt == 6'd0) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end else begin
                    cnt <= cnt - 6'd1;
                end
            end
        end
    end

    assign y = res;

endmodule
