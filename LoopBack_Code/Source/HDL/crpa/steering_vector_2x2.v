`timescale 1ns / 1ps

module steering_vector_2x2 #(
    parameter W = 16
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire                   valid_in,

    // ============================================================
    // ANGLES FROM PS
    //
    // Signed integer degrees
    // Valid range: -90 ... +90
    // ============================================================

    input wire signed [W-1:0] gps_angle,
    input wire signed [W-1:0] jam_angle,

    // ============================================================
    // GPS STEERING VECTOR
    //
    // aGPS = [ 1 ;
    //          exp(j*pi*sin(thetaGPS)) ]
    //
    // Q15
    // ============================================================

    output reg signed [W-1:0] gps_a1_i,
    output reg signed [W-1:0] gps_a1_q,

    output reg signed [W-1:0] gps_a2_i,
    output reg signed [W-1:0] gps_a2_q,

    // ============================================================
    // JAMMER STEERING VECTOR
    //
    // aJAM = [ 1 ;
    //          exp(j*pi*sin(thetaJAM)) ]
    //
    // Q15
    // ============================================================

    output reg signed [W-1:0] jam_a1_i,
    output reg signed [W-1:0] jam_a1_q,

    output reg signed [W-1:0] jam_a2_i,
    output reg signed [W-1:0] jam_a2_q,

    output reg valid_out
);

    // ============================================================
    // LUT FUNCTION
    //
    // Returns cos(pi*sin(|angle|)) in Q15.
    //
    // cos(pi*sin(-theta)) = cos(pi*sin(theta))
    //
    // Therefore negative angles use the same cosine value.
    // ============================================================

    function signed [15:0] angle_cos;
        input signed [15:0] angle;

        integer a;

        begin

            if (angle < 0)
                a = -angle;
            else
                a = angle;

            if (a > 90)
                a = 90;

            case (a)

                0:  angle_cos = 32767;
                1:  angle_cos = 32718;
                2:  angle_cos = 32570;
                3:  angle_cos = 32325;
                4:  angle_cos = 31983;
                5:  angle_cos = 31546;
                6:  angle_cos = 31016;
                7:  angle_cos = 30395;
                8:  angle_cos = 29685;
                9:  angle_cos = 28889;

                10: angle_cos = 28011;
                11: angle_cos = 27054;
                12: angle_cos = 26022;
                13: angle_cos = 24919;
                14: angle_cos = 23750;
                15: angle_cos = 22519;
                16: angle_cos = 21231;
                17: angle_cos = 19890;
                18: angle_cos = 18501;
                19: angle_cos = 17071;

                20: angle_cos = 15603;
                21: angle_cos = 14103;
                22: angle_cos = 12577;
                23: angle_cos = 11029;
                24: angle_cos = 9464;
                25: angle_cos = 7887;
                26: angle_cos = 6305;
                27: angle_cos = 4720;
                28: angle_cos = 3138;
                29: angle_cos = 1563;

                30: angle_cos = 0;
                31: angle_cos = -1547;
                32: angle_cos = -3075;
                33: angle_cos = -4580;
                34: angle_cos = -6058;
                35: angle_cos = -7507;
                36: angle_cos = -8923;
                37: angle_cos = -10303;
                38: angle_cos = -11646;
                39: angle_cos = -12949;

                40: angle_cos = -14211;
                41: angle_cos = -15429;
                42: angle_cos = -16603;
                43: angle_cos = -17731;
                44: angle_cos = -18812;
                45: angle_cos = -19847;
                46: angle_cos = -20834;
                47: angle_cos = -21774;
                48: angle_cos = -22665;
                49: angle_cos = -23510;

                50: angle_cos = -24308;
                51: angle_cos = -25059;
                52: angle_cos = -25765;
                53: angle_cos = -26426;
                54: angle_cos = -27044;
                55: angle_cos = -27619;
                56: angle_cos = -28153;
                57: angle_cos = -28648;
                58: angle_cos = -29104;
                59: angle_cos = -29523;

                60: angle_cos = -29907;
                61: angle_cos = -30258;
                62: angle_cos = -30576;
                63: angle_cos = -30865;
                64: angle_cos = -31125;
                65: angle_cos = -31358;
                66: angle_cos = -31566;
                67: angle_cos = -31750;
                68: angle_cos = -31913;
                69: angle_cos = -32056;

                70: angle_cos = -32181;
                71: angle_cos = -32288;
                72: angle_cos = -32380;
                73: angle_cos = -32459;
                74: angle_cos = -32525;
                75: angle_cos = -32579;
                76: angle_cos = -32624;
                77: angle_cos = -32661;
                78: angle_cos = -32690;
                79: angle_cos = -32712;

                80: angle_cos = -32730;
                81: angle_cos = -32742;
                82: angle_cos = -32752;
                83: angle_cos = -32758;
                84: angle_cos = -32762;
                85: angle_cos = -32765;
                86: angle_cos = -32766;
                87: angle_cos = -32767;
                88: angle_cos = -32767;
                89: angle_cos = -32767;
                90: angle_cos = -32767;

                default: angle_cos = 32767;

            endcase

        end
    endfunction


    // ============================================================
    // LUT FUNCTION
    //
    // Returns sin(pi*sin(|angle|)) in Q15.
    //
    // sin(pi*sin(-theta)) = -sin(pi*sin(theta))
    //
    // Therefore sign is restored for negative angles.
    // ============================================================

    function signed [15:0] angle_sin;
        input signed [15:0] angle;

        integer a;
        reg signed [15:0] temp;

        begin

            if (angle < 0)
                a = -angle;
            else
                a = angle;

            if (a > 90)
                a = 90;

            case (a)

                0:  temp = 0;
                1:  temp = 1796;
                2:  temp = 3585;
                3:  temp = 5363;
                4:  temp = 7123;
                5:  temp = 8860;
                6:  temp = 10568;
                7:  temp = 12241;
                8:  temp = 13874;
                9:  temp = 15463;

                10: temp = 17002;
                11: temp = 18487;
                12: temp = 19913;
                13: temp = 21277;
                14: temp = 22574;
                15: temp = 23803;
                16: temp = 24959;
                17: temp = 26040;
                18: temp = 27044;
                19: temp = 27969;

                20: temp = 28814;
                21: temp = 29577;
                22: temp = 30257;
                23: temp = 30855;
                24: temp = 31371;
                25: temp = 31804;
                26: temp = 32155;
                27: temp = 32425;
                28: temp = 32616;
                29: temp = 32730;

                30: temp = 32767;
                31: temp = 32730;
                32: temp = 32622;
                33: temp = 32445;
                34: temp = 32202;
                35: temp = 31896;
                36: temp = 31529;
                37: temp = 31105;
                38: temp = 30628;
                39: temp = 30100;

                40: temp = 29525;
                41: temp = 28907;
                42: temp = 28249;
                43: temp = 27555;
                44: temp = 26829;
                45: temp = 26072;
                46: temp = 25291;
                47: temp = 24487;
                48: temp = 23663;
                49: temp = 22824;

                50: temp = 21973;
                51: temp = 21112;
                52: temp = 20245;
                53: temp = 19373;
                54: temp = 18501;
                55: temp = 17631;
                56: temp = 16765;
                57: temp = 15905;
                58: temp = 15055;
                59: temp = 14215;

                60: temp = 13388;
                61: temp = 12576;
                62: temp = 11780;
                63: temp = 11002;
                64: temp = 10244;
                65: temp = 9506;
                66: temp = 8791;
                67: temp = 8098;
                68: temp = 7431;
                69: temp = 6788;

                70: temp = 6171;
                71: temp = 5581;
                72: temp = 5018;
                73: temp = 4484;
                74: temp = 3978;
                75: temp = 3501;
                76: temp = 3053;
                77: temp = 2636;
                78: temp = 2248;
                79: temp = 1890;

                80: temp = 1563;
                81: temp = 1267;
                82: temp = 1002;
                83: temp = 767;
                84: temp = 564;
                85: temp = 392;
                86: temp = 251;
                87: temp = 141;
                88: temp = 63;
                89: temp = 16;
                90: temp = 0;

                default: temp = 0;

            endcase

            if (angle < 0)
                angle_sin = -temp;
            else
                angle_sin = temp;

        end
    endfunction


    // ============================================================
    // MAIN
    // ============================================================

    always @(posedge clk) begin

        if (rst) begin

            gps_a1_i <= 0;
            gps_a1_q <= 0;
            gps_a2_i <= 0;
            gps_a2_q <= 0;

            jam_a1_i <= 0;
            jam_a1_q <= 0;
            jam_a2_i <= 0;
            jam_a2_q <= 0;

            valid_out <= 0;

        end
        else begin

            valid_out <= valid_in;

            if (valid_in) begin

                // =================================================
                // FIRST ELEMENT
                //
                // a[0] = 1 + j0
                //
                // =================================================

                gps_a1_i <= 32767;
                gps_a1_q <= 0;

                jam_a1_i <= 32767;
                jam_a1_q <= 0;

                // =================================================
                // SECOND ELEMENT
                //
                // a[1] =
                // cos(pi*sin(theta))
                // +
                // j sin(pi*sin(theta))
                //
                // =================================================

                gps_a2_i <= angle_cos(gps_angle);
                gps_a2_q <= angle_sin(gps_angle);

                jam_a2_i <= angle_cos(jam_angle);
                jam_a2_q <= angle_sin(jam_angle);

            end

        end

    end

endmodule
