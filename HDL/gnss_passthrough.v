`timescale 1ns/1ps

// ============================================================================
// gnss_passthrough.v
//
// ANTSDR E310 GNSS passthrough + 2-antenna robust LCMV (PIER C 2008)
//
// NORMAL MODE:
//      lcmv_enable = 0
//          RX1/RX2 -> FIFO -> TX1/TX2
//
// LCMV MODE:
//      lcmv_enable = 1
//          1. Automatically start one LCMV calculation
//          2. Calculate covariance and LCMV weights
//          3. Store weights
//          4. Beamformer continuously processes RX
//          5. Weighted output -> FIFO -> TX1/TX2
//
// IMPORTANT:
//      No external LCMV START command/toggle is required.
//
//      0 -> 1 on CONTROL[4]
//          automatically starts one complete LCMV calculation.
//
//      While CONTROL[4] remains 1:
//          calculated weights remain active.
//          beamformer continuously produces weighted output.
//
//      1 -> 0:
//          immediately returns to normal passthrough.
//
//      0 -> 1 again:
//          a fresh LCMV calculation is performed.
//
// AXI-Lite configuration:
//   0x00 CORE_ID
//   0x04 VERSION
//   0x08 SCRATCH
//   0x0C CONTROL
//   0x10 STATUS
//   0x14 RX_COUNT_CH0
//   0x18 TX_COUNT_CH0
//   0x1C OVERFLOW_COUNT
//   0x20 UNDERFLOW_COUNT
//   0x24 RX_SNAPSHOT_CH0
//   0x28 TX_SNAPSHOT_CH0
//   0x2C FIFO_LEVEL
//   0x30 RX_COUNT_CH1
//   0x34 TX_COUNT_CH1
//   0x38 RX_SNAPSHOT_CH1
//   0x40 GPS ANGLE   - LCMV look direction (integer deg)
//   0x44 JAM ANGLE   - LCMV null direction (integer deg, from PS / external DOA)
//   0x48 LCMV CONTROL - legacy/reserved, ignored
//   0x50 W1 REAL
//   0x54 W1 IMAG
//   0x58 W2 REAL
//   0x5C W2 IMAG
//
// CONTROL bits:
//   bit 0 = passthrough enable
//   bit 1 = mute
//   bit 2 = swap I/Q
//   bit 3 = CH1 copy CH0
//   bit 4 = LCMV enable
//   bit 8 = counter clear
//
// ============================================================================

module gnss_passthrough #(
    parameter FIFO_ADDR_WIDTH = 5
)(
    // ========================================================================
    // AXI CLOCK / RESET
    // ========================================================================

    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,

    input  wire [7:0]  s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,

    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,

    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [7:0]  s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,

    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // ========================================================================
    // DATAPATH CLOCK / RESET
    // ========================================================================

    input  wire        clk,
    input  wire        rst,

    // ========================================================================
    // ADC RX
    // ========================================================================

    input  wire [15:0] adc_data_i0,
    input  wire [15:0] adc_data_q0,
    input  wire        adc_valid_i0,
    input  wire        adc_valid_q0,
    input  wire        adc_enable_i0,
    input  wire        adc_enable_q0,

    input  wire [15:0] adc_data_i1,
    input  wire [15:0] adc_data_q1,
    input  wire        adc_valid_i1,
    input  wire        adc_valid_q1,
    input  wire        adc_enable_i1,
    input  wire        adc_enable_q1,

    // ========================================================================
    // DMA DAC DATA
    // ========================================================================

    input  wire [15:0] dma_dac_data_i0,
    input  wire [15:0] dma_dac_data_q0,
    input  wire [15:0] dma_dac_data_i1,
    input  wire [15:0] dma_dac_data_q1,

    // ========================================================================
    // DAC TX
    // ========================================================================

    output wire [15:0] dac_data_i0,
    output wire [15:0] dac_data_q0,
    output wire [15:0] dac_data_i1,
    output wire [15:0] dac_data_q1,

    input  wire        dac_valid_i0,
    input  wire        dac_valid_q0,
    input  wire        dac_enable_i0,
    input  wire        dac_enable_q0,

    input  wire        dac_valid_i1,
    input  wire        dac_valid_q1,
    input  wire        dac_enable_i1,
    input  wire        dac_enable_q1
);

    // ========================================================================
    // CONSTANTS
    // ========================================================================

    localparam [31:0] CORE_ID = 32'h47435031;
    localparam [31:0] VERSION = 32'h00010003;

    localparam integer FIFO_DEPTH = (1 << FIFO_ADDR_WIDTH);

    // ========================================================================
    // AXI REGISTERS
    // ========================================================================

    reg [31:0] reg_control;
    reg [31:0] reg_scratch;

    reg signed [15:0] reg_gps_angle;
    reg signed [15:0] reg_jam_angle;

    // Legacy register.
    // Kept for software compatibility, but NOT used to start LCMV.
    reg [31:0] reg_lcmv_control;

    // ========================================================================
    // AXI WRITE
    // ========================================================================

    reg        aw_hold;
    reg [7:0]  awaddr_hold;

    reg        w_hold;
    reg [31:0] wdata_hold;
    reg [3:0]  wstrb_hold;

    assign s_axi_awready = !aw_hold && !s_axi_bvalid;
    assign s_axi_wready  = !w_hold  && !s_axi_bvalid;

    always @(posedge s_axi_aclk) begin

        if (!s_axi_aresetn) begin

            aw_hold       <= 1'b0;
            awaddr_hold   <= 8'd0;

            w_hold        <= 1'b0;
            wdata_hold    <= 32'd0;
            wstrb_hold    <= 4'd0;

            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;

            reg_control      <= 32'd0;
            reg_scratch      <= 32'd0;

            reg_gps_angle    <= 16'sd30;
            reg_jam_angle    <= 16'sd60;

            reg_lcmv_control <= 32'd0;

        end
        else begin

            // ---------------------------------------------------------------
            // Capture address
            // ---------------------------------------------------------------

            if (s_axi_awvalid && s_axi_awready) begin

                aw_hold     <= 1'b1;
                awaddr_hold <= s_axi_awaddr;

            end

            // ---------------------------------------------------------------
            // Capture data
            // ---------------------------------------------------------------

            if (s_axi_wvalid && s_axi_wready) begin

                w_hold     <= 1'b1;
                wdata_hold <= s_axi_wdata;
                wstrb_hold <= s_axi_wstrb;

            end

            // ---------------------------------------------------------------
            // Execute write
            // ---------------------------------------------------------------

            if (aw_hold && w_hold && !s_axi_bvalid) begin

                case (awaddr_hold)

                    // -------------------------------------------------------
                    // CONTROL
                    // -------------------------------------------------------

                    8'h0C: begin

                        if (wstrb_hold[0])
                            reg_control[7:0] <= wdata_hold[7:0];

                        if (wstrb_hold[1])
                            reg_control[15:8] <= wdata_hold[15:8];

                        if (wstrb_hold[2])
                            reg_control[23:16] <= wdata_hold[23:16];

                        if (wstrb_hold[3])
                            reg_control[31:24] <= wdata_hold[31:24];

                    end

                    // -------------------------------------------------------
                    // SCRATCH
                    // -------------------------------------------------------

                    8'h08: begin

                        if (wstrb_hold[0])
                            reg_scratch[7:0] <= wdata_hold[7:0];

                        if (wstrb_hold[1])
                            reg_scratch[15:8] <= wdata_hold[15:8];

                        if (wstrb_hold[2])
                            reg_scratch[23:16] <= wdata_hold[23:16];

                        if (wstrb_hold[3])
                            reg_scratch[31:24] <= wdata_hold[31:24];

                    end

                    // -------------------------------------------------------
                    // GPS ANGLE
                    // -------------------------------------------------------

                    8'h40: begin

                        if (wstrb_hold[0])
                            reg_gps_angle[7:0] <= wdata_hold[7:0];

                        if (wstrb_hold[1])
                            reg_gps_angle[15:8] <= wdata_hold[15:8];

                    end

                    // -------------------------------------------------------
                    // JAM ANGLE
                    // -------------------------------------------------------

                    8'h44: begin

                        if (wstrb_hold[0])
                            reg_jam_angle[7:0] <= wdata_hold[7:0];

                        if (wstrb_hold[1])
                            reg_jam_angle[15:8] <= wdata_hold[15:8];

                    end

                    // -------------------------------------------------------
                    // LEGACY LCMV CONTROL
                    //
                    // Preserved only for software compatibility.
                    // It does NOT start LCMV.
                    // -------------------------------------------------------

                    8'h48: begin

                        if (wstrb_hold[0])
                            reg_lcmv_control[7:0] <= wdata_hold[7:0];

                        if (wstrb_hold[1])
                            reg_lcmv_control[15:8] <= wdata_hold[15:8];

                        if (wstrb_hold[2])
                            reg_lcmv_control[23:16] <= wdata_hold[23:16];

                        if (wstrb_hold[3])
                            reg_lcmv_control[31:24] <= wdata_hold[31:24];

                    end

                    default: begin
                    end

                endcase

                aw_hold      <= 1'b0;
                w_hold       <= 1'b0;

                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;

            end

            if (s_axi_bvalid && s_axi_bready)
                s_axi_bvalid <= 1'b0;

        end
    end

    // ========================================================================
    // STATUS / FIFO SIGNALS
    // ========================================================================

    wire fifo0_empty;
    wire fifo0_full;

    wire fifo1_empty;
    wire fifo1_full;

    wire [FIFO_ADDR_WIDTH:0] fifo_level0;
    wire [FIFO_ADDR_WIDTH:0] fifo_level1;

    reg overflow_sticky;
    reg underflow_sticky;

    reg [31:0] rx_count_ch0;
    reg [31:0] tx_count_ch0;

    reg [31:0] rx_count_ch1;
    reg [31:0] tx_count_ch1;

    reg [15:0] rx_snapshot_ch0;
    reg [15:0] tx_snapshot_ch0;
    reg [15:0] rx_snapshot_ch1;

    wire [31:0] status_reg;

    // Weight status registers.
    reg signed [31:0] w1_re_status;
    reg signed [31:0] w1_im_status;
    reg signed [31:0] w2_re_status;
    reg signed [31:0] w2_im_status;

    // ========================================================================
    // AXI READ
    // ========================================================================

    assign s_axi_arready = !s_axi_rvalid;

    always @(posedge s_axi_aclk) begin

        if (!s_axi_aresetn) begin

            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
            s_axi_rresp  <= 2'b00;

        end
        else begin

            if (s_axi_arvalid && s_axi_arready) begin

                case (s_axi_araddr)

                    8'h00:
                        s_axi_rdata <= CORE_ID;

                    8'h04:
                        s_axi_rdata <= VERSION;

                    8'h08:
                        s_axi_rdata <= reg_scratch;

                    8'h0C:
                        s_axi_rdata <= reg_control;

                    8'h10:
                        s_axi_rdata <= status_reg;

                    8'h14:
                        s_axi_rdata <= rx_count_ch0;

                    8'h18:
                        s_axi_rdata <= tx_count_ch0;

                    8'h1C:
                        s_axi_rdata <= {31'd0,overflow_sticky};

                    8'h20:
                        s_axi_rdata <= {31'd0,underflow_sticky};

                    8'h24:
                        s_axi_rdata <= {16'd0,rx_snapshot_ch0};

                    8'h28:
                        s_axi_rdata <= {16'd0,tx_snapshot_ch0};

                    8'h2C:
                        s_axi_rdata <=
                            {{(32-FIFO_ADDR_WIDTH-1){1'b0}},
                             fifo_level0};

                    8'h30:
                        s_axi_rdata <= rx_count_ch1;

                    8'h34:
                        s_axi_rdata <= tx_count_ch1;

                    8'h38:
                        s_axi_rdata <= {16'd0,rx_snapshot_ch1};

                    8'h40:
                        s_axi_rdata <=
                            {{16{reg_gps_angle[15]}},reg_gps_angle};

                    8'h44:
                        s_axi_rdata <=
                            {{16{reg_jam_angle[15]}},reg_jam_angle};

                    8'h48:
                        s_axi_rdata <= reg_lcmv_control;

                    8'h50:
                        s_axi_rdata <= w1_re_status;

                    8'h54:
                        s_axi_rdata <= w1_im_status;

                    8'h58:
                        s_axi_rdata <= w2_re_status;

                    8'h5C:
                        s_axi_rdata <= w2_im_status;

                    default:
                        s_axi_rdata <= 32'hDEAD_BEEF;

                endcase

                s_axi_rresp  <= 2'b00;
                s_axi_rvalid <= 1'b1;

            end

            if (s_axi_rvalid && s_axi_rready)
                s_axi_rvalid <= 1'b0;

        end
    end

    // ========================================================================
    // CONTROL CDC
    //
    // AXI clock -> datapath clock
    // ========================================================================

    reg [31:0] ctrl_sync1;
    reg [31:0] ctrl_sync;

    always @(posedge clk) begin

        if (rst) begin

            ctrl_sync1 <= 32'd0;
            ctrl_sync  <= 32'd0;

        end
        else begin

            ctrl_sync1 <= reg_control;
            ctrl_sync  <= ctrl_sync1;

        end

    end

    // ========================================================================
    // ANGLE CDC
    // ========================================================================

    reg signed [15:0] gps_angle_sync1;
    reg signed [15:0] gps_angle_sync;

    reg signed [15:0] jam_angle_sync1;
    reg signed [15:0] jam_angle_sync;

    always @(posedge clk) begin

        if (rst) begin

            gps_angle_sync1 <= 16'sd30;
            gps_angle_sync  <= 16'sd30;

            jam_angle_sync1 <= 16'sd60;
            jam_angle_sync  <= 16'sd60;

        end
        else begin

            gps_angle_sync1 <= reg_gps_angle;
            gps_angle_sync  <= gps_angle_sync1;

            jam_angle_sync1 <= reg_jam_angle;
            jam_angle_sync  <= jam_angle_sync1;

        end

    end

    // ========================================================================
    // CONTROL BITS
    // ========================================================================

    wire pass_en  = ctrl_sync[0];
    wire mute     = ctrl_sync[1];
    wire swap_iq  = ctrl_sync[2];
    wire ch1_copy = ctrl_sync[3];

    // ONLY USER LCMV CONTROL
    wire lcmv_enable = ctrl_sync[4];

    wire cnt_clear = ctrl_sync[8];

    // ========================================================================
    // LCMV CORE
    // ========================================================================

    wire lcmv_busy;
    wire lcmv_weights_valid;

    wire signed [31:0] lcmv_w1_re;
    wire signed [31:0] lcmv_w1_im;
    wire signed [31:0] lcmv_w2_re;
    wire signed [31:0] lcmv_w2_im;

    wire lcmv_beam_valid;

    wire signed [31:0] lcmv_y_re;
    wire signed [31:0] lcmv_y_im;

    lcmv_crpa_core #(
        .IQ_WIDTH    (16),
        .ACC_WIDTH   (64),
        .COV_WIDTH   (32),
        .COV_SAMPLES (1024),
        .COV_SHIFT   (10)
    ) u_lcmv_core (

        .clk  (clk),
        .rst  (rst),

        // ---------------------------------------------------------------
        // ONLY LCMV CONTROL
        // ---------------------------------------------------------------

        .enable(lcmv_enable),

        // ---------------------------------------------------------------
        // RX
        // ---------------------------------------------------------------

        .rx_valid(adc_valid_i0 & adc_valid_i1),

        .rx1_i($signed(adc_data_i0)),
        .rx1_q($signed(adc_data_q0)),

        .rx2_i($signed(adc_data_i1)),
        .rx2_q($signed(adc_data_q1)),

        // ---------------------------------------------------------------
        // ANGLES
        // ---------------------------------------------------------------

        .gps_angle(gps_angle_sync),
        .jam_angle(jam_angle_sync),

        // ---------------------------------------------------------------
        // STATUS
        // ---------------------------------------------------------------

        .busy(lcmv_busy),
        .weights_valid(lcmv_weights_valid),

        // ---------------------------------------------------------------
        // WEIGHTS
        // ---------------------------------------------------------------

        .w1_re(lcmv_w1_re),
        .w1_im(lcmv_w1_im),

        .w2_re(lcmv_w2_re),
        .w2_im(lcmv_w2_im),

        // ---------------------------------------------------------------
        // BEAMFORMER OUTPUT
        // ---------------------------------------------------------------

        .beam_valid(lcmv_beam_valid),

        .y_re(lcmv_y_re),
        .y_im(lcmv_y_im)

    );

    // ========================================================================
    // STORE LAST VALID LCMV WEIGHTS
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            w1_re_status <= 32'sd0;
            w1_im_status <= 32'sd0;

            w2_re_status <= 32'sd0;
            w2_im_status <= 32'sd0;

        end
        else begin

            // Only meaningful while valid.
            // This is a status mirror, not part of the TX datapath.

            if (lcmv_weights_valid) begin

                w1_re_status <= lcmv_w1_re;
                w1_im_status <= lcmv_w1_im;

                w2_re_status <= lcmv_w2_re;
                w2_im_status <= lcmv_w2_im;

            end

        end
    end

   // ========================================================================
// LCMV QUANTIZATION
//
// lcmv_beamformer_2ch already converts its internal accumulated result
// to an integer ADC-scale output using:
//
//     Y_re = sum_re >>> 16
//     Y_im = sum_im >>> 16
//
// Therefore lcmv_y_re / lcmv_y_im arriving here are already integer-scale
// samples and MUST NOT be shifted by another 16 bits.
//
// Convert directly to signed 12-bit ADC/DAC range.
//
// Range:
//     -2048 ... +2047
// ========================================================================

function [11:0] lcmv_to_s12;

    input signed [31:0] x;

    begin

        if (x > 32'sd2047)

            lcmv_to_s12 = 12'd2047;

        else if (x < -32'sd2048)

            lcmv_to_s12 = 12'h800;

        else

            lcmv_to_s12 = x[11:0];

    end

endfunction

wire [11:0] lcmv_i12 =
    lcmv_to_s12(lcmv_y_re);

wire [11:0] lcmv_q12 =
    lcmv_to_s12(lcmv_y_im);

wire [15:0] lcmv_i16 =
    {{4{lcmv_i12[11]}},lcmv_i12};

wire [15:0] lcmv_q16 =
    {{4{lcmv_q12[11]}},lcmv_q12};

    // ========================================================================
    // LCMV ACTIVE
    //
    // IMPORTANT:
    //
    // enable = 1 alone does NOT immediately replace passthrough.
    //
    // We wait until the complete LCMV calculation has produced valid weights.
    // ========================================================================

    wire lcmv_active =
        lcmv_enable && lcmv_weights_valid;

    // ========================================================================
    // PROCESSING SOURCE
    // ========================================================================

    wire [15:0] proc_i0 =
        lcmv_active ? lcmv_i16 : adc_data_i0;

    wire [15:0] proc_q0 =
        lcmv_active ? lcmv_q16 : adc_data_q0;

    wire [15:0] proc_i1 =
        lcmv_active ? lcmv_i16 : adc_data_i1;

    wire [15:0] proc_q1 =
        lcmv_active ? lcmv_q16 : adc_data_q1;

    // ========================================================================
    // FIFO MEMORY
    // ========================================================================

    (* ram_style = "distributed" *)
    reg [31:0] fifo0 [0:FIFO_DEPTH-1];

    (* ram_style = "distributed" *)
    reg [31:0] fifo1 [0:FIFO_DEPTH-1];

    reg [FIFO_ADDR_WIDTH:0] wptr0;
    reg [FIFO_ADDR_WIDTH:0] rptr0;

    reg [FIFO_ADDR_WIDTH:0] wptr1;
    reg [FIFO_ADDR_WIDTH:0] rptr1;

    reg [31:0] rd_data0;
    reg [31:0] rd_data1;

    assign fifo_level0 = wptr0 - rptr0;
    assign fifo_level1 = wptr1 - rptr1;

    assign fifo0_empty = (fifo_level0 == 0);
    assign fifo1_empty = (fifo_level1 == 0);

    assign fifo0_full = (fifo_level0 == FIFO_DEPTH);
    assign fifo1_full = (fifo_level1 == FIFO_DEPTH);

    // ========================================================================
    // FIFO PRIMING
    // ========================================================================

    reg primed0;
    reg primed1;

    always @(posedge clk) begin

        if (rst || !pass_en) begin

            primed0 <= 1'b0;
            primed1 <= 1'b0;

        end
        else begin

            if (fifo_level0 >= (FIFO_DEPTH >> 1))
                primed0 <= 1'b1;

            if (fifo_level1 >= (FIFO_DEPTH >> 1))
                primed1 <= 1'b1;

            if (fifo0_empty)
                primed0 <= 1'b0;

            if (fifo1_empty)
                primed1 <= 1'b0;

        end
    end

    // ========================================================================
    // WRITE ENABLE
    // ========================================================================

    wire lcmv_wr_valid =
        lcmv_active && lcmv_beam_valid;

    wire wr_en0 =
        pass_en &&
        (lcmv_active ? lcmv_wr_valid : adc_valid_i0) &&
        adc_enable_i0;

    wire wr_en1 =
        pass_en &&
        (lcmv_active ? lcmv_wr_valid : adc_valid_i1) &&
        adc_enable_i1;

    // ========================================================================
    // READ ENABLE
    // ========================================================================

    wire rd_en0 =
        pass_en &&
        dac_valid_i0 &&
        primed0;

    wire rd_en1 =
        pass_en &&
        dac_valid_i1 &&
        primed1;

    // ========================================================================
    // FIFO
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            wptr0 <= 0;
            rptr0 <= 0;

            wptr1 <= 0;
            rptr1 <= 0;

            rd_data0 <= 32'd0;
            rd_data1 <= 32'd0;

        end
        else begin

            // ---------------------------------------------------------------
            // FIFO 0 WRITE
            // ---------------------------------------------------------------

            if (wr_en0 && !fifo0_full) begin

                fifo0[wptr0[FIFO_ADDR_WIDTH-1:0]]
                    <= {proc_q0,proc_i0};

                wptr0 <= wptr0 + 1'b1;

            end

            // ---------------------------------------------------------------
            // FIFO 0 READ
            // ---------------------------------------------------------------

            if (rd_en0 && !fifo0_empty) begin

                rd_data0 <=
                    fifo0[rptr0[FIFO_ADDR_WIDTH-1:0]];

                rptr0 <= rptr0 + 1'b1;

            end

            // ---------------------------------------------------------------
            // FIFO 1 WRITE
            // ---------------------------------------------------------------

            if (wr_en1 && !fifo1_full) begin

                fifo1[wptr1[FIFO_ADDR_WIDTH-1:0]]
                    <= {proc_q1,proc_i1};

                wptr1 <= wptr1 + 1'b1;

            end

            // ---------------------------------------------------------------
            // FIFO 1 READ
            // ---------------------------------------------------------------

            if (rd_en1 && !fifo1_empty) begin

                rd_data1 <=
                    fifo1[rptr1[FIFO_ADDR_WIDTH-1:0]];

                rptr1 <= rptr1 + 1'b1;

            end

        end
    end

    // ========================================================================
    // COUNTERS
    // ========================================================================

    always @(posedge clk) begin

        if (rst) begin

            rx_count_ch0 <= 32'd0;
            tx_count_ch0 <= 32'd0;

            rx_count_ch1 <= 32'd0;
            tx_count_ch1 <= 32'd0;

            overflow_sticky <= 1'b0;
            underflow_sticky <= 1'b0;

            rx_snapshot_ch0 <= 16'd0;
            tx_snapshot_ch0 <= 16'd0;
            rx_snapshot_ch1 <= 16'd0;

        end
        else begin

            if (cnt_clear) begin

                rx_count_ch0 <= 32'd0;
                tx_count_ch0 <= 32'd0;

                rx_count_ch1 <= 32'd0;
                tx_count_ch1 <= 32'd0;

                overflow_sticky <= 1'b0;
                underflow_sticky <= 1'b0;

            end
            else begin

                if (wr_en0 && !fifo0_full) begin

                    rx_count_ch0 <= rx_count_ch0 + 1'b1;
                    rx_snapshot_ch0 <= proc_i0;

                end

                if (wr_en1 && !fifo1_full) begin

                    rx_count_ch1 <= rx_count_ch1 + 1'b1;
                    rx_snapshot_ch1 <= proc_i1;

                end

                if (rd_en0 && !fifo0_empty) begin

                    tx_count_ch0 <= tx_count_ch0 + 1'b1;
                    tx_snapshot_ch0 <= rd_data0[15:0];

                end

                if (rd_en1 && !fifo1_empty) begin

                    tx_count_ch1 <= tx_count_ch1 + 1'b1;

                end

                if (wr_en0 && fifo0_full)
                    overflow_sticky <= 1'b1;

                if (wr_en1 && fifo1_full)
                    overflow_sticky <= 1'b1;

                if (rd_en0 && fifo0_empty)
                    underflow_sticky <= 1'b1;

                if (rd_en1 && fifo1_empty)
                    underflow_sticky <= 1'b1;

            end
        end
    end

    // ========================================================================
    // STATUS
    // ========================================================================

    assign status_reg = {
        15'd0,
        pass_en,
        7'd0,
        underflow_sticky,
        overflow_sticky,
        fifo1_full,
        fifo1_empty,
        fifo0_full,
        fifo0_empty,
        dac_enable_q0,
        dac_enable_i0,
        adc_enable_q0,
        adc_enable_i0
    };

    // ========================================================================
    // OUTPUT DATA
    // ========================================================================

    wire [15:0] pt_i0 =
        swap_iq ? rd_data0[31:16] :
                  rd_data0[15:0];

    wire [15:0] pt_q0 =
        swap_iq ? rd_data0[15:0] :
                  rd_data0[31:16];

    wire [31:0] src1 =
        ch1_copy ? rd_data0 :
                   rd_data1;

    wire [15:0] pt_i1 =
        swap_iq ? src1[31:16] :
                  src1[15:0];

    wire [15:0] pt_q1 =
        swap_iq ? src1[15:0] :
                  src1[31:16];

    // ========================================================================
    // AD9361 DAC FORMAT
    // ========================================================================

    wire [15:0] al_i0 =
        {pt_i0[11:0],4'b0000};

    wire [15:0] al_q0 =
        {pt_q0[11:0],4'b0000};

    wire [15:0] al_i1 =
        {pt_i1[11:0],4'b0000};

    wire [15:0] al_q1 =
        {pt_q1[11:0],4'b0000};

    // ========================================================================
    // DAC OUTPUT
    //
    // pass_en remains the normal FIFO path enable.
    //
    // LCMV does NOT require a second start control.
    //
    // ========================================================================

    assign dac_data_i0 =
        mute ? 16'd0 :
        pass_en ? al_i0 :
        dma_dac_data_i0;

    assign dac_data_q0 =
        mute ? 16'd0 :
        pass_en ? al_q0 :
        dma_dac_data_q0;

    assign dac_data_i1 =
        mute ? 16'd0 :
        pass_en ? al_i1 :
        dma_dac_data_i1;

    assign dac_data_q1 =
        mute ? 16'd0 :
        pass_en ? al_q1 :
        dma_dac_data_q1;

endmodule