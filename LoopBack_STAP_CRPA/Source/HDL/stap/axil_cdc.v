// ============================================================================
//  axil_cdc.v  --  AXI4-Lite clock-domain bridge (single outstanding transfer)
//  ANTSDR E310 V1 GNSS CRPA pipeline.  Original work for this project.
// ----------------------------------------------------------------------------
//  Slave side  : s_clk (sys_cpu_clk, PS7 FCLK_CLK0, 100 MHz)
//  Master side : m_clk (axi_ad9361/l_clk, the sample domain)
//
//  PROTOCOL (safe by construction, not by timing)
//    1. The slave side accepts one AW+W or one AR, latches address / data /
//       direction into req_* registers and flips req_tog.
//    2. req_tog crosses into m_clk through two flops. On its edge the master
//       side copies req_* (stable since step 1) and runs the AXI transfer.
//    3. The master side latches the read data into ack_rdata and flips ack_tog.
//    4. ack_tog crosses back through two flops; on its edge the slave side
//       returns BRESP or RDATA (ack_rdata stable since step 3).
//
//  DEAD-CLOCK PROTECTION
//    l_clk only runs once the AD9361 interface is up. A CPU access that never
//    completes would stall the PS7 GP0 port, so every transfer has a timeout of
//    TIMEOUT s_clk cycles. On timeout the slave side answers OKAY with read data
//    TIMEOUT_DATA (0xDEADC10C) -- not SLVERR, because a bus error on GP0 raises
//    a data abort in the bare-metal firmware. A late ack is absorbed: a new
//    transfer is only forwarded when the previous one has been acknowledged.
//    `timeouts` counts such events.
//
//  CONSTRAINTS (Source/XDC/gnss_stap_cdc.xdc)
//    set_false_path        -to *req_meta_reg* / *ack_meta_reg*
//    set_max_delay -datapath_only  req_* -> m-side copy, ack_rdata -> s-side
// ============================================================================
`timescale 1ns/100ps

module axil_cdc #(
  parameter AW           = 16,
  parameter TIMEOUT      = 4096,
  parameter TIMEOUT_DATA = 32'hDEADC10C
) (
  // ---- slave side (s_clk) ----
  input               s_clk,
  input               s_resetn,
  input      [AW-1:0] s_awaddr,
  input               s_awvalid,
  output reg          s_awready,
  input      [31:0]   s_wdata,
  input               s_wvalid,
  output reg          s_wready,
  output reg          s_bvalid,
  output     [1:0]    s_bresp,
  input               s_bready,
  input      [AW-1:0] s_araddr,
  input               s_arvalid,
  output reg          s_arready,
  output reg [31:0]   s_rdata,
  output reg          s_rvalid,
  output     [1:0]    s_rresp,
  input               s_rready,
  output reg [15:0]   timeouts,
  // ---- master side (m_clk) ----
  input               m_clk,
  input               m_resetn,
  output reg [AW-1:0] m_awaddr,
  output reg          m_awvalid,
  input               m_awready,
  output reg [31:0]   m_wdata,
  output reg          m_wvalid,
  input               m_wready,
  input               m_bvalid,
  output reg          m_bready,
  output reg [AW-1:0] m_araddr,
  output reg          m_arvalid,
  input               m_arready,
  input      [31:0]   m_rdata,
  input               m_rvalid,
  output reg          m_rready
);

  assign s_bresp = 2'b00;
  assign s_rresp = 2'b00;

  // ==========================================================================
  //  slave side
  // ==========================================================================
  reg  [AW-1:0] req_addr  = {AW{1'b0}};
  reg  [31:0]   req_wdata = 32'd0;
  reg           req_write = 1'b0;
  reg           req_tog   = 1'b0;

  (* ASYNC_REG = "TRUE" *) reg ack_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg ack_sync = 1'b0;
  wire          outstanding = (req_tog != ack_sync);   // previous not yet acked

  localparam S_IDLE = 2'd0, S_WAIT = 2'd1, S_RESP = 2'd2;
  reg  [1:0]    s_state = S_IDLE;
  reg           aw_have = 1'b0, w_have = 1'b0;
  reg  [AW-1:0] aw_addr_q = {AW{1'b0}};
  reg  [31:0]   w_data_q  = 32'd0;
  reg  [15:0]   tcnt = 16'd0;
  reg           cur_write = 1'b0;

  wire [31:0]   ack_rdata_s;    // from the master side, stable while ack_tog settles

  always @(posedge s_clk) begin
    ack_meta <= ack_tog;
    ack_sync <= ack_meta;
  end

  always @(posedge s_clk) begin
    if (!s_resetn) begin
      s_state   <= S_IDLE;
      s_awready <= 1'b0;
      s_wready  <= 1'b0;
      s_arready <= 1'b0;
      s_bvalid  <= 1'b0;
      s_rvalid  <= 1'b0;
      s_rdata   <= 32'd0;
      aw_have   <= 1'b0;
      w_have    <= 1'b0;
      tcnt      <= 16'd0;
      timeouts  <= 16'd0;
      // req_tog is NOT reset: it must stay consistent with the master side.
    end else begin
      s_awready <= 1'b0;
      s_wready  <= 1'b0;
      s_arready <= 1'b0;
      case (s_state)
      S_IDLE: begin
        // collect AW and W (any order); AR has priority only when no write
        if (s_awvalid && !aw_have && !s_awready) begin
          s_awready <= 1'b1; aw_have <= 1'b1; aw_addr_q <= s_awaddr;
        end
        if (s_wvalid && !w_have && !s_wready) begin
          s_wready <= 1'b1;  w_have <= 1'b1;  w_data_q <= s_wdata;
        end
        if (aw_have && w_have) begin
          aw_have   <= 1'b0;
          w_have    <= 1'b0;
          cur_write <= 1'b1;
          tcnt      <= 16'd0;
          if (outstanding) begin           // clock dead from an earlier timeout
            s_state  <= S_RESP;
            s_bvalid <= 1'b1;
            timeouts <= timeouts + 1'b1;
          end else begin
            req_addr  <= aw_addr_q;
            req_wdata <= w_data_q;
            req_write <= 1'b1;
            req_tog   <= ~req_tog;
            s_state   <= S_WAIT;
          end
        end else if (s_arvalid && !aw_have && !w_have && !s_arready) begin
          s_arready <= 1'b1;
          cur_write <= 1'b0;
          tcnt      <= 16'd0;
          if (outstanding) begin
            s_state  <= S_RESP;
            s_rvalid <= 1'b1;
            s_rdata  <= TIMEOUT_DATA;
            timeouts <= timeouts + 1'b1;
          end else begin
            req_addr  <= s_araddr;
            req_write <= 1'b0;
            req_tog   <= ~req_tog;
            s_state   <= S_WAIT;
          end
        end
      end
      S_WAIT: begin
        tcnt <= tcnt + 1'b1;
        if (!outstanding) begin                       // acknowledged
          s_state <= S_RESP;
          if (cur_write) s_bvalid <= 1'b1;
          else begin     s_rvalid <= 1'b1; s_rdata <= ack_rdata_s; end
        end else if (tcnt == TIMEOUT - 1) begin       // l_clk not running
          s_state  <= S_RESP;
          timeouts <= timeouts + 1'b1;
          if (cur_write) s_bvalid <= 1'b1;
          else begin     s_rvalid <= 1'b1; s_rdata <= TIMEOUT_DATA; end
        end
      end
      S_RESP: begin
        if (s_bvalid && s_bready) begin s_bvalid <= 1'b0; s_state <= S_IDLE; end
        if (s_rvalid && s_rready) begin s_rvalid <= 1'b0; s_state <= S_IDLE; end
      end
      default: s_state <= S_IDLE;
      endcase
    end
  end

  // ==========================================================================
  //  master side
  // ==========================================================================
  (* ASYNC_REG = "TRUE" *) reg req_meta = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg req_sync = 1'b0;
  reg           ack_tog   = 1'b0;      // never reset (see slave side)
  reg  [31:0]   ack_rdata = 32'd0;
  assign ack_rdata_s = ack_rdata;

  localparam M_IDLE = 2'd0, M_WR = 2'd1, M_RD = 2'd2;
  reg  [1:0]    m_state = M_IDLE;
  reg           aw_done = 1'b0, w_done = 1'b0;

  always @(posedge m_clk) begin
    req_meta <= req_tog;
    req_sync <= req_meta;
  end

  always @(posedge m_clk) begin
    if (!m_resetn) begin
      m_state   <= M_IDLE;
      m_awvalid <= 1'b0;
      m_wvalid  <= 1'b0;
      m_bready  <= 1'b0;
      m_arvalid <= 1'b0;
      m_rready  <= 1'b0;
      aw_done   <= 1'b0;
      w_done    <= 1'b0;
    end else begin
      case (m_state)
      M_IDLE: if (req_sync != ack_tog) begin
        if (req_write) begin
          m_awaddr  <= req_addr;  m_awvalid <= 1'b1;
          m_wdata   <= req_wdata; m_wvalid  <= 1'b1;
          m_bready  <= 1'b1;
          aw_done   <= 1'b0;      w_done    <= 1'b0;
          m_state   <= M_WR;
        end else begin
          m_araddr  <= req_addr;  m_arvalid <= 1'b1;
          m_rready  <= 1'b1;
          m_state   <= M_RD;
        end
      end
      M_WR: begin
        if (m_awvalid && m_awready) m_awvalid <= 1'b0;
        if (m_wvalid  && m_wready ) m_wvalid  <= 1'b0;
        if (m_bvalid && m_bready) begin
          m_bready <= 1'b0;
          ack_tog  <= ~ack_tog;
          m_state  <= M_IDLE;
        end
      end
      M_RD: begin
        if (m_arvalid && m_arready) m_arvalid <= 1'b0;
        if (m_rvalid && m_rready) begin
          m_rready  <= 1'b0;
          ack_rdata <= m_rdata;
          ack_tog   <= ~ack_tog;
          m_state   <= M_IDLE;
        end
      end
      default: m_state <= M_IDLE;
      endcase
    end
  end

endmodule
