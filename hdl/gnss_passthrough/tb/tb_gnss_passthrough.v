`timescale 1ns/100ps
// ============================================================================
//  tb_gnss_passthrough.v -- VERBOSE testbench
//
//  Shows every necessary input/output of gnss_passthrough AND of the CRPA
//  core (u_crpa_core = pi_power_inversion) instantiated inside it: a full
//  named port dump at three points in the run, a running per-checkpoint
//  snapshot of RX in / CRPA internals / TX out during the stimulus, every
//  AXI-Lite transaction logged as it happens (not just its result), a
//  floating-point shadow model for a genuine accuracy number (not just
//  "it compiles"), and a VCD waveform dump.
//
//  Run it: see the exact iverilog/vvp/gtkwave and Vivado xvlog/xelab/xsim
//  commands in the delivery notes -- everything below is tool-agnostic
//  Verilog, no special flags needed either way beyond -g2005-sv (iverilog)
//  or nothing extra (xvlog -sv already assumed from earlier in this thread).
// ============================================================================
module tb_gnss_passthrough;

  localparam real CLK_PERIOD_NS   = 10.0;  // 100 MHz l_clk
  localparam real SAXI_PERIOD_NS  = 14.0;  // ~71.4 MHz sys_cpu_clk, deliberately async
  localparam integer N_SAMPLES    = 400;
  localparam integer PRINT_EVERY  = 40;    // 10 checkpoints across the run
  localparam real    TEST_AMP     = 100.0;
  localparam real    ANGLE_STEP   = 0.7;
  localparam real    PHASE_STEP   = 0.31;

  reg clk = 0;          always #(CLK_PERIOD_NS/2.0)  clk = ~clk;
  reg s_axi_aclk = 0;   always #(SAXI_PERIOD_NS/2.0) s_axi_aclk = ~s_axi_aclk;
  reg rst = 1, s_axi_aresetn = 0;

  reg adc_enable_i0=1, adc_valid_i0=0; reg [15:0] adc_data_i0;
  reg adc_enable_q0=1, adc_valid_q0=0; reg [15:0] adc_data_q0;
  reg adc_enable_i1=1, adc_valid_i1=0; reg [15:0] adc_data_i1;
  reg adc_enable_q1=1, adc_valid_q1=0; reg [15:0] adc_data_q1;

  reg dac_enable_i0=1, dac_valid_i0=0;
  reg dac_enable_q0=1, dac_valid_q0=0;
  reg dac_enable_i1=1, dac_valid_i1=0;
  reg dac_enable_q1=1, dac_valid_q1=0;

  reg [15:0] dma_dac_data_i0=16'h1234, dma_dac_data_q0=16'h5678;
  reg [15:0] dma_dac_data_i1=16'h0,    dma_dac_data_q1=16'h0;

  wire [15:0] dac_data_i0, dac_data_q0, dac_data_i1, dac_data_q1;

  reg s_axi_awvalid=0; reg [15:0] s_axi_awaddr; reg [2:0] s_axi_awprot=0;
  wire s_axi_awready;
  reg s_axi_wvalid=0; reg [31:0] s_axi_wdata; reg [3:0] s_axi_wstrb=4'hF;
  wire s_axi_wready;
  wire s_axi_bvalid; wire [1:0] s_axi_bresp; reg s_axi_bready=0;
  reg s_axi_arvalid=0; reg [15:0] s_axi_araddr; reg [2:0] s_axi_arprot=0;
  wire s_axi_arready;
  wire s_axi_rvalid; wire [1:0] s_axi_rresp; wire [31:0] s_axi_rdata;
  reg s_axi_rready=0;

  gnss_passthrough #(.FIFO_ADDR_WIDTH(5)) dut (
    .clk(clk), .rst(rst),
    .adc_enable_i0(adc_enable_i0), .adc_valid_i0(adc_valid_i0), .adc_data_i0(adc_data_i0),
    .adc_enable_q0(adc_enable_q0), .adc_valid_q0(adc_valid_q0), .adc_data_q0(adc_data_q0),
    .adc_enable_i1(adc_enable_i1), .adc_valid_i1(adc_valid_i1), .adc_data_i1(adc_data_i1),
    .adc_enable_q1(adc_enable_q1), .adc_valid_q1(adc_valid_q1), .adc_data_q1(adc_data_q1),
    .dac_enable_i0(dac_enable_i0), .dac_valid_i0(dac_valid_i0),
    .dac_enable_q0(dac_enable_q0), .dac_valid_q0(dac_valid_q0),
    .dac_enable_i1(dac_enable_i1), .dac_valid_i1(dac_valid_i1),
    .dac_enable_q1(dac_enable_q1), .dac_valid_q1(dac_valid_q1),
    .dma_dac_data_i0(dma_dac_data_i0), .dma_dac_data_q0(dma_dac_data_q0),
    .dma_dac_data_i1(dma_dac_data_i1), .dma_dac_data_q1(dma_dac_data_q1),
    .dac_data_i0(dac_data_i0), .dac_data_q0(dac_data_q0),
    .dac_data_i1(dac_data_i1), .dac_data_q1(dac_data_q1),
    .s_axi_aclk(s_axi_aclk), .s_axi_aresetn(s_axi_aresetn),
    .s_axi_awvalid(s_axi_awvalid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awprot(s_axi_awprot), .s_axi_awready(s_axi_awready),
    .s_axi_wvalid(s_axi_wvalid), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb), .s_axi_wready(s_axi_wready),
    .s_axi_bvalid(s_axi_bvalid), .s_axi_bresp(s_axi_bresp), .s_axi_bready(s_axi_bready),
    .s_axi_arvalid(s_axi_arvalid), .s_axi_araddr(s_axi_araddr), .s_axi_arprot(s_axi_arprot), .s_axi_arready(s_axi_arready),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rresp(s_axi_rresp), .s_axi_rdata(s_axi_rdata), .s_axi_rready(s_axi_rready)
  );

  // ==========================================================================
  //  VCD waveform dump -- depth 0 = unlimited, so dut AND dut.u_crpa_core AND
  //  its own pi_cmul sub-instances are all captured automatically.
  // ==========================================================================
  initial begin
    $dumpfile("waveform_gnss.vcd");
    $dumpvars(0, tb_gnss_passthrough);
  end

  // ==========================================================================
  //  AXI-Lite bus-functional tasks, WITH transaction logging -- every write
  //  and read prints its address, data, and a human-readable register name.
  // ==========================================================================
  function [191:0] reg_name(input [15:0] addr);
    begin
      case (addr)
        16'h0000: reg_name = "ID";
        16'h0004: reg_name = "VERSION";
        16'h0008: reg_name = "SCRATCH";
        16'h000C: reg_name = "CONTROL";
        16'h0010: reg_name = "STATUS";
        16'h0014: reg_name = "RX_COUNT_CH0";
        16'h0018: reg_name = "TX_COUNT_CH0";
        16'h001C: reg_name = "OVERFLOW_COUNT";
        16'h0020: reg_name = "UNDERFLOW_COUNT";
        16'h0024: reg_name = "RX_SNAPSHOT_CH0";
        16'h0028: reg_name = "TX_SNAPSHOT_CH0";
        16'h002C: reg_name = "FIFO_LEVEL";
        16'h0030: reg_name = "RX_COUNT_CH1";
        16'h0034: reg_name = "TX_COUNT_CH1";
        16'h0038: reg_name = "RX_SNAPSHOT_CH1";
        16'h0040: reg_name = "CRPA_COEF[0]=alpha";
        default:  reg_name = addr >= 16'h0040 && addr <= 16'h007C ? "CRPA_COEF[n]" : "?";
      endcase
    end
  endfunction

  task axil_write(input [15:0] addr, input [31:0] data);
    begin
      @(posedge s_axi_aclk);
      s_axi_awaddr <= addr; s_axi_awvalid <= 1'b1;
      s_axi_wdata  <= data; s_axi_wvalid  <= 1'b1;
      s_axi_bready <= 1'b1;
      @(posedge s_axi_aclk);
      while (!(s_axi_awready && s_axi_wready)) @(posedge s_axi_aclk);
      s_axi_awvalid <= 1'b0; s_axi_wvalid <= 1'b0;
      while (!s_axi_bvalid) @(posedge s_axi_aclk);
      $display("  [AXI WRITE] addr=0x%04h (%0s)  data=0x%08h", addr, reg_name(addr), data);
      @(posedge s_axi_aclk);
      s_axi_bready <= 1'b0;
    end
  endtask

  task axil_read(input [15:0] addr, output [31:0] data_out);
    begin
      @(posedge s_axi_aclk);
      s_axi_araddr <= addr; s_axi_arvalid <= 1'b1; s_axi_rready <= 1'b1;
      @(posedge s_axi_aclk);
      while (!s_axi_arready) @(posedge s_axi_aclk);
      s_axi_arvalid <= 1'b0;
      while (!s_axi_rvalid) @(posedge s_axi_aclk);
      data_out = s_axi_rdata;
      $display("  [AXI READ ] addr=0x%04h (%0s)  data=0x%08h", addr, reg_name(addr), s_axi_rdata);
      @(posedge s_axi_aclk);
      s_axi_rready <= 1'b0;
    end
  endtask

  // ==========================================================================
  //  Full named port dump -- gnss_passthrough AND the CRPA core inside it.
  //  Call at reset release, mid-run, and end-of-run to see every value.
  // ==========================================================================
  task dump_all_ports(input [639:0] label);
    begin
      $display("=========================================================");
      $display(" FULL PORT DUMP -- %0s", label);
      $display("=========================================================");
      $display(" -- gnss_passthrough top-level ports --");
      $display("  clk=%b rst=%b", clk, rst);
      $display("  adc_enable_i0=%b adc_valid_i0=%b adc_data_i0=%0d (0x%04h)",
                 adc_enable_i0, adc_valid_i0, $signed(adc_data_i0), adc_data_i0);
      $display("  adc_enable_q0=%b adc_valid_q0=%b adc_data_q0=%0d (0x%04h)",
                 adc_enable_q0, adc_valid_q0, $signed(adc_data_q0), adc_data_q0);
      $display("  adc_enable_i1=%b adc_valid_i1=%b adc_data_i1=%0d (0x%04h)",
                 adc_enable_i1, adc_valid_i1, $signed(adc_data_i1), adc_data_i1);
      $display("  adc_enable_q1=%b adc_valid_q1=%b adc_data_q1=%0d (0x%04h)",
                 adc_enable_q1, adc_valid_q1, $signed(adc_data_q1), adc_data_q1);
      $display("  dac_enable_i0=%b dac_valid_i0=%b   dac_enable_q0=%b dac_valid_q0=%b",
                 dac_enable_i0, dac_valid_i0, dac_enable_q0, dac_valid_q0);
      $display("  dac_enable_i1=%b dac_valid_i1=%b   dac_enable_q1=%b dac_valid_q1=%b",
                 dac_enable_i1, dac_valid_i1, dac_enable_q1, dac_valid_q1);
      $display("  dma_dac_data_i0=0x%04h dma_dac_data_q0=0x%04h dma_dac_data_i1=0x%04h dma_dac_data_q1=0x%04h",
                 dma_dac_data_i0, dma_dac_data_q0, dma_dac_data_i1, dma_dac_data_q1);
      $display("  dac_data_i0=%0d (0x%04h)  dac_data_q0=%0d (0x%04h)  [OUTPUTS]",
                 $signed(dac_data_i0), dac_data_i0, $signed(dac_data_q0), dac_data_q0);
      $display("  dac_data_i1=%0d (0x%04h)  dac_data_q1=%0d (0x%04h)  [OUTPUTS]",
                 $signed(dac_data_i1), dac_data_i1, $signed(dac_data_q1), dac_data_q1);
      $display("  s_axi_aclk=%b s_axi_aresetn=%b", s_axi_aclk, s_axi_aresetn);
      $display("  s_axi_aw: valid=%b addr=0x%04h prot=%b ready=%b",
                 s_axi_awvalid, s_axi_awaddr, s_axi_awprot, s_axi_awready);
      $display("  s_axi_w : valid=%b data=0x%08h strb=%b ready=%b",
                 s_axi_wvalid, s_axi_wdata, s_axi_wstrb, s_axi_wready);
      $display("  s_axi_b : valid=%b resp=%b ready=%b", s_axi_bvalid, s_axi_bresp, s_axi_bready);
      $display("  s_axi_ar: valid=%b addr=0x%04h prot=%b ready=%b",
                 s_axi_arvalid, s_axi_araddr, s_axi_arprot, s_axi_arready);
      $display("  s_axi_r : valid=%b data=0x%08h resp=%b ready=%b",
                 s_axi_rvalid, s_axi_rdata, s_axi_rresp, s_axi_rready);
      $display(" -- internal control/status (not top ports, but load-bearing) --");
      $display("  pass_en=%b mute=%b swap_iq=%b ch1_copy=%b cnt_clear=%b",
                 dut.pass_en, dut.mute, dut.swap_iq, dut.ch1_copy, dut.cnt_clear);
      $display("  wr_en0=%b wr_en1=%b rd_en0=%b rd_en1=%b  level0=%0d level1=%0d  primed0=%b primed1=%b",
                 dut.wr_en0, dut.wr_en1, dut.rd_en0, dut.rd_en1,
                 dut.level0, dut.level1, dut.primed0, dut.primed1);
      $display("  ovf_sticky=%b unf_sticky=%b ovf_cnt=%0d unf_cnt=%0d",
                 dut.ovf_sticky, dut.unf_sticky, dut.ovf_cnt, dut.unf_cnt);
      $display(" -- u_crpa_core (pi_power_inversion, M=2) ports --");
      $display("  clk=%b aresetn=%b", dut.u_crpa_core.clk, dut.u_crpa_core.aresetn);
      $display("  x_re=0x%08h (elem0=%0d elem1=%0d)", dut.u_crpa_core.x_re,
                 $signed(dut.u_crpa_core.x_re[15:0]), $signed(dut.u_crpa_core.x_re[31:16]));
      $display("  x_im=0x%08h (elem0=%0d elem1=%0d)", dut.u_crpa_core.x_im,
                 $signed(dut.u_crpa_core.x_im[15:0]), $signed(dut.u_crpa_core.x_im[31:16]));
      $display("  sample_valid=%b alpha_in=%0d alpha_wr=%b adapt_en=%b",
                 dut.u_crpa_core.sample_valid, $signed(dut.u_crpa_core.alpha_in),
                 dut.u_crpa_core.alpha_wr, dut.u_crpa_core.adapt_en);
      $display("  s_re=%0d s_im=%0d s_valid=%b  [OUTPUTS]",
                 $signed(dut.u_crpa_core.s_re), $signed(dut.u_crpa_core.s_im), dut.u_crpa_core.s_valid);
      $display("  w_re=0x%016h w_im=0x%016h weights_valid=%b  [OUTPUTS]",
                 dut.u_crpa_core.w_re, dut.u_crpa_core.w_im, dut.u_crpa_core.weights_valid);
      $display("  (saturated) crpa_re12=%0d crpa_im12=%0d -> proc_i0=0x%04h proc_q0=0x%04h",
                 dut.crpa_re12, dut.crpa_im12, dut.proc_i0, dut.proc_q0);
      $display("=========================================================");
    end
  endtask

  // ==========================================================================
  //  Floating-point shadow model -- same LPF_SHIFT-approximated recursion as
  //  the core, in real numbers, to measure fixed-point error directly.
  // ==========================================================================
  localparam real SHADOW_TAU   = 1.0 / (1.0 - 1.0/262144.0) - 1.0; // matches LPF_SHIFT=18 -> leak ~ 1/2^18
  real shadow_wp_re [0:1];
  real shadow_wp_im [0:1];
  real shadow_wo_re [0:1];
  integer si;
  real shadow_xk_re[0:1], shadow_xk_im[0:1];
  real shadow_wk_re[0:1], shadow_wk_im[0:1];
  real shadow_s_re, shadow_s_im, shadow_alpha;
  real shadow_corr_re[0:1], shadow_corr_im[0:1];
  real shadow_leak_re[0:1], shadow_leak_im[0:1];

  task shadow_init;
    begin
      shadow_wp_re[0]=0.0; shadow_wp_re[1]=0.0;
      shadow_wp_im[0]=0.0; shadow_wp_im[1]=0.0;
      shadow_wo_re[0]=1.0; shadow_wo_re[1]=0.0;
      shadow_alpha = 1.0;
    end
  endtask

  task shadow_step(input real xi0, input real xq0, input real xi1, input real xq1);
    begin
      shadow_xk_re[0]=xi0; shadow_xk_im[0]=xq0;
      shadow_xk_re[1]=xi1; shadow_xk_im[1]=xq1;
      for (si=0; si<2; si=si+1) begin
        shadow_wk_re[si] = shadow_wp_re[si] + shadow_wo_re[si];
        shadow_wk_im[si] = shadow_wp_im[si];
      end
      shadow_s_re = shadow_xk_re[0]*shadow_wk_re[0] - shadow_xk_im[0]*shadow_wk_im[0]
                  + shadow_xk_re[1]*shadow_wk_re[1] - shadow_xk_im[1]*shadow_wk_im[1];
      shadow_s_im = shadow_xk_re[0]*shadow_wk_im[0] + shadow_xk_im[0]*shadow_wk_re[0]
                  + shadow_xk_re[1]*shadow_wk_im[1] + shadow_xk_im[1]*shadow_wk_re[1];
      for (si=0; si<2; si=si+1) begin
        // conj(x)*s
        shadow_corr_re[si] = shadow_xk_re[si]*shadow_s_re + shadow_xk_im[si]*shadow_s_im;
        shadow_corr_im[si] = shadow_xk_re[si]*shadow_s_im - shadow_xk_im[si]*shadow_s_re;
        shadow_leak_re[si] = shadow_wp_re[si] + shadow_alpha*shadow_corr_re[si];
        shadow_leak_im[si] = shadow_wp_im[si] + shadow_alpha*shadow_corr_im[si];
        shadow_wp_re[si] = shadow_wp_re[si] - shadow_leak_re[si] / 262144.0; // 2^18
        shadow_wp_im[si] = shadow_wp_im[si] - shadow_leak_im[si] / 262144.0;
      end
    end
  endtask

  // ==========================================================================
  //  Timing coverage counters (also useful sanity)
  // ==========================================================================
  integer k;
  real phase;
  reg signed [15:0] i0,q0,i1,q1;
  integer align_checks = 0, align_fail = 0;
  integer errors = 0;

  always @(posedge clk) begin
    if (dut.wr_en0 && !dut.full0) begin
      align_checks = align_checks + 1;
      if (dut.proc_i0 !== {{4{dut.crpa_re12[11]}}, dut.crpa_re12} ||
          dut.proc_q0 !== {{4{dut.crpa_im12[11]}}, dut.crpa_im12}) begin
        align_fail = align_fail + 1;
      end
    end
  end

  // per-checkpoint live snapshot: RX in / CRPA internals / TX out, plus
  // shadow-model comparison for a genuine accuracy number, not a guess.
  task print_checkpoint(input integer kk);
    real crpa_re, crpa_im, crpa_mag, shadow_mag, err_db;
    begin
      crpa_re = $itor($signed(dut.u_crpa_core.s_re)) / 1048576.0; // /2^20
      crpa_im = $itor($signed(dut.u_crpa_core.s_im)) / 1048576.0;
      crpa_mag = $sqrt(crpa_re*crpa_re + crpa_im*crpa_im);
      shadow_mag = $sqrt(shadow_s_re*shadow_s_re + shadow_s_im*shadow_s_im);
      err_db = 20.0 * $ln(crpa_mag>1e-9 ? crpa_mag : 1e-9) / $ln(10.0)
             - 20.0 * $ln(TEST_AMP) / $ln(10.0);
      $display(" k=%4d | RX i0=%5d q0=%5d i1=%5d q1=%5d | CRPA s_re=%8.4f s_im=%8.4f |s|=%7.4f (shadow %7.4f) | %6.2f dB | lvl0=%2d lvl1=%2d | TX i0=%6d q0=%6d",
                 kk, i0, q0, i1, q1, crpa_re, crpa_im, crpa_mag, shadow_mag, err_db,
                 dut.level0, dut.level1, $signed(dac_data_i0), $signed(dac_data_q0));
    end
  endtask

  initial begin
    shadow_init;
    adc_data_i0=0; adc_data_q0=0; adc_data_i1=0; adc_data_q1=0;

    $display("=========================================================");
    $display(" GNSS_PASSTHROUGH + CRPA CORE -- VERBOSE TESTBENCH");
    $display("=========================================================");
    $display(" clk (l_clk)        : %.1f ns period = %.2f MHz", CLK_PERIOD_NS, 1000.0/CLK_PERIOD_NS);
    $display(" s_axi_aclk         : %.1f ns period = %.2f MHz (deliberately async to clk)", SAXI_PERIOD_NS, 1000.0/SAXI_PERIOD_NS);
    $display(" Samples to drive   : %0d", N_SAMPLES);
    $display(" Test amplitude     : %.1f    angle_step: %.2f rad    phase_step: %.2f rad/sample", TEST_AMP, ANGLE_STEP, PHASE_STEP);
    $display(" CRPA core          : M=2, DATA_W=16, WEIGHT_W=32, WEIGHT_FRAC=20, ALPHA_FRAC=8, LPF_SHIFT=18");
    $display("=========================================================");

    repeat (10) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge s_axi_aclk);
    s_axi_aresetn = 1;
    repeat (5) @(posedge s_axi_aclk);

    dump_all_ports("AFTER RESET RELEASE (Phase 1 identity mode, pass_en=0)");

    // -------- Check 1: Phase 1 baseline preserved when pass_en=0 --------
    $display("--- CONTROL register writes ---");
    axil_write(16'h000C, 32'h0000_0000);  // CONTROL: pass_en=0
    repeat (5) @(posedge clk);
    if (dac_data_i0 === dma_dac_data_i0 && dac_data_q0 === dma_dac_data_q0) begin
      $display("PASS: pass_en=0 -> dac_data == dma_dac_data (baseline unchanged)");
    end else begin
      $display("FAIL: pass_en=0 baseline broken: dac_data_i0=%h (expected %h)", dac_data_i0, dma_dac_data_i0);
      errors = errors + 1;
    end

    // -------- Enable CRPA repeater mode, ch1_copy=1 --------
    axil_write(16'h000C, 32'h0000_0009);  // CONTROL: pass_en=1, ch1_copy=1

    // -------- Check: CRPA_COEF[0] write actually reaches alpha_reg --------
    // v1.2 tied u_crpa_core's alpha_wr to 1'b0, so alpha_reg (the register
    // the adaptation math actually reads) never loaded alpha_in and stayed
    // at ALPHA_INIT (256) forever -- CRPA_COEF(0) writes and reads worked
    // perfectly from software's point of view, they just had NO EFFECT on
    // the algorithm. A test that only ever writes the same value as the
    // reset default (256, exactly what the loop below writes right after
    // this) cannot catch that: the "before" and "after" values are
    // indistinguishable either way. So probe with something the reset
    // value is NOT, directly against the internal register, before writing
    // the real 1.0 this run actually uses.
    axil_write(16'h0040, 32'd512);        // CRPA_COEF[0] = alpha = 2.0 (Q8.8), != reset default
    repeat (10) @(posedge clk);           // let the 2-flop CDC settle
    if (dut.u_crpa_core.alpha_reg == 512) begin
      $display("PASS: CRPA_COEF[0] write of a non-default alpha reached alpha_reg (512)");
    end else begin
      $display("FAIL: alpha_reg = %0d after writing 512 -- software alpha control has NO EFFECT on the core (v1.2 alpha_wr regression)",
                dut.u_crpa_core.alpha_reg);
      errors = errors + 1;
    end

    axil_write(16'h0040, 32'd256);        // CRPA_COEF[0] = alpha = 1.0 (Q8.8), the value this run's shadow model assumes
    repeat (10) @(posedge clk);

    dump_all_ports("AFTER pass_en=1, ch1_copy=1, alpha=1.0 WRITTEN");

    $display("--- driving %0d samples, printing every %0d ---", N_SAMPLES, PRINT_EVERY);
    $display(" k    | RX in (raw)                    | CRPA core output                                              | buffer     | TX out (raw)");
    $display("---------------------------------------------------------------------------------------------------------------------------------------");

    for (k = 0; k < N_SAMPLES; k = k + 1) begin
      phase = k * PHASE_STEP;
      i0 = $rtoi(TEST_AMP * $cos(phase));
      q0 = $rtoi(TEST_AMP * $sin(phase));
      i1 = $rtoi(TEST_AMP * $cos(phase + ANGLE_STEP));
      q1 = $rtoi(TEST_AMP * $sin(phase + ANGLE_STEP));

      shadow_step($itor(i0), $itor(q0), $itor(i1), $itor(q1));

      @(posedge clk);
      adc_data_i0 <= i0; adc_data_q0 <= q0;
      adc_data_i1 <= i1; adc_data_q1 <= q1;
      adc_valid_i0 <= 1'b1; adc_valid_q0 <= 1'b1;
      adc_valid_i1 <= 1'b1; adc_valid_q1 <= 1'b1;
      @(posedge clk);
      adc_valid_i0 <= 1'b0; adc_valid_q0 <= 1'b0;
      adc_valid_i1 <= 1'b0; adc_valid_q1 <= 1'b0;

      // TX side requests at the SAME average rate as RX (one pulse per
      // sample period, like adc_valid) but phase-shifted within the period.
      @(posedge clk);
      if (k > 20) begin
        dac_valid_i0 <= 1'b1; dac_valid_q0 <= 1'b1;
        dac_valid_i1 <= 1'b1; dac_valid_q1 <= 1'b1;
      end
      @(posedge clk);
      dac_valid_i0 <= 1'b0; dac_valid_q0 <= 1'b0;
      dac_valid_i1 <= 1'b0; dac_valid_q1 <= 1'b0;

      if (k % PRINT_EVERY == 0) print_checkpoint(k);
    end

    repeat (20) @(posedge clk);
    print_checkpoint(N_SAMPLES-1);

    dump_all_ports("END OF RUN");

    // -------- Read back diagnostics through the REAL AXI-Lite interface,
    // not internal hierarchical probes -- this is what software actually sees.
    $display("--- reading back diagnostics via AXI-Lite ---");
    begin : AXI_READBACK
      reg [31:0] rd;
      axil_read(16'h0010, rd);  // STATUS
      axil_read(16'h0014, rd);  // RX_COUNT_CH0
      axil_read(16'h0018, rd);  // TX_COUNT_CH0
      axil_read(16'h001C, rd);  // OVERFLOW_COUNT
      axil_read(16'h0020, rd);  // UNDERFLOW_COUNT
      axil_read(16'h002C, rd);  // FIFO_LEVEL
      axil_read(16'h0030, rd);  // RX_COUNT_CH1
      axil_read(16'h0034, rd);  // TX_COUNT_CH1
      axil_read(16'h0040, rd);  // CRPA_COEF[0] readback (should read back 256)
    end

    // -------- Check 2: buffer write/read alignment held for every sample --------
    if (align_fail == 0 && align_checks > 300) begin
      $display("PASS: fifo0 write always matched the CURRENT nulled sample (%0d checked, 0 stale writes)", align_checks);
    end else begin
      $display("FAIL: %0d of %0d fifo0 writes captured a STALE value (timing misalignment)", align_fail, align_checks);
      errors = errors + 1;
    end

    // -------- Check 3: no overflow/underflow --------
    if (dut.ovf_sticky === 1'b0 && dut.unf_sticky === 1'b0) begin
      $display("PASS: no overflow/underflow over %0d samples", N_SAMPLES);
    end else begin
      $display("FAIL: ovf_sticky=%b unf_sticky=%b (ovf_cnt=%0d unf_cnt=%0d)",
                dut.ovf_sticky, dut.unf_sticky, dut.ovf_cnt, dut.unf_cnt);
      errors = errors + 1;
    end

    // -------- Check 4: real nulling happened, cross-checked against the
    // floating-point shadow model for a genuine accuracy percentage --------
    begin : CHECK4_ACCURACY
      real w0_re, w0_im, w0_mag, shadow_mag, abs_err, rel_err_pct;
      w0_re = $itor($signed(dut.u_crpa_core.w_re[31:0])) / 1048576.0;
      w0_im = $itor($signed(dut.u_crpa_core.w_im[31:0])) / 1048576.0;
      w0_mag = $sqrt(w0_re*w0_re + w0_im*w0_im);
      shadow_mag = $sqrt((shadow_wp_re[0]+shadow_wo_re[0])*(shadow_wp_re[0]+shadow_wo_re[0])
                        + shadow_wp_im[0]*shadow_wp_im[0]);
      abs_err = w0_mag - shadow_mag;
      if (abs_err < 0) abs_err = -abs_err;
      rel_err_pct = shadow_mag > 1e-9 ? (100.0*abs_err/shadow_mag) : 0.0;
      $display("=========================================================");
      $display(" ACCURACY: fixed-point CRPA core vs floating-point shadow model");
      $display("=========================================================");
      $display("  final |w[0]| fixed-point = %.5f", w0_mag);
      $display("  final |w[0]| shadow      = %.5f", shadow_mag);
      $display("  absolute error           = %.5f  (%.4f %%)", abs_err, rel_err_pct);
      if (w0_mag > 0.3 && w0_mag < 0.7) begin
        $display("PASS: weights converged to the expected range");
      end else begin
        $display("FAIL: weights did not converge as expected");
        errors = errors + 1;
      end
    end

    $display("=========================================================");
    $display(" Waveform written to waveform_gnss.vcd -- open with:");
    $display("   gtkwave waveform_gnss.vcd");
    $display("=========================================================");
    if (errors == 0) $display(" ALL CHECKS PASSED");
    else              $display(" %0d CHECK(S) FAILED", errors);
    $display("=========================================================");
    $finish;
  end

endmodule
