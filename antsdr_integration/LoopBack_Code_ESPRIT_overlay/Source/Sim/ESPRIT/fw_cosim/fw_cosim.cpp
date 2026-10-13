// ============================================================================
//  fw_cosim.cpp -- firmware / RTL co-simulation of the ESPRIT DOA monitor
// ----------------------------------------------------------------------------
//  Runs the UNMODIFIED Source/Firmware/app_gnss_e310/gnss_doa.c against the
//  UNMODIFIED gnss_passthrough RTL (Verilator model):
//    gnss_pt_read / gnss_pt_write -> real AXI4-Lite transactions
//    no_os_mdelay / no_os_udelay  -> simulated time (100 MHz AXI clock)
//  RX1/RX2 carry a synthetic far-field QPSK source at a chosen angle plus a
//  fixed "board" phase offset on RX2 and AWGN, 12-bit, 30.72 MS/s.
//
//  Sequence: boot check (probe + self-test S1..S6, live enable), live angle
//  readout with the board offset present, calibration with a common signal
//  (true phase 0), live readout at several angles after calibration, and
//  the test-generator command. Prints FW_COSIM_RESULT: PASS|FAIL.
//
//  HLS mode (-DHLS_MODEL=0|1, GNSS_DOA_IMPL=hls in run_fw_cosim.sh): the RTL
//  is built with HLS_CORE = 1 and the Vitis HLS C++ kernel
//  (Source/HLS/esprit_hls, esprit_iu_hls or esprit_m_hls) is called as the
//  external HLS IP on m_axis_doa / s_axis_res, with a compute stall per
//  window, so the same firmware runs against the HLS block-design variant.
//
//  Build/run: ./run_fw_cosim.sh  (needs verilator >= 4.2 and g++)
// ============================================================================
#include "Vgnss_passthrough.h"
#include "verilated.h"
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <random>

extern "C" {
#include "gnss_doa.h"
}



static Vgnss_passthrough *top;
static uint64_t t_ps = 0;                       // simulated time
static const uint64_t ACLK_HALF = 5000;         // 100 MHz
static const uint64_t LCLK_HALF = 4069;         // 122.88 MHz
static uint64_t next_a = ACLK_HALF, next_l = LCLK_HALF;
static unsigned lph = 0;

#ifdef HLS_MODEL
#include <deque>
#include "esprit_hls.h"
// ---- the HLS IP as a cycle-level stand-in ------------------------------------
//  s_axis_x : accepts one snapshot per aclk while not busy (tready = !busy)
//  compute  : when 2^log2_snap snapshots are buffered, run the C++ kernel
//             (exactly what Vitis HLS synthesises) and stall HLS_BUSY cycles
//  m_axis_y : one result word per aclk
static std::deque<uint64_t> hls_in;
static std::deque<uint32_t> hls_out;
static int  hls_busy = 0;
static bool hls_res_driven = false;
static const int HLS_BUSY = 400;               // ~ 4 us at 100 MHz per window
static unsigned long hls_windows = 0;

static void hls_after_rising_edge(bool pre_tvalid, uint64_t pre_tdata, bool pre_rst_n) {
    if (!pre_rst_n) {
        hls_in.clear(); hls_out.clear(); hls_busy = 0; hls_res_driven = false;
    } else {
        if (hls_res_driven && !hls_out.empty()) hls_out.pop_front();    // consumed (tready = 1)
        if (pre_tvalid && top->m_axis_doa_tready) hls_in.push_back(pre_tdata);
        if (hls_busy > 0) hls_busy--;
        int L = top->doa_cfg_log2_snap;
        if (L < ESPRIT_MIN_LOG2) L = ESPRIT_MIN_LOG2;
        if (L > ESPRIT_MAX_LOG2) L = ESPRIT_MAX_LOG2;
        size_t N = (size_t)1 << L;
        if (hls_busy == 0 && hls_in.size() >= N) {
            hls::stream<snap_t> in; hls::stream<word_t> out;
            for (size_t n = 0; n < N; n++) { in.write((snap_t)hls_in.front()); hls_in.pop_front(); }
#if HLS_MODEL == 0
            esprit_iu_hls(in, out, top->doa_cfg_log2_snap, top->doa_cfg_p_log2, top->doa_cfg_use_cov,
                          top->doa_cfg_mu_q30, (int)top->doa_cfg_k_q30);
#else
            esprit_m_hls(in, out, top->doa_cfg_log2_snap, top->doa_cfg_p_log2, top->doa_cfg_use_cov,
                         top->doa_cfg_mu_q30, (int)top->doa_cfg_k_q30, top->doa_cfg_uctol_q24);
#endif
            while (!out.empty()) hls_out.push_back((uint32_t)out.read());
            hls_busy = HLS_BUSY;
            hls_windows++;
        }
    }
    bool rst_n = top->doa_hls_rst_n;
    top->m_axis_doa_tready = (rst_n && hls_busy == 0) ? 1 : 0;
    hls_res_driven = rst_n && !hls_out.empty();
    top->s_axis_res_tvalid = hls_res_driven ? 1 : 0;
    top->s_axis_res_tdata  = hls_res_driven ? hls_out.front() : 0;
}
#endif

// ---- RF scene ---------------------------------------------------------------
static double scene_theta_deg = 0.0;            // true DOA
static double scene_offset_deg = 37.0;          // RX2 board/cable phase offset
static double scene_snr_db = 20.0;
static std::mt19937 rng(7);
static std::normal_distribution<double> gauss(0.0, 1.0);
static std::uniform_int_distribution<int> sym(0, 3);
static int cur_sym = 0, sym_cnt = 0;

static int16_t q12(double v) {
    long r = lround(v * 2048.0);
    if (r > 2047) r = 2047;
    if (r < -2048) r = -2048;
    return (int16_t)r;
}

static void drive_adc() {
    // one snapshot every 4 l_clk cycles (30.72 MS/s)
    if (lph == 0) {
        if (++sym_cnt >= 4) { sym_cnt = 0; cur_sym = sym(rng); }   // QPSK, 4 samples/symbol
        double ph  = M_PI / 4 + M_PI / 2 * cur_sym;
        double amp = 0.35;
        double dphi = -M_PI * sin(scene_theta_deg * M_PI / 180.0) + scene_offset_deg * M_PI / 180.0;
        double ns  = amp * pow(10.0, -scene_snr_db / 20.0) / sqrt(2.0);
        double x0i = amp * cos(ph) + ns * gauss(rng), x0q = amp * sin(ph) + ns * gauss(rng);
        double x1i = amp * cos(ph + dphi) + ns * gauss(rng), x1q = amp * sin(ph + dphi) + ns * gauss(rng);
        top->adc_data_i0 = (uint16_t)q12(x0i); top->adc_data_q0 = (uint16_t)q12(x0q);
        top->adc_data_i1 = (uint16_t)q12(x1i); top->adc_data_q1 = (uint16_t)q12(x1q);
        top->adc_valid_i0 = top->adc_valid_q0 = top->adc_valid_i1 = top->adc_valid_q1 = 1;
    } else {
        top->adc_valid_i0 = top->adc_valid_q0 = top->adc_valid_i1 = top->adc_valid_q1 = 0;
    }
    top->dac_valid_i0 = top->dac_valid_q0 = top->dac_valid_i1 = top->dac_valid_q1 = (lph == 2);
    lph = (lph + 1) & 3;
}

// advance to the next clock edge (either clock)
static void step_edge() {
    if (next_a <= next_l) {
        t_ps = next_a; next_a += ACLK_HALF;
#ifdef HLS_MODEL
        bool rising = !top->s_axi_aclk;
        bool pre_tvalid = top->m_axis_doa_tvalid;
        uint64_t pre_tdata = top->m_axis_doa_tdata;
        bool pre_rst_n = top->doa_hls_rst_n;
        top->s_axi_aclk = !top->s_axi_aclk;
        top->eval();
        if (rising) hls_after_rising_edge(pre_tvalid, pre_tdata, pre_rst_n);
#else
        top->s_axi_aclk = !top->s_axi_aclk;
#endif
    } else {
        t_ps = next_l; next_l += LCLK_HALF;
        top->clk = !top->clk;
        if (top->clk) drive_adc();             // inputs change after the rising edge
    }
    top->eval();
}

// run until the next rising edge of aclk
static void aclk_cycle() {
    do { step_edge(); } while (!(top->s_axi_aclk == 1 && t_ps == next_a - ACLK_HALF));
}

static void aclk_cycles(uint64_t n) { while (n--) aclk_cycle(); }

// ---- firmware platform hooks -------------------------------------------------
extern "C" uint32_t gnss_pt_read(uint32_t off) {
    top->s_axi_araddr = off; top->s_axi_arvalid = 1;
    aclk_cycle();
    int guard = 0;
    while (!top->s_axi_rvalid && guard++ < 100) aclk_cycle();
    uint32_t v = top->s_axi_rdata;
    top->s_axi_arvalid = 0;
    aclk_cycle();
    return v;
}

extern "C" void gnss_pt_write(uint32_t off, uint32_t v) {
    top->s_axi_awaddr = off; top->s_axi_wdata = v; top->s_axi_wstrb = 0xF;
    top->s_axi_awvalid = 1; top->s_axi_wvalid = 1;
    aclk_cycle();
    int guard = 0;
    while (!top->s_axi_awready && guard++ < 100) aclk_cycle();
    top->s_axi_awvalid = 0; top->s_axi_wvalid = 0;
    guard = 0;
    while (!top->s_axi_bvalid && guard++ < 100) aclk_cycle();
    aclk_cycle();
}

extern "C" void no_os_mdelay(uint32_t ms) { aclk_cycles(100000ULL * ms); }
extern "C" void no_os_udelay(uint32_t us) { aclk_cycles(100ULL * us); }

// ---- checks ------------------------------------------------------------------
static int fails = 0;
static void check(const char *what, bool ok) {
    printf("FW_COSIM: %-62s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) fails++;
}

// estimators present in this build (DOA_CAPS bit 0 = I-UESPRIT, bit 1 = M-ESPRIT)
static bool has_i = true, has_m = true;

// avg_deg(true) = the primary estimator: M-ESPRIT if built, else I-UESPRIT
static double avg_deg(bool m) {
    if (m && !has_m) m = false;
    return (int32_t)gnss_pt_read(m ? DOA_REG_M_AVG_DEG : DOA_REG_I_AVG_DEG) / 65536.0;
}
static void wait_blocks(int n) {
    uint32_t reg = has_m ? DOA_REG_M_AVG_SEQ : DOA_REG_I_AVG_SEQ;
    uint32_t b = gnss_pt_read(reg);
    while ((int)(gnss_pt_read(reg) - b) < n) no_os_udelay(50);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vgnss_passthrough;
    top->clk = 0; top->s_axi_aclk = 0; top->rst = 1; top->s_axi_aresetn = 0;
    top->adc_enable_i0 = top->adc_enable_q0 = top->adc_enable_i1 = top->adc_enable_q1 = 1;
    top->dac_enable_i0 = top->dac_enable_q0 = top->dac_enable_i1 = top->dac_enable_q1 = 1;
    top->s_axi_bready = 1; top->s_axi_rready = 1;
    top->eval();
    aclk_cycles(20);
    top->rst = 0; top->s_axi_aresetn = 1;
    aclk_cycles(20);

    printf("FW_COSIM: ===== firmware gnss_doa.c  <->  RTL gnss_passthrough v1.2 =====\n");
    printf("FW_COSIM: scene: QPSK source, SNR %.0f dB, RX2 board offset %+.1f deg\n",
           scene_snr_db, scene_offset_deg);
    {
        uint32_t caps = gnss_pt_read(DOA_REG_CAPS);
        has_i = (caps & 1U) != 0; has_m = (caps & 2U) != 0;
        printf("FW_COSIM: estimators built: %s%s\n", has_i ? "I-UESPRIT " : "", has_m ? "M-ESPRIT" : "");
    }

    // 1. boot check exactly as main.c calls it
    scene_theta_deg = 10.0;
    int32_t r = gnss_doa_boot_check();
    check("boot: gnss_doa_boot_check() self-test S1..S6 returns DOA_OK", r == DOA_OK);

    // 2. live, uncalibrated: the board offset biases the angle
    wait_blocks(2);
    printf("FW_COSIM: --- doa?  (live, true 10 deg, NOT calibrated) ---\n");
    gnss_doa_print_result();
    double unc = avg_deg(true);
    double exp_unc = -asin(fmin(1.0, fmax(-1.0, (-M_PI * sin(10.0 * M_PI / 180) + scene_offset_deg * M_PI / 180) / M_PI))) * 180 / M_PI;
    printf("FW_COSIM: uncalibrated %s %.3f deg, expected with offset %.3f deg\n", has_m ? "M-ESPRIT" : "I-UESPRIT", unc, exp_unc);
    check("live uncalibrated angle shows the board offset (within 0.5 deg)", fabs(unc - exp_unc) < 0.5);

    // 3. calibration with a common signal (true phase 0)
    printf("FW_COSIM: --- doa_cal=1 (splitter: both RX see the same signal) ---\n");
    scene_theta_deg = 0.0;
    r = gnss_doa_calibrate();
    check("calibration completes", r == DOA_OK);
    wait_blocks(2);
    check("after calibration the common signal reads 0 deg (within 0.3)", fabs(avg_deg(true)) < 0.3);

    // 4. live angles after calibration
    const double angles[] = { -40.0, -15.0, 25.0, 50.0 };
    for (double a : angles) {
        scene_theta_deg = a;
        wait_blocks(2);
        double di = avg_deg(false), dm = avg_deg(true);
        printf("FW_COSIM: live true %+6.1f deg -> I-UESPRIT %+8.3f  M-ESPRIT %+8.3f\n", a, di, dm);
        char msg[96];
        snprintf(msg, sizeof msg, "live %+.0f deg after calibration, built estimators within 0.5 deg", a);
        check(msg, (!has_i || fabs(di - a) < 0.5) && (!has_m || fabs(dm - a) < 0.5));
    }
    printf("FW_COSIM: --- doa_monitor=3 ---\n");
    gnss_doa_monitor(3);

    // 5. test-generator command and register dump paths
    printf("FW_COSIM: --- doa_test=-22.5 ---\n");
    r = gnss_doa_test_angle(-22.5);
    check("doa_test=-22.5 completes", r == DOA_OK);
    // gnss_doa_test_angle() restored live mode; the reported value was printed
    // above. Re-measure through the register path to check it independently.
    {
        uint32_t c = gnss_pt_read(DOA_REG_CTRL), cal = gnss_pt_read(DOA_REG_CAL);
        check("doa_test restores live mode and calibration",
              !(c & DOA_CTRL_SRC_TEST) && (c & DOA_CTRL_ENABLE) && cal != 0x00007FFFU);
        gnss_pt_write(DOA_REG_CTRL, 0); gnss_pt_write(DOA_REG_CAL, 0x00007FFF);
        gnss_pt_write(DOA_REG_CTRL, c | DOA_CTRL_SRC_TEST);
        wait_blocks(2);
        if (has_m) check("test generator at -22.5 deg: M-ESPRIT within 0.05 deg", fabs(avg_deg(true) + 22.5) < 0.05);
        if (has_i) check("test generator at -22.5 deg: I-UESPRIT within 0.05 deg", fabs(avg_deg(false) + 22.5) < 0.05);
        gnss_pt_write(DOA_REG_CTRL, 0); gnss_pt_write(DOA_REG_CAL, cal); gnss_pt_write(DOA_REG_CTRL, c);
    }
    gnss_doa_print_status();

#ifdef HLS_MODEL
    printf("FW_COSIM: HLS kernel (%s) processed %lu windows\n",
           HLS_MODEL == 0 ? "esprit_iu_hls" : "esprit_m_hls", hls_windows);
    check("estimator results came from the HLS kernel (DOA_CAPS bit 2 set)",
          (gnss_pt_read(DOA_REG_CAPS) & 4U) != 0 && hls_windows > 0);
#endif
    printf("FW_COSIM: simulated time %.3f ms\n", t_ps / 1e9);
    printf("FW_COSIM_RESULT: %s\n", fails ? "FAIL" : "PASS");
    delete top;
    return fails ? 1 : 0;
}
