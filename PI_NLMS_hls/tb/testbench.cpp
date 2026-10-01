#include "pi_nlms.h"
#include "hls_stream.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include <cstdint>

int main() {
    // ------------------------------------------------------------------------
    // Simulation Parameters for 50 MHz Target Sample Rate
    // ------------------------------------------------------------------------
    const double FS = 50.0e6;                   // 50 MHz Sampling Rate
    const double T_SIM = 0.010;                 // 10 ms simulation time
    const int NUM_SAMPLES = (int)(FS * T_SIM);   // 500,000 samples total

    const int ACCUM_LEN = 1024;
    const double JAMMER_FREQ = 5.0e6;            // 5.0 MHz CW Jammer offset
    const double JAMMER_AMP = 12000.0;           // ~-12 dBFS (16-bit max is 32767)
    const double NOISE_AMP = 200.0;              // AWGN noise level
    const double PHASE_DIFF = M_PI / 3.0;        // 60-degree spatial phase delta

    const int16_t MU_SHIFT_CTRL = 0;             // Matches devmem 0x40000010 0x0
    const int16_t RESERVED_CTRL = 0;

    // AXI Streams for DUT interface
    hls::stream<axis_t> in1_stream("in1_stream");
    hls::stream<axis_t> in2_stream("in2_stream");
    hls::stream<axis_t> out_stream("out_stream");

    std::ofstream csv_file("pi_nlms_50mhz_results.csv");
    csv_file << "sample,in1_i,in1_q,in2_i,in2_q,out_i,out_q,out_power\n";

    std::cout << "========================================================\n";
    std::cout << " Running pi_nlms C-Simulation at Fs = 50 MHz\n";
    std::cout << " Total Samples: " << NUM_SAMPLES << " (" << T_SIM * 1000.0 << " ms)\n";
    std::cout << " Weight Update Interval: " << (ACCUM_LEN / FS) * 1e6 << " us\n";
    std::cout << "========================================================\n";

    double initial_power = 0.0;
    double final_power = 0.0;
    const int eval_window = 2048;
    const int late_window_start = NUM_SAMPLES - eval_window;

    int output_count = 0;

    // ------------------------------------------------------------------------
    // Sample-by-Sample Stream Simulation
    // ------------------------------------------------------------------------
    for (int n = 0; n < NUM_SAMPLES; n++) {
        double t = (double)n / FS;

        // Generate correlated jammer signals with channel phase difference
        double j_i1 = JAMMER_AMP * cos(2.0 * M_PI * JAMMER_FREQ * t);
        double j_q1 = JAMMER_AMP * sin(2.0 * M_PI * JAMMER_FREQ * t);

        double j_i2 = JAMMER_AMP * cos(2.0 * M_PI * JAMMER_FREQ * t + PHASE_DIFF);
        double j_q2 = JAMMER_AMP * sin(2.0 * M_PI * JAMMER_FREQ * t + PHASE_DIFF);

        // Thermal noise
        double n_i1 = ((rand() % 2000) - 1000) / 1000.0 * NOISE_AMP;
        double n_q1 = ((rand() % 2000) - 1000) / 1000.0 * NOISE_AMP;
        double n_i2 = ((rand() % 2000) - 1000) / 1000.0 * NOISE_AMP;
        double n_q2 = ((rand() % 2000) - 1000) / 1000.0 * NOISE_AMP;

        int16_t in1_i = (int16_t)(j_i1 + n_i1);
        int16_t in1_q = (int16_t)(j_q1 + n_q1);
        int16_t in2_i = (int16_t)(j_i2 + n_i2);
        int16_t in2_q = (int16_t)(j_q2 + n_q2);

        // Pack into AXI-Stream words (Lower 16-bits = I, Upper 16-bits = Q)
        axis_t val1, val2;
        val1.data = ((uint32_t)(uint16_t)in1_q << 16) | ((uint32_t)(uint16_t)in1_i & 0xFFFFu);
        val1.keep = -1;
        val1.strb = -1;
        val1.last = (n == NUM_SAMPLES - 1) ? 1 : 0;

        val2.data = ((uint32_t)(uint16_t)in2_q << 16) | ((uint32_t)(uint16_t)in2_i & 0xFFFFu);
        val2.keep = -1;
        val2.strb = -1;
        val2.last = (n == NUM_SAMPLES - 1) ? 1 : 0;

        // Push inputs into streaming interface
        in1_stream.write(val1);
        in2_stream.write(val2);

        // Execute top-level HLS function
        pi_nlms(in1_stream, in2_stream, MU_SHIFT_CTRL, RESERVED_CTRL, out_stream);

        // Read output from streaming interface
        if (!out_stream.empty()) {
            axis_t val_out = out_stream.read();

            int16_t out_i = (int16_t)(uint16_t)(val_out.data & 0xFFFFu);
            int16_t out_q = (int16_t)(uint16_t)((val_out.data >> 16) & 0xFFFFu);

            double pwr = (double)out_i * (double)out_i + (double)out_q * (double)out_q;

            // Log every 20th sample to CSV
            if (output_count % 20 == 0) {
                csv_file << output_count << "," << in1_i << "," << in1_q << ","
                         << in2_i << "," << in2_q << ","
                         << out_i << "," << out_q << "," << pwr << "\n";
            }

            // Power logging for pre- vs post-convergence
            if (output_count >= ACCUM_LEN && output_count < ACCUM_LEN + eval_window) {
                initial_power += pwr;
            } else if (output_count >= late_window_start) {
                final_power += pwr;
            }

            output_count++;
        }
    }

    csv_file.close();

    initial_power /= eval_window;
    final_power /= eval_window;

    double suppression_db = 10.0 * log10((initial_power + 1e-6) / (final_power + 1e-6));

    std::cout << "\n--- TEST RESULTS (50 MHz) ---" << std::endl;
    std::cout << "Initial Output Power (Before Nulling) : " << initial_power << std::endl;
    std::cout << "Final Output Power   (After Nulling)  : " << final_power << std::endl;
    std::cout << "Jammer Suppression Depth               : " << suppression_db << " dB" << std::endl;

    if (suppression_db > 15.0) {
        std::cout << "\nSTATUS: PASS - Core converged and suppressed jammer at 50 MHz!" << std::endl;
        return 0;
    } else {
        std::cout << "\nSTATUS: FAIL - Convergence failed at 50 MHz." << std::endl;
        return 1;
    }
}
