`timescale 1ns/1ps
// ============================================================================
// Self-checking testbench for the FPGA rewrite.
// Trick: CLK_HZ = 1 MHz and SAMPLE_DIV = 1  ->  FS = 1 MHz, so the tuning
// words / frequencies are IDENTICAL to the board build (100 MHz / 100), but
// simulation needs 100x fewer clock cycles.
// Run:  iverilog -g2005 -o sim tb_adaptive_sonar_hw.v adaptive_sonar_top.v \
//                       param_controller.v dds_waveform_gen.v && vvp sim
// ============================================================================
module tb_adaptive_sonar_hw;

    reg        clk = 0;
    reg        rst = 1;
    reg  [7:0] depth = 8'd50, salinity = 8'd50, noise_level = 8'd50;

    wire signed [7:0] sample_out;
    wire        [7:0] dac_data;
    wire        sample_valid, tx_active, pulse_start;
    wire [1:0]  waveform_mode, active_mode;
    wire [4:0]  rom_addr_debug;

    adaptive_sonar_top #(.CLK_HZ(1_000_000), .SAMPLE_DIV(1)) dut (
        .clk(clk), .rst(rst),
        .depth(depth), .salinity(salinity), .noise_level(noise_level),
        .sample_out(sample_out), .dac_data(dac_data), .sample_valid(sample_valid),
        .waveform_mode(waveform_mode), .active_mode(active_mode),
        .tx_active(tx_active), .pulse_start(pulse_start),
        .rom_addr_debug(rom_addr_debug)
    );

    always #500 clk = ~clk;                     // 1 MHz

    // ---------------- measurement: zero crossings & peak ----------------
    integer     xings;   real t_first, t_last;
    integer     peak;    reg signed [7:0] prev;
    real        meas_hz;
    always @(posedge clk) begin
        if (sample_valid) begin
            if ((prev < 0 && sample_out >= 0) || (prev >= 0 && sample_out < 0)) begin
                if (xings == 0) t_first = $realtime;
                t_last = $realtime;
                xings  = xings + 1;
            end
            if ((sample_out < 0 ? -sample_out : sample_out) > peak)
                peak = (sample_out < 0) ? -sample_out : sample_out;
        end
        prev <= sample_out;
    end
    task clear_meas; begin xings = 0; peak = 0; t_first = 0; t_last = 0; end endtask

    // ---------------- check: mode never changes mid-pulse ----------------
    reg [1:0] mode_at_start; integer mid_pulse_errors = 0;
    always @(posedge clk) begin
        if (pulse_start) mode_at_start <= active_mode;
        else if (tx_active && active_mode !== mode_at_start) mid_pulse_errors = mid_pulse_errors + 1;
    end

    integer errors = 0;

    task scenario;
        input [7:0] d, s, n;
        input [1:0] exp_mode;
        input real  exp_hz;        // expected mean frequency
        input real  tol;           // fractional tolerance
        input [8*12-1:0] name;
        begin
            depth = d; salinity = s; noise_level = n;
            @(posedge pulse_start);            // this ping may still carry old params
            @(posedge pulse_start);            // this one must use the new ones
            clear_meas;
            #1;
            if (active_mode !== exp_mode) begin
                $display("FAIL %0s: mode=%0d expected %0d", name, active_mode, exp_mode);
                errors = errors + 1;
            end
            @(negedge sample_valid);
            meas_hz = (xings > 1) ? ((xings - 1) / 2.0) / ((t_last - t_first) * 1e-9) : 0.0;
            $display("%0s: mode=%0d  mean f=%0.1f kHz  peak=%0d  (expect ~%0.1f kHz)",
                     name, active_mode, meas_hz/1e3, peak, exp_hz/1e3);
            if (meas_hz < exp_hz*(1.0-tol) || meas_hz > exp_hz*(1.0+tol)) begin
                $display("FAIL %0s: frequency out of range", name);
                errors = errors + 1;
            end
            if (peak < 20) begin
                $display("FAIL %0s: output amplitude too small (%0d)", name, peak);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("adaptive_sonar_hw.vcd");
        $dumpvars(0, tb_adaptive_sonar_hw);
        clear_meas; prev = 0;
        #3000 rst = 0;

        //            depth sal  noise  mode  Hz      tol   name
        scenario(8'd50,  8'd50, 8'd50,  2'd0, 60e3,  0.03, "calm");
        scenario(8'd50,  8'd50, 8'd200, 2'd1, 86e3,  0.08, "noisy");
        scenario(8'd200, 8'd50, 8'd50,  2'd0, 48e3,  0.03, "deep");
        scenario(8'd50,  8'd220,8'd50,  2'd2, 80e3,  0.08, "salinity");
        scenario(8'd50,  8'd50, 8'd50,  2'd0, 60e3,  0.03, "calm again");

        if (mid_pulse_errors != 0) begin
            $display("FAIL: mode changed mid-pulse %0d times", mid_pulse_errors);
            errors = errors + 1;
        end
        if (errors == 0) $display("ALL CHECKS PASSED");
        else             $display("%0d CHECK(S) FAILED", errors);
        $finish;
    end

    initial begin #100_000_000 $display("TIMEOUT"); $finish; end   // 100 ms sim guard
endmodule
