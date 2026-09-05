`timescale 1ns/1ps
// ============================================================================
// Testbench: tb_adaptive_sonar
// ----------------------------------------------------------------------------
// Applies a sequence of simulated environmental "scenarios" to the DUT
// *within the same simulation run*, so GTKWave shows one continuous trace
// where the output waveform visibly changes the moment the input changes.
// That single trace is the direct proof of "real-time adaptive" behaviour --
// note the cycle count between a stimulus edge and the parameter/waveform
// changing in the .vcd, and quote it (e.g. "adapts within N clock cycles").
// ============================================================================

module tb_adaptive_sonar;

    reg clk = 0;
    reg rst;
    reg [7:0] depth, salinity, noise_level;

    wire signed [7:0] sample_out;
    wire              sample_valid;
    wire [1:0]        waveform_mode;
    wire [4:0]        rom_addr_debug;

    adaptive_sonar_top dut (
        .clk(clk), .rst(rst),
        .depth(depth), .salinity(salinity), .noise_level(noise_level),
        .sample_out(sample_out), .sample_valid(sample_valid),
        .waveform_mode(waveform_mode), .rom_addr_debug(rom_addr_debug)
    );

    // 100 MHz clock (10ns period)
    always #5 clk = ~clk;

    initial begin
        $dumpfile("adaptive_sonar.vcd");
        $dumpvars(0, tb_adaptive_sonar);

        rst = 1; depth = 8'd50; salinity = 8'd50; noise_level = 8'd50;
        #20 rst = 0;

        // Scenario 1: calm, shallow water -> default CW
        depth = 8'd40; salinity = 8'd60; noise_level = 8'd30;
        #2000;

        // Scenario 2: deep water -> low-frequency CW, long pulse
        depth = 8'd180; salinity = 8'd60; noise_level = 8'd30;
        #2000;

        // Scenario 3: noisy shallow water -> LFM up-chirp
        depth = 8'd40; salinity = 8'd60; noise_level = 8'd200;
        #2000;

        // Scenario 4: high salinity -> LFM down-chirp
        depth = 8'd40; salinity = 8'd220; noise_level = 8'd30;
        #2000;

        // Back to calm baseline -> confirms it reverts, not just latches once
        depth = 8'd40; salinity = 8'd60; noise_level = 8'd30;
        #2000;

        $finish;
    end

    initial begin
        $monitor("t=%0t depth=%0d noise=%0d salinity=%0d mode=%0d addr=%0d sample=%0d",
                 $time, depth, noise_level, salinity, waveform_mode, rom_addr_debug, sample_out);
    end

endmodule
