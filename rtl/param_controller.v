// ============================================================================
// Adaptive Parameter Controller
// ----------------------------------------------------------------------------
// Maps simulated environmental sensor inputs (depth, salinity, noise) into
// sonar waveform control parameters (frequency, chirp rate, amplitude, pulse
// width, waveform mode). This block is fully synchronous (one clock cycle
// from sensor change -> new parameters latched), which is what lets us make
// a concrete, measurable claim about "real-time" adaptation latency.
// ============================================================================

module param_controller #(
    parameter FREQ_WIDTH   = 24,
    parameter SENSOR_WIDTH = 8
)(
    input  wire                            clk,
    input  wire                            rst,
    input  wire [SENSOR_WIDTH-1:0]         depth,        // simulated depth sensor
    input  wire [SENSOR_WIDTH-1:0]         salinity,     // simulated salinity sensor
    input  wire [SENSOR_WIDTH-1:0]         noise_level,  // simulated ambient noise

    output reg  [FREQ_WIDTH-1:0]           freq_word,     // base DDS tuning word
    output reg  signed [FREQ_WIDTH-1:0]    chirp_rate,    // per-cycle freq ramp (0 = CW)
    output reg  [7:0]                      amplitude,     // output amplitude scale
    output reg  [15:0]                     pulse_width,   // pulse length in clk cycles
    output reg  [1:0]                      waveform_mode  // 0=CW 1=up-chirp 2=down-chirp
);

    // Threshold constants represent design intent for the demo, not
    // calibrated physical values -- swap these for real sensor ranges later.
    localparam DEPTH_DEEP    = 8'd160;
    localparam NOISE_HIGH    = 8'd150;
    localparam SALINITY_HIGH = 8'd180;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            freq_word     <= {FREQ_WIDTH{1'b0}};
            chirp_rate    <= {FREQ_WIDTH{1'b0}};
            amplitude     <= 8'd0;
            pulse_width   <= 16'd0;
            waveform_mode <= 2'd0;
        end else begin
            // ---------------------------------------------------------
            // Priority-encoded adaptation rules. Only one condition wins
            // per cycle -- this keeps the logic simple and its latency
            // trivially provable in a waveform trace (see testbench).
            // ---------------------------------------------------------
            if (depth > DEPTH_DEEP) begin
                // Deep water: lower frequency, longer pulse for range
                freq_word     <= 24'd800000;
                chirp_rate    <= 24'sd0;
                amplitude     <= 8'd200;
                pulse_width   <= 16'd2000;
                waveform_mode <= 2'd0; // CW - simple, robust at range
            end else if (noise_level > NOISE_HIGH) begin
                // Noisy environment: LFM up-chirp for pulse-compression gain
                freq_word     <= 24'd1200000;
                chirp_rate    <= 24'sd400;
                amplitude     <= 8'd255;
                pulse_width   <= 16'd1200;
                waveform_mode <= 2'd1; // up-chirp
            end else if (salinity > SALINITY_HIGH) begin
                // High salinity shifts sound speed -> shift band, down-chirp
                freq_word     <= 24'd1500000;
                chirp_rate    <= -24'sd300;
                amplitude     <= 8'd230;
                pulse_width   <= 16'd800;
                waveform_mode <= 2'd2; // down-chirp
            end else begin
                // Calm / shallow default condition
                freq_word     <= 24'd1000000;
                chirp_rate    <= 24'sd0;
                amplitude     <= 8'd180;
                pulse_width   <= 16'd600;
                waveform_mode <= 2'd0;
            end
        end
    end

endmodule
