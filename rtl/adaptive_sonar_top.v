// ============================================================================
// Adaptive Sonar Transmitter - Top Level
// ----------------------------------------------------------------------------
// Ties param_controller (decision logic) to dds_waveform_gen (synthesis)
// and generates the pulse-on/pulse-off timing from pulse_width so the
// design repeats pulses continuously -- useful for a live demo trace.
// ============================================================================

module adaptive_sonar_top #(
    parameter FREQ_WIDTH   = 24,
    parameter SENSOR_WIDTH = 8,
    parameter SAMPLE_WIDTH = 8
)(
    input  wire                            clk,
    input  wire                            rst,
    input  wire [SENSOR_WIDTH-1:0]         depth,
    input  wire [SENSOR_WIDTH-1:0]         salinity,
    input  wire [SENSOR_WIDTH-1:0]         noise_level,

    output wire signed [SAMPLE_WIDTH-1:0]  sample_out,
    output wire                            sample_valid,
    output wire [1:0]                      waveform_mode, // exposed for observation/demo
    output wire [4:0]                      rom_addr_debug // exposed for GTKWave debugging
);

    wire [FREQ_WIDTH-1:0]        freq_word;
    wire signed [FREQ_WIDTH-1:0] chirp_rate;
    wire [7:0]                   amplitude;
    wire [15:0]                  pulse_width;

    reg  [15:0] pulse_counter;
    wire        pulse_enable = (pulse_counter < pulse_width);

    param_controller #(
        .FREQ_WIDTH(FREQ_WIDTH), .SENSOR_WIDTH(SENSOR_WIDTH)
    ) u_param (
        .clk(clk), .rst(rst),
        .depth(depth), .salinity(salinity), .noise_level(noise_level),
        .freq_word(freq_word), .chirp_rate(chirp_rate),
        .amplitude(amplitude), .pulse_width(pulse_width),
        .waveform_mode(waveform_mode)
    );

    dds_waveform_gen #(
        .FREQ_WIDTH(FREQ_WIDTH), .SAMPLE_WIDTH(SAMPLE_WIDTH)
    ) u_dds (
        .clk(clk), .rst(rst), .enable(pulse_enable),
        .freq_word(freq_word), .chirp_rate(chirp_rate), .amplitude(amplitude),
        .sample_out(sample_out), .sample_valid(sample_valid),
        .rom_addr_debug(rom_addr_debug)
    );

    always @(posedge clk or posedge rst) begin
        if (rst)
            pulse_counter <= 16'd0;
        else if (pulse_counter < pulse_width)
            pulse_counter <= pulse_counter + 16'd1;
        else
            pulse_counter <= 16'd0; // auto-repeat pulse for a continuous demo trace
    end

endmodule
