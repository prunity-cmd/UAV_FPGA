// ============================================================================
// Adaptive Parameter Controller  (FPGA-oriented rewrite)
// ----------------------------------------------------------------------------
// Maps environmental sensor inputs (depth, salinity, noise) to waveform
// parameters. Improvements over the first version:
//
//  * Sensor inputs are treated as asynchronous: 2-FF synchroniser plus a
//    "two consecutive equal samples" filter (USE_SYNC=1). Set USE_SYNC=0 in
//    simulation/testbench to bypass and get minimum latency.
//  * Schmitt-trigger hysteresis on every threshold, so a noisy sensor sitting
//    on a threshold cannot make the waveform mode chatter.
//  * Tuning is done in physical units (Hz, samples). Tuning words are computed
//    at elaboration from FS_HZ, so changing the sample rate keeps the
//    frequencies correct instead of silently scaling them.
//  * Reset values are the "calm" defaults (never zero), and params_valid tells
//    the top level when the first real decision is available.
//
// NOTE: outputs change at any clock; the top level latches them only at a
// pulse boundary, so a waveform is never altered mid-pulse.
// ============================================================================

module param_controller #(
    parameter FREQ_WIDTH   = 24,
    parameter SENSOR_WIDTH = 8,          // thresholds below assume >= 8
    parameter FS_HZ        = 1_000_000,  // DDS tick rate (Hz)
    parameter USE_SYNC     = 1,

    // ---- thresholds (raw sensor counts, demo values) ----
    parameter integer DEPTH_DEEP    = 160,
    parameter integer NOISE_HIGH    = 150,
    parameter integer SALINITY_HIGH = 180,
    parameter integer HYST          = 6,

    // ---- waveform tuning, physical units ----
    // Calm / shallow: CW
    parameter integer F_CALM_HZ   = 60_000,
    parameter integer AMP_CALM    = 180,
    parameter integer PW_CALM     = 600,    // pulse length, ticks
    parameter integer LST_CALM    = 1500,   // listen window, ticks
    // Deep: lower frequency CW, longer pulse
    parameter integer F_DEEP_HZ   = 48_000,
    parameter integer AMP_DEEP    = 200,
    parameter integer PW_DEEP     = 2000,
    parameter integer LST_DEEP    = 4000,
    // Noisy: LFM up-chirp (start freq + swept bandwidth)
    parameter integer F_NOISY_HZ  = 72_000,
    parameter integer BW_NOISY_HZ = 28_000,
    parameter integer AMP_NOISY   = 255,
    parameter integer PW_NOISY    = 1200,
    parameter integer LST_NOISY   = 3000,
    // High salinity: LFM down-chirp
    parameter integer F_SAL_HZ    = 90_000,
    parameter integer BW_SAL_HZ   = 20_000,
    parameter integer AMP_SAL     = 230,
    parameter integer PW_SAL      = 800,
    parameter integer LST_SAL     = 2000
    // listen window sets max range:  R = c * (LST/FS_HZ) / 2   (c ~ 1500 m/s)
)(
    input  wire                            clk,
    input  wire                            rst,          // synchronous, active high
    input  wire [SENSOR_WIDTH-1:0]         depth,
    input  wire [SENSOR_WIDTH-1:0]         salinity,
    input  wire [SENSOR_WIDTH-1:0]         noise_level,

    output reg  [FREQ_WIDTH-1:0]           freq_word,     // base DDS tuning word
    output reg  signed [FREQ_WIDTH-1:0]    chirp_rate,    // tuning-word step per tick (0 = CW)
    output reg  [7:0]                      amplitude,
    output reg  [15:0]                     pulse_width,   // transmit length, ticks
    output reg  [15:0]                     listen_width,  // receive/listen length, ticks
    output reg  [1:0]                      waveform_mode, // 0=CW 1=up-chirp 2=down-chirp
    output reg                             params_valid
);

    // ---- elaboration-time constants (64-bit to avoid overflow) ----
    localparam [63:0] TWO_N = 64'd1 << FREQ_WIDTH;

    localparam [FREQ_WIDTH-1:0] W_CALM  = (F_CALM_HZ  * TWO_N) / FS_HZ;
    localparam [FREQ_WIDTH-1:0] W_DEEP  = (F_DEEP_HZ  * TWO_N) / FS_HZ;
    localparam [FREQ_WIDTH-1:0] W_NOISY = (F_NOISY_HZ * TWO_N) / FS_HZ;
    localparam [FREQ_WIDTH-1:0] W_SAL   = (F_SAL_HZ   * TWO_N) / FS_HZ;
    // chirp rate = swept tuning-word span / pulse length
    localparam [FREQ_WIDTH-1:0] R_NOISY = ((BW_NOISY_HZ * TWO_N) / FS_HZ) / PW_NOISY;
    localparam [FREQ_WIDTH-1:0] R_SAL   = ((BW_SAL_HZ   * TWO_N) / FS_HZ) / PW_SAL;

    // ------------------------------------------------------------------
    // Input conditioning
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [SENSOR_WIDTH-1:0] d_q1, d_q2, s_q1, s_q2, n_q1, n_q2;
    reg [SENSOR_WIDTH-1:0] d_q3, s_q3, n_q3;
    reg [SENSOR_WIDTH-1:0] d_f,  s_f,  n_f;    // filtered values

    always @(posedge clk) begin
        d_q1 <= depth;       d_q2 <= d_q1;  d_q3 <= d_q2;
        s_q1 <= salinity;    s_q2 <= s_q1;  s_q3 <= s_q2;
        n_q1 <= noise_level; n_q2 <= n_q1;  n_q3 <= n_q2;
        if (d_q2 == d_q3) d_f <= d_q2;         // accept only stable samples
        if (s_q2 == s_q3) s_f <= s_q2;
        if (n_q2 == n_q3) n_f <= n_q2;
    end

    wire [SENSOR_WIDTH-1:0] d_i = USE_SYNC ? d_f : depth;
    wire [SENSOR_WIDTH-1:0] s_i = USE_SYNC ? s_f : salinity;
    wire [SENSOR_WIDTH-1:0] n_i = USE_SYNC ? n_f : noise_level;

    // ------------------------------------------------------------------
    // Schmitt-trigger condition flags
    // ------------------------------------------------------------------
    reg deep, noisy, salty;

    always @(posedge clk) begin
        if (rst) begin
            deep <= 1'b0; noisy <= 1'b0; salty <= 1'b0;
        end else begin
            if      (d_i > DEPTH_DEEP    + HYST) deep  <= 1'b1;
            else if (d_i < DEPTH_DEEP    - HYST) deep  <= 1'b0;

            if      (n_i > NOISE_HIGH    + HYST) noisy <= 1'b1;
            else if (n_i < NOISE_HIGH    - HYST) noisy <= 1'b0;

            if      (s_i > SALINITY_HIGH + HYST) salty <= 1'b1;
            else if (s_i < SALINITY_HIGH - HYST) salty <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // Warm-up: don't declare parameters valid until the input pipeline and
    // flags have settled on real sensor data.
    // ------------------------------------------------------------------
    reg [2:0] warm;
    always @(posedge clk) begin
        if (rst) begin
            warm <= 3'd0; params_valid <= 1'b0;
        end else if (warm != 3'd7) begin
            warm <= warm + 3'd1;
        end else begin
            params_valid <= 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // Priority-encoded decision (deep > noisy > salinity > calm), registered
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            freq_word     <= W_CALM;
            chirp_rate    <= {FREQ_WIDTH{1'b0}};
            amplitude     <= AMP_CALM;
            pulse_width   <= PW_CALM;
            listen_width  <= LST_CALM;
            waveform_mode <= 2'd0;
        end else if (deep) begin
            freq_word     <= W_DEEP;
            chirp_rate    <= {FREQ_WIDTH{1'b0}};
            amplitude     <= AMP_DEEP;
            pulse_width   <= PW_DEEP;
            listen_width  <= LST_DEEP;
            waveform_mode <= 2'd0;              // CW: simple, robust at range
        end else if (noisy) begin
            freq_word     <= W_NOISY;
            chirp_rate    <= $signed(R_NOISY);
            amplitude     <= AMP_NOISY;
            pulse_width   <= PW_NOISY;
            listen_width  <= LST_NOISY;
            waveform_mode <= 2'd1;              // up-chirp: pulse-compression gain
        end else if (salty) begin
            freq_word     <= W_SAL;
            chirp_rate    <= -$signed(R_SAL);
            amplitude     <= AMP_SAL;
            pulse_width   <= PW_SAL;
            listen_width  <= LST_SAL;
            waveform_mode <= 2'd2;              // down-chirp
        end else begin
            freq_word     <= W_CALM;
            chirp_rate    <= {FREQ_WIDTH{1'b0}};
            amplitude     <= AMP_CALM;
            pulse_width   <= PW_CALM;
            listen_width  <= LST_CALM;
            waveform_mode <= 2'd0;
        end
    end

endmodule
