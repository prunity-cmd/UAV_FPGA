// ============================================================================
// Adaptive Sonar Transmitter - Top Level  (FPGA-oriented rewrite)
// ----------------------------------------------------------------------------
//  * Reset synchroniser (async assert, sync release) -- one internal sync reset.
//  * Sample-rate clock enable (`tick`) = CLK_HZ / SAMPLE_DIV. Single clock domain.
//  * Real ping cycle:  TX (pulse_width ticks)  ->  LISTEN (listen_width ticks)
//    -> repeat. The old design transmitted almost continuously; a sonar needs a
//    listen window for echoes.
//  * Parameters are LATCHED at the start of each pulse, so a sensor change (or
//    a glitch) can never alter a waveform mid-pulse. Adaptation therefore takes
//    effect on the next ping (worst-case latency = one ping period).
//  * Raised-cosine-like linear ramp on pulse edges to cut spectral splatter.
//
// Board hookup (Edge Artix-7): drive DAC from `dac_data` (offset binary), and
// use `pulse_start` / `tx_active` for T/R switching or an ADC trigger.
// Set SAMPLE_DIV to match the DAC's max update rate: 100 MHz/100 = 1 MSPS.
// For quick simulation use SAMPLE_DIV=1 (and expect FS_HZ = CLK_HZ).
// ============================================================================

module adaptive_sonar_top #(
    parameter CLK_HZ       = 100_000_000,
    parameter SAMPLE_DIV   = 100,          // tick every SAMPLE_DIV clocks
    parameter FREQ_WIDTH   = 24,
    parameter SENSOR_WIDTH = 8,
    parameter SAMPLE_WIDTH = 8,            // match your DAC (<= 12)
    parameter USE_SYNC     = 1             // 0 = bypass input sync (simulation)
)(
    input  wire                            clk,
    input  wire                            rst,           // async assert, active high
    input  wire [SENSOR_WIDTH-1:0]         depth,
    input  wire [SENSOR_WIDTH-1:0]         salinity,
    input  wire [SENSOR_WIDTH-1:0]         noise_level,

    output wire signed [SAMPLE_WIDTH-1:0]  sample_out,    // two's complement
    output wire [SAMPLE_WIDTH-1:0]         dac_data,      // offset binary -> unsigned DAC
    output wire                            sample_valid,  // aligned with sample_out
    output wire [1:0]                      waveform_mode, // controller's CURRENT decision
    output wire [1:0]                      active_mode,   // mode of the pulse being sent
    output wire                            tx_active,     // transmit window
    output wire                            pulse_start,   // 1-clk strobe at start of each ping
    output wire [4:0]                      rom_addr_debug
);

    localparam FS_HZ = CLK_HZ / SAMPLE_DIV;

    // ------------------------------------------------------------------
    // Reset synchroniser
    // ------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sr = 2'b11;
    always @(posedge clk or posedge rst) begin
        if (rst) rst_sr <= 2'b11;
        else     rst_sr <= {rst_sr[0], 1'b0};
    end
    wire rst_s = rst_sr[1];

    // ------------------------------------------------------------------
    // Sample tick (clock enable)
    // ------------------------------------------------------------------
    localparam DIV_W = (SAMPLE_DIV > 1) ? $clog2(SAMPLE_DIV) : 1;
    reg [DIV_W-1:0] div_cnt;
    wire tick = (div_cnt == SAMPLE_DIV - 1);

    always @(posedge clk) begin
        if (rst_s || tick) div_cnt <= {DIV_W{1'b0}};
        else               div_cnt <= div_cnt + 1'b1;
    end

    // ------------------------------------------------------------------
    // Parameter controller
    // ------------------------------------------------------------------
    wire [FREQ_WIDTH-1:0]        freq_word;
    wire signed [FREQ_WIDTH-1:0] chirp_rate;
    wire [7:0]                   amplitude;
    wire [15:0]                  pulse_width, listen_width;
    wire                         params_valid;

    param_controller #(
        .FREQ_WIDTH(FREQ_WIDTH), .SENSOR_WIDTH(SENSOR_WIDTH),
        .FS_HZ(FS_HZ), .USE_SYNC(USE_SYNC)
    ) u_param (
        .clk(clk), .rst(rst_s),
        .depth(depth), .salinity(salinity), .noise_level(noise_level),
        .freq_word(freq_word), .chirp_rate(chirp_rate),
        .amplitude(amplitude), .pulse_width(pulse_width),
        .listen_width(listen_width),
        .waveform_mode(waveform_mode), .params_valid(params_valid)
    );

    // ------------------------------------------------------------------
    // Ping sequencer: INIT -> TX -> LISTEN -> TX -> ...  (advances on tick)
    // ------------------------------------------------------------------
    localparam [1:0] S_INIT = 2'd0, S_TX = 2'd1, S_LISTEN = 2'd2;

    reg [1:0]                    state;
    reg [15:0]                   cnt;
    reg [FREQ_WIDTH-1:0]         freq_l;
    reg signed [FREQ_WIDTH-1:0]  chirp_l;
    reg [7:0]                    amp_l;
    reg [15:0]                   pw_l, listen_l;
    reg [1:0]                    mode_l;

    wire last_tx     = ({1'b0, cnt} + 17'd1 >= {1'b0, pw_l});
    wire last_listen = ({1'b0, cnt} + 17'd1 >= {1'b0, listen_l});

    always @(posedge clk) begin
        if (rst_s) begin
            state    <= S_INIT;
            cnt      <= 16'd0;
            freq_l   <= {FREQ_WIDTH{1'b0}};
            chirp_l  <= {FREQ_WIDTH{1'b0}};
            amp_l    <= 8'd0;
            pw_l     <= 16'd1;
            listen_l <= 16'd1;
            mode_l   <= 2'd0;
        end else if (tick) begin
            case (state)
                S_INIT: begin
                    if (params_valid) begin
                        freq_l <= freq_word;  chirp_l  <= chirp_rate;
                        amp_l  <= amplitude;  pw_l     <= pulse_width;
                        listen_l <= listen_width; mode_l <= waveform_mode;
                        cnt    <= 16'd0;
                        state  <= S_TX;
                    end
                end
                S_TX: begin
                    if (last_tx) begin cnt <= 16'd0; state <= S_LISTEN; end
                    else         cnt <= cnt + 16'd1;
                end
                S_LISTEN: begin
                    if (last_listen) begin
                        freq_l <= freq_word;  chirp_l  <= chirp_rate;   // latch for next ping
                        amp_l  <= amplitude;  pw_l     <= pulse_width;
                        listen_l <= listen_width; mode_l <= waveform_mode;
                        cnt    <= 16'd0;
                        state  <= S_TX;
                    end else cnt <= cnt + 16'd1;
                end
                default: state <= S_INIT;
            endcase
        end
    end

    wire active = (state == S_TX);
    wire first  = active && (cnt == 16'd0);

    // Edge ramp: 0 at first/last sample, rising over 63 samples.
    wire [15:0] rem  = pw_l - 16'd1 - cnt;
    wire [5:0]  r_up = (cnt >= 16'd63) ? 6'd63 : cnt[5:0];
    wire [5:0]  r_dn = (rem >= 16'd63) ? 6'd63 : rem[5:0];
    wire [5:0]  r6   = (r_up < r_dn) ? r_up : r_dn;
    wire [7:0]  ramp = {r6, r6[5:4]};

    assign tx_active   = active;
    assign active_mode = mode_l;
    assign pulse_start = tick && first;

    // ------------------------------------------------------------------
    // DDS
    // ------------------------------------------------------------------
    dds_waveform_gen #(
        .FREQ_WIDTH(FREQ_WIDTH), .SAMPLE_WIDTH(SAMPLE_WIDTH)
    ) u_dds (
        .clk(clk), .rst(rst_s), .tick(tick),
        .active(active), .first(first),
        .freq_word(freq_l), .chirp_rate(chirp_l),
        .amplitude(amp_l), .ramp(ramp),
        .sample_out(sample_out), .dac_data(dac_data),
        .sample_valid(sample_valid),
        .rom_addr_debug(rom_addr_debug)
    );

endmodule
