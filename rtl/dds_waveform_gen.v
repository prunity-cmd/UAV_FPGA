// ============================================================================
// DDS waveform generator  (FPGA-oriented rewrite)
// ----------------------------------------------------------------------------
// CW / LFM chirp synthesis: phase accumulator -> quarter-wave sine ROM ->
// amplitude scaling -> DAC-ready output.
//
//  * Everything advances on `tick` (a clock-enable at the DAC sample rate), so
//    the whole design runs on ONE clock -- no derived clocks.
//  * Quarter-wave ROM (64 x 11 bit, half-sample offset => exact symmetry, no DC
//    offset) gives a 256-point sine from 64 stored values.
//  * Amplitude is applied with real multipliers (maps to DSP48). The old
//    "signed multiply collapsed to 0/-1" problem was an Icarus/width issue; here
//    every operand is explicitly sized and signed, and the shift is rounded so
//    there is no -1 LSB bias.
//  * Pipeline is 4 ticks deep. sample_valid is aligned with sample_out.
//  * dac_data is offset-binary (mid-scale = 0 V) for unsigned DACs.
//
// Frequency:  f_out = freq_word * f_tick / 2^FREQ_WIDTH
// Chirp:      freq_word_current += chirp_rate  every tick
// SAMPLE_WIDTH must be <= 12 (ROM precision).
// ============================================================================

module dds_waveform_gen #(
    parameter FREQ_WIDTH    = 24,
    parameter ROM_ADDR_BITS = 5,      // width of rom_addr_debug only
    parameter SAMPLE_WIDTH  = 8
)(
    input  wire                            clk,
    input  wire                            rst,          // synchronous, active high
    input  wire                            tick,         // sample-rate clock enable
    input  wire                            active,       // inside a transmit pulse
    input  wire                            first,        // first sample of a pulse (reload phase)
    input  wire [FREQ_WIDTH-1:0]           freq_word,    // held constant during a pulse
    input  wire signed [FREQ_WIDTH-1:0]    chirp_rate,   // held constant during a pulse
    input  wire [7:0]                      amplitude,    // held constant during a pulse
    input  wire [7:0]                      ramp,         // 0..255 pulse-edge envelope (from top)

    output reg  signed [SAMPLE_WIDTH-1:0]  sample_out,   // two's complement
    output reg  [SAMPLE_WIDTH-1:0]         dac_data,     // offset binary for unsigned DAC
    output reg                             sample_valid,
    output wire [ROM_ADDR_BITS-1:0]        rom_addr_debug
);

    localparam OUT_SHIFT = 20 - SAMPLE_WIDTH;   // 11-bit ROM x 8-bit gain = 19 bit -> SAMPLE_WIDTH

    // ------------------------------------------------------------------
    // Quarter-wave sine ROM: round(2047*sin((k+0.5)*pi/2/64)), k = 0..63
    // (precomputed constants -- no $sin, synthesises to LUTs)
    // ------------------------------------------------------------------
    function [10:0] sine_q;
        input [5:0] k;
        begin
            case (k)
            6'd0: sine_q = 11'd25;
            6'd1: sine_q = 11'd75;
            6'd2: sine_q = 11'd126;
            6'd3: sine_q = 11'd176;
            6'd4: sine_q = 11'd226;
            6'd5: sine_q = 11'd275;
            6'd6: sine_q = 11'd325;
            6'd7: sine_q = 11'd375;
            6'd8: sine_q = 11'd424;
            6'd9: sine_q = 11'd473;
            6'd10: sine_q = 11'd522;
            6'd11: sine_q = 11'd570;
            6'd12: sine_q = 11'd618;
            6'd13: sine_q = 11'd666;
            6'd14: sine_q = 11'd713;
            6'd15: sine_q = 11'd760;
            6'd16: sine_q = 11'd807;
            6'd17: sine_q = 11'd852;
            6'd18: sine_q = 11'd898;
            6'd19: sine_q = 11'd943;
            6'd20: sine_q = 11'd987;
            6'd21: sine_q = 11'd1031;
            6'd22: sine_q = 11'd1074;
            6'd23: sine_q = 11'd1116;
            6'd24: sine_q = 11'd1158;
            6'd25: sine_q = 11'd1199;
            6'd26: sine_q = 11'd1239;
            6'd27: sine_q = 11'd1279;
            6'd28: sine_q = 11'd1318;
            6'd29: sine_q = 11'd1356;
            6'd30: sine_q = 11'd1393;
            6'd31: sine_q = 11'd1430;
            6'd32: sine_q = 11'd1465;
            6'd33: sine_q = 11'd1500;
            6'd34: sine_q = 11'd1533;
            6'd35: sine_q = 11'd1566;
            6'd36: sine_q = 11'd1598;
            6'd37: sine_q = 11'd1629;
            6'd38: sine_q = 11'd1659;
            6'd39: sine_q = 11'd1688;
            6'd40: sine_q = 11'd1716;
            6'd41: sine_q = 11'd1743;
            6'd42: sine_q = 11'd1769;
            6'd43: sine_q = 11'd1793;
            6'd44: sine_q = 11'd1817;
            6'd45: sine_q = 11'd1840;
            6'd46: sine_q = 11'd1861;
            6'd47: sine_q = 11'd1881;
            6'd48: sine_q = 11'd1901;
            6'd49: sine_q = 11'd1919;
            6'd50: sine_q = 11'd1936;
            6'd51: sine_q = 11'd1951;
            6'd52: sine_q = 11'd1966;
            6'd53: sine_q = 11'd1979;
            6'd54: sine_q = 11'd1992;
            6'd55: sine_q = 11'd2003;
            6'd56: sine_q = 11'd2012;
            6'd57: sine_q = 11'd2021;
            6'd58: sine_q = 11'd2028;
            6'd59: sine_q = 11'd2035;
            6'd60: sine_q = 11'd2039;
            6'd61: sine_q = 11'd2043;
            6'd62: sine_q = 11'd2046;
            6'd63: sine_q = 11'd2047;
            default: sine_q = 11'd0;
            endcase
        end
    endfunction

    // ------------------------------------------------------------------
    // Stage 0: phase accumulator + chirp frequency ramp
    // ------------------------------------------------------------------
    reg [FREQ_WIDTH-1:0]        phase_acc;
    reg [FREQ_WIDTH-1:0]        freq_cur;
    reg                         active0;
    reg [7:0]                   ramp0;

    assign rom_addr_debug = phase_acc[FREQ_WIDTH-1 -: ROM_ADDR_BITS];

    always @(posedge clk) begin
        if (rst) begin
            phase_acc <= {FREQ_WIDTH{1'b0}};
            freq_cur  <= {FREQ_WIDTH{1'b0}};
            active0   <= 1'b0;
            ramp0     <= 8'd0;
        end else if (tick) begin
            active0 <= active;
            ramp0   <= ramp;
            if (active && first) begin
                phase_acc <= {FREQ_WIDTH{1'b0}};   // every pulse starts phase-coherent
                freq_cur  <= freq_word;
            end else if (active) begin
                phase_acc <= phase_acc + freq_cur;
                freq_cur  <= freq_cur + chirp_rate; // wraps mod 2^N; chirp span is bounded by pulse length
            end
        end
    end

    // ------------------------------------------------------------------
    // Stage 1: ROM lookup (quadrant fold) + envelope gain
    // ------------------------------------------------------------------
    wire [1:0] quad  = phase_acc[FREQ_WIDTH-1 -: 2];
    wire [5:0] kidx  = phase_acc[FREQ_WIDTH-3 -: 6];
    wire [5:0] k_eff = quad[0] ? ~kidx : kidx;          // mirror odd quadrants
    wire [15:0] gain_full = amplitude * ramp0;          // 8x8 unsigned

    reg [10:0] mag1;
    reg        neg1;
    reg [7:0]  gain1;
    reg        val1;

    always @(posedge clk) begin
        if (rst) begin
            mag1 <= 11'd0; neg1 <= 1'b0; gain1 <= 8'd0; val1 <= 1'b0;
        end else if (tick) begin
            mag1  <= sine_q(k_eff);
            neg1  <= quad[1];                            // lower half-cycle = negative
            gain1 <= active0 ? gain_full[15:8] : 8'd0;
            val1  <= active0;
        end
    end

    // ------------------------------------------------------------------
    // Stage 2: apply sign
    // ------------------------------------------------------------------
    reg signed [11:0] s12;
    reg        [7:0]  gain2;
    reg               val2;

    always @(posedge clk) begin
        if (rst) begin
            s12 <= 12'sd0; gain2 <= 8'd0; val2 <= 1'b0;
        end else if (tick) begin
            s12   <= neg1 ? -$signed({1'b0, mag1}) : $signed({1'b0, mag1});
            gain2 <= gain1;
            val2  <= val1;
        end
    end

    // ------------------------------------------------------------------
    // Stage 3: amplitude multiply (signed 12 x signed 9 -> 21 bit)
    // ------------------------------------------------------------------
    reg signed [20:0] prod;
    reg               val3;

    always @(posedge clk) begin
        if (rst) begin
            prod <= 21'sd0; val3 <= 1'b0;
        end else if (tick) begin
            prod <= s12 * $signed({1'b0, gain2});
            val3 <= val2;
        end
    end

    // ------------------------------------------------------------------
    // Stage 4: round, shift down to DAC width, offset-binary conversion
    // ------------------------------------------------------------------
    wire signed [20:0] prod_r   = prod + (21'sd1 <<< (OUT_SHIFT - 1));
    wire signed [20:0] shifted  = prod_r >>> OUT_SHIFT;
    wire signed [SAMPLE_WIDTH-1:0] scaled = shifted[SAMPLE_WIDTH-1:0];

    always @(posedge clk) begin
        if (rst) begin
            sample_out   <= {SAMPLE_WIDTH{1'b0}};
            dac_data     <= {1'b1, {(SAMPLE_WIDTH-1){1'b0}}};   // mid-scale
            sample_valid <= 1'b0;
        end else if (tick) begin
            sample_out   <= scaled;
            dac_data     <= {~scaled[SAMPLE_WIDTH-1], scaled[SAMPLE_WIDTH-2:0]};
            sample_valid <= val3;
        end
    end

endmodule
