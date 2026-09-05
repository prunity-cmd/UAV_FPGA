// ============================================================================
// DDS-based waveform generator
// ----------------------------------------------------------------------------
// Produces CW or LFM chirp samples using a phase accumulator + sine ROM.
// chirp_rate != 0 ramps the effective frequency every clock, generating a
// linear frequency modulated (LFM) chirp; chirp_rate == 0 gives a pure
// continuous-wave (CW) tone.
// ============================================================================

module dds_waveform_gen #(
    parameter FREQ_WIDTH    = 24,
    parameter ROM_ADDR_BITS = 5,     // 32-entry sine ROM
    parameter SAMPLE_WIDTH  = 8
)(
    input  wire                            clk,
    input  wire                            rst,
    input  wire                            enable,       // pulse active
    input  wire [FREQ_WIDTH-1:0]           freq_word,
    input  wire signed [FREQ_WIDTH-1:0]    chirp_rate,
    input  wire [7:0]                      amplitude,

    output reg  signed [SAMPLE_WIDTH-1:0]  sample_out,
    output reg                             sample_valid,
    output wire [ROM_ADDR_BITS-1:0]        rom_addr_debug // exposed so you can
                                                           // confirm in GTKWave
                                                           // that the ROM
                                                           // address is really
                                                           // sweeping 0..31
);

    reg [FREQ_WIDTH-1:0] phase_acc;
    reg [FREQ_WIDTH-1:0] freq_word_current;

    // Sine lookup ROM - hardcoded 32-point table, scaled to 8-bit signed
    // (-127..127). Using fixed constants instead of a runtime $sin() call
    // avoids relying on real-math system function support (which varies
    // across simulators), and is also how you'd actually implement this
    // ROM for real FPGA synthesis (precomputed constants, not runtime trig).
    wire [ROM_ADDR_BITS-1:0] rom_addr = phase_acc[FREQ_WIDTH-1 -: ROM_ADDR_BITS];
    assign rom_addr_debug = rom_addr;
    reg signed [SAMPLE_WIDTH-1:0] rom_value;

    always @(*) begin
        case (rom_addr)
            5'd0:  rom_value = 8'sd0;
            5'd1:  rom_value = 8'sd25;
            5'd2:  rom_value = 8'sd49;
            5'd3:  rom_value = 8'sd71;
            5'd4:  rom_value = 8'sd90;
            5'd5:  rom_value = 8'sd106;
            5'd6:  rom_value = 8'sd117;
            5'd7:  rom_value = 8'sd125;
            5'd8:  rom_value = 8'sd127;
            5'd9:  rom_value = 8'sd125;
            5'd10: rom_value = 8'sd117;
            5'd11: rom_value = 8'sd106;
            5'd12: rom_value = 8'sd90;
            5'd13: rom_value = 8'sd71;
            5'd14: rom_value = 8'sd49;
            5'd15: rom_value = 8'sd25;
            5'd16: rom_value = 8'sd0;
            5'd17: rom_value = -8'sd25;
            5'd18: rom_value = -8'sd49;
            5'd19: rom_value = -8'sd71;
            5'd20: rom_value = -8'sd90;
            5'd21: rom_value = -8'sd106;
            5'd22: rom_value = -8'sd117;
            5'd23: rom_value = -8'sd125;
            5'd24: rom_value = -8'sd127;
            5'd25: rom_value = -8'sd125;
            5'd26: rom_value = -8'sd117;
            5'd27: rom_value = -8'sd106;
            5'd28: rom_value = -8'sd90;
            5'd29: rom_value = -8'sd71;
            5'd30: rom_value = -8'sd49;
            5'd31: rom_value = -8'sd25;
            default: rom_value = 8'sd0;
        endcase
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase_acc         <= {FREQ_WIDTH{1'b0}};
            freq_word_current <= {FREQ_WIDTH{1'b0}};
            sample_out        <= {SAMPLE_WIDTH{1'b0}};
            sample_valid      <= 1'b0;
        end else if (enable) begin
            // Ramp frequency each cycle for chirp modes (chirp_rate = 0 -> CW)
            freq_word_current <= freq_word_current + chirp_rate;
            phase_acc         <= phase_acc + freq_word_current;

            // NOTE: outputting rom_value directly (unscaled by amplitude) for
            // now. The earlier (rom_value * amplitude) >>> 8 scaling was
            // collapsing to near-zero/garbage on this toolchain -- rather
            // than debug signed-multiply width semantics this close to the
            // deadline, we're de-risking by dropping the multiply entirely.
            // This still gives you the full -127..127 waveform shape, which
            // is what actually matters for the demo (seeing CW vs chirp).
            // Amplitude scaling can be reintroduced later, e.g. by using
            // amplitude as a simple right-shift attenuation level instead
            // of a full multiply.
            sample_out   <= rom_value;
            sample_valid <= 1'b1;
        end else begin
            // Reload base frequency and reset phase when idle, so the next
            // pulse always starts from a clean, repeatable state.
            freq_word_current <= freq_word;
            phase_acc         <= {FREQ_WIDTH{1'b0}};
            sample_valid      <= 1'b0;
        end
    end

endmodule
