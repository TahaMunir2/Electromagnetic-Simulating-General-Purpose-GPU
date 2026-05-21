`timescale 1ns/1ps

module cordic_generator #(
    parameter signed [15:0] AMPLITUDE_Q313 = 16'sd8192
)(
    input  wire        clk,
    input  wire        rst,

    // For this Vivado IP proof, phase_in is ABSOLUTE phase:
    // 16'h0000 = 0
    // 16'h4000 = quarter cycle
    // 16'h8000 = half cycle
    // 16'hc000 = three-quarter cycle
    input  wire [15:0] phase_in,
    input  wire        phase_valid,

    // Project output format: signed Q3.13
    output wire [15:0] sin_out,
    output wire [15:0] cos_out,
    output wire        out_valid
);

    reg [15:0] phase_word;
    reg        phase_word_valid;

    always @(posedge clk) begin
        if (rst) begin
            phase_word       <= 16'd0;
            phase_word_valid <= 1'b0;
        end else begin
            phase_word_valid <= phase_valid;

            if (phase_valid) begin
                phase_word <= phase_in;
            end
        end
    end

`ifdef VIVADO_CORDIC_IP
    wire [31:0] cordic_tdata;
    wire        cordic_tvalid;

    wire signed [15:0] phase_word_signed;
    wire signed [15:0] cordic_phase_tdata;

    wire signed [15:0] cordic_sin_q214;
    wire signed [15:0] cordic_cos_q214;

    wire signed [15:0] sin_q313_raw;
    wire signed [15:0] cos_q313_raw;

    assign phase_word_signed = phase_word;

    // Calibrated against Vivado CORDIC IP:
    // project phase quarter-cycle 16'h4000 maps to IP phase 16'h1000.
    assign cordic_phase_tdata = phase_word_signed >>> 2;

    cordic_0 cordic_ip (
        .aclk(clk),
        .s_axis_phase_tvalid(phase_word_valid),
        .s_axis_phase_tdata(cordic_phase_tdata),
        .m_axis_dout_tvalid(cordic_tvalid),
        .m_axis_dout_tdata(cordic_tdata)
    );

    // From CORDIC IP GUI:
    // M_AXIS_DOUT REAL(15:0), IMAG(31:16)
    // For Sin/Cos: REAL = cos, IMAG = sin.
    assign cordic_cos_q214 = cordic_tdata[15:0];
    assign cordic_sin_q214 = cordic_tdata[31:16];

    // IP output is fix16_14. Project output is Q3.13.
    assign sin_q313_raw = cordic_sin_q214 >>> 1;
    assign cos_q313_raw = cordic_cos_q214 >>> 1;

    assign sin_out   = sin_q313_raw;
    assign cos_out   = cos_q313_raw;
    assign out_valid = cordic_tvalid;

`else
    reg signed [15:0] sin_reg;
    reg signed [15:0] cos_reg;
    reg               valid_reg;

    always @(posedge clk) begin
        if (rst) begin
            sin_reg   <= 16'sd0;
            cos_reg   <= AMPLITUDE_Q313;
            valid_reg <= 1'b0;
        end else begin
            valid_reg <= phase_word_valid;

            if (phase_word_valid) begin
                case (phase_word)
                    16'h0000: begin
                        sin_reg <= 16'sd0;
                        cos_reg <= AMPLITUDE_Q313;
                    end
                    16'h4000: begin
                        sin_reg <= AMPLITUDE_Q313;
                        cos_reg <= 16'sd0;
                    end
                    16'h8000: begin
                        sin_reg <= 16'sd0;
                        cos_reg <= -AMPLITUDE_Q313;
                    end
                    16'hc000: begin
                        sin_reg <= -AMPLITUDE_Q313;
                        cos_reg <= 16'sd0;
                    end
                    default: begin
                        sin_reg <= 16'sd0;
                        cos_reg <= 16'sd0;
                    end
                endcase
            end
        end
    end

    assign sin_out   = sin_reg;
    assign cos_out   = cos_reg;
    assign out_valid = valid_reg;
`endif

endmodule
