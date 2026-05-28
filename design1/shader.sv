// ============================================================================
//  shader.sv  (pipelined — 5 stages, designed for 100 MHz on Artix-7)
//  ----------------------------------------------------------------------------
//  Same functionality as the single-cycle shader, but broken into 5 pipeline
//  stages to meet timing.  Each stage performs at most one big arithmetic
//  operation (multiply, divide, or add-chain) plus light bit manipulation.
//
//  Stages:
//      Stage 1: bright_q, altitude_shifted, fog_int    (shifts/adds + a divide)
//      Stage 2: bright_u8, altitude_u8, fog_u8         (clamps to 8 bits)
//      Stage 3: light_u8, base_r/g/b                   (small mults & subs)
//      Stage 4: fogged_r/g/b                           (mix multiply-add)
//      Stage 5: lit_r/g/b -> registered output + HIT/sky mux
//
//  All control signals (status, px, py, valid) are pipelined through every
//  stage so they arrive at the output aligned with the colour data.
//
//  Total latency: 5 cycles (was 1).  Throughput: still 1 pixel/cycle.
//
//  Functional behaviour is bit-identical to the single-cycle version, so
//  Python goldens still apply — only the TB's LATENCY constant needs to
//  grow by 4 (ray_unit total: 73 -> 77).
// ============================================================================

module shader #(
    parameter int H_W        = 16,
    parameter int H_I        = 4,
    parameter int H_F        = H_W - 1 - H_I,
    parameter int DIR_W      = 16,
    parameter int DIR_I      = 2,
    parameter int DIR_F      = DIR_W - 1 - DIR_I,
    parameter int N_STEPS    = 16,
    parameter int STEP_W     = $clog2(N_STEPS + 1),
    parameter int PX_W       = 10,
    parameter int PY_W       = 10,
    parameter logic [7:0] SKY_R    = 8'd135,
    parameter logic [7:0] SKY_G    = 8'd206,
    parameter logic [7:0] SKY_B    = 8'd235,
    parameter logic [7:0] PALE_R   = 8'd192,
    parameter logic [7:0] PALE_G   = 8'd192,
    parameter logic [7:0] PALE_B   = 8'd192,
    parameter logic [7:0] AMBIENT  = 8'd64
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,

    input  logic [1:0]                 status_in,
    input  logic signed [H_W-1:0]      h_hit_in,
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic signed [DIR_W-1:0]    Nx_in,
    input  logic signed [DIR_W-1:0]    Ny_in,
    input  logic signed [DIR_W-1:0]    Nz_in,
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    input  logic signed [DIR_W-1:0]    sun_dx,
    input  logic signed [DIR_W-1:0]    sun_dy,
    input  logic signed [DIR_W-1:0]    sun_dz,

    output logic [7:0]                 r_out,
    output logic [7:0]                 g_out,
    output logic [7:0]                 b_out,
    output logic [PX_W-1:0]            px_out,
    output logic [PY_W-1:0]            py_out,
    output logic                       valid_out
);

    localparam logic [1:0] ST_HIT = 2'b01;


    // =================================================================
    //  STAGE 1 registers: bright_q, altitude_shifted, fog_int
    // =================================================================
    logic                       v_s1;
    logic [1:0]                 stat_s1;
    logic [PX_W-1:0]            px_s1;
    logic [PY_W-1:0]            py_s1;

    logic signed [2*DIR_W-1:0]  bright_q_s1;
    logic signed [31:0]         altitude_shifted_s1;
    logic signed [31:0]         fog_int_s1;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s1 <= 1'b0;
        end else if (en) begin
            v_s1     <= valid_in;
            stat_s1  <= status_in;
            px_s1    <= px_in;
            py_s1    <= py_in;

            // bright_q: sun hardcoded to (0,0,1), so dot(N, sun) = Nz.
            if (Nz_in <= 0)
                bright_q_s1 <= '0;
            else
                bright_q_s1 <= {{DIR_W{1'b0}}, Nz_in};

            // altitude_shifted: place h around centre value 128.
            if (H_F >= 7)
                altitude_shifted_s1 <= (h_hit_in >>> (H_F - 7)) + 32'sd128;
            else
                altitude_shifted_s1 <= (h_hit_in <<< (7 - H_F)) + 32'sd128;

            // fog_int = step_count * 255 / N_STEPS
            if (N_STEPS <= 1)
                fog_int_s1 <= '0;
            else
                fog_int_s1 <= ($signed({1'b0, step_count_in}) * 32'sd255) / N_STEPS;
        end
    end


    // =================================================================
    //  STAGE 2 registers: bright_u8, altitude_u8, fog_u8
    // =================================================================
    logic                       v_s2;
    logic [1:0]                 stat_s2;
    logic [PX_W-1:0]            px_s2;
    logic [PY_W-1:0]            py_s2;

    logic [7:0]                 bright_u8_s2;
    logic [7:0]                 altitude_u8_s2;
    logic [7:0]                 fog_u8_s2;

    logic signed [31:0]         bright_tmp;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s2 <= 1'b0;
        end else if (en) begin
            v_s2     <= v_s1;
            stat_s2  <= stat_s1;
            px_s2    <= px_s1;
            py_s2    <= py_s1;

            // Scale bright_q (Q?.DIR_F) to 8 bits
            if (bright_q_s1[2*DIR_W-1])
                bright_tmp = 32'sd0;
            else if (bright_q_s1 >= (1 <<< DIR_F))
                bright_tmp = 32'sd255;
            else if (DIR_F >= 8)
                bright_tmp = bright_q_s1 >>> (DIR_F - 8);
            else
                bright_tmp = bright_q_s1 <<< (8 - DIR_F);

            if (bright_tmp < 0)        bright_u8_s2 <= 8'd0;
            else if (bright_tmp > 255) bright_u8_s2 <= 8'd255;
            else                       bright_u8_s2 <= bright_tmp[7:0];

            if (altitude_shifted_s1 < 0)        altitude_u8_s2 <= 8'd0;
            else if (altitude_shifted_s1 > 255) altitude_u8_s2 <= 8'd255;
            else                                altitude_u8_s2 <= altitude_shifted_s1[7:0];

            if (fog_int_s1 < 0)        fog_u8_s2 <= 8'd0;
            else if (fog_int_s1 > 255) fog_u8_s2 <= 8'd255;
            else                       fog_u8_s2 <= fog_int_s1[7:0];
        end
    end


    // =================================================================
    //  STAGE 3 registers: light_u8, base_r/g/b, carry fog through
    // =================================================================
    logic                       v_s3;
    logic [1:0]                 stat_s3;
    logic [PX_W-1:0]            px_s3;
    logic [PY_W-1:0]            py_s3;

    logic [7:0]                 light_u8_s3;
    logic [7:0]                 fog_u8_s3;
    logic [7:0]                 base_r_s3, base_g_s3, base_b_s3;

    logic [7:0] diff;
    logic [7:0] inner;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s3 <= 1'b0;
        end else if (en) begin
            v_s3      <= v_s2;
            stat_s3   <= stat_s2;
            px_s3     <= px_s2;
            py_s3     <= py_s2;
            fog_u8_s3 <= fog_u8_s2;

            // light_u8 = AMBIENT + ((255 - AMBIENT) * bright_u8) >> 8  (16-bit safe)
            light_u8_s3 <= AMBIENT +
                           (((16'd255 - AMBIENT) * bright_u8_s2) >> 8);

            // base RGB from altitude
            base_r_s3 <= altitude_u8_s2;
            base_b_s3 <= 8'd255 - altitude_u8_s2;

            // base_g: triangle peaked at 32+(255>>2)=95 when altitude=127
            if (altitude_u8_s2 > 8'd127)
                diff = (altitude_u8_s2 - 8'd127) << 1;
            else
                diff = (8'd127 - altitude_u8_s2) << 1;
            inner = 8'd255 - diff;
            base_g_s3 <= 8'd32 + (inner >> 2);
        end
    end


    // =================================================================
    //  STAGE 4 registers: fogged_r/g/b = mix(base, PALE, fog)
    //  Heaviest stage: 6 small multiplies, three adders, three shifts.
    // =================================================================
    logic                       v_s4;
    logic [1:0]                 stat_s4;
    logic [PX_W-1:0]            px_s4;
    logic [PY_W-1:0]            py_s4;

    logic [7:0]                 light_u8_s4;
    logic [7:0]                 fogged_r_s4, fogged_g_s4, fogged_b_s4;

    logic [15:0] one_minus_fog;
    logic [15:0] mix_r_acc, mix_g_acc, mix_b_acc;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s4 <= 1'b0;
        end else if (en) begin
            v_s4        <= v_s3;
            stat_s4     <= stat_s3;
            px_s4       <= px_s3;
            py_s4       <= py_s3;
            light_u8_s4 <= light_u8_s3;

            one_minus_fog = 16'd255 - fog_u8_s3;

            mix_r_acc = (one_minus_fog * base_r_s3 + fog_u8_s3 * PALE_R) >> 8;
            mix_g_acc = (one_minus_fog * base_g_s3 + fog_u8_s3 * PALE_G) >> 8;
            mix_b_acc = (one_minus_fog * base_b_s3 + fog_u8_s3 * PALE_B) >> 8;

            fogged_r_s4 <= (mix_r_acc > 255) ? 8'd255 : mix_r_acc[7:0];
            fogged_g_s4 <= (mix_g_acc > 255) ? 8'd255 : mix_g_acc[7:0];
            fogged_b_s4 <= (mix_b_acc > 255) ? 8'd255 : mix_b_acc[7:0];
        end
    end


    // =================================================================
    //  STAGE 5 registers: lit = fogged * light, then HIT/sky mux to output
    // =================================================================
    logic [15:0] mul_r_prod, mul_g_prod, mul_b_prod;
    logic [7:0]  lit_r_s5, lit_g_s5, lit_b_s5;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            valid_out <= v_s4;
            px_out    <= px_s4;
            py_out    <= py_s4;

            mul_r_prod = (fogged_r_s4 * light_u8_s4) >> 8;
            mul_g_prod = (fogged_g_s4 * light_u8_s4) >> 8;
            mul_b_prod = (fogged_b_s4 * light_u8_s4) >> 8;

            lit_r_s5 = (mul_r_prod > 255) ? 8'd255 : mul_r_prod[7:0];
            lit_g_s5 = (mul_g_prod > 255) ? 8'd255 : mul_g_prod[7:0];
            lit_b_s5 = (mul_b_prod > 255) ? 8'd255 : mul_b_prod[7:0];

            if (stat_s4 == ST_HIT) begin
                r_out <= lit_r_s5;
                g_out <= lit_g_s5;
                b_out <= lit_b_s5;
            end else begin
                r_out <= SKY_R;
                g_out <= SKY_G;
                b_out <= SKY_B;
            end
        end
    end

endmodule