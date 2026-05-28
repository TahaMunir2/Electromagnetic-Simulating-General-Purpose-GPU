// ============================================================================
//  normal.sv  (Design 1: bilinear shading)
//  ----------------------------------------------------------------------------
//  Reads 4 heightmap cells in the SQUARE pattern (bilinear corners):
//
//      (ix,   iy  )   (ix+1, iy  )
//      (ix,   iy+1)   (ix+1, iy+1)
//
//  Computes the smooth surface normal via FORWARD differences:
//
//      Nx = -(h10 - h00)
//      Ny = -(h01 - h00)
//      Nz = +1.0
//
//  Computes the bilinearly-interpolated surface height at (Px, Py):
//
//      xf = fractional part of world->cell mapping for Px   (Q0.FRAC_W)
//      yf = fractional part of world->cell mapping for Py
//
//      h_top    = lerp(h00, h10, xf)
//      h_bot    = lerp(h01, h11, xf)
//      h_interp = lerp(h_top, h_bot, yf)
//
//  This gives smooth shading (smooth normal) AND a smooth surface height
//  for altitude colouring — at zero BRAM cost compared to the previous
//  "plus" pattern (still 4 BRAM read ports).
//
//  4 BRAM read ports.  Synchronous BRAM (1-cycle read latency).
//
//  Pipeline (5 stages, latency = 5 cycles):
//      Stage 1: latch inputs, compute (ix, ix+1, iy, iy+1) with saturation
//               at GRID_N-1, extract xf/yf from Px_hit/Py_hit, drive BRAM
//               addresses.
//      Stage 2: BRAM is reading — carry payload + xf/yf forward.
//      Stage 3: BRAM data valid — latch h00, h10, h01, h11.
//      Stage 4: compute Nx, Ny (forward diff + saturate) and h_top, h_bot
//               (two parallel lerps).  Register all.
//      Stage 5: compute h_interp = lerp(h_top, h_bot, yf).  Drive outputs.
//
//  Total latency: 5 cycles (was 4).  Throughput: 1 pixel / cycle.
// ============================================================================

module normal #(
    parameter int GRID_N     = 256,
    parameter int IDX_W      = $clog2(GRID_N),

    parameter int POS_W      = 16,
    parameter int POS_I      = 4,
    parameter int POS_F      = POS_W - 1 - POS_I,

    parameter int H_W        = 16,
    parameter int H_I        = 4,
    parameter int H_F        = H_W - 1 - H_I,

    parameter int DIR_W      = 16,
    parameter int DIR_I      = 2,
    parameter int DIR_F      = DIR_W - 1 - DIR_I,

    parameter int PX_W       = 10,
    parameter int PY_W       = 10,
    parameter int STEP_W     = 5,

    // World half-extent (must match marcher / march_step).
    parameter logic signed [POS_W-1:0] WORLD_HALF = (1 <<< POS_F),

    // Fractional-position precision used for bilinear lerps.
    // 8 bits is plenty for visual smoothness at GRID_N=64..256 / 640x480.
    parameter int FRAC_W     = 8
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,           // ~stall

    // ----- From marcher -----
    input  logic [1:0]                 status_in,    // 01=HIT, 10=OFFGRID, 00=MISS
    input  logic [IDX_W-1:0]           ix_in,
    input  logic [IDX_W-1:0]           iy_in,
    input  logic signed [H_W-1:0]      h_hit_in,
    input  logic signed [POS_W-1:0]    Px_hit_in,    // world x at HIT
    input  logic signed [POS_W-1:0]    Py_hit_in,    // world y at HIT
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    // ----- 4-port BRAM interface.  SQUARE pattern. -----
    //   [0] = (ix,   iy  )    = h00
    //   [1] = (ix+1, iy  )    = h10
    //   [2] = (ix,   iy+1)    = h01
    //   [3] = (ix+1, iy+1)    = h11
    output logic [IDX_W*2-1:0]         bram_addr [4],
    output logic                       bram_re   [4],
    input  logic signed [H_W-1:0]      bram_dout [4],

    // ----- To shader -----
    output logic [1:0]                 status_out,
    output logic [IDX_W-1:0]           ix_out,
    output logic [IDX_W-1:0]           iy_out,
    output logic signed [H_W-1:0]      h_hit_out,        // raw cell height (kept for debug)
    output logic signed [H_W-1:0]      h_interp_out,     // bilinear smooth height
    output logic [STEP_W-1:0]          step_count_out,
    output logic signed [DIR_W-1:0]    Nx_out,
    output logic signed [DIR_W-1:0]    Ny_out,
    output logic signed [DIR_W-1:0]    Nz_out,
    output logic [PX_W-1:0]            px_out,
    output logic [PY_W-1:0]            py_out,
    output logic                       valid_out
);

    localparam logic [1:0] ST_HIT = 2'b01;
    localparam logic signed [DIR_W-1:0] NZ_CONST = (1 <<< DIR_F);

    // World->cell shift: see march_step.  (P + WORLD_HALF) >>> W2G_SHIFT
    // gives the integer cell; the low W2G_SHIFT bits are the sub-cell
    // fractional part as a Q0.W2G_SHIFT unsigned value.
    localparam int W2G_SHIFT = POS_F + 1 - $clog2(GRID_N);


    // =================================================================
    //  STAGE 1: latch inputs, compute (ix, ix+1, iy, iy+1), extract xf, yf.
    // =================================================================
    logic [1:0]                 stat_s1;
    logic [IDX_W-1:0]           ix_s1, iy_s1;
    logic signed [H_W-1:0]      hhit_s1;
    logic [STEP_W-1:0]          step_s1;
    logic [PX_W-1:0]            px_s1;
    logic [PY_W-1:0]            py_s1;
    logic                       v_s1;

    logic [IDX_W-1:0]           ix0_s1, ix1_s1;
    logic [IDX_W-1:0]           iy0_s1, iy1_s1;
    logic [FRAC_W-1:0]          xf_s1, yf_s1;

    // Combinational pre-compute from inputs (one cycle of slack to BRAM)
    logic [POS_W:0]             Px_shifted, Py_shifted;
    logic [FRAC_W-1:0]          xf_in, yf_in;

    always_comb begin
        // (P + WORLD_HALF), 17 bits unsigned (sign bit known to be 0 once shifted)
        Px_shifted = $signed({Px_hit_in[POS_W-1], Px_hit_in}) + $signed({1'b0, WORLD_HALF});
        Py_shifted = $signed({Py_hit_in[POS_W-1], Py_hit_in}) + $signed({1'b0, WORLD_HALF});
    end

    // The low W2G_SHIFT bits of (P + WORLD_HALF) are the sub-cell fraction
    // as a Q0.W2G_SHIFT unsigned value.  Generate picks the correct resize
    // for the FRAC_W >= W2G_SHIFT vs FRAC_W < W2G_SHIFT case at elab time.
    generate
        if (W2G_SHIFT >= FRAC_W) begin : g_frac_narrow
            // Drop low (W2G_SHIFT - FRAC_W) bits.
            assign xf_in = Px_shifted[W2G_SHIFT-1 -: FRAC_W];
            assign yf_in = Py_shifted[W2G_SHIFT-1 -: FRAC_W];
        end else begin : g_frac_pad
            // Zero-pad on the right.
            assign xf_in = { Px_shifted[W2G_SHIFT-1:0],
                             {(FRAC_W - W2G_SHIFT){1'b0}} };
            assign yf_in = { Py_shifted[W2G_SHIFT-1:0],
                             {(FRAC_W - W2G_SHIFT){1'b0}} };
        end
    endgenerate

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s1 <= 1'b0;
        end else if (en) begin
            // Pass through payload
            stat_s1  <= status_in;
            ix_s1    <= ix_in;
            iy_s1    <= iy_in;
            hhit_s1  <= h_hit_in;
            step_s1  <= step_count_in;
            px_s1    <= px_in;
            py_s1    <= py_in;
            v_s1     <= valid_in;

            // Corner indices (saturate at GRID_N-1; ix and iy are already >= 0)
            ix0_s1 <= ix_in;
            iy0_s1 <= iy_in;
            ix1_s1 <= (ix_in == GRID_N-1) ? ix_in : (ix_in + 1'b1);
            iy1_s1 <= (iy_in == GRID_N-1) ? iy_in : (iy_in + 1'b1);

            // Sub-cell fractions
            xf_s1 <= xf_in;
            yf_s1 <= yf_in;
        end
    end

    // BRAM addresses driven combinationally from stage-1 registers.
    // Address layout: {iy, ix} = iy*GRID_N + ix.
    assign bram_addr[0] = {iy0_s1, ix0_s1};   // h00 = (ix,   iy  )
    assign bram_addr[1] = {iy0_s1, ix1_s1};   // h10 = (ix+1, iy  )
    assign bram_addr[2] = {iy1_s1, ix0_s1};   // h01 = (ix,   iy+1)
    assign bram_addr[3] = {iy1_s1, ix1_s1};   // h11 = (ix+1, iy+1)

    assign bram_re[0] = v_s1;
    assign bram_re[1] = v_s1;
    assign bram_re[2] = v_s1;
    assign bram_re[3] = v_s1;


    // =================================================================
    //  STAGE 2: BRAM is reading — carry payload + xf/yf forward.
    // =================================================================
    logic [1:0]                 stat_s2;
    logic [IDX_W-1:0]           ix_s2, iy_s2;
    logic signed [H_W-1:0]      hhit_s2;
    logic [STEP_W-1:0]          step_s2;
    logic [PX_W-1:0]            px_s2;
    logic [PY_W-1:0]            py_s2;
    logic                       v_s2;
    logic [FRAC_W-1:0]          xf_s2, yf_s2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s2 <= 1'b0;
        end else if (en) begin
            stat_s2 <= stat_s1;
            ix_s2   <= ix_s1;
            iy_s2   <= iy_s1;
            hhit_s2 <= hhit_s1;
            step_s2 <= step_s1;
            px_s2   <= px_s1;
            py_s2   <= py_s1;
            v_s2    <= v_s1;
            xf_s2   <= xf_s1;
            yf_s2   <= yf_s1;
        end
    end


    // =================================================================
    //  STAGE 3: latch BRAM data into the four corner registers.
    // =================================================================
    logic [1:0]                 stat_s3;
    logic [IDX_W-1:0]           ix_s3, iy_s3;
    logic signed [H_W-1:0]      hhit_s3;
    logic [STEP_W-1:0]          step_s3;
    logic [PX_W-1:0]            px_s3;
    logic [PY_W-1:0]            py_s3;
    logic                       v_s3;
    logic [FRAC_W-1:0]          xf_s3, yf_s3;

    logic signed [H_W-1:0]      h00_s3, h10_s3, h01_s3, h11_s3;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s3 <= 1'b0;
        end else if (en) begin
            stat_s3 <= stat_s2;
            ix_s3   <= ix_s2;
            iy_s3   <= iy_s2;
            hhit_s3 <= hhit_s2;
            step_s3 <= step_s2;
            px_s3   <= px_s2;
            py_s3   <= py_s2;
            v_s3    <= v_s2;
            xf_s3   <= xf_s2;
            yf_s3   <= yf_s2;

            h00_s3 <= bram_dout[0];
            h10_s3 <= bram_dout[1];
            h01_s3 <= bram_dout[2];
            h11_s3 <= bram_dout[3];
        end
    end


    // =================================================================
    //  STAGE 4: compute Nx, Ny (forward diff + saturate) and the two
    //  intermediate lerps h_top, h_bot.  Register all.
    //
    //  Nx = -(h10 - h00), Ny = -(h01 - h00).
    //  h_top = h00 + xf*(h10-h00)
    //  h_bot = h01 + xf*(h11-h01)
    //
    //  Multiplier sizes: H_W+1 (signed diff) * FRAC_W (unsigned) -> H_W+FRAC_W+1.
    //  Defaults: 17 * 8 = 25 bits.  One DSP per lerp, two DSPs in parallel
    //  -> easy at 100 MHz.
    // =================================================================
    logic signed [H_W:0]           dx_h_comb, dy_h_comb;
    logic signed [H_W+DIR_W-1:0]   dx_shifted, dy_shifted;
    logic signed [DIR_W-1:0]       nx_calc, ny_calc;

    localparam logic signed [H_W+DIR_W-1:0] SAT_POS =
        $signed({1'b0, {(DIR_W-1){1'b1}}});                  // +32767
    localparam logic signed [H_W+DIR_W-1:0] SAT_NEG =
        $signed({{(H_W+1){1'b1}}, {(DIR_W-1){1'b0}}});       // -32768

    // Lerp helpers: 1 multiplier each.
    //   lerp(a, b, f) = a + ((b - a) * f) >>> FRAC_W
    // a, b are H_W signed.  (b-a) is H_W+1 signed.  f is FRAC_W unsigned.
    // Product is H_W+1+FRAC_W signed.
    logic signed [H_W:0]              diff_top_comb, diff_bot_comb;
    logic signed [H_W+FRAC_W:0]       prod_top_comb, prod_bot_comb;
    logic signed [H_W-1:0]            h_top_comb, h_bot_comb;

    always_comb begin
        // ---- Forward-difference normal ----
        dx_h_comb = $signed(h10_s3) - $signed(h00_s3);
        dy_h_comb = $signed(h01_s3) - $signed(h00_s3);

        if (DIR_F >= H_F) begin
            dx_shifted = $signed(dx_h_comb) <<< (DIR_F - H_F);
            dy_shifted = $signed(dy_h_comb) <<< (DIR_F - H_F);
        end else begin
            dx_shifted = $signed(dx_h_comb) >>> (H_F - DIR_F);
            dy_shifted = $signed(dy_h_comb) >>> (H_F - DIR_F);
        end

        // Saturate then negate (since N = -dh)
        if (dx_shifted > SAT_POS)
            nx_calc = -SAT_POS[DIR_W-1:0];
        else if (dx_shifted < SAT_NEG)
            nx_calc = -SAT_NEG[DIR_W-1:0];
        else
            nx_calc = -dx_shifted[DIR_W-1:0];

        if (dy_shifted > SAT_POS)
            ny_calc = -SAT_POS[DIR_W-1:0];
        else if (dy_shifted < SAT_NEG)
            ny_calc = -SAT_NEG[DIR_W-1:0];
        else
            ny_calc = -dy_shifted[DIR_W-1:0];

        // ---- Top/bottom lerps along x ----
        diff_top_comb = $signed(h10_s3) - $signed(h00_s3);
        diff_bot_comb = $signed(h11_s3) - $signed(h01_s3);
        prod_top_comb = diff_top_comb * $signed({1'b0, xf_s3});
        prod_bot_comb = diff_bot_comb * $signed({1'b0, xf_s3});
        // h00 + (prod >>> FRAC_W) — narrow to H_W with arithmetic shift
        h_top_comb = $signed(h00_s3) + prod_top_comb[H_W+FRAC_W-1 -: H_W];
        h_bot_comb = $signed(h01_s3) + prod_bot_comb[H_W+FRAC_W-1 -: H_W];
    end

    // Stage-4 registers
    logic [1:0]                 stat_s4;
    logic [IDX_W-1:0]           ix_s4, iy_s4;
    logic signed [H_W-1:0]      hhit_s4;
    logic [STEP_W-1:0]          step_s4;
    logic [PX_W-1:0]            px_s4;
    logic [PY_W-1:0]            py_s4;
    logic                       v_s4;
    logic [FRAC_W-1:0]          yf_s4;

    logic signed [DIR_W-1:0]    Nx_s4, Ny_s4, Nz_s4;
    logic signed [H_W-1:0]      h_top_s4, h_bot_s4;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s4 <= 1'b0;
        end else if (en) begin
            stat_s4 <= stat_s3;
            ix_s4   <= ix_s3;
            iy_s4   <= iy_s3;
            hhit_s4 <= hhit_s3;
            step_s4 <= step_s3;
            px_s4   <= px_s3;
            py_s4   <= py_s3;
            v_s4    <= v_s3;
            yf_s4   <= yf_s3;

            if (stat_s3 == ST_HIT) begin
                Nx_s4 <= nx_calc;
                Ny_s4 <= ny_calc;
                Nz_s4 <= NZ_CONST;
            end else begin
                Nx_s4 <= '0;
                Ny_s4 <= '0;
                Nz_s4 <= NZ_CONST;
            end

            h_top_s4 <= h_top_comb;
            h_bot_s4 <= h_bot_comb;
        end
    end


    // =================================================================
    //  STAGE 5: final lerp along y.  Drive outputs.
    //
    //  h_interp = h_top + ((h_bot - h_top) * yf) >>> FRAC_W
    // =================================================================
    logic signed [H_W:0]          diff_y_comb;
    logic signed [H_W+FRAC_W:0]   prod_y_comb;
    logic signed [H_W-1:0]        h_interp_comb;

    always_comb begin
        diff_y_comb   = $signed(h_bot_s4) - $signed(h_top_s4);
        prod_y_comb   = diff_y_comb * $signed({1'b0, yf_s4});
        h_interp_comb = $signed(h_top_s4) + prod_y_comb[H_W+FRAC_W-1 -: H_W];
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            status_out     <= stat_s4;
            ix_out         <= ix_s4;
            iy_out         <= iy_s4;
            h_hit_out      <= hhit_s4;
            step_count_out <= step_s4;
            px_out         <= px_s4;
            py_out         <= py_s4;
            valid_out      <= v_s4;

            Nx_out <= Nx_s4;
            Ny_out <= Ny_s4;
            Nz_out <= Nz_s4;

            // For non-HIT pixels h_interp is meaningless; the shader
            // ignores it via status_in.  Still output something defined.
            if (stat_s4 == ST_HIT)
                h_interp_out <= h_interp_comb;
            else
                h_interp_out <= hhit_s4;
        end
    end

endmodule
