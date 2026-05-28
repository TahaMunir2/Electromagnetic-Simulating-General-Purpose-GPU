// ============================================================================
//  normal2.sv  (Design 2: folded normal with 2 BRAM ports)
//  ----------------------------------------------------------------------------
//  Same algorithm as normal.sv (Design 1) but uses 2 BRAM ports instead
//  of 4, by spreading the 4 corner reads over 2 cycles.
//
//      Port A:  phase=0 -> (ix,   iy  ) = h00     phase=1 -> (ix+1, iy  ) = h10
//      Port B:  phase=0 -> (ix,   iy+1) = h01     phase=1 -> (ix+1, iy+1) = h11
//
//  This needs only 1 heightmap copy (= 2 BRAM18 tiles) instead of 2.
//
//  Pipeline (6 stages, latency = 6 cycles):
//      Stage 1: latch inputs, compute (ix, ix+1, iy, iy+1) and xf, yf.
//               Drive BRAM addresses (combinational from stage-1 regs
//               + phase mux).
//      Stage 2: BRAM is reading first half (h00, h01).
//      Stage 3: capture h00, h01.  BRAM reading second half (h10, h11).
//      Stage 4: capture h10, h11.
//      Stage 5: compute Nx, Ny, h_top, h_bot.
//      Stage 6: compute h_interp = lerp(h_top, h_bot, yf).  Drive outputs.
//
//  Throughput: 1 pixel / 2 cycles (matches marcher2).
// ============================================================================

module normal2 #(
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

    parameter logic signed [POS_W-1:0] WORLD_HALF = (1 <<< POS_F),
    parameter int FRAC_W     = 8
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,

    input  logic [1:0]                 status_in,
    input  logic [IDX_W-1:0]           ix_in,
    input  logic [IDX_W-1:0]           iy_in,
    input  logic signed [H_W-1:0]      h_hit_in,
    input  logic signed [POS_W-1:0]    Px_hit_in,
    input  logic signed [POS_W-1:0]    Py_hit_in,
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    // 2-port BRAM interface.
    //   port 0: alternates h00 (phase=0) / h10 (phase=1)
    //   port 1: alternates h01 (phase=0) / h11 (phase=1)
    output logic [IDX_W*2-1:0]         bram_addr [2],
    output logic                       bram_re   [2],
    input  logic signed [H_W-1:0]      bram_dout [2],

    output logic [1:0]                 status_out,
    output logic [IDX_W-1:0]           ix_out,
    output logic [IDX_W-1:0]           iy_out,
    output logic signed [H_W-1:0]      h_hit_out,
    output logic signed [H_W-1:0]      h_interp_out,
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
    localparam int W2G_SHIFT = POS_F + 1 - $clog2(GRID_N);

    // Internal phase counter (toggles every cycle when en=1).
    logic phase_in;
    always_ff @(posedge clk) begin
        if (!rst_n) phase_in <= 1'b0;
        else if (en) phase_in <= ~phase_in;
    end

    // =================================================================
    //  STAGE 1: latch inputs, compute corner indices and xf/yf.
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

    logic [POS_W:0]             Px_shifted, Py_shifted;
    logic [FRAC_W-1:0]          xf_in, yf_in;

    always_comb begin
        Px_shifted = $signed({Px_hit_in[POS_W-1], Px_hit_in}) + $signed({1'b0, WORLD_HALF});
        Py_shifted = $signed({Py_hit_in[POS_W-1], Py_hit_in}) + $signed({1'b0, WORLD_HALF});
    end

    generate
        if (W2G_SHIFT >= FRAC_W) begin : g_frac_narrow
            assign xf_in = Px_shifted[W2G_SHIFT-1 -: FRAC_W];
            assign yf_in = Py_shifted[W2G_SHIFT-1 -: FRAC_W];
        end else begin : g_frac_pad
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
            stat_s1  <= status_in;
            ix_s1    <= ix_in;
            iy_s1    <= iy_in;
            hhit_s1  <= h_hit_in;
            step_s1  <= step_count_in;
            px_s1    <= px_in;
            py_s1    <= py_in;
            v_s1     <= valid_in;

            ix0_s1 <= ix_in;
            iy0_s1 <= iy_in;
            ix1_s1 <= (ix_in == GRID_N-1) ? ix_in : (ix_in + 1'b1);
            iy1_s1 <= (iy_in == GRID_N-1) ? iy_in : (iy_in + 1'b1);

            xf_s1 <= xf_in;
            yf_s1 <= yf_in;
        end
    end

    // BRAM address mux: phase=0 -> x=ix0; phase=1 -> x=ix1
    assign bram_addr[0] = (phase_in == 1'b0) ? {iy0_s1, ix0_s1}    // h00
                                             : {iy0_s1, ix1_s1};   // h10
    assign bram_addr[1] = (phase_in == 1'b0) ? {iy1_s1, ix0_s1}    // h01
                                             : {iy1_s1, ix1_s1};   // h11
    assign bram_re[0] = v_s1;
    assign bram_re[1] = v_s1;


    // =================================================================
    //  STAGE 2: first reads in flight.  Carry payload + xf/yf.
    // =================================================================
    logic [1:0]                 stat_s2;
    logic [IDX_W-1:0]           ix_s2, iy_s2;
    logic signed [H_W-1:0]      hhit_s2;
    logic [STEP_W-1:0]          step_s2;
    logic [PX_W-1:0]            px_s2;
    logic [PY_W-1:0]            py_s2;
    logic                       v_s2;
    logic [FRAC_W-1:0]          xf_s2, yf_s2;
    logic                       phase_s1_reg;   // phase when read was issued in stage 1

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
            phase_s1_reg <= phase_in;   // remember which phase entered stage 1
        end
    end


    // =================================================================
    //  STAGE 3: first BRAM data arrives.  Capture h00,h01 or h10,h11
    //  depending on which phase issued the first read.
    // =================================================================
    logic [1:0]                 stat_s3;
    logic [IDX_W-1:0]           ix_s3, iy_s3;
    logic signed [H_W-1:0]      hhit_s3;
    logic [STEP_W-1:0]          step_s3;
    logic [PX_W-1:0]            px_s3;
    logic [PY_W-1:0]            py_s3;
    logic                       v_s3;
    logic [FRAC_W-1:0]          xf_s3, yf_s3;
    logic                       phase_s2_reg;

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
            phase_s2_reg <= phase_s1_reg;

            // First read was at phase_s1_reg.  If that was phase=0, ports
            // delivered h00/h01; if phase=1, they delivered h10/h11.
            if (phase_s1_reg == 1'b0) begin
                h00_s3 <= bram_dout[0];
                h01_s3 <= bram_dout[1];
            end else begin
                h10_s3 <= bram_dout[0];
                h11_s3 <= bram_dout[1];
            end
        end
    end


    // =================================================================
    //  STAGE 4: second BRAM data arrives.  Capture the other half.
    //  After this stage all 4 corners are captured.
    // =================================================================
    logic [1:0]                 stat_s4;
    logic [IDX_W-1:0]           ix_s4, iy_s4;
    logic signed [H_W-1:0]      hhit_s4;
    logic [STEP_W-1:0]          step_s4;
    logic [PX_W-1:0]            px_s4;
    logic [PY_W-1:0]            py_s4;
    logic                       v_s4;
    logic [FRAC_W-1:0]          xf_s4, yf_s4;

    logic signed [H_W-1:0]      h00_s4, h10_s4, h01_s4, h11_s4;

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
            xf_s4   <= xf_s3;
            yf_s4   <= yf_s3;

            // Carry already-captured corners forward
            h00_s4 <= h00_s3;
            h10_s4 <= h10_s3;
            h01_s4 <= h01_s3;
            h11_s4 <= h11_s3;

            // Capture the OTHER half this cycle.  Second read was at the
            // phase opposite to phase_s2_reg (which is phase_s1_reg
            // pipelined once).  Equivalently: data on bram_dout this
            // cycle is for phase = ~phase_s2_reg.
            if (phase_s2_reg == 1'b0) begin
                // First half was h00/h01, so second half is h10/h11
                h10_s4 <= bram_dout[0];
                h11_s4 <= bram_dout[1];
            end else begin
                h00_s4 <= bram_dout[0];
                h01_s4 <= bram_dout[1];
            end
        end
    end


    // =================================================================
    //  STAGE 5: compute Nx, Ny (forward diff + saturate) and the two
    //  intermediate lerps h_top, h_bot.  Identical to normal.sv stage 4.
    // =================================================================
    logic signed [H_W:0]           dx_h_comb, dy_h_comb;
    logic signed [H_W+DIR_W-1:0]   dx_shifted, dy_shifted;
    logic signed [DIR_W-1:0]       nx_calc, ny_calc;

    localparam logic signed [H_W+DIR_W-1:0] SAT_POS =
        $signed({1'b0, {(DIR_W-1){1'b1}}});
    localparam logic signed [H_W+DIR_W-1:0] SAT_NEG =
        $signed({{(H_W+1){1'b1}}, {(DIR_W-1){1'b0}}});

    logic signed [H_W:0]              diff_top_comb, diff_bot_comb;
    logic signed [H_W+FRAC_W:0]       prod_top_comb, prod_bot_comb;
    logic signed [H_W-1:0]            h_top_comb, h_bot_comb;

    always_comb begin
        dx_h_comb = $signed(h10_s4) - $signed(h00_s4);
        dy_h_comb = $signed(h01_s4) - $signed(h00_s4);

        if (DIR_F >= H_F) begin
            dx_shifted = $signed(dx_h_comb) <<< (DIR_F - H_F);
            dy_shifted = $signed(dy_h_comb) <<< (DIR_F - H_F);
        end else begin
            dx_shifted = $signed(dx_h_comb) >>> (H_F - DIR_F);
            dy_shifted = $signed(dy_h_comb) >>> (H_F - DIR_F);
        end

        if (dx_shifted > SAT_POS)      nx_calc = -SAT_POS[DIR_W-1:0];
        else if (dx_shifted < SAT_NEG) nx_calc = -SAT_NEG[DIR_W-1:0];
        else                           nx_calc = -dx_shifted[DIR_W-1:0];

        if (dy_shifted > SAT_POS)      ny_calc = -SAT_POS[DIR_W-1:0];
        else if (dy_shifted < SAT_NEG) ny_calc = -SAT_NEG[DIR_W-1:0];
        else                           ny_calc = -dy_shifted[DIR_W-1:0];

        diff_top_comb = $signed(h10_s4) - $signed(h00_s4);
        diff_bot_comb = $signed(h11_s4) - $signed(h01_s4);
        prod_top_comb = diff_top_comb * $signed({1'b0, xf_s4});
        prod_bot_comb = diff_bot_comb * $signed({1'b0, xf_s4});
        h_top_comb = $signed(h00_s4) + prod_top_comb[H_W+FRAC_W-1 -: H_W];
        h_bot_comb = $signed(h01_s4) + prod_bot_comb[H_W+FRAC_W-1 -: H_W];
    end

    logic [1:0]                 stat_s5;
    logic [IDX_W-1:0]           ix_s5, iy_s5;
    logic signed [H_W-1:0]      hhit_s5;
    logic [STEP_W-1:0]          step_s5;
    logic [PX_W-1:0]            px_s5;
    logic [PY_W-1:0]            py_s5;
    logic                       v_s5;
    logic [FRAC_W-1:0]          yf_s5;

    logic signed [DIR_W-1:0]    Nx_s5, Ny_s5, Nz_s5;
    logic signed [H_W-1:0]      h_top_s5, h_bot_s5;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_s5 <= 1'b0;
        end else if (en) begin
            stat_s5 <= stat_s4;
            ix_s5   <= ix_s4;
            iy_s5   <= iy_s4;
            hhit_s5 <= hhit_s4;
            step_s5 <= step_s4;
            px_s5   <= px_s4;
            py_s5   <= py_s4;
            v_s5    <= v_s4;
            yf_s5   <= yf_s4;

            if (stat_s4 == ST_HIT) begin
                Nx_s5 <= nx_calc;
                Ny_s5 <= ny_calc;
                Nz_s5 <= NZ_CONST;
            end else begin
                Nx_s5 <= '0;
                Ny_s5 <= '0;
                Nz_s5 <= NZ_CONST;
            end

            h_top_s5 <= h_top_comb;
            h_bot_s5 <= h_bot_comb;
        end
    end


    // =================================================================
    //  STAGE 6: final lerp along y.  Drive outputs.
    // =================================================================
    logic signed [H_W:0]          diff_y_comb;
    logic signed [H_W+FRAC_W:0]   prod_y_comb;
    logic signed [H_W-1:0]        h_interp_comb;

    always_comb begin
        diff_y_comb   = $signed(h_bot_s5) - $signed(h_top_s5);
        prod_y_comb   = diff_y_comb * $signed({1'b0, yf_s5});
        h_interp_comb = $signed(h_top_s5) + prod_y_comb[H_W+FRAC_W-1 -: H_W];
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            status_out     <= stat_s5;
            ix_out         <= ix_s5;
            iy_out         <= iy_s5;
            h_hit_out      <= hhit_s5;
            step_count_out <= step_s5;
            px_out         <= px_s5;
            py_out         <= py_s5;
            valid_out      <= v_s5;

            Nx_out <= Nx_s5;
            Ny_out <= Ny_s5;
            Nz_out <= Nz_s5;

            if (stat_s5 == ST_HIT)
                h_interp_out <= h_interp_comb;
            else
                h_interp_out <= hhit_s5;
        end
    end

endmodule
