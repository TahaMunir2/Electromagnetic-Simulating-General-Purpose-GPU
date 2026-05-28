// ============================================================================
//  march_step4.sv  (Design 4: bilinear-in-marcher, quarter-rate, 1 port/step)
//  ----------------------------------------------------------------------------
//  SAME ALGORITHM as march_step3 (true bilinear SDF lookup inside every step,
//  hits land on the smooth interpolated surface).  The ONLY change is read
//  scheduling: the 4 corner reads are folded across 4 cycles on a SINGLE BRAM
//  port instead of 2 cycles on 2 ports.
//
//      Design 3:  4 reads / 2 cycles / 2 ports  -> 32 ports -> 32 BRAM, 1px/2c
//      Design 4:  4 reads / 4 cycles / 1 port   -> 16 ports -> 16 BRAM, 1px/4c
//
//  The BRAM saving is bought purely with throughput: same total reads per
//  pixel, spread more thinly in time.  This brings the marcher BRAM cost back
//  down to Design 1's level while keeping Design 3's smooth silhouettes.
//
//  READ SCHEDULE (single port, one corner per cycle):
//      issue B0 : (ix0, iy0) = h00
//      issue B1 : (ix1, iy0) = h10
//      issue B2 : (ix0, iy1) = h01
//      issue B3 : (ix1, iy1) = h11
//  Each issued address is captured one cycle later (BRAM read latency).
//  After the 4th capture all corners are held; the lerp+compare then fires.
//
//  Because a flow-through pipeline cannot "hold" a pixel in one stage for 4
//  cycles, the 4 issue+capture steps are implemented as 4 cascaded pipeline
//  sub-stages (B0..B3), each driving the shared port on its OWN cycle and
//  capturing the previous sub-stage's read.  The marcher feeds a new real
//  pixel only every 4 cycles (quarter-rate), so the single port is never
//  contended between pixels.
//
//  PIPELINE (10 stages, latency = 10 cycles):
//      A    : advance position P += dt*D
//      B0   : compute ix0/iy0/ix1/iy1, xf, yf; issue addr(h00)
//      B1   : capture h00; issue addr(h10)
//      B2   : capture h10; issue addr(h01)
//      B3   : capture h01; issue addr(h11)
//      B4   : capture h11   -> all 4 corners held
//      D    : x-lerps  -> h_top, h_bot
//      D2   : y-lerp   -> h_interp, then below/cross compare
//      E    : output buffer
//      (A + B0..B4 + D + D2 + E = 1+5+1+1+1 = 10)
//
//  PRESERVED INVARIANTS (identical to march_step3):
//    - Frozen-ray pattern; Bug 2 fix (step 0 cannot HIT).
//    - prev_below / h_hit use the interpolated height h_interp.
//    - Lerp arithmetic reused verbatim so the Python golden matches bit-exact.
//
//  Port driving: each sub-stage drives bram_addr/bram_re ONLY on the cycle it
//  owns the port; outputs are OR-combined to the single port by the marcher
//  (only one sub-stage of a given step is mid-issue per cycle by construction).
// ============================================================================

module march_step4 #(
    parameter int POS_W      = 16,
    parameter int POS_I      = 4,
    parameter int POS_F      = POS_W - 1 - POS_I,

    parameter int DIR_W      = 16,
    parameter int DIR_I      = 2,
    parameter int DIR_F      = DIR_W - 1 - DIR_I,

    parameter int GRID_N     = 256,
    parameter int IDX_W      = $clog2(GRID_N),

    parameter int H_W        = 16,
    parameter int H_I        = 4,
    parameter int H_F        = H_W - 1 - H_I,

    parameter int STEP_W     = 5,
    parameter int FRAC_W     = 8,

    parameter logic signed [POS_W-1:0] WORLD_HALF = (1 <<< POS_F),
    parameter logic signed [POS_W-1:0] DT = (2 * WORLD_HALF) / GRID_N
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,

    // ----- Pipeline data in -----
    input  logic signed [POS_W-1:0]    Px_in,
    input  logic signed [POS_W-1:0]    Py_in,
    input  logic signed [POS_W-1:0]    Pz_in,
    input  logic signed [DIR_W-1:0]    Dx,
    input  logic signed [DIR_W-1:0]    Dy,
    input  logic signed [DIR_W-1:0]    Dz,
    input  logic [1:0]                 status_in,
    input  logic                       prev_below_in,
    input  logic signed [H_W-1:0]      h_hit_in,
    input  logic [IDX_W-1:0]           ix_hit_in,
    input  logic [IDX_W-1:0]           iy_hit_in,
    input  logic signed [POS_W-1:0]    Px_hit_in,
    input  logic signed [POS_W-1:0]    Py_hit_in,
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic                       valid_in,

    // ----- Single BRAM read port -----
    output logic [IDX_W*2-1:0]         bram_addr,
    output logic                       bram_re,
    input  logic signed [H_W-1:0]      bram_dout,

    // ----- Pipeline data out -----
    output logic signed [POS_W-1:0]    Px_out,
    output logic signed [POS_W-1:0]    Py_out,
    output logic signed [POS_W-1:0]    Pz_out,
    output logic signed [DIR_W-1:0]    Dx_out,
    output logic signed [DIR_W-1:0]    Dy_out,
    output logic signed [DIR_W-1:0]    Dz_out,
    output logic [1:0]                 status_out,
    output logic                       prev_below_out,
    output logic signed [H_W-1:0]      h_hit_out,
    output logic [IDX_W-1:0]           ix_hit_out,
    output logic [IDX_W-1:0]           iy_hit_out,
    output logic signed [POS_W-1:0]    Px_hit_out,
    output logic signed [POS_W-1:0]    Py_hit_out,
    output logic [STEP_W-1:0]          step_count_out,
    output logic                       valid_out
);

    localparam logic [1:0] ST_MARCHING = 2'b00;
    localparam logic [1:0] ST_HIT      = 2'b01;
    localparam logic [1:0] ST_OFFGRID  = 2'b10;


    // =================================================================
    //  STAGE A: advance position (identical to march_step3).
    // =================================================================
    logic signed [POS_W + DIR_W - 1 : 0]  raw_dtDx, raw_dtDy, raw_dtDz;
    logic signed [POS_W - 1 : 0]          inc_Px, inc_Py, inc_Pz;

    always_comb begin
        raw_dtDx = $signed(DT) * Dx;
        raw_dtDy = $signed(DT) * Dy;
        raw_dtDz = $signed(DT) * Dz;
        inc_Px = raw_dtDx[DIR_F + POS_W - 1 -: POS_W];
        inc_Py = raw_dtDy[DIR_F + POS_W - 1 -: POS_W];
        inc_Pz = raw_dtDz[DIR_F + POS_W - 1 -: POS_W];
    end

    logic signed [POS_W-1:0]   Px_A, Py_A, Pz_A;
    logic signed [DIR_W-1:0]   Dx_A, Dy_A, Dz_A;
    logic [1:0]                stat_A;
    logic                      prev_A;
    logic signed [H_W-1:0]     h_hit_A;
    logic [IDX_W-1:0]          ix_hit_A, iy_hit_A;
    logic signed [POS_W-1:0]   Px_hit_A, Py_hit_A;
    logic [STEP_W-1:0]         step_count_A;
    logic                      v_A;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_A <= 1'b0;
        end else if (en) begin
            if (status_in == ST_MARCHING) begin
                Px_A <= Px_in + inc_Px;
                Py_A <= Py_in + inc_Py;
                Pz_A <= Pz_in + inc_Pz;
            end else begin
                Px_A <= Px_in;
                Py_A <= Py_in;
                Pz_A <= Pz_in;
            end
            Dx_A         <= Dx;
            Dy_A         <= Dy;
            Dz_A         <= Dz;
            stat_A       <= status_in;
            prev_A       <= prev_below_in;
            h_hit_A      <= h_hit_in;
            ix_hit_A     <= ix_hit_in;
            iy_hit_A     <= iy_hit_in;
            Px_hit_A     <= Px_hit_in;
            Py_hit_A     <= Py_hit_in;
            step_count_A <= step_count_in;
            v_A          <= valid_in;
        end
    end


    // =================================================================
    //  STAGE B0: index + fractional weights + off-grid; ISSUE h00.
    //  (Index/frac arithmetic identical to march_step3 stage B.)
    // =================================================================
    localparam int W2G_SHIFT = POS_F + 1 - $clog2(GRID_N);

    logic signed [POS_W:0]  ix_raw, iy_raw;
    logic                   offgrid_B;
    logic [IDX_W-1:0]       ix0_B, iy0_B, ix1_B, iy1_B;
    logic [POS_W:0]         Px_shifted, Py_shifted;
    logic [FRAC_W-1:0]      xf_B, yf_B;

    always_comb begin
        ix_raw = $signed({Px_A[POS_W-1], Px_A}) + $signed({1'b0, WORLD_HALF});
        iy_raw = $signed({Py_A[POS_W-1], Py_A}) + $signed({1'b0, WORLD_HALF});
        ix_raw = ix_raw >>> W2G_SHIFT;
        iy_raw = iy_raw >>> W2G_SHIFT;
        offgrid_B = (ix_raw < 0) || (ix_raw >= GRID_N) ||
                    (iy_raw < 0) || (iy_raw >= GRID_N);
        Px_shifted = $signed({Px_A[POS_W-1], Px_A}) + $signed({1'b0, WORLD_HALF});
        Py_shifted = $signed({Py_A[POS_W-1], Py_A}) + $signed({1'b0, WORLD_HALF});
    end

    assign ix0_B = ix_raw[IDX_W-1:0];
    assign iy0_B = iy_raw[IDX_W-1:0];
    assign ix1_B = (ix0_B == GRID_N-1) ? ix0_B : (ix0_B + 1'b1);
    assign iy1_B = (iy0_B == GRID_N-1) ? iy0_B : (iy0_B + 1'b1);

    generate
        if (W2G_SHIFT >= FRAC_W) begin : g_frac_narrow
            assign xf_B = Px_shifted[W2G_SHIFT-1 -: FRAC_W];
            assign yf_B = Py_shifted[W2G_SHIFT-1 -: FRAC_W];
        end else begin : g_frac_pad
            assign xf_B = { Px_shifted[W2G_SHIFT-1:0], {(FRAC_W - W2G_SHIFT){1'b0}} };
            assign yf_B = { Py_shifted[W2G_SHIFT-1:0], {(FRAC_W - W2G_SHIFT){1'b0}} };
        end
    endgenerate

    // --- B0 registers (carry payload + the 4 corner addresses + frac) ---
    logic signed [POS_W-1:0]  Px_B0, Py_B0, Pz_B0;
    logic signed [DIR_W-1:0]  Dx_B0, Dy_B0, Dz_B0;
    logic [1:0]               stat_B0;
    logic                     prev_B0;
    logic signed [H_W-1:0]    h_hit_B0;
    logic [IDX_W-1:0]         ix_hit_B0, iy_hit_B0;
    logic signed [POS_W-1:0]  Px_hit_B0, Py_hit_B0;
    logic [STEP_W-1:0]        step_count_B0;
    logic                     v_B0;
    logic [FRAC_W-1:0]        xf_B0, yf_B0;
    // Latched corner addresses so later sub-stages can re-issue them.
    logic [IDX_W-1:0]         ix0_q, iy0_q, ix1_q, iy1_q;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_B0 <= 1'b0;
        end else if (en) begin
            Px_B0   <= Px_A;
            Py_B0   <= Py_A;
            Pz_B0   <= Pz_A;
            Dx_B0   <= Dx_A;
            Dy_B0   <= Dy_A;
            Dz_B0   <= Dz_A;
            prev_B0 <= prev_A;
            v_B0    <= v_A;
            xf_B0   <= xf_B;
            yf_B0   <= yf_B;
            ix0_q   <= ix0_B;
            iy0_q   <= iy0_B;
            ix1_q   <= ix1_B;
            iy1_q   <= iy1_B;

            if (stat_A != ST_MARCHING)
                stat_B0 <= stat_A;
            else if (offgrid_B)
                stat_B0 <= ST_OFFGRID;
            else
                stat_B0 <= ST_MARCHING;

            if (stat_A == ST_MARCHING && !offgrid_B) begin
                ix_hit_B0 <= ix0_B;
                iy_hit_B0 <= iy0_B;
                Px_hit_B0 <= Px_A;
                Py_hit_B0 <= Py_A;
            end else begin
                ix_hit_B0 <= ix_hit_A;
                iy_hit_B0 <= iy_hit_A;
                Px_hit_B0 <= Px_hit_A;
                Py_hit_B0 <= Py_hit_A;
            end

            h_hit_B0 <= h_hit_A;

            if (stat_A == ST_MARCHING)
                step_count_B0 <= step_count_A + 1'b1;
            else
                step_count_B0 <= step_count_A;
        end
    end

    // B0 owns the port this cycle: issue h00 = (ix0, iy0).
    // (Driven combinationally from B0's registered addresses.)
    wire issue_active_B0 = v_B0 && (stat_B0 == ST_MARCHING);


    // =================================================================
    //  Generic carry struct for sub-stages B1..B4.  Each sub-stage:
    //    - captures the previous issue's bram_dout into the right corner reg
    //    - issues the NEXT corner address on the shared port
    //  We carry all four corner addresses + the captured corners forward.
    // =================================================================
    // ---- B1: capture h00, issue h10 ----
    logic signed [POS_W-1:0]  Px_B1, Py_B1, Pz_B1;
    logic signed [DIR_W-1:0]  Dx_B1, Dy_B1, Dz_B1;
    logic [1:0]               stat_B1;
    logic                     prev_B1;
    logic signed [H_W-1:0]    h_hit_B1;
    logic [IDX_W-1:0]         ix_hit_B1, iy_hit_B1;
    logic signed [POS_W-1:0]  Px_hit_B1, Py_hit_B1;
    logic [STEP_W-1:0]        step_count_B1;
    logic                     v_B1;
    logic [FRAC_W-1:0]        xf_B1, yf_B1;
    logic [IDX_W-1:0]         ix0_B1, iy0_B1, ix1_B1, iy1_B1;
    logic signed [H_W-1:0]    h00_B1;

    always_ff @(posedge clk) begin
        if (!rst_n) v_B1 <= 1'b0;
        else if (en) begin
            Px_B1<=Px_B0; Py_B1<=Py_B0; Pz_B1<=Pz_B0;
            Dx_B1<=Dx_B0; Dy_B1<=Dy_B0; Dz_B1<=Dz_B0;
            stat_B1<=stat_B0; prev_B1<=prev_B0; v_B1<=v_B0;
            h_hit_B1<=h_hit_B0; ix_hit_B1<=ix_hit_B0; iy_hit_B1<=iy_hit_B0;
            Px_hit_B1<=Px_hit_B0; Py_hit_B1<=Py_hit_B0; step_count_B1<=step_count_B0;
            xf_B1<=xf_B0; yf_B1<=yf_B0;
            ix0_B1<=ix0_q; iy0_B1<=iy0_q; ix1_B1<=ix1_q; iy1_B1<=iy1_q;
            h00_B1 <= bram_dout;            // capture h00 (issued in B0)
        end
    end
    wire issue_active_B1 = v_B1 && (stat_B1 == ST_MARCHING);

    // ---- B2: capture h10, issue h01 ----
    logic signed [POS_W-1:0]  Px_B2, Py_B2, Pz_B2;
    logic signed [DIR_W-1:0]  Dx_B2, Dy_B2, Dz_B2;
    logic [1:0]               stat_B2;
    logic                     prev_B2;
    logic signed [H_W-1:0]    h_hit_B2;
    logic [IDX_W-1:0]         ix_hit_B2, iy_hit_B2;
    logic signed [POS_W-1:0]  Px_hit_B2, Py_hit_B2;
    logic [STEP_W-1:0]        step_count_B2;
    logic                     v_B2;
    logic [FRAC_W-1:0]        xf_B2, yf_B2;
    logic [IDX_W-1:0]         ix0_B2, iy0_B2, ix1_B2, iy1_B2;
    logic signed [H_W-1:0]    h00_B2, h10_B2;

    always_ff @(posedge clk) begin
        if (!rst_n) v_B2 <= 1'b0;
        else if (en) begin
            Px_B2<=Px_B1; Py_B2<=Py_B1; Pz_B2<=Pz_B1;
            Dx_B2<=Dx_B1; Dy_B2<=Dy_B1; Dz_B2<=Dz_B1;
            stat_B2<=stat_B1; prev_B2<=prev_B1; v_B2<=v_B1;
            h_hit_B2<=h_hit_B1; ix_hit_B2<=ix_hit_B1; iy_hit_B2<=iy_hit_B1;
            Px_hit_B2<=Px_hit_B1; Py_hit_B2<=Py_hit_B1; step_count_B2<=step_count_B1;
            xf_B2<=xf_B1; yf_B2<=yf_B1;
            ix0_B2<=ix0_B1; iy0_B2<=iy0_B1; ix1_B2<=ix1_B1; iy1_B2<=iy1_B1;
            h00_B2 <= h00_B1;
            h10_B2 <= bram_dout;           // capture h10 (issued in B1)
        end
    end
    wire issue_active_B2 = v_B2 && (stat_B2 == ST_MARCHING);

    // ---- B3: capture h01, issue h11 ----
    logic signed [POS_W-1:0]  Px_B3, Py_B3, Pz_B3;
    logic signed [DIR_W-1:0]  Dx_B3, Dy_B3, Dz_B3;
    logic [1:0]               stat_B3;
    logic                     prev_B3;
    logic signed [H_W-1:0]    h_hit_B3;
    logic [IDX_W-1:0]         ix_hit_B3, iy_hit_B3;
    logic signed [POS_W-1:0]  Px_hit_B3, Py_hit_B3;
    logic [STEP_W-1:0]        step_count_B3;
    logic                     v_B3;
    logic [FRAC_W-1:0]        xf_B3, yf_B3;
    logic [IDX_W-1:0]         ix1_B3, iy1_B3;
    logic signed [H_W-1:0]    h00_B3, h10_B3, h01_B3;

    always_ff @(posedge clk) begin
        if (!rst_n) v_B3 <= 1'b0;
        else if (en) begin
            Px_B3<=Px_B2; Py_B3<=Py_B2; Pz_B3<=Pz_B2;
            Dx_B3<=Dx_B2; Dy_B3<=Dy_B2; Dz_B3<=Dz_B2;
            stat_B3<=stat_B2; prev_B3<=prev_B2; v_B3<=v_B2;
            h_hit_B3<=h_hit_B2; ix_hit_B3<=ix_hit_B2; iy_hit_B3<=iy_hit_B2;
            Px_hit_B3<=Px_hit_B2; Py_hit_B3<=Py_hit_B2; step_count_B3<=step_count_B2;
            xf_B3<=xf_B2; yf_B3<=yf_B2;
            ix1_B3<=ix1_B2; iy1_B3<=iy1_B2;
            h00_B3 <= h00_B2;
            h10_B3 <= h10_B2;
            h01_B3 <= bram_dout;           // capture h01 (issued in B2)
        end
    end
    wire issue_active_B3 = v_B3 && (stat_B3 == ST_MARCHING);

    // ---- B4: capture h11 -> all four corners held ----
    logic signed [POS_W-1:0]  Px_B4, Py_B4, Pz_B4;
    logic signed [DIR_W-1:0]  Dx_B4, Dy_B4, Dz_B4;
    logic [1:0]               stat_B4;
    logic                     prev_B4;
    logic signed [H_W-1:0]    h_hit_B4;
    logic [IDX_W-1:0]         ix_hit_B4, iy_hit_B4;
    logic signed [POS_W-1:0]  Px_hit_B4, Py_hit_B4;
    logic [STEP_W-1:0]        step_count_B4;
    logic                     v_B4;
    logic [FRAC_W-1:0]        xf_B4, yf_B4;
    logic signed [H_W-1:0]    h00_B4, h10_B4, h01_B4, h11_B4;

    always_ff @(posedge clk) begin
        if (!rst_n) v_B4 <= 1'b0;
        else if (en) begin
            Px_B4<=Px_B3; Py_B4<=Py_B3; Pz_B4<=Pz_B3;
            Dx_B4<=Dx_B3; Dy_B4<=Dy_B3; Dz_B4<=Dz_B3;
            stat_B4<=stat_B3; prev_B4<=prev_B3; v_B4<=v_B3;
            h_hit_B4<=h_hit_B3; ix_hit_B4<=ix_hit_B3; iy_hit_B4<=iy_hit_B3;
            Px_hit_B4<=Px_hit_B3; Py_hit_B4<=Py_hit_B3; step_count_B4<=step_count_B3;
            xf_B4<=xf_B3; yf_B4<=yf_B3;
            h00_B4 <= h00_B3;
            h10_B4 <= h10_B3;
            h01_B4 <= h01_B3;
            h11_B4 <= bram_dout;           // capture h11 (issued in B3)
        end
    end

    // ---- B5: valid/payload alignment stage.  The valid/payload chain reaches
    //  B4 one cycle BEFORE h11 settles into the B4 registers (h11 lands from
    //  BRAM on the edge that also sets v_B4, so on the v_B4=1 cycle the B4
    //  corner regs still hold the PREVIOUS pixel).  We delay valid+payload by
    //  one cycle here; stage D then reads the four corners LIVE from the B4
    //  registers (which now hold THIS pixel's corners) gated by v_B5.  The
    //  corners are deliberately NOT re-registered here.
    logic signed [POS_W-1:0]  Px_B5, Py_B5, Pz_B5;
    logic signed [DIR_W-1:0]  Dx_B5, Dy_B5, Dz_B5;
    logic [1:0]               stat_B5;
    logic                     prev_B5;
    logic signed [H_W-1:0]    h_hit_B5;
    logic [IDX_W-1:0]         ix_hit_B5, iy_hit_B5;
    logic signed [POS_W-1:0]  Px_hit_B5, Py_hit_B5;
    logic [STEP_W-1:0]        step_count_B5;
    logic                     v_B5;
    logic [FRAC_W-1:0]        xf_B5, yf_B5;

    always_ff @(posedge clk) begin
        if (!rst_n) v_B5 <= 1'b0;
        else if (en) begin
            Px_B5<=Px_B4; Py_B5<=Py_B4; Pz_B5<=Pz_B4;
            Dx_B5<=Dx_B4; Dy_B5<=Dy_B4; Dz_B5<=Dz_B4;
            stat_B5<=stat_B4; prev_B5<=prev_B4; v_B5<=v_B4;
            h_hit_B5<=h_hit_B4; ix_hit_B5<=ix_hit_B4; iy_hit_B5<=iy_hit_B4;
            Px_hit_B5<=Px_hit_B4; Py_hit_B5<=Py_hit_B4; step_count_B5<=step_count_B4;
            xf_B5<=xf_B4; yf_B5<=yf_B4;
        end
    end


    // =================================================================
    //  PORT DRIVE: a step occupies the shared port across 4 consecutive
    //  cycles via its 4 sub-stages.  Exactly one of B0..B3 is mid-issue
    //  per cycle for a given pixel; the marcher's quarter-rate input gate
    //  guarantees no two pixels of the SAME step collide.
    //
    //  Issue priority (only one is active at a time for a given pixel, but
    //  we make the mux explicit and stable):
    //      B0 -> h00 (ix0,iy0)
    //      B1 -> h10 (ix1,iy0)
    //      B2 -> h01 (ix0,iy1)
    //      B3 -> h11 (ix1,iy1)
    // =================================================================
    always_comb begin
        // Default: no read.
        bram_addr = '0;
        bram_re   = 1'b0;
        if (issue_active_B0) begin
            bram_addr = {iy0_q,  ix0_q};   // h00
            bram_re   = 1'b1;
        end else if (issue_active_B1) begin
            bram_addr = {iy0_B1, ix1_B1};  // h10
            bram_re   = 1'b1;
        end else if (issue_active_B2) begin
            bram_addr = {iy1_B2, ix0_B2};  // h01
            bram_re   = 1'b1;
        end else if (issue_active_B3) begin
            bram_addr = {iy1_B3, ix1_B3};  // h11
            bram_re   = 1'b1;
        end
    end


    // =================================================================
    //  STAGE D: x-direction lerps (identical arithmetic to march_step3).
    // =================================================================
    logic signed [H_W:0]            diff_top_comb, diff_bot_comb;
    logic signed [H_W+FRAC_W:0]     prod_top_comb, prod_bot_comb;
    logic signed [H_W-1:0]          h_top_comb, h_bot_comb;

    always_comb begin
        diff_top_comb = $signed(h10_B4) - $signed(h00_B4);
        diff_bot_comb = $signed(h11_B4) - $signed(h01_B4);
        prod_top_comb = diff_top_comb * $signed({1'b0, xf_B4});
        prod_bot_comb = diff_bot_comb * $signed({1'b0, xf_B4});
        h_top_comb = $signed(h00_B4) + prod_top_comb[H_W+FRAC_W-1 -: H_W];
        h_bot_comb = $signed(h01_B4) + prod_bot_comb[H_W+FRAC_W-1 -: H_W];
    end

    logic signed [POS_W-1:0]  Px_D, Py_D, Pz_D;
    logic signed [DIR_W-1:0]  Dx_D, Dy_D, Dz_D;
    logic [1:0]               stat_D;
    logic                     prev_D;
    logic signed [H_W-1:0]    h_hit_D;
    logic [IDX_W-1:0]         ix_hit_D, iy_hit_D;
    logic signed [POS_W-1:0]  Px_hit_D, Py_hit_D;
    logic [STEP_W-1:0]        step_count_D;
    logic                     v_D;
    logic [FRAC_W-1:0]        yf_D;
    logic signed [H_W-1:0]    h_top_D, h_bot_D;

    always_ff @(posedge clk) begin
        if (!rst_n) v_D <= 1'b0;
        else if (en) begin
            // Payload + valid come from the B5 alignment stage so they are
            // co-incident with the B4 corner registers holding THIS pixel's
            // four corners.  The lerp (above) reads the corners + xf live from
            // the B4 registers on this same cycle.
            Px_D<=Px_B5; Py_D<=Py_B5; Pz_D<=Pz_B5;
            Dx_D<=Dx_B5; Dy_D<=Dy_B5; Dz_D<=Dz_B5;
            stat_D<=stat_B5; prev_D<=prev_B5; v_D<=v_B5;
            h_hit_D<=h_hit_B5; ix_hit_D<=ix_hit_B5; iy_hit_D<=iy_hit_B5;
            Px_hit_D<=Px_hit_B5; Py_hit_D<=Py_hit_B5; step_count_D<=step_count_B5;
            yf_D<=yf_B5;
            h_top_D <= h_top_comb;
            h_bot_D <= h_bot_comb;
        end
    end


    // =================================================================
    //  STAGE D2: y-lerp + hit test (identical to march_step3 stage D2).
    // =================================================================
    localparam int H_ALIGN_SHIFT = POS_F - H_F;

    logic signed [H_W:0]          diff_y_comb;
    logic signed [H_W+FRAC_W:0]   prod_y_comb;
    logic signed [H_W-1:0]        h_interp_comb;
    logic signed [POS_W-1:0]      h_aligned;
    logic                         below_D2;
    logic                         crossed_D2;

    always_comb begin
        diff_y_comb   = $signed(h_bot_D) - $signed(h_top_D);
        prod_y_comb   = diff_y_comb * $signed({1'b0, yf_D});
        h_interp_comb = $signed(h_top_D) + prod_y_comb[H_W+FRAC_W-1 -: H_W];

        h_aligned = $signed(h_interp_comb) <<< H_ALIGN_SHIFT;
        below_D2  = (Pz_D < h_aligned);
        crossed_D2 = (stat_D == ST_MARCHING)
                  && (below_D2 != prev_D)
                  && (step_count_D != '0);
    end

    logic signed [POS_W-1:0]  Px_D2, Py_D2, Pz_D2;
    logic signed [DIR_W-1:0]  Dx_D2, Dy_D2, Dz_D2;
    logic [1:0]               stat_D2;
    logic                     prev_D2;
    logic signed [H_W-1:0]    h_hit_D2;
    logic [IDX_W-1:0]         ix_hit_D2, iy_hit_D2;
    logic signed [POS_W-1:0]  Px_hit_D2, Py_hit_D2;
    logic [STEP_W-1:0]        step_count_D2;
    logic                     v_D2;

    always_ff @(posedge clk) begin
        if (!rst_n) v_D2 <= 1'b0;
        else if (en) begin
            Px_D2<=Px_D; Py_D2<=Py_D; Pz_D2<=Pz_D;
            Dx_D2<=Dx_D; Dy_D2<=Dy_D; Dz_D2<=Dz_D;
            step_count_D2<=step_count_D; v_D2<=v_D;

            if (stat_D == ST_MARCHING && crossed_D2) begin
                stat_D2   <= ST_HIT;
                h_hit_D2  <= h_interp_comb;
                ix_hit_D2 <= ix_hit_D;
                iy_hit_D2 <= iy_hit_D;
                Px_hit_D2 <= Px_hit_D;
                Py_hit_D2 <= Py_hit_D;
            end else begin
                stat_D2   <= stat_D;
                h_hit_D2  <= h_hit_D;
                ix_hit_D2 <= ix_hit_D;
                iy_hit_D2 <= iy_hit_D;
                Px_hit_D2 <= Px_hit_D;
                Py_hit_D2 <= Py_hit_D;
            end

            if (stat_D == ST_MARCHING)
                prev_D2 <= below_D2;
            else
                prev_D2 <= prev_D;
        end
    end


    // =================================================================
    //  STAGE E: output buffer (passthrough).
    // =================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) valid_out <= 1'b0;
        else if (en) begin
            Px_out<=Px_D2; Py_out<=Py_D2; Pz_out<=Pz_D2;
            Dx_out<=Dx_D2; Dy_out<=Dy_D2; Dz_out<=Dz_D2;
            status_out<=stat_D2; prev_below_out<=prev_D2;
            h_hit_out<=h_hit_D2; ix_hit_out<=ix_hit_D2; iy_hit_out<=iy_hit_D2;
            Px_hit_out<=Px_hit_D2; Py_hit_out<=Py_hit_D2; step_count_out<=step_count_D2;
            valid_out<=v_D2;
        end
    end

endmodule
