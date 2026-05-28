// ============================================================================
//  march_step.sv  (rewritten — clean, readable, no helper functions)
//  ----------------------------------------------------------------------------
//  One iteration of the heightmap ray-march.
//
//  Each step:
//      - advances the ray:   P_new = P_old + dt * D
//      - looks up the terrain height at the new (x, y)
//      - decides one of:
//            MARCHING  -> ray is still going
//            HIT       -> ray crossed the terrain surface
//            OFF_GRID  -> ray walked off the heightmap edge
//
//  Frozen-ray pattern: once a ray is HIT or OFF_GRID, all downstream
//  march_step instances pass it through unchanged.
//
//  Pipeline (4 stages, latency = 4 cycles):
//      Stage A:  advance position P + dt*D  (if MARCHING)
//      Stage B:  compute grid indices, issue BRAM read, off-grid check
//      Stage C:  receive BRAM data (1-cycle BRAM read latency)
//      Stage D:  compare Pz vs h, detect surface crossing, update status
//
//  Conventions:
//      x, y  : horizontal heightmap axes
//      z     : height (vertical)
//      Position is Q(POS_I).(POS_F) signed.
//      Direction is Q(DIR_I).(DIR_F) signed.
//      Heightmap values are Q(H_I).(H_F) signed.
//
//  Bug 2 fix: step 0 never declares a HIT, it only establishes prev_below.
// ============================================================================

module march_step #(
    // ----- Position fixed-point format -----
    parameter int POS_W      = 16,
    parameter int POS_I      = 4,
    parameter int POS_F      = POS_W - 1 - POS_I,    // = 11

    // ----- Direction fixed-point format -----
    parameter int DIR_W      = 16,
    parameter int DIR_I      = 2,
    parameter int DIR_F      = DIR_W - 1 - DIR_I,    // = 13

    // ----- Heightmap geometry -----
    parameter int GRID_N     = 256,
    parameter int IDX_W      = $clog2(GRID_N),

    // ----- Heightmap value format -----
    parameter int H_W        = 16,
    parameter int H_I        = 4,
    parameter int H_F        = H_W - 1 - H_I,        // = 11

    // ----- Hit-distance proxy -----
    parameter int STEP_W     = 5,

    // ----- World extents -----
    // World x in [-WORLD_HALF, +WORLD_HALF].  Heightmap covers the same range.
    parameter logic signed [POS_W-1:0] WORLD_HALF = (1 <<< POS_F),

    // ----- Step size -----
    // dt = 2*WORLD_HALF / GRID_N  =>  one heightmap cell per step.
    parameter logic signed [POS_W-1:0] DT = (2 * WORLD_HALF) / GRID_N
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,           // ~stall

    // ----- Pipeline data in -----
    input  logic signed [POS_W-1:0]    Px_in,
    input  logic signed [POS_W-1:0]    Py_in,
    input  logic signed [POS_W-1:0]    Pz_in,
    input  logic signed [DIR_W-1:0]    Dx,
    input  logic signed [DIR_W-1:0]    Dy,
    input  logic signed [DIR_W-1:0]    Dz,
    input  logic [1:0]                 status_in,    // 00=MARCHING, 01=HIT, 10=OFF
    input  logic                       prev_below_in,
    input  logic signed [H_W-1:0]      h_hit_in,
    input  logic [IDX_W-1:0]           ix_hit_in,
    input  logic [IDX_W-1:0]           iy_hit_in,
    input  logic signed [POS_W-1:0]    Px_hit_in,     // world x at the moment of HIT
    input  logic signed [POS_W-1:0]    Py_hit_in,     // world y at the moment of HIT
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic                       valid_in,

    // ----- BRAM read port (issued in stage B, returned in stage C) -----
    output logic [IDX_W*2-1:0]         bram_addr,    // {iy, ix}
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
    //  STAGE A: advance position.  P_new = P_in + dt * D  (if MARCHING).
    //  Frozen rays pass through unchanged.
    //
    //  dt is a constant, so dt*D collapses to a small shift/add network
    //  (no real multiplier).
    //
    //  dt is Q4.11.  D is Q2.13.  Product is Q6.24.  We shift right by
    //  DIR_F = 13 to land back in Q4.11 (position format).
    // =================================================================

    // Full-width raw products for one cycle (32-bit each)
    logic signed [POS_W + DIR_W - 1 : 0]  raw_dtDx, raw_dtDy, raw_dtDz;
    // After alignment to POS_F: 16-bit increment to add to P
    logic signed [POS_W - 1 : 0]          inc_Px, inc_Py, inc_Pz;

    always_comb begin
        raw_dtDx = $signed(DT) * Dx;
        raw_dtDy = $signed(DT) * Dy;
        raw_dtDz = $signed(DT) * Dz;
        // Take the slice that puts the result in POS_F format.
        inc_Px = raw_dtDx[DIR_F + POS_W - 1 -: POS_W];
        inc_Py = raw_dtDy[DIR_F + POS_W - 1 -: POS_W];
        inc_Pz = raw_dtDz[DIR_F + POS_W - 1 -: POS_W];
    end

    // Stage A registers
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
                // Frozen ray — position doesn't change
                Px_A <= Px_in;
                Py_A <= Py_in;
                Pz_A <= Pz_in;
            end

            // Pass-through payload
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
    //  STAGE B: convert world position -> grid index, check off-grid,
    //           issue BRAM read.
    //
    //  Grid index from world coordinate:
    //      cell = (P + WORLD_HALF) * GRID_N / (2*WORLD_HALF)
    //
    //  With WORLD_HALF = 2^POS_F = 1.0 and GRID_N a power of 2, this
    //  reduces to:  cell = (P + WORLD_HALF) >> (POS_F + 1 - log2(GRID_N))
    //  i.e. just a bit slice.
    // =================================================================

    // Local constants (cleaner than re-deriving each time)
    localparam int W2G_SHIFT = POS_F + 1 - $clog2(GRID_N);

    // Raw grid coordinates (POS_W+1 bits signed — extra bit catches overflow)
    logic signed [POS_W:0]  ix_raw, iy_raw;
    logic                   offgrid_B;
    logic [IDX_W-1:0]       ix_B, iy_B;

    always_comb begin
        ix_raw = $signed({Px_A[POS_W-1], Px_A}) + $signed({1'b0, WORLD_HALF});
        iy_raw = $signed({Py_A[POS_W-1], Py_A}) + $signed({1'b0, WORLD_HALF});
        ix_raw = ix_raw >>> W2G_SHIFT;
        iy_raw = iy_raw >>> W2G_SHIFT;
        offgrid_B = (ix_raw < 0) || (ix_raw >= GRID_N) ||
                    (iy_raw < 0) || (iy_raw >= GRID_N);
    end

    assign ix_B = ix_raw[IDX_W-1:0];
    assign iy_B = iy_raw[IDX_W-1:0];

    // BRAM port (combinational from stage A)
    assign bram_addr = {iy_B, ix_B};
    assign bram_re   = v_A && (stat_A == ST_MARCHING) && !offgrid_B;

    // Stage B registers
    logic signed [POS_W-1:0]  Px_B, Py_B, Pz_B;
    logic signed [DIR_W-1:0]  Dx_B, Dy_B, Dz_B;
    logic [1:0]               stat_B;
    logic                     prev_B;
    logic signed [H_W-1:0]    h_hit_B;
    logic [IDX_W-1:0]         ix_hit_B, iy_hit_B;
    logic signed [POS_W-1:0]  Px_hit_B, Py_hit_B;
    logic [STEP_W-1:0]        step_count_B;
    logic                     v_B;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_B <= 1'b0;
        end else if (en) begin
            // Pass through everything
            Px_B   <= Px_A;
            Py_B   <= Py_A;
            Pz_B   <= Pz_A;
            Dx_B   <= Dx_A;
            Dy_B   <= Dy_A;
            Dz_B   <= Dz_A;
            prev_B <= prev_A;
            v_B    <= v_A;

            // Status update for this stage:
            //   Frozen ray -> hold status.
            //   Marching ray that just walked off -> OFF_GRID.
            //   Otherwise still MARCHING.
            if (stat_A != ST_MARCHING)
                stat_B <= stat_A;
            else if (offgrid_B)
                stat_B <= ST_OFFGRID;
            else
                stat_B <= ST_MARCHING;

            // Capture the indices we just queried (used by stage D if HIT)
            if (stat_A == ST_MARCHING && !offgrid_B) begin
                ix_hit_B <= ix_B;
                iy_hit_B <= iy_B;
                Px_hit_B <= Px_A;
                Py_hit_B <= Py_A;
            end else begin
                ix_hit_B <= ix_hit_A;
                iy_hit_B <= iy_hit_A;
                Px_hit_B <= Px_hit_A;
                Py_hit_B <= Py_hit_A;
            end

            // Carry the existing h_hit (will be overwritten in stage D if HIT)
            h_hit_B <= h_hit_A;

            // Step counter increments while marching
            if (stat_A == ST_MARCHING)
                step_count_B <= step_count_A + 1'b1;
            else
                step_count_B <= step_count_A;
        end
    end


    // =================================================================
    //  STAGE C: BRAM read latency.  Capture bram_dout, pass everything else.
    // =================================================================

    logic signed [POS_W-1:0]  Px_C, Py_C, Pz_C;
    logic signed [DIR_W-1:0]  Dx_C, Dy_C, Dz_C;
    logic [1:0]               stat_C;
    logic                     prev_C;
    logic signed [H_W-1:0]    h_hit_C;
    logic [IDX_W-1:0]         ix_hit_C, iy_hit_C;
    logic signed [POS_W-1:0]  Px_hit_C, Py_hit_C;
    logic [STEP_W-1:0]        step_count_C;
    logic                     v_C;
    logic signed [H_W-1:0]    h_bram_C;          // BRAM data arrives here

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_C <= 1'b0;
        end else if (en) begin
            Px_C         <= Px_B;
            Py_C         <= Py_B;
            Pz_C         <= Pz_B;
            Dx_C         <= Dx_B;
            Dy_C         <= Dy_B;
            Dz_C         <= Dz_B;
            stat_C       <= stat_B;
            prev_C       <= prev_B;
            h_hit_C      <= h_hit_B;
            ix_hit_C     <= ix_hit_B;
            iy_hit_C     <= iy_hit_B;
            Px_hit_C     <= Px_hit_B;
            Py_hit_C     <= Py_hit_B;
            step_count_C <= step_count_B;
            v_C          <= v_B;
            h_bram_C     <= bram_dout;          // 1-cycle BRAM read result
        end
    end


    // =================================================================
    //  STAGE D: compare Pz vs h.  Detect surface crossing.  Update status.
    //
    //  Pz is Q(POS_I).(POS_F).  h is Q(H_I).(H_F).  Align h to Pz's format.
    //  With defaults both are Q4.11, so no shift needed (H_ALIGN_SHIFT = 0).
    //
    //  Surface crossing: below flipped between this step and the previous.
    //  BUG 2 FIX: a HIT can only be declared at step >= 1.  Step 0 is just
    //  there to establish prev_below.
    // =================================================================

    localparam int H_ALIGN_SHIFT = POS_F - H_F;

    logic signed [POS_W-1:0]  h_aligned;
    logic                     below_D;
    logic                     crossed_D;

    always_comb begin
        h_aligned = $signed(h_bram_C) <<< H_ALIGN_SHIFT;
        below_D   = (Pz_C < h_aligned);
        // A HIT requires:
        //   - the ray is still actively marching
        //   - "below" flipped versus previous step
        //   - we're not at step 0 (need at least one prior step to compare)
        crossed_D = (stat_C == ST_MARCHING)
                 && (below_D != prev_C)
                 && (step_count_C != '0);   // Bug 2 fix: step 0 cannot HIT
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            // Pass-through position and direction
            Px_out         <= Px_C;
            Py_out         <= Py_C;
            Pz_out         <= Pz_C;
            Dx_out         <= Dx_C;
            Dy_out         <= Dy_C;
            Dz_out         <= Dz_C;
            step_count_out <= step_count_C;
            valid_out      <= v_C;

            // Status transition: only MARCHING -> HIT here.
            // Anything already non-MARCHING stays as is.
            if (stat_C == ST_MARCHING && crossed_D) begin
                status_out <= ST_HIT;
                h_hit_out  <= h_bram_C;
                ix_hit_out <= ix_hit_C;
                iy_hit_out <= iy_hit_C;
                Px_hit_out <= Px_hit_C;
                Py_hit_out <= Py_hit_C;
            end else begin
                status_out <= stat_C;
                h_hit_out  <= h_hit_C;
                ix_hit_out <= ix_hit_C;
                iy_hit_out <= iy_hit_C;
                Px_hit_out <= Px_hit_C;
                Py_hit_out <= Py_hit_C;
            end

            // prev_below for the next step:
            //   If still marching, update with the current step's below.
            //   If frozen, hold whatever it was.
            if (stat_C == ST_MARCHING)
                prev_below_out <= below_D;
            else
                prev_below_out <= prev_C;
        end
    end

endmodule