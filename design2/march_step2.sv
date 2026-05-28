// ============================================================================
//  march_step2.sv  (Design 2: half-rate, BRAM-port sharing variant)
//  ----------------------------------------------------------------------------
//  Same algorithm as march_step.  Two changes:
//
//   1. New parameter MY_PHASE (0 or 1).  Indicates whether this instance
//      is the "even" or "odd" partner of a pair sharing one BRAM port.
//
//   2. New input phase_in (1 bit).  Toggles every cycle.  The marcher2
//      drives this from its own phase counter.
//
//      - bram_re is gated on phase_in == MY_PHASE (only assert read when
//        we own the shared port this cycle).
//      - Stage C only captures BRAM data on the cycle when our data is
//        valid on bram_dout.  Because BRAM has 1-cycle read latency, that
//        is the cycle AFTER our address-issue cycle, i.e. when
//        phase_in != MY_PHASE.
//
//  All other pipeline stages still tick every cycle.  Half the slots
//  carry "bubble" pixels (valid_in = 0); their compute is harmless.
//
//  5-stage internal pipeline (latency = 5 cycles, +1 vs Design 1).
//  The extra stage (E) is a phase-alignment buffer that makes adjacent
//  steps' read phases differ by 1, enabling BRAM-port sharing.
//
//  Throughput: 1 real pixel / 2 cycles when chained in marcher2.
//
//  Note: marcher2 gates valid_in to step 0 on phase == 0 so that a real
//  pixel only enters when MY_PHASE==0 instances will read it.
// ============================================================================

module march_step2 #(
    // ----- Pair-phase parameter (0 = even partner, 1 = odd partner) -----
    parameter bit MY_PHASE   = 1'b0,
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
    input  logic                       phase_in,     // toggles every cycle (from marcher2)

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

    // BRAM port (combinational from stage A).  In Design 2 we only assert
    // bram_re on the cycles when we own the shared port (phase_in == MY_PHASE).
    // The address line is still driven every cycle (the marcher mux picks
    // which step's address goes onto the port).
    assign bram_addr = {iy_B, ix_B};
    assign bram_re   = v_A && (stat_A == ST_MARCHING) && !offgrid_B
                       && (phase_in == MY_PHASE);

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
    //
    //  In Design 2 the shared bram_dout line carries OUR step's data only
    //  on the cycle AFTER our address-issue cycle (= phase_in != MY_PHASE),
    //  so h_bram_C latches only on that phase.  All other registers tick
    //  every cycle as in Design 1.
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

            // BRAM data is for THIS step on the cycle when phase_in != MY_PHASE
            // (i.e. one cycle after our address-issue cycle).
            if (phase_in != MY_PHASE)
                h_bram_C <= bram_dout;
            // else: hold previous value (keeps our captured data alive
            //       until stage D uses it on the next cycle).
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

    // Stage D registers (intermediate — passed to stage E next cycle)
    logic signed [POS_W-1:0]  Px_D, Py_D, Pz_D;
    logic signed [DIR_W-1:0]  Dx_D, Dy_D, Dz_D;
    logic [1:0]               stat_D;
    logic                     prev_D;
    logic signed [H_W-1:0]    h_hit_D;
    logic [IDX_W-1:0]         ix_hit_D, iy_hit_D;
    logic signed [POS_W-1:0]  Px_hit_D, Py_hit_D;
    logic [STEP_W-1:0]        step_count_D;
    logic                     v_D;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            v_D <= 1'b0;
        end else if (en) begin
            Px_D         <= Px_C;
            Py_D         <= Py_C;
            Pz_D         <= Pz_C;
            Dx_D         <= Dx_C;
            Dy_D         <= Dy_C;
            Dz_D         <= Dz_C;
            step_count_D <= step_count_C;
            v_D          <= v_C;

            if (stat_C == ST_MARCHING && crossed_D) begin
                stat_D   <= ST_HIT;
                h_hit_D  <= h_bram_C;
                ix_hit_D <= ix_hit_C;
                iy_hit_D <= iy_hit_C;
                Px_hit_D <= Px_hit_C;
                Py_hit_D <= Py_hit_C;
            end else begin
                stat_D   <= stat_C;
                h_hit_D  <= h_hit_C;
                ix_hit_D <= ix_hit_C;
                iy_hit_D <= iy_hit_C;
                Px_hit_D <= Px_hit_C;
                Py_hit_D <= Py_hit_C;
            end

            if (stat_C == ST_MARCHING)
                prev_D <= below_D;
            else
                prev_D <= prev_C;
        end
    end


    // =================================================================
    //  STAGE E: phase-alignment buffer.
    //
    //  Reason: each step's stage A is 5 cycles downstream of the previous
    //  step's stage A (was 4 in Design 1).  5 is odd, so adjacent steps'
    //  read phases differ by 1 — enabling BRAM-port sharing.
    //
    //  Pure passthrough.  No logic.
    // =================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            Px_out         <= Px_D;
            Py_out         <= Py_D;
            Pz_out         <= Pz_D;
            Dx_out         <= Dx_D;
            Dy_out         <= Dy_D;
            Dz_out         <= Dz_D;
            status_out     <= stat_D;
            prev_below_out <= prev_D;
            h_hit_out      <= h_hit_D;
            ix_hit_out     <= ix_hit_D;
            iy_hit_out     <= iy_hit_D;
            Px_hit_out     <= Px_hit_D;
            Py_hit_out     <= Py_hit_D;
            step_count_out <= step_count_D;
            valid_out      <= v_D;
        end
    end

endmodule