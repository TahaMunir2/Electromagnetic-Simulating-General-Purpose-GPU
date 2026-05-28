// ============================================================================
//  normal4.sv  (Design 4: simplified normal, 2 BRAM ports / 1 copy)
//  ----------------------------------------------------------------------------
//  In Designs 3 and 4 the marcher already produces a bilinearly-interpolated surface
//  height (h_hit is smooth), so this module no longer needs to interpolate the
//  height for shading.  It ONLY needs the surface slope to build the normal.
//
//  It computes a forward-difference normal from 3 corners:
//
//      Nx = -(h(ix+1, iy)   - h(ix, iy))
//      Ny = -(h(ix,   iy+1) - h(ix, iy))
//      Nz = +1.0
//
//  Reads are spread over 2 cycles on 2 ports (1 heightmap copy = 2 tiles):
//
//                       phase = 0              phase = 1
//          port 0       (ix0, iy0) = h00       (ix1, iy0) = h10
//          port 1       (ix0, iy1) = h01       (unused / re-read h00)
//
//  Only h00, h10, h01 are needed (h11 is not used for forward differences).
//  We still issue port1 on phase1 (harmless re-read) to keep the address
//  muxing trivial and symmetric with normal2/3.
//
//  Pipeline (5 stages, latency = 5):
//      Stage 1: latch inputs, compute ix0/ix1/iy0/iy1; drive ports.
//      Stage 2: first reads in flight.
//      Stage 3: capture first column (h00,h01 if phase0 / h10 if phase1).
//      Stage 4: capture second column -> h00,h10,h01 all held.
//      Stage 5: compute Nx,Ny (forward diff + saturate); drive outputs.
//
//  h_hit (smooth, from the marcher) is passed straight through unchanged and
//  re-exported as h_interp_out so the shader interface is unchanged vs Design 2.
//
//  Throughput: 1 pixel / 2 cycles.   BRAM: 1 copy (2 tiles).
// ============================================================================

module normal4 #(
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
    parameter int STEP_W     = 5
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,

    input  logic [1:0]                 status_in,
    input  logic [IDX_W-1:0]           ix_in,
    input  logic [IDX_W-1:0]           iy_in,
    input  logic signed [H_W-1:0]      h_hit_in,      // already smooth from marcher3
    input  logic [STEP_W-1:0]          step_count_in,
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    // 2-port BRAM interface (1 heightmap copy).
    output logic [IDX_W*2-1:0]         bram_addr [2],
    output logic                       bram_re   [2],
    input  logic signed [H_W-1:0]      bram_dout [2],

    output logic [1:0]                 status_out,
    output logic [IDX_W-1:0]           ix_out,
    output logic [IDX_W-1:0]           iy_out,
    output logic signed [H_W-1:0]      h_hit_out,
    output logic signed [H_W-1:0]      h_interp_out,  // = h_hit (smooth) passthrough
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

    // Internal phase counter (toggles every cycle when en=1).
    logic phase_in;
    always_ff @(posedge clk) begin
        if (!rst_n) phase_in <= 1'b0;
        else if (en) phase_in <= ~phase_in;
    end

    // =================================================================
    //  STAGE 1: latch inputs, compute corner indices.
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
        end
    end

    // BRAM address mux: phase=0 -> x=ix0 (h00,h01); phase=1 -> x=ix1 (h10).
    // port1 on phase1 re-reads h00's row (harmless; result discarded).
    assign bram_addr[0] = (phase_in == 1'b0) ? {iy0_s1, ix0_s1}    // h00
                                             : {iy0_s1, ix1_s1};   // h10
    assign bram_addr[1] = (phase_in == 1'b0) ? {iy1_s1, ix0_s1}    // h01
                                             : {iy0_s1, ix0_s1};   // (unused)
    assign bram_re[0] = v_s1;
    assign bram_re[1] = v_s1;


    // =================================================================
    //  STAGE 2: first reads in flight.
    // =================================================================
    logic [1:0]                 stat_s2;
    logic [IDX_W-1:0]           ix_s2, iy_s2;
    logic signed [H_W-1:0]      hhit_s2;
    logic [STEP_W-1:0]          step_s2;
    logic [PX_W-1:0]            px_s2;
    logic [PY_W-1:0]            py_s2;
    logic                       v_s2;
    logic                       phase_s1_reg;

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
            phase_s1_reg <= phase_in;
        end
    end


    // =================================================================
    //  STAGE 3: first BRAM data arrives.
    //    phase_s1_reg==0 -> ports delivered h00 (port0), h01 (port1)
    //    phase_s1_reg==1 -> port0 delivered h10
    // =================================================================
    logic [1:0]                 stat_s3;
    logic [IDX_W-1:0]           ix_s3, iy_s3;
    logic signed [H_W-1:0]      hhit_s3;
    logic [STEP_W-1:0]          step_s3;
    logic [PX_W-1:0]            px_s3;
    logic [PY_W-1:0]            py_s3;
    logic                       v_s3;
    logic                       phase_s2_reg;

    logic signed [H_W-1:0]      h00_s3, h10_s3, h01_s3;

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
            phase_s2_reg <= phase_s1_reg;

            if (phase_s1_reg == 1'b0) begin
                h00_s3 <= bram_dout[0];
                h01_s3 <= bram_dout[1];
            end else begin
                h10_s3 <= bram_dout[0];
            end
        end
    end


    // =================================================================
    //  STAGE 4: second BRAM data arrives -> capture the other half.
    // =================================================================
    logic [1:0]                 stat_s4;
    logic [IDX_W-1:0]           ix_s4, iy_s4;
    logic signed [H_W-1:0]      hhit_s4;
    logic [STEP_W-1:0]          step_s4;
    logic [PX_W-1:0]            px_s4;
    logic [PY_W-1:0]            py_s4;
    logic                       v_s4;

    logic signed [H_W-1:0]      h00_s4, h10_s4, h01_s4;

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

            // Carry already-captured corners
            h00_s4 <= h00_s3;
            h10_s4 <= h10_s3;
            h01_s4 <= h01_s3;

            // Second read was the opposite phase to phase_s2_reg.
            if (phase_s2_reg == 1'b0) begin
                // first half was h00/h01 -> now h10 (port0)
                h10_s4 <= bram_dout[0];
            end else begin
                // first half was h10 -> now h00 (port0), h01 (port1)
                h00_s4 <= bram_dout[0];
                h01_s4 <= bram_dout[1];
            end
        end
    end


    // =================================================================
    //  STAGE 5: forward-difference normal + saturate.  Drive outputs.
    //  Identical saturation logic to normal2 stage 5.
    // =================================================================
    logic signed [H_W:0]           dx_h_comb, dy_h_comb;
    logic signed [H_W+DIR_W-1:0]   dx_shifted, dy_shifted;
    logic signed [DIR_W-1:0]       nx_calc, ny_calc;

    localparam logic signed [H_W+DIR_W-1:0] SAT_POS =
        $signed({1'b0, {(DIR_W-1){1'b1}}});
    localparam logic signed [H_W+DIR_W-1:0] SAT_NEG =
        $signed({{(H_W+1){1'b1}}, {(DIR_W-1){1'b0}}});

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
    end

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            status_out     <= stat_s4;
            ix_out         <= ix_s4;
            iy_out         <= iy_s4;
            h_hit_out      <= hhit_s4;
            h_interp_out   <= hhit_s4;     // smooth height passthrough
            step_count_out <= step_s4;
            px_out         <= px_s4;
            py_out         <= py_s4;
            valid_out      <= v_s4;

            if (stat_s4 == ST_HIT) begin
                Nx_out <= nx_calc;
                Ny_out <= ny_calc;
                Nz_out <= NZ_CONST;
            end else begin
                Nx_out <= '0;
                Ny_out <= '0;
                Nz_out <= NZ_CONST;
            end
        end
    end

endmodule
