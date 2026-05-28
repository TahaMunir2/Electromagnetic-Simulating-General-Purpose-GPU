// ============================================================================
//  marcher.sv
//  ----------------------------------------------------------------------------
//  Top-level ray marcher.  Chains N_STEPS copies of `march_step` in series.
//
//  Each step advances the ray by `dt`, looks up the heightmap, and sets
//  status to HIT / OFF_GRID / MARCHING.  Once a ray's status is non-MARCHING,
//  later stages just pass it through unchanged ("frozen ray" pattern).
//
//  Inputs:
//      - ray origin O and direction D from ray_gen
//      - heightmap BRAM read ports (one per step)
//  Outputs:
//      - hit status, hit indices, hit height
//      - step_count as a lightweight distance hint for the shader
//
//  Pipeline:
//      - per step: 4 cycles
//      - total:    4 * N_STEPS cycles latency
//      - throughput: 1 pixel / cycle (when not stalled)
//
//  This is the "fully unrolled, fixed-N steps" pattern.  Modelled directly
//  on QUICK_MAFS's raymarcher (their N=6; ours can be much larger because
//  our SDF is just a BRAM lookup, not a 26-cycle Perlin pipeline).
// ============================================================================

module design1_marcher #(
    // ----- Geometry / format parameters (must match ray_gen, heightmap) -----
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

    parameter int PX_W       = 10,
    parameter int PY_W       = 10,

    // ----- Number of unrolled march steps -----
    // Each step costs 1 BRAM port + a tiny amount of logic.  Pick at
    // synthesis time.  Plot a histogram of step counts in your reference
    // model and set N_STEPS at the knee of the distribution.
    parameter int N_STEPS    = 16,
    parameter int STEP_W     = $clog2(N_STEPS + 1),

    // ----- Throughput / quality knobs (inert for Design 1) -----
    // FOLD = 1 -> 1 pixel/cycle (Design 1, 2 -> 1/2, 4 -> 1/4)
    // INTERP_IN_MARCHER = 0 -> nearest-neighbour SDF lookup inside marcher
    //                       (Design 1, 2).  1 -> bilinear (Design 3, 4).
    parameter int  FOLD              = 1,
    parameter bit  INTERP_IN_MARCHER = 1'b0,

    // ----- Step size dt -----
    parameter logic signed [POS_W-1:0] WORLD_HALF = (1 <<< POS_F),
    parameter logic signed [POS_W-1:0] DT         = (2 * WORLD_HALF) / GRID_N
)(
    input  logic                       clk,
    input  logic                       rst_n,
    input  logic                       en,           // pipeline enable (~stall)

    // ----- From ray_gen -----
    input  logic signed [DIR_W-1:0]    Dx_in,
    input  logic signed [DIR_W-1:0]    Dy_in,
    input  logic signed [DIR_W-1:0]    Dz_in,
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    // ----- Camera origin (constant per frame, broadcast) -----
    input  logic signed [POS_W-1:0]    Ox,
    input  logic signed [POS_W-1:0]    Oy,
    input  logic signed [POS_W-1:0]    Oz,

    // ----- Heightmap BRAM ports.  One read port per march step. -----
    output logic [IDX_W*2-1:0]         bram_addr [N_STEPS],
    output logic                       bram_re   [N_STEPS],
    input  logic signed [H_W-1:0]      bram_dout [N_STEPS],

    // ----- Pipeline data out (to normal stage) -----
    output logic [1:0]                 status_out,
    output logic [IDX_W-1:0]           ix_hit_out,
    output logic [IDX_W-1:0]           iy_hit_out,
    output logic signed [H_W-1:0]      h_hit_out,
    output logic signed [POS_W-1:0]    Px_hit_out,
    output logic signed [POS_W-1:0]    Py_hit_out,
    output logic [STEP_W-1:0]          step_count_out,
    output logic [PX_W-1:0]            px_out,
    output logic [PY_W-1:0]            py_out,
    output logic                       valid_out
);

    // -----------------------------------------------------------------
    //  Build N_STEPS+1 internal "wires" for the inter-stage data.
    //  step[0] is the input to step 0; step[N_STEPS] is the final output.
    // -----------------------------------------------------------------
    logic signed [POS_W-1:0]    Px_chain   [N_STEPS+1];
    logic signed [POS_W-1:0]    Py_chain   [N_STEPS+1];
    logic signed [POS_W-1:0]    Pz_chain   [N_STEPS+1];
    logic signed [DIR_W-1:0]    Dx_chain   [N_STEPS+1];
    logic signed [DIR_W-1:0]    Dy_chain   [N_STEPS+1];
    logic signed [DIR_W-1:0]    Dz_chain   [N_STEPS+1];
    logic [1:0]                 stat_chain [N_STEPS+1];
    logic                       prev_chain [N_STEPS+1];
    logic signed [H_W-1:0]      hH_chain   [N_STEPS+1];
    logic [IDX_W-1:0]           ixH_chain  [N_STEPS+1];
    logic [IDX_W-1:0]           iyH_chain  [N_STEPS+1];
    logic signed [POS_W-1:0]    PxH_chain  [N_STEPS+1];
    logic signed [POS_W-1:0]    PyH_chain  [N_STEPS+1];
    logic [STEP_W-1:0]          step_chain [N_STEPS+1];
    logic                       v_chain    [N_STEPS+1];

    // -----------------------------------------------------------------
    //  Initialise chain[0] from the inputs: ray starts at the camera.
    // -----------------------------------------------------------------
    assign Px_chain[0]   = Ox;
    assign Py_chain[0]   = Oy;
    assign Pz_chain[0]   = Oz;
    assign Dx_chain[0]   = Dx_in;
    assign Dy_chain[0]   = Dy_in;
    assign Dz_chain[0]   = Dz_in;
    assign stat_chain[0] = 2'b00;             // ST_MARCHING
    assign prev_chain[0] = 1'b0;              // doesn't matter (step 0 forces compare result)
    assign hH_chain[0]   = '0;
    assign ixH_chain[0]  = '0;
    assign iyH_chain[0]  = '0;
    assign PxH_chain[0]  = '0;
    assign PyH_chain[0]  = '0;
    assign step_chain[0] = '0;
    assign v_chain[0]    = valid_in;

    // -----------------------------------------------------------------
    //  Instantiate N_STEPS march_step blocks in series.
    // -----------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < N_STEPS; gi++) begin : g_march
            design1_march_step #(
                .POS_W      (POS_W),
                .POS_I      (POS_I),
                .POS_F      (POS_F),
                .DIR_W      (DIR_W),
                .DIR_I      (DIR_I),
                .DIR_F      (DIR_F),
                .GRID_N     (GRID_N),
                .IDX_W      (IDX_W),
                .H_W        (H_W),
                .H_I        (H_I),
                .H_F        (H_F),
                .STEP_W     (STEP_W),
                .WORLD_HALF (WORLD_HALF),
                .DT         (DT)
            ) u_step (
                .clk            (clk),
                .rst_n          (rst_n),
                .en             (en),

                .Px_in          (Px_chain[gi]),
                .Py_in          (Py_chain[gi]),
                .Pz_in          (Pz_chain[gi]),
                .Dx             (Dx_chain[gi]),
                .Dy             (Dy_chain[gi]),
                .Dz             (Dz_chain[gi]),
                .status_in      (stat_chain[gi]),
                .prev_below_in  (prev_chain[gi]),
                .h_hit_in       (hH_chain[gi]),
                .ix_hit_in      (ixH_chain[gi]),
                .iy_hit_in      (iyH_chain[gi]),
                .Px_hit_in      (PxH_chain[gi]),
                .Py_hit_in      (PyH_chain[gi]),
                .step_count_in  (step_chain[gi]),
                .valid_in       (v_chain[gi]),

                .bram_addr      (bram_addr[gi]),
                .bram_re        (bram_re[gi]),
                .bram_dout      (bram_dout[gi]),

                .Px_out         (Px_chain[gi+1]),
                .Py_out         (Py_chain[gi+1]),
                .Pz_out         (Pz_chain[gi+1]),
                .Dx_out         (Dx_chain[gi+1]),
                .Dy_out         (Dy_chain[gi+1]),
                .Dz_out         (Dz_chain[gi+1]),
                .status_out     (stat_chain[gi+1]),
                .prev_below_out (prev_chain[gi+1]),
                .h_hit_out      (hH_chain[gi+1]),
                .ix_hit_out     (ixH_chain[gi+1]),
                .iy_hit_out     (iyH_chain[gi+1]),
                .Px_hit_out     (PxH_chain[gi+1]),
                .Py_hit_out     (PyH_chain[gi+1]),
                .step_count_out (step_chain[gi+1]),
                .valid_out      (v_chain[gi+1])
            );
        end
    endgenerate

    // -----------------------------------------------------------------
    //  Tap the end of the chain for the marcher's outputs.
    // -----------------------------------------------------------------
    assign status_out = stat_chain[N_STEPS];
    assign ix_hit_out = ixH_chain[N_STEPS];
    assign iy_hit_out = iyH_chain[N_STEPS];
    assign h_hit_out  = hH_chain[N_STEPS];
    assign Px_hit_out = PxH_chain[N_STEPS];
    assign Py_hit_out = PyH_chain[N_STEPS];
    assign step_count_out = step_chain[N_STEPS];
    assign valid_out  = v_chain[N_STEPS];

    // -----------------------------------------------------------------
    //  Pixel-coordinate delay line.  The marcher latency is
    //  4 * N_STEPS cycles, so we need a delay line of that length
    //  to keep (px, py) aligned with the data emerging from the chain.
    // -----------------------------------------------------------------
    localparam int LATENCY = 4 * N_STEPS;

    logic [PX_W-1:0]  px_pipe [LATENCY-1:0];
    logic [PY_W-1:0]  py_pipe [LATENCY-1:0];

    always_ff @(posedge clk) begin
        if (en) begin
            px_pipe[0] <= px_in;
            py_pipe[0] <= py_in;
            for (int i = 1; i < LATENCY; i++) begin
                px_pipe[i] <= px_pipe[i-1];
                py_pipe[i] <= py_pipe[i-1];
            end
        end
    end

    assign px_out = px_pipe[LATENCY-1];
    assign py_out = py_pipe[LATENCY-1];

endmodule
