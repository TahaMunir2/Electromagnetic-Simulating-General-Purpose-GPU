// ============================================================================
//  marcher2.sv  (Design 2: half-rate, BRAM-port sharing)
//  ----------------------------------------------------------------------------
//  Chains N_STEPS copies of `march_step2` in series.  Adjacent steps pair
//  up to SHARE one BRAM port:
//
//      pair k = (step 2k, step 2k+1)  -->  shared_bram[k]
//
//  A 1-bit `phase` counter toggles every cycle.  Each step has 5 internal
//  pipeline stages; consecutive steps' read phases naturally alternate.
//
//  Step k's MY_PHASE is set so that its bram_re lines up with the cycle
//  on which the mux selects its address.  Empirically (see timing probe):
//      MY_PHASE for step k = (k+1) & 1   --> even k -> 1, odd k -> 0
//
//  A new pixel enters step 0 every 2 cycles (valid_in gated on phase=1).
//
//  Outputs:
//      - hit status, hit indices, hit height, Px_hit, Py_hit
//      - step_count
//      - N_STEPS/2 shared BRAM port outputs (8 instead of 16 for N_STEPS=16)
//
//  Pipeline latency: 5 * N_STEPS cycles (was 4 * N_STEPS).
//  Throughput: 1 pixel / 2 cycles.
// ============================================================================

module marcher2 #(
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

    // ----- Number of unrolled march steps (must be EVEN for pairing) -----
    parameter int N_STEPS    = 16,
    parameter int STEP_W     = $clog2(N_STEPS + 1),

    // ----- Inert knobs (matched against marcher's interface) -----
    parameter int  FOLD              = 2,
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

    // ----- Heightmap BRAM ports.  N_STEPS/2 SHARED read ports. -----
    // Each port serves 2 march_step2 instances on alternating cycles.
    output logic [IDX_W*2-1:0]         bram_addr [N_STEPS/2],
    output logic                       bram_re   [N_STEPS/2],
    input  logic signed [H_W-1:0]      bram_dout [N_STEPS/2],

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
    //  Phase counter — toggles every cycle.  Drives address muxing and
    //  per-step phase_in.
    // -----------------------------------------------------------------
    logic phase;
    always_ff @(posedge clk) begin
        if (!rst_n)
            phase <= 1'b0;
        else if (en)
            phase <= ~phase;
    end

    // -----------------------------------------------------------------
    //  Initialise chain[0] from the inputs: ray starts at the camera.
    //  valid_in only enters the chain on phase=1 (half-rate input).
    // -----------------------------------------------------------------
    assign Px_chain[0]   = Ox;
    assign Py_chain[0]   = Oy;
    assign Pz_chain[0]   = Oz;
    assign Dx_chain[0]   = Dx_in;
    assign Dy_chain[0]   = Dy_in;
    assign Dz_chain[0]   = Dz_in;
    assign stat_chain[0] = 2'b00;             // ST_MARCHING
    assign prev_chain[0] = 1'b0;
    assign hH_chain[0]   = '0;
    assign ixH_chain[0]  = '0;
    assign iyH_chain[0]  = '0;
    assign PxH_chain[0]  = '0;
    assign PyH_chain[0]  = '0;
    assign step_chain[0] = '0;
    assign v_chain[0]    = valid_in && (phase == 1'b0);

    // -----------------------------------------------------------------
    //  Per-step BRAM signals.  Each march_step2 drives its own bram_addr
    //  and bram_re every cycle.  The marcher2 muxes pairs onto the
    //  shared output ports based on phase.
    // -----------------------------------------------------------------
    logic [IDX_W*2-1:0]         step_addr [N_STEPS];
    logic                       step_re   [N_STEPS];

    // -----------------------------------------------------------------
    //  Instantiate N_STEPS march_step2 blocks in series.
    //  MY_PHASE alternates: even index -> 1, odd index -> 0.
    //  (Determined empirically — see tb_timing in dev notes.)
    //  In each pair (2k, 2k+1) the two steps thus have opposite phases.
    // -----------------------------------------------------------------
    genvar gi;
    generate
        for (gi = 0; gi < N_STEPS; gi++) begin : g_march
            localparam bit STEP_PHASE = ((gi & 1) == 0) ? 1'b1 : 1'b0;
            // Each step gets bram_dout from its pair's shared port.
            // Pair index = gi/2.
            wire signed [H_W-1:0] step_dout = bram_dout[gi >> 1];

            march_step2 #(
                .MY_PHASE   (STEP_PHASE),
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
                .phase_in       (phase),

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

                .bram_addr      (step_addr[gi]),
                .bram_re        (step_re[gi]),
                .bram_dout      (step_dout),

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
    //  Address mux per pair: even step (MY_PHASE=1) drives the shared
    //  port on phase=1; odd step (MY_PHASE=0) drives on phase=0.
    //  bram_re is the OR of the two partners' re signals (only one can
    //  be asserted on any given cycle by construction).
    // -----------------------------------------------------------------
    generate
        for (gi = 0; gi < N_STEPS/2; gi++) begin : g_pair
            // even = step 2k (MY_PHASE=1), odd = step 2k+1 (MY_PHASE=0)
            assign bram_addr[gi] = (phase == 1'b1) ? step_addr[2*gi]
                                                   : step_addr[2*gi+1];
            assign bram_re[gi]   = (phase == 1'b1) ? step_re[2*gi]
                                                   : step_re[2*gi+1];
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
    //  Pixel-coordinate delay line.  Each step has 5 internal stages in
    //  Design 2 (vs 4 in Design 1), so latency = 5 * N_STEPS.
    // -----------------------------------------------------------------
    localparam int LATENCY = 5 * N_STEPS;

    logic [PX_W-1:0]  px_pipe [LATENCY];
    logic [PY_W-1:0]  py_pipe [LATENCY];

    always_ff @(posedge clk) begin
        if (en) begin
            px_pipe[0] <= px_in;
            py_pipe[0] <= py_in;
        end
    end

    genvar pi;
    generate
        for (pi = 1; pi < LATENCY; pi++) begin : g_pxpipe
            always_ff @(posedge clk) begin
                if (en) begin
                    px_pipe[pi] <= px_pipe[pi-1];
                    py_pipe[pi] <= py_pipe[pi-1];
                end
            end
        end
    endgenerate

    assign px_out = px_pipe[LATENCY-1];
    assign py_out = py_pipe[LATENCY-1];

endmodule