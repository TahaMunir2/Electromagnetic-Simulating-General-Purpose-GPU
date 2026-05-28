// ============================================================================
//  ray_unit.sv
//  ----------------------------------------------------------------------------
//  Single-pixel renderer wrapper.
//
//  Chains the four core modules end-to-end:
//
//      pixel coords (px, py) + camera params
//                  │
//                  ▼
//          ┌──────────────┐
//          │   ray_gen    │  computes ray direction D from (px, py)
//          └──────┬───────┘
//                 │ (Dx, Dy, Dz, px, py, valid)
//                 ▼
//          ┌──────────────┐
//          │   marcher    │  marches N_STEPS steps through the heightmap
//          │              │  16 BRAM ports
//          └──────┬───────┘
//                 │ (status, ix_hit, iy_hit, h_hit, step_count, px, py, valid)
//                 ▼
//          ┌──────────────┐
//          │   normal     │  reads 4 neighbours, computes surface normal
//          │              │  4 BRAM ports
//          └──────┬───────┘
//                 │ (status, h_hit, step_count, Nx, Ny, Nz, px, py, valid)
//                 ▼
//          ┌──────────────┐
//          │   shader     │  Lambert + altitude colour + fog
//          └──────┬───────┘
//                 │ (r, g, b, px, py, valid)
//                 ▼
//             output
//
//  This wrapper exposes:
//      - the input side of ray_gen
//      - the output side of shader
//      - all 20 BRAM ports (16 from marcher + 4 from normal)
//
//  No emit module yet (that will go after shader for AXI-Stream handshake).
//  No top-level AXI-Lite interface (camera params come in as discrete ports
//  for now).
//
//  Total pipeline latency:
//      ray_gen  :  4 cycles
//      marcher  : 64 cycles (16 march_steps * 4 stages)
//      normal   :  5 cycles (was 4 — added 1 stage for bilinear lerp)
//      shader   :  5 cycles
//      TOTAL    : 78 cycles
//
//  Throughput (after fill): 1 pixel/cycle.
// ============================================================================

module design1_ray_unit #(
    // ----- Image / screen geometry -----
    parameter int W           = 640,
    parameter int H           = 480,

    // ----- Fixed-point formats (must be consistent across submodules) -----
    parameter int POS_W       = 16,
    parameter int POS_I       = 2,
    parameter int POS_F       = POS_W - 1 - POS_I,

    parameter int DIR_W       = 16,
    parameter int DIR_I       = 2,
    parameter int DIR_F       = DIR_W - 1 - DIR_I,

    parameter int UV_W        = 16,
    parameter int UV_I        = 1,

    parameter int K_W         = 16,
    parameter int K_I         = 0,
    parameter logic signed [K_W-1:0] K_U = 16'sd137,
    parameter logic signed [K_W-1:0] K_V = 16'sd137,

    // ----- Heightmap geometry -----
    parameter int GRID_N      = 256,
    parameter int IDX_W       = $clog2(GRID_N),

    parameter int H_W         = 16,
    parameter int H_I         = 2,
    parameter int H_F         = H_W - 1 - H_I,

    // ----- Marcher depth -----
    parameter int N_STEPS     = 16,
    parameter int STEP_W      = $clog2(N_STEPS + 1),

    // ----- Pixel coord widths -----
    parameter int PX_W        = $clog2(W),
    parameter int PY_W        = $clog2(H),

    // ----- Shader knobs -----
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
    input  logic                       en,           // ~stall

    // ----- Camera params (constant per frame, set by CPU via AXI-Lite later) -----
    input  logic signed [POS_W-1:0]    Ox,
    input  logic signed [POS_W-1:0]    Oy,
    input  logic signed [POS_W-1:0]    Oz,
    input  logic signed [DIR_W-1:0]    fwd_x,
    input  logic signed [DIR_W-1:0]    fwd_y,
    input  logic signed [DIR_W-1:0]    fwd_z,
    input  logic signed [DIR_W-1:0]    right_x,
    input  logic signed [DIR_W-1:0]    right_y,
    input  logic signed [DIR_W-1:0]    right_z,
    input  logic signed [DIR_W-1:0]    up_x,
    input  logic signed [DIR_W-1:0]    up_y,
    input  logic signed [DIR_W-1:0]    up_z,

    // ----- Sun direction (currently unused — sun is hardcoded in shader) -----
    input  logic signed [DIR_W-1:0]    sun_dx,
    input  logic signed [DIR_W-1:0]    sun_dy,
    input  logic signed [DIR_W-1:0]    sun_dz,

    // ----- Pixel input -----
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    // ----- Marcher BRAM ports (16 ports) -----
    output logic [IDX_W*2-1:0]         marcher_bram_addr [N_STEPS],
    output logic                       marcher_bram_re   [N_STEPS],
    input  logic signed [H_W-1:0]      marcher_bram_dout [N_STEPS],

    // ----- Normal BRAM ports (4 ports) -----
    output logic [IDX_W*2-1:0]         normal_bram_addr  [4],
    output logic                       normal_bram_re    [4],
    input  logic signed [H_W-1:0]      normal_bram_dout  [4],

    // ----- Pixel output -----
    output logic [7:0]                 r_out,
    output logic [7:0]                 g_out,
    output logic [7:0]                 b_out,
    output logic [PX_W-1:0]            px_out,
    output logic [PY_W-1:0]            py_out,
    output logic                       valid_out
);


    // =================================================================
    //  ray_gen  ->  marcher
    // =================================================================
    logic signed [DIR_W-1:0]    rg_Dx, rg_Dy, rg_Dz;
    logic [PX_W-1:0]            rg_px;
    logic [PY_W-1:0]            rg_py;
    logic                       rg_valid;
    logic                       rg_stall;   // back-pressure into ray_gen

    // No back-pressure for now; tie to 0.  Will be driven by emit later.
    assign rg_stall = 1'b0;

    design1_ray_gen #(
        .W      (W),
        .H      (H),
        .DIR_W  (DIR_W),
        .DIR_I  (DIR_I),
        .UV_W   (UV_W),
        .UV_I   (UV_I),
        .K_W    (K_W),
        .K_I    (K_I),
        .K_U    (K_U),
        .K_V    (K_V)
    ) u_ray_gen (
        .clk        (clk),
        .rst_n      (rst_n),
        .px_in      (px_in),
        .py_in      (py_in),
        .valid_in   (valid_in),
        .fwd_x      (fwd_x), .fwd_y(fwd_y), .fwd_z(fwd_z),
        .right_x    (right_x), .right_y(right_y), .right_z(right_z),
        .up_x       (up_x), .up_y(up_y), .up_z(up_z),
        .stall      (rg_stall),
        .Dx         (rg_Dx), .Dy(rg_Dy), .Dz(rg_Dz),
        .px_out     (rg_px),
        .py_out     (rg_py),
        .valid_out  (rg_valid)
    );


    // =================================================================
    //  marcher  ->  normal
    // =================================================================
    logic [1:0]                 mc_status;
    logic [IDX_W-1:0]           mc_ix_hit, mc_iy_hit;
    logic signed [H_W-1:0]      mc_h_hit;
    logic signed [POS_W-1:0]    mc_Px_hit, mc_Py_hit;
    logic [STEP_W-1:0]          mc_step_count;
    logic [PX_W-1:0]            mc_px;
    logic [PY_W-1:0]            mc_py;
    logic                       mc_valid;

    design1_marcher #(
        .POS_W      (POS_W),
        .POS_I      (POS_I),
        .DIR_W      (DIR_W),
        .DIR_I      (DIR_I),
        .GRID_N     (GRID_N),
        .H_W        (H_W),
        .H_I        (H_I),
        .N_STEPS    (N_STEPS),
        .PX_W       (PX_W),
        .PY_W       (PY_W)
    ) u_marcher (
        .clk        (clk),
        .rst_n      (rst_n),
        .en         (en),
        .Dx_in      (rg_Dx),
        .Dy_in      (rg_Dy),
        .Dz_in      (rg_Dz),
        .px_in      (rg_px),
        .py_in      (rg_py),
        .valid_in   (rg_valid),
        .Ox         (Ox),
        .Oy         (Oy),
        .Oz         (Oz),
        .bram_addr  (marcher_bram_addr),
        .bram_re    (marcher_bram_re),
        .bram_dout  (marcher_bram_dout),
        .status_out (mc_status),
        .ix_hit_out (mc_ix_hit),
        .iy_hit_out (mc_iy_hit),
        .h_hit_out  (mc_h_hit),
        .Px_hit_out (mc_Px_hit),
        .Py_hit_out (mc_Py_hit),
        .step_count_out (mc_step_count),
        .px_out     (mc_px),
        .py_out     (mc_py),
        .valid_out  (mc_valid)
    );


    // =================================================================
    //  normal  ->  shader
    // =================================================================
    logic [1:0]                 nm_status;
    logic [IDX_W-1:0]           nm_ix, nm_iy;
    logic signed [H_W-1:0]      nm_h_hit;
    logic signed [H_W-1:0]      nm_h_interp;
    logic [STEP_W-1:0]          nm_step_count;
    logic signed [DIR_W-1:0]    nm_Nx, nm_Ny, nm_Nz;
    logic [PX_W-1:0]            nm_px;
    logic [PY_W-1:0]            nm_py;
    logic                       nm_valid;

    design1_normal #(
        .GRID_N     (GRID_N),
        .POS_W      (POS_W),
        .POS_I      (POS_I),
        .H_W        (H_W),
        .H_I        (H_I),
        .DIR_W      (DIR_W),
        .DIR_I      (DIR_I),
        .PX_W       (PX_W),
        .PY_W       (PY_W),
        .STEP_W     (STEP_W)
    ) u_normal (
        .clk        (clk),
        .rst_n      (rst_n),
        .en         (en),
        .status_in  (mc_status),
        .ix_in      (mc_ix_hit),
        .iy_in      (mc_iy_hit),
        .h_hit_in   (mc_h_hit),
        .Px_hit_in  (mc_Px_hit),
        .Py_hit_in  (mc_Py_hit),
        .step_count_in (mc_step_count),
        .px_in      (mc_px),
        .py_in      (mc_py),
        .valid_in   (mc_valid),
        .bram_addr  (normal_bram_addr),
        .bram_re    (normal_bram_re),
        .bram_dout  (normal_bram_dout),
        .status_out (nm_status),
        .ix_out     (nm_ix),
        .iy_out     (nm_iy),
        .h_hit_out  (nm_h_hit),
        .h_interp_out (nm_h_interp),
        .step_count_out (nm_step_count),
        .Nx_out     (nm_Nx),
        .Ny_out     (nm_Ny),
        .Nz_out     (nm_Nz),
        .px_out     (nm_px),
        .py_out     (nm_py),
        .valid_out  (nm_valid)
    );


    // =================================================================
    //  shader  ->  outputs
    // =================================================================
    design1_shader #(
        .H_W        (H_W),
        .H_I        (H_I),
        .DIR_W      (DIR_W),
        .DIR_I      (DIR_I),
        .N_STEPS    (N_STEPS),
        .PX_W       (PX_W),
        .PY_W       (PY_W),
        .SKY_R      (SKY_R),
        .SKY_G      (SKY_G),
        .SKY_B      (SKY_B),
        .PALE_R     (PALE_R),
        .PALE_G     (PALE_G),
        .PALE_B     (PALE_B),
        .AMBIENT    (AMBIENT)
    ) u_shader (
        .clk        (clk),
        .rst_n      (rst_n),
        .en         (en),
        .status_in  (nm_status),
        .h_hit_in   (nm_h_interp),    // bilinear smooth height (was nm_h_hit)
        .step_count_in (nm_step_count),
        .Nx_in      (nm_Nx),
        .Ny_in      (nm_Ny),
        .Nz_in      (nm_Nz),
        .px_in      (nm_px),
        .py_in      (nm_py),
        .valid_in   (nm_valid),
        .sun_dx     (sun_dx),
        .sun_dy     (sun_dy),
        .sun_dz     (sun_dz),
        .r_out      (r_out),
        .g_out      (g_out),
        .b_out      (b_out),
        .px_out     (px_out),
        .py_out     (py_out),
        .valid_out  (valid_out)
    );

endmodule
