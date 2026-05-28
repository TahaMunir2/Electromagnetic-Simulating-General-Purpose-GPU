// ============================================================================
//  ray_unit_top.sv  —  Vivado implementation wrapper for timing analysis
// ============================================================================
//  Wraps design1_ray_unit with:
//    - 20 inferred block RAMs (16 marcher + 4 normal)
//    - a free-running pixel counter driving valid_in/px/py
//    - hardcoded (but non-trivial) camera parameters
//
//  GRID_N is reduced to 64 (from 256) so all 20 BRAMs fit on an
//  Artix-7 100T / Zynq-7020 class device (~40 RAMB36).
//  To target a larger part, increase GRID_N back to 256 here and in the
//  design1_ray_unit instantiation below.
//
//  Ports are kept minimal — only clock, active-low reset, and the
//  pixel output bus.  All I/O timing is false-pathed in constraints.xdc.
// ============================================================================

module design1_ray_unit_top (
    input  logic        clk,
    input  logic        rst_n,

    output logic [7:0]  r_out,
    output logic [7:0]  g_out,
    output logic [7:0]  b_out,
    output logic [9:0]  px_out,    // PX_W = $clog2(640) = 10
    output logic [8:0]  py_out,    // PY_W = $clog2(480) = 9
    output logic        valid_out
);

    // -------------------------------------------------------------------------
    //  Design-point parameters
    // -------------------------------------------------------------------------
    localparam int W       = 640;
    localparam int H       = 480;
    localparam int PX_W    = 10;   // $clog2(640)
    localparam int PY_W    = 9;    // $clog2(480)

    localparam int GRID_N  = 64;   // keep ≤64 on Artix-7 / Zynq-7020
    localparam int IDX_W   = 6;    // $clog2(64)
    localparam int ADDR_W  = IDX_W * 2;   // 12 bits

    localparam int N_STEPS = 16;
    localparam int H_W     = 16;
    localparam int DIR_W   = 16;
    localparam int POS_W   = 16;

    // Fixed-point 1.0 in Q2.13 (DIR_F = 13, 1.0 = 2^13 = 8192)
    localparam logic signed [DIR_W-1:0] ONE  = 16'sd8192;
    localparam logic signed [DIR_W-1:0] ZERO = 16'sd0;

    // Camera: close above the -X/-Y side, looking diagonally toward map centre
    // with a 45-degree downward pitch. Values are Q2.13. The height is kept
    // low enough that the current 16-step marcher reaches the terrain.
    localparam logic signed [POS_W-1:0] OX = -16'sd2867;   // -0.350
    localparam logic signed [POS_W-1:0] OY = -16'sd2867;   // -0.350
    localparam logic signed [POS_W-1:0] OZ =  16'sd3686;   //  0.450

    localparam logic signed [DIR_W-1:0] FWD_X   =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] FWD_Y   =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] FWD_Z   = -16'sd5793;  // -0.707
    localparam logic signed [DIR_W-1:0] RIGHT_X =  16'sd5793;  //  0.707
    localparam logic signed [DIR_W-1:0] RIGHT_Y = -16'sd5793;  // -0.707
    localparam logic signed [DIR_W-1:0] RIGHT_Z =  16'sd0;
    localparam logic signed [DIR_W-1:0] UP_X    =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] UP_Y    =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] UP_Z    =  16'sd5793;  //  0.707

    // Sun direction (normalised approx.) pointing up and forward:
    //   (0, 0.707, 0.707) ≈ (0, 5793, 5793) in Q2.13
    localparam logic signed [DIR_W-1:0] SUN_D = 16'sd5793;

    // -------------------------------------------------------------------------
    //  Free-running pixel counter — drives valid/px/py into design1_ray_unit
    // -------------------------------------------------------------------------
    logic [PX_W-1:0] px_cnt;
    logic [PY_W-1:0] py_cnt;
    logic            valid_in;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            px_cnt   <= '0;
            py_cnt   <= '0;
            valid_in <= 1'b0;
        end else begin
            valid_in <= 1'b1;
            if (px_cnt == W - 1) begin
                px_cnt <= '0;
                py_cnt <= (py_cnt == H - 1) ? '0 : py_cnt + 1'b1;
            end else begin
                px_cnt <= px_cnt + 1'b1;
            end
        end
    end

    // -------------------------------------------------------------------------
    //  BRAM arrays — one instance per marcher port, one per normal port
    // -------------------------------------------------------------------------
    logic [ADDR_W-1:0]         mb_addr [N_STEPS];
    logic                      mb_re   [N_STEPS];
    logic signed [H_W-1:0]     mb_dout [N_STEPS];

    logic [ADDR_W-1:0]         nb_addr [4];
    logic                      nb_re   [4];
    logic signed [H_W-1:0]     nb_dout [4];

    genvar gi;
    generate
        for (gi = 0; gi < N_STEPS; gi++) begin : g_marcher_bram
            design1_heightmap_bram #(
                .ADDR_W (ADDR_W),
                .DATA_W (H_W)
            ) u_bram (
                .clk  (clk),
                .addr (mb_addr[gi]),
                .re   (mb_re[gi]),
                .dout (mb_dout[gi])
            );
        end

        for (gi = 0; gi < 4; gi++) begin : g_normal_bram
            design1_heightmap_bram #(
                .ADDR_W (ADDR_W),
                .DATA_W (H_W)
            ) u_bram (
                .clk  (clk),
                .addr (nb_addr[gi]),
                .re   (nb_re[gi]),
                .dout (nb_dout[gi])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    //  design1_ray_unit instance
    // -------------------------------------------------------------------------
    design1_ray_unit #(
        .W        (W),
        .H        (H),
        .GRID_N   (GRID_N),
        .N_STEPS  (N_STEPS),
        .H_W      (H_W),
        .H_I      (2),
        .DIR_W    (DIR_W),
        .DIR_I    (2),
        .POS_W    (POS_W),
        .POS_I    (2)
    ) u_ray_unit (
        .clk                (clk),
        .rst_n              (rst_n),
        .en                 (1'b1),

        // Camera parameters
        .Ox                 (OX),
        .Oy                 (OY),
        .Oz                 (OZ),
        .fwd_x              (FWD_X),
        .fwd_y              (FWD_Y),
        .fwd_z              (FWD_Z),
        .right_x            (RIGHT_X),
        .right_y            (RIGHT_Y),
        .right_z            (RIGHT_Z),
        .up_x               (UP_X),
        .up_y               (UP_Y),
        .up_z               (UP_Z),

        // Sun direction
        .sun_dx             (ZERO),
        .sun_dy             (SUN_D),
        .sun_dz             (SUN_D),

        // Pixel input
        .px_in              (px_cnt),
        .py_in              (py_cnt),
        .valid_in           (valid_in),

        // Marcher BRAMs
        .marcher_bram_addr  (mb_addr),
        .marcher_bram_re    (mb_re),
        .marcher_bram_dout  (mb_dout),

        // Normal BRAMs
        .normal_bram_addr   (nb_addr),
        .normal_bram_re     (nb_re),
        .normal_bram_dout   (nb_dout),

        // Pixel output
        .r_out              (r_out),
        .g_out              (g_out),
        .b_out              (b_out),
        .px_out             (px_out),
        .py_out             (py_out),
        .valid_out          (valid_out)
    );

endmodule
