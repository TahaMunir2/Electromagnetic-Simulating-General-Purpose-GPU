// PYNQ-Z1 HDMI wrapper for Design 1 of the ray renderer.
//
// Required Vivado IP:
//   - clk_wiz_0:
//       input  clk_in1 = 125 MHz PYNQ-Z1 PL clock
//       output clk_out1 = 25.000 MHz or 25.175 MHz pixel clock
//       output clk_out2 = 125.000 MHz or 125.875 MHz serial clock
//   - rgb2dvi_0:
//       Digilent rgb2dvi configured for 7-series TMDS output

module design1_ray_unit_hdmi_top (
    input  logic       clk,
    input  logic       rst,

    output logic       hdmi_tx_clk_p,
    output logic       hdmi_tx_clk_n,
    output logic [2:0] hdmi_tx_p,
    output logic [2:0] hdmi_tx_n
);

    localparam int W              = 640;
    localparam int H              = 480;
    localparam int PX_W           = 10;
    localparam int PY_W           = 9;
    localparam int RENDER_LATENCY = 78;

    localparam int GRID_N  = 64;
    localparam int IDX_W   = 6;
    localparam int ADDR_W  = IDX_W * 2;

    localparam int N_STEPS = 16;
    localparam int H_W     = 16;
    localparam int DIR_W   = 16;
    localparam int POS_W   = 16;

    localparam logic signed [DIR_W-1:0] ONE  = 16'sd8192;
    localparam logic signed [DIR_W-1:0] ZERO = 16'sd0;

    // Camera: close above the -X/-Y side, looking diagonally toward map centre
    // with a 45-degree downward pitch. Values are Q2.13. The height is kept
    // low enough that the current 16-step marcher reaches the terrain.
    localparam logic signed [POS_W-1:0] OX = -16'sd2867;  // -0.350
    localparam logic signed [POS_W-1:0] OY = -16'sd2867;  // -0.350
    localparam logic signed [POS_W-1:0] OZ =  16'sd3686;  //  0.450

    localparam logic signed [DIR_W-1:0] FWD_X   =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] FWD_Y   =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] FWD_Z   = -16'sd5793;  // -0.707
    localparam logic signed [DIR_W-1:0] RIGHT_X =  16'sd5793;  //  0.707
    localparam logic signed [DIR_W-1:0] RIGHT_Y = -16'sd5793;  // -0.707
    localparam logic signed [DIR_W-1:0] RIGHT_Z =  16'sd0;
    localparam logic signed [DIR_W-1:0] UP_X    =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] UP_Y    =  16'sd4096;  //  0.500
    localparam logic signed [DIR_W-1:0] UP_Z    =  16'sd5793;  //  0.707

    localparam logic signed [DIR_W-1:0] SUN_D = 16'sd5793;

    logic clk_pix;
    logic clk_5x;
    logic clk_locked;

    clk_wiz_0 u_clk_wiz (
        .clk_in1  (clk),
        .reset    (rst),
        .clk_out1 (clk_pix),
        .clk_out2 (clk_5x),
        .locked   (clk_locked)
    );

    logic rst_pix_n;
    assign rst_pix_n = ~rst & clk_locked;

    logic [9:0] sx;
    logic [9:0] sy;
    logic       hsync;
    logic       vsync;
    logic       active_video;

    design1_video_timing_640x480 u_timing (
        .clk_pix      (clk_pix),
        .rst_n        (rst_pix_n),
        .sx           (sx),
        .sy           (sy),
        .hsync        (hsync),
        .vsync        (vsync),
        .active_video (active_video)
    );

    logic       hsync_pipe [0:RENDER_LATENCY-1];
    logic       vsync_pipe [0:RENDER_LATENCY-1];
    logic       de_pipe    [0:RENDER_LATENCY-1];

    always_ff @(posedge clk_pix) begin
        if (!rst_pix_n) begin
            for (int i = 0; i < RENDER_LATENCY; i++) begin
                hsync_pipe[i] <= 1'b1;
                vsync_pipe[i] <= 1'b1;
                de_pipe[i]    <= 1'b0;
            end
        end else begin
            hsync_pipe[0] <= hsync;
            vsync_pipe[0] <= vsync;
            de_pipe[0]    <= active_video;

            for (int i = 1; i < RENDER_LATENCY; i++) begin
                hsync_pipe[i] <= hsync_pipe[i-1];
                vsync_pipe[i] <= vsync_pipe[i-1];
                de_pipe[i]    <= de_pipe[i-1];
            end
        end
    end

    logic [7:0] ray_r;
    logic [7:0] ray_g;
    logic [7:0] ray_b;
    logic [9:0] ray_px;
    logic [8:0] ray_py;
    logic       ray_valid;

    logic [ADDR_W-1:0]     mb_addr [N_STEPS];
    logic                  mb_re   [N_STEPS];
    logic signed [H_W-1:0] mb_dout [N_STEPS];

    logic [ADDR_W-1:0]     nb_addr [4];
    logic                  nb_re   [4];
    logic signed [H_W-1:0] nb_dout [4];

    genvar gi;
    generate
        for (gi = 0; gi < N_STEPS; gi++) begin : g_marcher_bram
            design1_heightmap_bram #(
                .ADDR_W (ADDR_W),
                .DATA_W (H_W)
            ) u_bram (
                .clk  (clk_pix),
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
                .clk  (clk_pix),
                .addr (nb_addr[gi]),
                .re   (nb_re[gi]),
                .dout (nb_dout[gi])
            );
        end
    endgenerate

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
        .clk                (clk_pix),
        .rst_n              (rst_pix_n),
        .en                 (1'b1),

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

        .sun_dx             (ZERO),
        .sun_dy             (SUN_D),
        .sun_dz             (SUN_D),

        .px_in              (active_video ? sx[PX_W-1:0] : '0),
        .py_in              (active_video ? sy[PY_W-1:0] : '0),
        .valid_in           (active_video),

        .marcher_bram_addr  (mb_addr),
        .marcher_bram_re    (mb_re),
        .marcher_bram_dout  (mb_dout),

        .normal_bram_addr   (nb_addr),
        .normal_bram_re     (nb_re),
        .normal_bram_dout   (nb_dout),

        .r_out              (ray_r),
        .g_out              (ray_g),
        .b_out              (ray_b),
        .px_out             (ray_px),
        .py_out             (ray_py),
        .valid_out          (ray_valid)
    );

    logic       hdmi_de;
    logic       hdmi_hsync;
    logic       hdmi_vsync;
    logic [7:0] hdmi_r;
    logic [7:0] hdmi_g;
    logic [7:0] hdmi_b;

    assign hdmi_de    = de_pipe[RENDER_LATENCY-1] & ray_valid;
    assign hdmi_hsync = hsync_pipe[RENDER_LATENCY-1];
    assign hdmi_vsync = vsync_pipe[RENDER_LATENCY-1];

    assign hdmi_r = hdmi_de ? ray_r : 8'd0;
    assign hdmi_g = hdmi_de ? ray_g : 8'd0;
    assign hdmi_b = hdmi_de ? ray_b : 8'd0;

    rgb2dvi_0 u_rgb2dvi (
        .TMDS_Clk_p  (hdmi_tx_clk_p),
        .TMDS_Clk_n  (hdmi_tx_clk_n),
        .TMDS_Data_p (hdmi_tx_p),
        .TMDS_Data_n (hdmi_tx_n),
        .aRst        (!rst_pix_n),
        .vid_pData   ({hdmi_r, hdmi_b, hdmi_g}),
        .vid_pVDE    (hdmi_de),
        .vid_pHSync  (hdmi_hsync),
        .vid_pVSync  (hdmi_vsync),
        .PixelClk    (clk_pix),
        .SerialClk   (clk_5x)
    );

endmodule
