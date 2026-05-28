// 640x480 @ 60 Hz video timing.
// Pixel clock is nominally 25.175 MHz; 25.000 MHz is accepted by many displays.

module design1_video_timing_640x480 (
    input  logic       clk_pix,
    input  logic       rst_n,

    output logic [9:0] sx,
    output logic [9:0] sy,
    output logic       hsync,
    output logic       vsync,
    output logic       active_video
);

    localparam int H_ACTIVE = 640;
    localparam int H_FRONT  = 16;
    localparam int H_SYNC   = 96;
    localparam int H_BACK   = 48;
    localparam int H_TOTAL  = H_ACTIVE + H_FRONT + H_SYNC + H_BACK;

    localparam int V_ACTIVE = 480;
    localparam int V_FRONT  = 10;
    localparam int V_SYNC   = 2;
    localparam int V_BACK   = 33;
    localparam int V_TOTAL  = V_ACTIVE + V_FRONT + V_SYNC + V_BACK;

    always_ff @(posedge clk_pix) begin
        if (!rst_n) begin
            sx <= 10'd0;
            sy <= 10'd0;
        end else if (sx == H_TOTAL - 1) begin
            sx <= 10'd0;
            sy <= (sy == V_TOTAL - 1) ? 10'd0 : sy + 10'd1;
        end else begin
            sx <= sx + 10'd1;
        end
    end

    assign active_video = (sx < H_ACTIVE) && (sy < V_ACTIVE);

    // VGA/DVI 640x480 uses negative sync polarity.
    assign hsync = ~((sx >= H_ACTIVE + H_FRONT) &&
                     (sx <  H_ACTIVE + H_FRONT + H_SYNC));

    assign vsync = ~((sy >= V_ACTIVE + V_FRONT) &&
                     (sy <  V_ACTIVE + V_FRONT + V_SYNC));

endmodule
