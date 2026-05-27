`timescale 1ns/1ps

module bz #(
    parameter FP_WIDTH = 16,
    parameter FRAC_BITS = 13
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] ca,
    input  logic signed [FP_WIDTH-1:0] cb,
    input  logic signed [FP_WIDTH-1:0] bz_old,
    input  logic signed [FP_WIDTH-1:0] ey_left,
    input  logic signed [FP_WIDTH-1:0] ey_right,
    input  logic signed [FP_WIDTH-1:0] ex_left,
    input  logic signed [FP_WIDTH-1:0] ex_right,
    output logic signed [FP_WIDTH-1:0] bz_new
);

    logic signed [FP_WIDTH-1:0] bz_1_reg;
    logic signed [FP_WIDTH:0] difference_reg;
    logic signed [2*FP_WIDTH:0] bz_ca_untruncated;
    logic signed [2*FP_WIDTH:0] bz_cb_untruncated;
    logic signed [FP_WIDTH-1:0] bz_ca_truncated;
    logic signed [FP_WIDTH-1:0] bz_cb_truncated;
    logic signed [FP_WIDTH-1:0] bz_ca_reg;
    logic signed [FP_WIDTH-1:0] bz_cb_reg;

    always_ff @(posedge clk) begin
        difference_reg <= (ey_right - ey_left) - (ex_right - ex_left);
        bz_1_reg       <= bz_old;
        bz_ca_reg      <= bz_ca_truncated;
        bz_cb_reg      <= bz_cb_truncated;
        bz_new         <= bz_ca_reg + bz_cb_reg;
    end

    assign bz_ca_untruncated = ca * bz_1_reg;
    assign bz_cb_untruncated = cb * difference_reg;
    assign bz_ca_truncated   = $signed(bz_ca_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);
    assign bz_cb_truncated   = $signed(bz_cb_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);

endmodule
