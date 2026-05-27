`timescale 1ns/1ps

module ex #(
    parameter FP_WIDTH = 16,
    parameter FRAC_BITS = 13
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] ca,
    input  logic signed [FP_WIDTH-1:0] cb,
    input  logic signed [FP_WIDTH-1:0] ex_old,
    input  logic signed [FP_WIDTH-1:0] bz_left,
    input  logic signed [FP_WIDTH-1:0] bz_right,
    output logic signed [FP_WIDTH-1:0] ex_new
);

    logic signed [FP_WIDTH-1:0] ex_1_reg;
    logic signed [FP_WIDTH:0] difference_reg;
    logic signed [2*FP_WIDTH:0] ex_ca_untruncated;
    logic signed [2*FP_WIDTH:0] ex_cb_untruncated;
    logic signed [FP_WIDTH-1:0] ex_ca_truncated;
    logic signed [FP_WIDTH-1:0] ex_cb_truncated;
    logic signed [FP_WIDTH-1:0] ex_ca_reg;
    logic signed [FP_WIDTH-1:0] ex_cb_reg;

    always_ff @(posedge clk) begin
        difference_reg <= bz_right - bz_left;
        ex_1_reg       <= ex_old;
        ex_ca_reg      <= ex_ca_truncated;
        ex_cb_reg      <= ex_cb_truncated;
        ex_new         <= ex_ca_reg - ex_cb_reg;
    end

    assign ex_ca_untruncated = ca * ex_1_reg;
    assign ex_cb_untruncated = cb * difference_reg;
    assign ex_ca_truncated   = $signed(ex_ca_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);
    assign ex_cb_truncated   = $signed(ex_cb_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);

endmodule
