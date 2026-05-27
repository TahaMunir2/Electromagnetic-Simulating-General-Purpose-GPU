`timescale 1ns/1ps

module ey #(
    parameter FP_WIDTH = 16,
    parameter FRAC_BITS = 13
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] ca,
    input  logic signed [FP_WIDTH-1:0] cb,
    input  logic signed [FP_WIDTH-1:0] ey_old,
    input  logic signed [FP_WIDTH-1:0] bz_left,
    input  logic signed [FP_WIDTH-1:0] bz_right,
    output logic signed [FP_WIDTH-1:0] ey_new
);

    logic signed [FP_WIDTH-1:0] ey_1_reg;
    logic signed [FP_WIDTH:0] difference_reg;
    logic signed [2*FP_WIDTH:0] ey_ca_untruncated;
    logic signed [2*FP_WIDTH:0] ey_cb_untruncated;
    logic signed [FP_WIDTH-1:0] ey_ca_truncated;
    logic signed [FP_WIDTH-1:0] ey_cb_truncated;
    logic signed [FP_WIDTH-1:0] ey_ca_reg;
    logic signed [FP_WIDTH-1:0] ey_cb_reg;

    always_ff @(posedge clk) begin
        difference_reg <= bz_right - bz_left;
        ey_1_reg       <= ey_old;
        ey_ca_reg      <= ey_ca_truncated;
        ey_cb_reg      <= ey_cb_truncated;
        ey_new         <= ey_ca_reg + ey_cb_reg;
    end

    assign ey_ca_untruncated = ca * ey_1_reg;
    assign ey_cb_untruncated = cb * difference_reg;
    assign ey_ca_truncated   = $signed(ey_ca_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);
    assign ey_cb_truncated   = $signed(ey_cb_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);

endmodule
