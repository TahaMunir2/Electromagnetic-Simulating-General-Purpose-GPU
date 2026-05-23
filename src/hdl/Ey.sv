`timescale 1ns/1ps

module ey #(
    parameter FP_WIDTH = 16,
    parameter FRAC_BITS = 13
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] C_E,
    input  logic signed [FP_WIDTH-1:0] ey_old,
    input  logic signed [FP_WIDTH-1:0] bz_left,
    input  logic signed [FP_WIDTH-1:0] bz_right,
    output logic signed [FP_WIDTH-1:0] ey_new
);

    logic signed [FP_WIDTH-1:0] ey_1_reg;
    logic signed [FP_WIDTH:0] difference_reg;
    logic signed [2*FP_WIDTH:0] ey_untruncated;
    logic signed [FP_WIDTH-1:0] ey_truncated;

    always_ff @(posedge clk) begin
        difference_reg <= bz_right - bz_left;
        ey_1_reg       <= ey_old;
        ey_new         <= ey_1_reg - ey_truncated;
    end

    assign ey_untruncated = C_E * difference_reg;
    assign ey_truncated   = $signed(ey_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);

endmodule
