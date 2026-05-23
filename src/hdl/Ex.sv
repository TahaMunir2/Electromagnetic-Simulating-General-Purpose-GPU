`timescale 1ns/1ps

module ex #(
    parameter FP_WIDTH = 16,
    parameter FRAC_BITS = 13
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] C_E,
    input  logic signed [FP_WIDTH-1:0] ex_old,
    input  logic signed [FP_WIDTH-1:0] bz_left,
    input  logic signed [FP_WIDTH-1:0] bz_right,
    output logic signed [FP_WIDTH-1:0] ex_new
);

    logic signed [FP_WIDTH-1:0] ex_1_reg;
    logic signed [FP_WIDTH:0] difference_reg;
    logic signed [2*FP_WIDTH:0] ex_untruncated;
    logic signed [FP_WIDTH-1:0] ex_truncated;

    always_ff @(posedge clk) begin
        difference_reg <= bz_right - bz_left;
        ex_1_reg       <= ex_old;
        ex_new         <= ex_1_reg + ex_truncated;
    end

    assign ex_untruncated = C_E * difference_reg;
    assign ex_truncated   = $signed(ex_untruncated[FRAC_BITS+FP_WIDTH-1:FRAC_BITS]);

endmodule
