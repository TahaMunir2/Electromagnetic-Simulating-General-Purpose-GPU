module bz#(
    parameter FP_WIDTH = 16,    
    parameter FRAC_BITS = 13
)(
    input logic clk,
    input logic signed [FP_WIDTH - 1:0] C_B,
    input logic signed [FP_WIDTH - 1:0] bz_old,
    input logic signed [FP_WIDTH - 1:0] ey_left,
    input logic signed [FP_WIDTH - 1:0] ey_right,
    output logic signed [FP_WIDTH - 1:0] bz_new 
);

logic signed [FP_WIDTH - 1:0] bz_1_reg;
logic signed [FP_WIDTH:0] difference_reg; // one bit wider to prevent overflow
logic signed [2 * FP_WIDTH:0] bz_untruncated; // twice as long + 1 for overflow due to multiplication
logic signed [FP_WIDTH - 1 :0] bz_truncated; 


always_ff @(posedge clk) begin
    difference_reg <= ey_right - ey_left;
    bz_1_reg <= bz_old;
    bz_new <= bz_1_reg - bz_truncated;
end


    assign bz_untruncated = (C_B * difference_reg);
    assign bz_truncated = $signed(bz_untruncated[FRAC_BITS + FP_WIDTH - 1 : FRAC_BITS]);

endmodule