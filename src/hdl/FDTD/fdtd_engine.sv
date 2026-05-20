module fdtd_engine#(
parameter FP_WIDTH = 16
)(
    input  logic clk,
    input  logic signed [FP_WIDTH-1:0] bz_val,
    input  logic signed [FP_WIDTH-1:0] ey_val,
    output logic signed [FP_WIDTH-1:0] bz_new,
    output logic signed [FP_WIDTH-1:0] ey_new

);

    bz #(.FP_WIDTH(FP_WIDTH)) bz (
        .clk(clk),
        .C_B(16'sd2867),       // hardwired for now
        .bz_old(bz_val),        
        .ey_left(ey_val),      
        .ey_right(ey_val), 
        .bz_new(bz_new)         
    );

    ey #(.FP_WIDTH(FP_WIDTH)) ey (
        .clk(clk),
        .C_E(16'sd717),         // hardwired for now
        .ey_old(ey_val),        
        .bz_left(bz_val),      
        .bz_right(bz_val),
        .ey_new(ey_new)        
    );

endmodule