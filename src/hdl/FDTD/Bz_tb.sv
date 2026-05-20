`timescale 1ns/1ps

module bz_tb;
    parameter FP_WIDTH = 16;
    parameter FRAC_BITS = 13;

    logic clk;
    logic signed [FP_WIDTH-1:0] C_B;
    logic signed [FP_WIDTH-1:0] bz_old;
    logic signed [FP_WIDTH-1:0] ey_left;
    logic signed [FP_WIDTH-1:0] ey_right;
    logic signed [FP_WIDTH-1:0] bz_new;

    bz #(
        .FP_WIDTH(FP_WIDTH),
        .FRAC_BITS(FRAC_BITS)
    ) bz_test (
        .clk(clk),
        .C_B(C_B),
        .bz_old(bz_old),
        .ey_left(ey_left),
        .ey_right(ey_right),
        .bz_new(bz_new)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, bz_tb);

        C_B = 16'sd2867; // 0.35 
        bz_old = 0;
        ey_left = 0;
        ey_right = 0;
        
        #40; 

        ey_left  = 16'sd8192; // 1.0 
        ey_right = 16'sd8192; // 1.0 
        bz_old   = 16'sd4096; // 0.5 
        
        #30;

        ey_left  = 16'sd4096; // 0.5
        ey_right = 16'sd8192; // 1.0
        
        #30; 
        // expect 2663 in dec

        $display("Simulation Complete. Open dump.vcd in GTKWave.");
        $finish;
    end
endmodule