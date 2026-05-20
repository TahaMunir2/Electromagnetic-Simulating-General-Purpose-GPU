`timescale 1ns/1ps

module ey_tb;
    parameter FP_WIDTH = 16;
    parameter FRAC_BITS = 13;

    logic clk;
    logic signed [FP_WIDTH-1:0] C_E;
    logic signed [FP_WIDTH-1:0] ey_old;
    logic signed [FP_WIDTH-1:0] bz_left;
    logic signed [FP_WIDTH-1:0] bz_right;
    logic signed [FP_WIDTH-1:0] ey_new;

    ey #(
        .FP_WIDTH(FP_WIDTH),
        .FRAC_BITS(FRAC_BITS)
    ) ey_test (
        .clk(clk),
        .C_E(C_E),
        .ey_old(ey_old),
        .bz_left(bz_left),
        .bz_right(bz_right),
        .ey_new(ey_new)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk; 
    end

    initial begin
        $dumpfile("dump.vcd");
        $dumpvars(0, ey_tb);

        C_E = 16'sd717;  // 0.0875
        ey_old = 0;
        bz_left = 0;
        bz_right = 0;
        
        #40; 

        bz_left  = 16'sd8192; // 1.0 
        bz_right = 16'sd8192; // 1.0 
        ey_old   = 16'sd4096; // 0.5
        
        #30; 

        bz_left  = 16'sd4096; // 0.5
        bz_right = 16'sd8192; // 1.0
        
        #30; 
        // expect 4454 in dec
        $display("Simulation Complete. Open dump.vcd in GTKWave.");
        $finish;
    end
endmodule