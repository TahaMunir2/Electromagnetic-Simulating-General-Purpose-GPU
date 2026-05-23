`timescale 1ns/1ps

module fdtd_engine #(
    parameter FP_WIDTH = 16
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] C_E,
    input  logic signed [FP_WIDTH-1:0] C_B,
    input  logic signed [FP_WIDTH-1:0] ey_old,
    input  logic signed [FP_WIDTH-1:0] ex_old,
    input  logic signed [FP_WIDTH-1:0] bz_left,
    input  logic signed [FP_WIDTH-1:0] bz_right,
    input  logic signed [FP_WIDTH-1:0] bz_old,
    input  logic signed [FP_WIDTH-1:0] ey_left,
    input  logic signed [FP_WIDTH-1:0] ey_right,
    input  logic signed [FP_WIDTH-1:0] ex_left,
    input  logic signed [FP_WIDTH-1:0] ex_right,
    output logic signed [FP_WIDTH-1:0] ex_new,
    output logic signed [FP_WIDTH-1:0] ey_new,
    output logic signed [FP_WIDTH-1:0] bz_new
);

    bz #(.FP_WIDTH(FP_WIDTH)) u_bz (
        .clk(clk),
        .C_B(C_B),
        .bz_old(bz_old),
        .ey_left(ey_left),
        .ey_right(ey_right),
        .ex_left(ex_left),
        .ex_right(ex_right),
        .bz_new(bz_new)
    );

    ey #(.FP_WIDTH(FP_WIDTH)) u_ey (
        .clk(clk),
        .C_E(C_E),
        .ey_old(ey_old),
        .bz_left(bz_left),
        .bz_right(bz_right),
        .ey_new(ey_new)
    );

    ex #(.FP_WIDTH(FP_WIDTH)) u_ex (
        .clk(clk),
        .C_E(C_E),
        .ex_old(ex_old),
        .bz_left(bz_left),
        .bz_right(bz_right),
        .ex_new(ex_new)
    );

endmodule
