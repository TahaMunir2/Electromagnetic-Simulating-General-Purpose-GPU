`timescale 1ns/1ps

module fdtd_engine #(
    parameter FP_WIDTH = 16
)(
    input  logic                       clk,
    input  logic signed [FP_WIDTH-1:0] ca_ey,
    input  logic signed [FP_WIDTH-1:0] cb_ey,
    input  logic signed [FP_WIDTH-1:0] ca_ex,
    input  logic signed [FP_WIDTH-1:0] cb_ex,
    input  logic signed [FP_WIDTH-1:0] ca_bz,
    input  logic signed [FP_WIDTH-1:0] cb_bz,
    input  logic signed [FP_WIDTH-1:0] ey_old,
    input  logic signed [FP_WIDTH-1:0] ex_old,
    input  logic signed [FP_WIDTH-1:0] bz_left_ey,
    input  logic signed [FP_WIDTH-1:0] bz_left_ex,
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
        .ca(ca_bz),
        .cb(cb_bz),
        .bz_old(bz_old),
        .ey_left(ey_left),
        .ey_right(ey_right),
        .ex_left(ex_left),
        .ex_right(ex_right),
        .bz_new(bz_new)
    );

    ey #(.FP_WIDTH(FP_WIDTH)) u_ey (
        .clk(clk),
        .ca(ca_ey),
        .cb(cb_ey),
        .ey_old(ey_old),
        .bz_left(bz_left_ey),
        .bz_right(bz_right),
        .ey_new(ey_new)
    );

    ex #(.FP_WIDTH(FP_WIDTH)) u_ex (
        .clk(clk),
        .ca(ca_ex),
        .cb(cb_ex),
        .ex_old(ex_old),
        .bz_left(bz_left_ex),
        .bz_right(bz_right),
        .ex_new(ex_new)
    );

endmodule
