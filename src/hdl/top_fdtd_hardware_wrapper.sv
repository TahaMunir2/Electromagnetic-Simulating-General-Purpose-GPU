`timescale 1ns/1ps

module top_fdtd_hardware_wrapper (
    input  logic        clk,
    input  logic        rst,
    input  logic        start,
    output logic        busy,
    output logic        done,
    output logic [15:0] iteration_count,
    output logic [15:0] ey_probe,
    output logic [15:0] ex_probe,
    output logic [15:0] bz_probe
);

    localparam int CELLS = 64;
    localparam int DATA_WIDTH = 16;
    localparam int ADDR_WIDTH = 6;

    localparam logic [15:0] NUM_ITERATIONS = 16'd4;
    localparam logic [15:0] PHASE_STEP = 16'h4000;
    localparam logic [2*ADDR_WIDTH-1:0] SOURCE_ADDR = 12'd520;
    localparam logic [2*ADDR_WIDTH-1:0] PROBE_ADDR  = 12'd520;
    localparam logic signed [DATA_WIDTH-1:0] C_E = 16'sd717;
    localparam logic signed [DATA_WIDTH-1:0] C_B = 16'sd2867;

    logic [3:0] state_debug_unused;
    logic [2*ADDR_WIDTH-1:0] cell_debug_unused;
    logic signed [DATA_WIDTH-1:0] source_sample_unused;
    logic source_sample_valid_unused;
    logic signed [DATA_WIDTH-1:0] ey_probe_signed;
    logic signed [DATA_WIDTH-1:0] ex_probe_signed;
    logic signed [DATA_WIDTH-1:0] bz_probe_signed;

    top_fdtd_system #(
        .CELLS(CELLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_system (
        .clk(clk),
        .rst(rst),
        .start(start),
        .num_iterations(NUM_ITERATIONS),
        .phase_step(PHASE_STEP),
        .source_addr(SOURCE_ADDR),
        .probe_addr(PROBE_ADDR),
        .C_E(C_E),
        .C_B(C_B),
        .busy(busy),
        .done(done),
        .state_debug(state_debug_unused),
        .iteration_count(iteration_count),
        .cell_debug(cell_debug_unused),
        .source_sample(source_sample_unused),
        .source_sample_valid(source_sample_valid_unused),
        .ey_probe(ey_probe_signed),
        .ex_probe(ex_probe_signed),
        .bz_probe(bz_probe_signed)
    );

    assign ey_probe = ey_probe_signed;
    assign ex_probe = ex_probe_signed;
    assign bz_probe = bz_probe_signed;

endmodule
