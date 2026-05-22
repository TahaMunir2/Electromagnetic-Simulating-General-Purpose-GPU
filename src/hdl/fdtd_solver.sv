`timescale 1ns/1ps

module fdtd_solver #(
    parameter CELLS = 64,
    parameter CELL_WIDTH = 6,
    parameter DATA_WIDTH = 16
)(
    input  wire clk,
    input  wire rst,
    input  wire signed [DATA_WIDTH-1:0] C_E,
    input  wire signed [DATA_WIDTH-1:0] C_B,
    input  wire [DATA_WIDTH-1:0] source_in,
    input  wire                  source_valid,
    input  wire [CELL_WIDTH-1:0] source_idx,
    output logic [CELL_WIDTH-1:0] ey_rd_addr,
    input  wire  [DATA_WIDTH-1:0] ey_rd_dout,
    output logic [CELL_WIDTH-1:0] ey_wr_addr,
    output logic [DATA_WIDTH-1:0] ey_wr_data,
    output logic                  ey_we,
    output logic [CELL_WIDTH-1:0] bz_rd_addr,
    input  wire  [DATA_WIDTH-1:0] bz_rd_dout,
    output logic [CELL_WIDTH-1:0] bz_wr_addr,
    output logic [DATA_WIDTH-1:0] bz_wr_data,
    output logic                  bz_we,
    input  wire solver_enable,
    output logic solver_done
);

    logic signed [DATA_WIDTH-1:0] engine_ey_old;
    logic signed [DATA_WIDTH-1:0] engine_bz_right;
    logic signed [DATA_WIDTH-1:0] engine_bz_old;
    logic signed [DATA_WIDTH-1:0] engine_ey_right;
    wire  signed [DATA_WIDTH-1:0] engine_ey_new;
    wire  signed [DATA_WIDTH-1:0] engine_bz_new;
    logic signed [DATA_WIDTH-1:0] prev_bz;
    logic signed [DATA_WIDTH-1:0] prev_ey;
    logic        [CELL_WIDTH:0]   counter;
    logic        [CELL_WIDTH-1:0] cell_idx;

    fdtd_engine #(.FP_WIDTH(DATA_WIDTH)) fdtd_engine (
        .clk(clk),
        .C_E(C_E),
        .C_B(C_B),
        .ey_old(engine_ey_old),
        .bz_left(prev_bz),
        .bz_right(engine_bz_right),
        .bz_old(engine_bz_old),
        .ey_left(prev_ey),
        .ey_right(engine_ey_right),
        .ey_new(engine_ey_new),
        .bz_new(engine_bz_new)
    );

always_comb begin
    cell_idx = counter[CELL_WIDTH-1:0];
    if (counter < CELLS) begin
        ey_rd_addr = cell_idx;
        bz_rd_addr = cell_idx + 1'b1;
    end else begin
        bz_rd_addr = cell_idx;
        ey_rd_addr = cell_idx + 1'b1;
    end

    engine_ey_old   = ey_rd_dout;
    engine_bz_right = bz_rd_dout;
    engine_ey_right = ey_rd_dout;
    engine_bz_old   = bz_rd_dout;
end

always_ff @(posedge clk) begin
    ey_we       <= 1'b0;
    bz_we       <= 1'b0;
    solver_done <= 1'b0;

    prev_bz <= bz_rd_dout;
    prev_ey <= ey_rd_dout;
    ey_wr_addr <= cell_idx - 3'd3;
    bz_wr_addr <= cell_idx - 3'd3;

    if (rst || !solver_enable) begin
        counter <= '0;
    end else begin
        if (counter == 2 * CELLS - 1) solver_done <= 1'b1;

        if (counter < CELLS) begin
            ey_we <= 1'b1;
            if (ey_wr_addr == 0 || ey_wr_addr == CELLS-1)
                ey_wr_data <= '0;
            else if (source_valid && ey_wr_addr == source_idx)
                ey_wr_data <= source_in;
            else
                ey_wr_data <= engine_ey_new;
        end else begin
            bz_we      <= 1'b1;
            bz_wr_data <= engine_bz_new;
        end

        counter <= counter + 1'b1;
    end
end

endmodule
