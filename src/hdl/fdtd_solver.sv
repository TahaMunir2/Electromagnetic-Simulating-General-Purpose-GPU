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
    input  wire [2*CELL_WIDTH-1:0]  source_addr,
    output logic [2*CELL_WIDTH-1:0] ey_rd_addr,
    input  wire  [DATA_WIDTH-1:0]   ey_rd_dout,
    output logic [2*CELL_WIDTH-1:0] ey_wr_addr,
    output logic [DATA_WIDTH-1:0]   ey_wr_data,
    output logic                    ey_we,
    output logic [2*CELL_WIDTH-1:0] ex_rd_addr,
    input  wire  [DATA_WIDTH-1:0]   ex_rd_dout,
    output logic [2*CELL_WIDTH-1:0] ex_wr_addr,
    output logic [DATA_WIDTH-1:0]   ex_wr_data,
    output logic                    ex_we,
    output logic [2*CELL_WIDTH-1:0] bz_rd_addr,
    input  wire  [DATA_WIDTH-1:0]   bz_rd_dout,
    output logic [2*CELL_WIDTH-1:0] bz_wr_addr,
    output logic [DATA_WIDTH-1:0]   bz_wr_data,
    output logic                    bz_we,
    input  wire solver_enable,
    output logic solver_done,
    output logic [2*CELL_WIDTH-1:0] bz_adj_rd_addr,
    input  wire  [DATA_WIDTH-1:0]   bz_adj_dout,
    output logic [2*CELL_WIDTH-1:0] ey_adj_rd_addr,
    input  wire  [DATA_WIDTH-1:0]   ey_adj_dout
);

    logic signed [DATA_WIDTH-1:0] engine_ey_old;
    logic signed [DATA_WIDTH-1:0] engine_ex_old;
    logic signed [DATA_WIDTH-1:0] engine_bz_right;
    logic signed [DATA_WIDTH-1:0] engine_bz_old;
    logic signed [DATA_WIDTH-1:0] engine_ey_right;
    logic signed [DATA_WIDTH-1:0] engine_ex_right;
    logic signed [DATA_WIDTH-1:0] engine_ey_left;
    wire  signed [DATA_WIDTH-1:0] engine_ey_new;
    wire  signed [DATA_WIDTH-1:0] engine_ex_new;
    wire  signed [DATA_WIDTH-1:0] engine_bz_new;
    logic signed [DATA_WIDTH-1:0] prev_bz;
    logic signed [DATA_WIDTH-1:0] prev_ey;
    logic signed [DATA_WIDTH-1:0] prev_ex;
    logic        [2*CELL_WIDTH+1:0] counter;
    logic        [2*CELL_WIDTH-1:0] cell_addr;
    wire         [2*CELL_WIDTH-1:0] wr_cell = cell_addr - 3'd3;
    localparam   [2*CELL_WIDTH:0] grid_size = CELLS*CELLS;
    logic        [CELL_WIDTH-1:0] row;
    logic        [CELL_WIDTH-1:0] column;
    logic signed [DATA_WIDTH-1:0] engine_bz_left;

    

    fdtd_engine #(.FP_WIDTH(DATA_WIDTH)) fdtd_engine (
        .clk(clk),
        .C_E(C_E),
        .C_B(C_B),
        .ey_old(engine_ey_old),
        .ex_old(engine_ex_old),
        .bz_left(engine_bz_left),
        .bz_right(engine_bz_right),
        .bz_old(engine_bz_old),
        .ey_left(engine_ey_left),
        .ey_right(engine_ey_right),
        .ex_left(prev_ex),
        .ex_right(engine_ex_right),
        .ex_new(engine_ex_new),
        .ey_new(engine_ey_new),
        .bz_new(engine_bz_new)
    );

always_comb begin
    cell_addr = counter[2*CELL_WIDTH-1:0];
    row = counter[2*CELL_WIDTH-1:CELL_WIDTH];
    column = counter[CELL_WIDTH-1:0];
    bz_adj_rd_addr  = '0;
    ey_adj_rd_addr  = '0;
    ey_rd_addr      = '0;
    ex_rd_addr      = '0;
    engine_ey_left  = prev_ey;
    engine_ey_right = ey_rd_dout;

    if(counter < grid_size) begin
        bz_adj_rd_addr = cell_addr - CELLS; 
        ey_rd_addr = cell_addr;
        bz_rd_addr = cell_addr;
        engine_bz_left = bz_adj_dout;
    end else if (counter < 2 * grid_size) begin
        ex_rd_addr = cell_addr;
        engine_bz_left = prev_bz;
        bz_rd_addr = cell_addr;
    end else begin
        bz_rd_addr = cell_addr;
        ey_rd_addr = cell_addr;
        ex_rd_addr = cell_addr + 1'b1;
        ey_adj_rd_addr = cell_addr + CELLS; 
        engine_bz_left = prev_bz;
        engine_ey_right = ey_adj_dout;
        engine_ey_left  = ey_rd_dout;
    end

    engine_ey_old   = ey_rd_dout;
    engine_ex_old   = ex_rd_dout;
    engine_bz_right = bz_rd_dout;
    engine_ex_right = ex_rd_dout;
    engine_bz_old   = bz_rd_dout;
end

always_ff @(posedge clk) begin
    ey_we       <= 1'b0;
    ex_we       <= 1'b0;
    bz_we       <= 1'b0;
    solver_done <= 1'b0;

    prev_bz <= bz_rd_dout;
    prev_ey <= ey_rd_dout;
    prev_ex <= ex_rd_dout;
    ey_wr_addr <= wr_cell;
    ex_wr_addr <= wr_cell;
    bz_wr_addr <= wr_cell;

    if (rst || !solver_enable) begin
        counter <= '0;
    end else begin
        if (counter == 3 * grid_size - 1) solver_done <= 1'b1;

        if (counter < grid_size) begin
            ey_we <= 1'b1;
            if (row == 0 || row == CELLS-1) ey_wr_data <= '0;
            else if (source_valid && wr_cell == source_addr) ey_wr_data <= source_in;
            else ey_wr_data <= engine_ey_new;
        end
        
        else if (counter < 2 * grid_size) begin
            ex_we <= 1'b1;
            if (column == 0 || column == CELLS-1) ex_wr_data <= '0;
            else ex_wr_data <= engine_ex_new;          

        end
        else begin
            bz_we      <= 1'b1;
            bz_wr_data <= engine_bz_new;
        end
        counter <= counter + 1'b1;


    end
end

endmodule