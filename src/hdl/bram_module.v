`timescale 1ns/1ps

/**
 * BRAM Module - Field Storage for 1D FDTD Solver
 *
 * Owner: Yi
 *
 * Stores Ey and Bz field values for a 64-cell 1D FDTD grid.
 * Each field has two synchronous read ports and one synchronous write port.
 *
 * This gives the FDTD datapath four read address inputs total:
 *   - Ey read port 0
 *   - Ey read port 1
 *   - Bz read port 0
 *   - Bz read port 1
 *
 * The implementation uses replicated memories so each field can support two
 * simultaneous reads while keeping one write interface.
 */

module bram_module #(
    parameter DEPTH = 64,
    parameter WIDTH = 16,
    parameter ADDR_WIDTH = 6
)(
    input  wire clk,
    input  wire rst,

    // Ey read ports
    input  wire [ADDR_WIDTH-1:0] ey_rd_addr_0,
    output wire [WIDTH-1:0]      ey_rd_data_0,
    input  wire [ADDR_WIDTH-1:0] ey_rd_addr_1,
    output wire [WIDTH-1:0]      ey_rd_data_1,

    // Bz read ports
    input  wire [ADDR_WIDTH-1:0] bz_rd_addr_0,
    output wire [WIDTH-1:0]      bz_rd_data_0,
    input  wire [ADDR_WIDTH-1:0] bz_rd_addr_1,
    output wire [WIDTH-1:0]      bz_rd_data_1,

    // Ey write port
    input  wire                  ey_we,
    input  wire [ADDR_WIDTH-1:0] ey_wr_addr,
    input  wire [WIDTH-1:0]      ey_wr_data,

    // Bz write port
    input  wire                  bz_we,
    input  wire [ADDR_WIDTH-1:0] bz_wr_addr,
    input  wire [WIDTH-1:0]      bz_wr_data
);

    (* ram_style = "block" *) reg [WIDTH-1:0] ey_mem_0 [0:DEPTH-1];
    (* ram_style = "block" *) reg [WIDTH-1:0] ey_mem_1 [0:DEPTH-1];
    (* ram_style = "block" *) reg [WIDTH-1:0] bz_mem_0 [0:DEPTH-1];
    (* ram_style = "block" *) reg [WIDTH-1:0] bz_mem_1 [0:DEPTH-1];

    reg [WIDTH-1:0] ey_rd_data_0_reg;
    reg [WIDTH-1:0] ey_rd_data_1_reg;
    reg [WIDTH-1:0] bz_rd_data_0_reg;
    reg [WIDTH-1:0] bz_rd_data_1_reg;

    integer init_idx;

    initial begin
        ey_rd_data_0_reg = {WIDTH{1'b0}};
        ey_rd_data_1_reg = {WIDTH{1'b0}};
        bz_rd_data_0_reg = {WIDTH{1'b0}};
        bz_rd_data_1_reg = {WIDTH{1'b0}};

        for (init_idx = 0; init_idx < DEPTH; init_idx = init_idx + 1) begin
            ey_mem_0[init_idx] = {WIDTH{1'b0}};
            ey_mem_1[init_idx] = {WIDTH{1'b0}};
            bz_mem_0[init_idx] = {WIDTH{1'b0}};
            bz_mem_1[init_idx] = {WIDTH{1'b0}};
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            ey_rd_data_0_reg <= {WIDTH{1'b0}};
            ey_rd_data_1_reg <= {WIDTH{1'b0}};
            bz_rd_data_0_reg <= {WIDTH{1'b0}};
            bz_rd_data_1_reg <= {WIDTH{1'b0}};
        end else begin
            if (ey_we) begin
                ey_mem_0[ey_wr_addr] <= ey_wr_data;
                ey_mem_1[ey_wr_addr] <= ey_wr_data;
            end

            if (bz_we) begin
                bz_mem_0[bz_wr_addr] <= bz_wr_data;
                bz_mem_1[bz_wr_addr] <= bz_wr_data;
            end

            if (ey_we && (ey_rd_addr_0 == ey_wr_addr)) begin
                ey_rd_data_0_reg <= ey_wr_data;
            end else begin
                ey_rd_data_0_reg <= ey_mem_0[ey_rd_addr_0];
            end

            if (ey_we && (ey_rd_addr_1 == ey_wr_addr)) begin
                ey_rd_data_1_reg <= ey_wr_data;
            end else begin
                ey_rd_data_1_reg <= ey_mem_1[ey_rd_addr_1];
            end

            if (bz_we && (bz_rd_addr_0 == bz_wr_addr)) begin
                bz_rd_data_0_reg <= bz_wr_data;
            end else begin
                bz_rd_data_0_reg <= bz_mem_0[bz_rd_addr_0];
            end

            if (bz_we && (bz_rd_addr_1 == bz_wr_addr)) begin
                bz_rd_data_1_reg <= bz_wr_data;
            end else begin
                bz_rd_data_1_reg <= bz_mem_1[bz_rd_addr_1];
            end
        end
    end

    assign ey_rd_data_0 = ey_rd_data_0_reg;
    assign ey_rd_data_1 = ey_rd_data_1_reg;
    assign bz_rd_data_0 = bz_rd_data_0_reg;
    assign bz_rd_data_1 = bz_rd_data_1_reg;

endmodule
