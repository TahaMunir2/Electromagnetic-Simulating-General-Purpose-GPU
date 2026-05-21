`timescale 1ns/1ps

module tb_bram_module;
    localparam DEPTH = 64;
    localparam WIDTH = 16;
    localparam ADDR_WIDTH = 6;

    reg clk = 1'b0;
    reg rst = 1'b1;

    reg [ADDR_WIDTH-1:0] ey_rd_addr_0 = {ADDR_WIDTH{1'b0}};
    reg [ADDR_WIDTH-1:0] ey_rd_addr_1 = {ADDR_WIDTH{1'b0}};
    wire [WIDTH-1:0]     ey_rd_data_0;
    wire [WIDTH-1:0]     ey_rd_data_1;

    reg [ADDR_WIDTH-1:0] bz_rd_addr_0 = {ADDR_WIDTH{1'b0}};
    reg [ADDR_WIDTH-1:0] bz_rd_addr_1 = {ADDR_WIDTH{1'b0}};
    wire [WIDTH-1:0]     bz_rd_data_0;
    wire [WIDTH-1:0]     bz_rd_data_1;

    reg                  ey_we = 1'b0;
    reg [ADDR_WIDTH-1:0] ey_wr_addr = {ADDR_WIDTH{1'b0}};
    reg [WIDTH-1:0]      ey_wr_data = {WIDTH{1'b0}};

    reg                  bz_we = 1'b0;
    reg [ADDR_WIDTH-1:0] bz_wr_addr = {ADDR_WIDTH{1'b0}};
    reg [WIDTH-1:0]      bz_wr_data = {WIDTH{1'b0}};

    bram_module #(
        .DEPTH(DEPTH),
        .WIDTH(WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .ey_rd_addr_0(ey_rd_addr_0),
        .ey_rd_data_0(ey_rd_data_0),
        .ey_rd_addr_1(ey_rd_addr_1),
        .ey_rd_data_1(ey_rd_data_1),
        .bz_rd_addr_0(bz_rd_addr_0),
        .bz_rd_data_0(bz_rd_data_0),
        .bz_rd_addr_1(bz_rd_addr_1),
        .bz_rd_data_1(bz_rd_data_1),
        .ey_we(ey_we),
        .ey_wr_addr(ey_wr_addr),
        .ey_wr_data(ey_wr_data),
        .bz_we(bz_we),
        .bz_wr_addr(bz_wr_addr),
        .bz_wr_data(bz_wr_data)
    );

    always #5 clk = ~clk;

    task check_word;
        input [WIDTH-1:0] actual;
        input [WIDTH-1:0] expected;
        input [8*40-1:0] label;
        begin
            if (actual !== expected) begin
                $display("BRAM_FAIL %0s expected=%h actual=%h", label, expected, actual);
                $finish;
            end
        end
    endtask

    initial begin
        $dumpfile("bram_module.vcd");
        $dumpvars(0, tb_bram_module);

        repeat (2) @(posedge clk);
        rst = 1'b0;

        @(negedge clk);
        ey_we = 1'b1;
        ey_wr_addr = 6'd3;
        ey_wr_data = 16'h1111;
        bz_we = 1'b1;
        bz_wr_addr = 6'd5;
        bz_wr_data = 16'haaaa;

        @(posedge clk);
        #1;

        @(negedge clk);
        ey_wr_addr = 6'd4;
        ey_wr_data = 16'h2222;
        bz_wr_addr = 6'd6;
        bz_wr_data = 16'hbbbb;

        @(posedge clk);
        #1;

        @(negedge clk);
        ey_we = 1'b0;
        bz_we = 1'b0;
        ey_rd_addr_0 = 6'd3;
        ey_rd_addr_1 = 6'd4;
        bz_rd_addr_0 = 6'd5;
        bz_rd_addr_1 = 6'd6;

        @(posedge clk);
        #1;
        check_word(ey_rd_data_0, 16'h1111, "ey read port 0");
        check_word(ey_rd_data_1, 16'h2222, "ey read port 1");
        check_word(bz_rd_data_0, 16'haaaa, "bz read port 0");
        check_word(bz_rd_data_1, 16'hbbbb, "bz read port 1");

        @(negedge clk);
        ey_rd_addr_0 = 6'd7;
        ey_rd_addr_1 = 6'd7;
        ey_we = 1'b1;
        ey_wr_addr = 6'd7;
        ey_wr_data = 16'h3333;
        bz_rd_addr_0 = 6'd8;
        bz_rd_addr_1 = 6'd8;
        bz_we = 1'b1;
        bz_wr_addr = 6'd8;
        bz_wr_data = 16'hcccc;

        @(posedge clk);
        #1;
        check_word(ey_rd_data_0, 16'h3333, "ey same-cycle bypass port 0");
        check_word(ey_rd_data_1, 16'h3333, "ey same-cycle bypass port 1");
        check_word(bz_rd_data_0, 16'hcccc, "bz same-cycle bypass port 0");
        check_word(bz_rd_data_1, 16'hcccc, "bz same-cycle bypass port 1");

        @(negedge clk);
        ey_we = 1'b0;
        bz_we = 1'b0;
        ey_rd_addr_0 = 6'd10;
        ey_rd_addr_1 = 6'd11;
        bz_rd_addr_0 = 6'd12;
        bz_rd_addr_1 = 6'd13;

        @(posedge clk);
        #1;
        check_word(ey_rd_data_0, 16'h0000, "ey zero init port 0");
        check_word(ey_rd_data_1, 16'h0000, "ey zero init port 1");
        check_word(bz_rd_data_0, 16'h0000, "bz zero init port 0");
        check_word(bz_rd_data_1, 16'h0000, "bz zero init port 1");

        $display("BRAM_PASS");
        $finish;
    end
endmodule
