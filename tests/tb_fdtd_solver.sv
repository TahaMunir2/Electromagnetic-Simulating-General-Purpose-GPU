`timescale 1ns/1ps

module tb_fdtd_solver;

    localparam CELLS      = 64;
    localparam CELL_WIDTH = 6;
    localparam DATA_WIDTH = 16;
    localparam FRAC_BITS  = 13;
    localparam GRID       = CELLS * CELLS;

    localparam signed [DATA_WIDTH-1:0] C_E = 16'sd717;
    localparam signed [DATA_WIDTH-1:0] C_B = 16'sd2867;

    logic clk = 1'b0;
    logic rst = 1'b1;
    always #5 clk = ~clk;

    logic signed [DATA_WIDTH-1:0] ey_mem [0:GRID-1];
    logic signed [DATA_WIDTH-1:0] ex_mem [0:GRID-1];
    logic signed [DATA_WIDTH-1:0] bz_mem [0:GRID-1];

    logic [2*CELL_WIDTH-1:0] ey_rd_addr, ey_wr_addr, ey_adj_rd_addr;
    logic [DATA_WIDTH-1:0]   ey_rd_dout, ey_wr_data, ey_adj_dout;
    logic                    ey_we;

    logic [2*CELL_WIDTH-1:0] ex_rd_addr, ex_wr_addr;
    logic [DATA_WIDTH-1:0]   ex_rd_dout, ex_wr_data;
    logic                    ex_we;

    logic [2*CELL_WIDTH-1:0] bz_rd_addr, bz_wr_addr, bz_adj_rd_addr;
    logic [DATA_WIDTH-1:0]   bz_rd_dout, bz_wr_data, bz_adj_dout;
    logic                    bz_we;

    logic solver_enable, solver_done;
    logic [DATA_WIDTH-1:0]   source_in;
    logic                    source_valid;
    logic [2*CELL_WIDTH-1:0] source_addr;

    always_ff @(posedge clk) begin
        if (ey_we) ey_mem[ey_wr_addr] <= ey_wr_data;
        if (ex_we) ex_mem[ex_wr_addr] <= ex_wr_data;
        if (bz_we) bz_mem[bz_wr_addr] <= bz_wr_data;

        ey_rd_dout  <= (ey_we && ey_rd_addr     == ey_wr_addr) ? ey_wr_data : ey_mem[ey_rd_addr];
        ey_adj_dout <= (ey_we && ey_adj_rd_addr == ey_wr_addr) ? ey_wr_data : ey_mem[ey_adj_rd_addr];
        ex_rd_dout  <= (ex_we && ex_rd_addr     == ex_wr_addr) ? ex_wr_data : ex_mem[ex_rd_addr];
        bz_rd_dout  <= (bz_we && bz_rd_addr     == bz_wr_addr) ? bz_wr_data : bz_mem[bz_rd_addr];
        bz_adj_dout <= (bz_we && bz_adj_rd_addr == bz_wr_addr) ? bz_wr_data : bz_mem[bz_adj_rd_addr];
    end

    fdtd_solver #(
        .CELLS(CELLS),
        .CELL_WIDTH(CELL_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .C_E(C_E),
        .C_B(C_B),
        .source_in(source_in),
        .source_valid(source_valid),
        .source_addr(source_addr),
        .ey_rd_addr(ey_rd_addr),
        .ey_rd_dout(ey_rd_dout),
        .ey_wr_addr(ey_wr_addr),
        .ey_wr_data(ey_wr_data),
        .ey_we(ey_we),
        .ex_rd_addr(ex_rd_addr),
        .ex_rd_dout(ex_rd_dout),
        .ex_wr_addr(ex_wr_addr),
        .ex_wr_data(ex_wr_data),
        .ex_we(ex_we),
        .bz_rd_addr(bz_rd_addr),
        .bz_rd_dout(bz_rd_dout),
        .bz_wr_addr(bz_wr_addr),
        .bz_wr_data(bz_wr_data),
        .bz_we(bz_we),
        .solver_enable(solver_enable),
        .solver_done(solver_done),
        .bz_adj_rd_addr(bz_adj_rd_addr),
        .bz_adj_dout(bz_adj_dout),
        .ey_adj_rd_addr(ey_adj_rd_addr),
        .ey_adj_dout(ey_adj_dout)
    );

    function automatic integer flat(input integer row, input integer col);
        flat = row * CELLS + col;
    endfunction

    task wait_done;
        input integer max_cycles;
        integer i;
        begin
            i = 0;
            while (!solver_done && i < max_cycles) begin
                @(posedge clk);
                i = i + 1;
            end
            if (!solver_done) begin
                $display("FAIL: solver_done never asserted (timeout %0d cycles)", max_cycles);
                $finish;
            end
        end
    endtask

    integer row, col, addr;
    integer cycles_taken;
    logic signed [DATA_WIDTH-1:0] ey_val;

    initial begin
        for (int i = 0; i < GRID; i++) begin
            ey_mem[i] = '0;
            ex_mem[i] = '0;
            bz_mem[i] = '0;
        end

        solver_enable = 1'b0;
        source_valid  = 1'b0;
        source_in     = '0;
        source_addr   = flat(8, 8);

        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (2) @(posedge clk);

        $display("TEST 1: solver_done timing");

        source_in     = 16'sd8192;
        source_valid  = 1'b1;
        solver_enable = 1'b1;

        cycles_taken = 0;
        @(posedge clk);
        while (!solver_done) begin
            @(posedge clk);
            cycles_taken = cycles_taken + 1;
        end
        cycles_taken = cycles_taken + 1;

        $display("  solver_done after %0d cycles (expected %0d)", cycles_taken, 3*GRID);
        if (cycles_taken !== 3*GRID) begin
            $display("  FAIL: wrong cycle count");
            $finish;
        end
        $display("  PASS");

        $display("TEST 2: Ey source injection at (8,8)");
        ey_val = ey_mem[flat(8,8)];
        $display("  Ey[8][8] = %0d (raw), expected ~8192", $signed(ey_val));
        if (ey_val == '0) begin
            $display("  FAIL: Ey[8][8] is still zero after source injection");
            $finish;
        end
        $display("  PASS");

        $display("TEST 3: Ey boundary (rows 0 and 63 forced zero)");
        for (col = 0; col < CELLS; col++) begin
            if (ey_mem[flat(0, col)] !== '0) begin
                $display("  FAIL: Ey[0][%0d] = %0d, expected 0", col, $signed(ey_mem[flat(0,col)]));
                $finish;
            end
            if (ey_mem[flat(CELLS-1, col)] !== '0) begin
                $display("  FAIL: Ey[63][%0d] = %0d, expected 0", col, $signed(ey_mem[flat(CELLS-1,col)]));
                $finish;
            end
        end
        $display("  PASS");

        $display("TEST 4: Ex boundary (cols 0 and 63 forced zero)");
        for (row = 0; row < CELLS; row++) begin
            if (ex_mem[flat(row, 0)] !== '0) begin
                $display("  FAIL: Ex[%0d][0] = %0d, expected 0", row, $signed(ex_mem[flat(row,0)]));
                $finish;
            end
            if (ex_mem[flat(row, CELLS-1)] !== '0) begin
                $display("  FAIL: Ex[%0d][63] = %0d, expected 0", row, $signed(ex_mem[flat(row,CELLS-1)]));
                $finish;
            end
        end
        $display("  PASS");

        $display("TEST 5: second iteration triggers solver_done again");
        solver_enable = 1'b0;
        @(posedge clk);
        solver_enable = 1'b1;
        source_valid  = 1'b1;

        cycles_taken = 0;
        while (!solver_done && cycles_taken < 3*GRID + 10) begin
            @(posedge clk);
            cycles_taken = cycles_taken + 1;
        end

        if (!solver_done) begin
            $display("  FAIL: solver_done did not fire (timeout after %0d cycles)", cycles_taken);
            $finish;
        end
        $display("  solver_done fired after %0d cycles", cycles_taken);
        $display("  PASS");

        $display("TEST 6: rst halts solver mid-run");
        solver_enable = 1'b1;
        repeat (100) @(posedge clk);
        rst = 1'b1;
        @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);
        solver_enable = 1'b0;
        repeat (10) @(posedge clk);
        if (ey_we || ex_we || bz_we) begin
            $display("  FAIL: write enable asserted while solver disabled after rst");
            $finish;
        end
        $display("  PASS");

        $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
