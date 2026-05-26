`timescale 1ns/1ps

module tb_fdtd_solver;

    localparam CELLS      = 192;
    localparam CELL_WIDTH = 8;
    localparam DATA_WIDTH = 16;
    localparam GRID       = CELLS * CELLS;

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

    integer row, col;
    integer cycles_taken;
    logic signed [DATA_WIDTH-1:0] ey_val;
    logic signed [DATA_WIDTH-1:0] pml_val;
    logic signed [DATA_WIDTH-1:0] int_val;

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
            $display("  FAIL");
            $finish;
        end
        $display("  PASS");

        $display("TEST 2: Ey source injection at (8,8)");
        ey_val = ey_mem[flat(8,8)];
        $display("  Ey[8][8] = %0d", $signed(ey_val));
        if (ey_val == '0) begin
            $display("  FAIL: Ey[8][8] still zero after source injection");
            $finish;
        end
        $display("  PASS");

        $display("TEST 3: Ey boundary rows 0 and %0d forced zero", CELLS-1);
        for (col = 0; col < CELLS; col++) begin
            if (ey_mem[flat(0, col)] !== '0) begin
                $display("  FAIL: Ey[0][%0d] = %0d", col, $signed(ey_mem[flat(0,col)]));
                $finish;
            end
            if (ey_mem[flat(CELLS-1, col)] !== '0) begin
                $display("  FAIL: Ey[%0d][%0d] = %0d", CELLS-1, col, $signed(ey_mem[flat(CELLS-1,col)]));
                $finish;
            end
        end
        $display("  PASS");

        $display("TEST 4: Ex boundary cols 0 and %0d forced zero", CELLS-1);
        for (row = 0; row < CELLS; row++) begin
            if (ex_mem[flat(row, 0)] !== '0) begin
                $display("  FAIL: Ex[%0d][0] = %0d", row, $signed(ex_mem[flat(row,0)]));
                $finish;
            end
            if (ex_mem[flat(row, CELLS-1)] !== '0) begin
                $display("  FAIL: Ex[%0d][%0d] = %0d", row, CELLS-1, $signed(ex_mem[flat(row,CELLS-1)]));
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

        $display("TEST 7: PML damping (uniform ey=8192, bz=0)");
        for (int i = 0; i < GRID; i++) begin
            ey_mem[i] = 16'sd8192;
            ex_mem[i] = '0;
            bz_mem[i] = '0;
        end

        rst = 1'b1;
        @(posedge clk);
        rst           = 1'b0;
        source_valid  = 1'b0;
        solver_enable = 1'b1;
        repeat (2) @(posedge clk);

        wait_done(3*GRID + 10);

        pml_val = ey_mem[flat(2, 96)];
        int_val = ey_mem[flat(10, 96)];
        $display("  Ey[2][96]  (PML row, d=3) = %0d", $signed(pml_val));
        $display("  Ey[10][96] (interior)     = %0d", $signed(int_val));

        if (int_val !== 16'sd8192) begin
            $display("  FAIL: interior cell modified (expected 8192, got %0d)", $signed(int_val));
            $finish;
        end
        if (int_val <= pml_val) begin
            $display("  FAIL: PML cell not smaller than interior");
            $finish;
        end
        if (ey_mem[flat(0, 96)] !== '0) begin
            $display("  FAIL: boundary row 0 not zero after PML run");
            $finish;
        end
        $display("  PASS");

        $display("ALL TESTS PASSED");
        $finish;
    end

endmodule
