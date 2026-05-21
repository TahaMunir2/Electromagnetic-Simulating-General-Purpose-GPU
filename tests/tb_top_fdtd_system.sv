`timescale 1ns/1ps

module tb_top_fdtd_system;
    localparam CELLS = 64;
    localparam DATA_WIDTH = 16;
    localparam ADDR_WIDTH = 6;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic start = 1'b0;
    logic [15:0] num_iterations = 16'd1;
    logic [15:0] phase_step = 16'h4000;
    logic [ADDR_WIDTH-1:0] source_addr = 6'd8;
    logic [ADDR_WIDTH-1:0] probe_addr = 6'd8;
    logic signed [DATA_WIDTH-1:0] C_E = 16'sd717;
    logic signed [DATA_WIDTH-1:0] C_B = 16'sd2867;

    logic busy;
    logic done;
    logic [3:0] state_debug;
    logic [15:0] iteration_count;
    logic [ADDR_WIDTH-1:0] cell_debug;
    logic signed [DATA_WIDTH-1:0] source_sample;
    logic source_sample_valid;
    logic signed [DATA_WIDTH-1:0] ey_probe;
    logic signed [DATA_WIDTH-1:0] bz_probe;
    integer cycles;

    top_fdtd_system #(
        .CELLS(CELLS),
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .num_iterations(num_iterations),
        .phase_step(phase_step),
        .source_addr(source_addr),
        .probe_addr(probe_addr),
        .C_E(C_E),
        .C_B(C_B),
        .busy(busy),
        .done(done),
        .state_debug(state_debug),
        .iteration_count(iteration_count),
        .cell_debug(cell_debug),
        .source_sample(source_sample),
        .source_sample_valid(source_sample_valid),
        .ey_probe(ey_probe),
        .bz_probe(bz_probe)
    );

    always #5 clk = ~clk;

    function integer abs_int;
        input integer value;
        begin
            abs_int = (value < 0) ? -value : value;
        end
    endfunction

    task check_close;
        input signed [DATA_WIDTH-1:0] actual;
        input signed [DATA_WIDTH-1:0] expected;
        input integer tolerance;
        input [8*40-1:0] label;
        integer diff;
        begin
            diff = actual - expected;
            if (abs_int(diff) > tolerance) begin
                $display("TOP_FAIL %0s expected=%0d actual=%0d diff=%0d",
                         label, expected, actual, diff);
                $finish;
            end
        end
    endtask

    initial begin
        $display("TOP_TB_START");

        repeat (5) @(posedge clk);
        rst = 1'b0;

        @(negedge clk);
        start = 1'b1;

        @(negedge clk);
        start = 1'b0;

        cycles = 0;
        while ((done !== 1'b1) && (cycles < 1200)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end

        if (done !== 1'b1) begin
            $display("TOP_FAIL timeout state=%0d cell=%0d iter=%0d busy=%b source_valid=%b",
                     state_debug, cell_debug, iteration_count, busy, source_sample_valid);
            $finish;
        end

        repeat (2) @(posedge clk);

        if (iteration_count !== 16'd1) begin
            $display("TOP_FAIL expected one completed iteration, got %0d", iteration_count);
            $finish;
        end

        if (source_sample_valid !== 1'b1) begin
            $display("TOP_FAIL source sample was not marked valid");
            $finish;
        end

        check_close(source_sample, 16'sd8192, 16, "source sample");
        check_close(ey_probe, 16'sd8192, 16, "Ey source cell");

        $display("TOP_SAMPLE source=%0d ey_probe=%0d bz_probe=%0d iter=%0d state=%0d cell=%0d",
                 source_sample, ey_probe, bz_probe, iteration_count, state_debug, cell_debug);
        $display("TOP_PASS");
        $display("TOP_TB_DONE");
        $finish;
    end
endmodule
