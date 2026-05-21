`timescale 1ns/1ps

module tb_top_fdtd_hardware_wrapper;
    logic clk = 1'b0;
    logic rst = 1'b1;
    logic start = 1'b0;
    logic busy;
    logic done;
    logic [15:0] iteration_count;
    logic [15:0] ey_probe;
    logic [15:0] bz_probe;

    integer cycles;

    top_fdtd_hardware_wrapper dut (
        .clk(clk),
        .rst(rst),
        .start(start),
        .busy(busy),
        .done(done),
        .iteration_count(iteration_count),
        .ey_probe(ey_probe),
        .bz_probe(bz_probe)
    );

    always #5 clk = ~clk;

    initial begin
        $display("HW_WRAPPER_TB_START");

        repeat (5) @(posedge clk);
        rst = 1'b0;

        @(negedge clk);
        start = 1'b1;

        @(negedge clk);
        start = 1'b0;

        cycles = 0;
        while ((done !== 1'b1) && (cycles < 8000)) begin
            @(posedge clk);
            cycles = cycles + 1;
        end

        if (done !== 1'b1) begin
            $display("HW_WRAPPER_FAIL timeout busy=%b iter=%0d", busy, iteration_count);
            $finish;
        end

        repeat (2) @(posedge clk);

        if (iteration_count !== 16'd4) begin
            $display("HW_WRAPPER_FAIL expected 4 iterations, got %0d", iteration_count);
            $finish;
        end

        $display("HW_WRAPPER_SAMPLE ey_probe=%0d bz_probe=%0d iter=%0d",
                 $signed(ey_probe), $signed(bz_probe), iteration_count);
        $display("HW_WRAPPER_PASS");
        $display("HW_WRAPPER_TB_DONE");
        $finish;
    end
endmodule
