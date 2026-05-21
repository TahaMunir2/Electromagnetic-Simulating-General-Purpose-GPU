`timescale 1ns/1ps

module tb_top_fdtd_reference;
    localparam CELLS = 64;
    localparam DATA_WIDTH = 16;
    localparam ADDR_WIDTH = 6;
    parameter integer NUM_ITERATIONS_PARAM = 4;

    logic clk = 1'b0;
    logic rst = 1'b1;
    logic start = 1'b0;
    logic [15:0] num_iterations = NUM_ITERATIONS_PARAM[15:0];
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
    integer idx;
    integer ey_file;
    integer bz_file;
    integer dump_used_build_dir;

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

    task dump_memory;
        begin
            ey_file = $fopen("build/top_fdtd_reference_ey.txt", "w");
            bz_file = $fopen("build/top_fdtd_reference_bz.txt", "w");
            dump_used_build_dir = 1;

            if ((ey_file == 0) || (bz_file == 0)) begin
                if (ey_file != 0) begin
                    $fclose(ey_file);
                end

                if (bz_file != 0) begin
                    $fclose(bz_file);
                end

                ey_file = $fopen("top_fdtd_reference_ey.txt", "w");
                bz_file = $fopen("top_fdtd_reference_bz.txt", "w");
                dump_used_build_dir = 0;

                if ((ey_file == 0) || (bz_file == 0)) begin
                    $display("TOP_REFERENCE_FAIL could not open output dump files");
                    $finish;
                end
            end

            for (idx = 0; idx < CELLS; idx = idx + 1) begin
                $fwrite(ey_file, "%0d\n", $signed(dut.u_bram.ey_mem_0[idx]));
                $fwrite(bz_file, "%0d\n", $signed(dut.u_bram.bz_mem_0[idx]));
            end

            $fclose(ey_file);
            $fclose(bz_file);

            if (dump_used_build_dir) begin
                $display("TOP_REFERENCE_DUMP ey=build/top_fdtd_reference_ey.txt bz=build/top_fdtd_reference_bz.txt");
            end else begin
                $display("TOP_REFERENCE_DUMP ey=top_fdtd_reference_ey.txt bz=top_fdtd_reference_bz.txt");
            end
        end
    endtask

    initial begin
        $display("TOP_REFERENCE_TB_START");

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
            $display("TOP_REFERENCE_FAIL timeout state=%0d cell=%0d iter=%0d busy=%b source_valid=%b",
                     state_debug, cell_debug, iteration_count, busy, source_sample_valid);
            $finish;
        end

        repeat (2) @(posedge clk);

        if (iteration_count !== num_iterations) begin
            $display("TOP_REFERENCE_FAIL expected iterations=%0d actual=%0d",
                     num_iterations, iteration_count);
            $finish;
        end

        dump_memory();

        $display("TOP_REFERENCE_TB_DONE");
        $finish;
    end
endmodule
