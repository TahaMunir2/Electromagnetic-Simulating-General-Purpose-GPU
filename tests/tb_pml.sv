`timescale 1ns/1ps

module tb_pml;

    localparam DATA_WIDTH = 16;
    localparam CELL_WIDTH = 6;
    localparam PML_SIZE   = 6;

    logic signed [CELL_WIDTH-1:0] d;
    logic signed [DATA_WIDTH-1:0] ca;
    logic signed [DATA_WIDTH-1:0] cb_e;
    logic signed [DATA_WIDTH-1:0] cb_bz;

    pml #(
        .DATA_WIDTH(DATA_WIDTH),
        .CELL_WIDTH(CELL_WIDTH),
        .PML_SIZE(PML_SIZE)
    ) dut (
        .d(d),
        .ca(ca),
        .cb_e(cb_e),
        .cb_bz(cb_bz)
    );

    int pass_count = 0;
    int fail_count = 0;

    logic signed [DATA_WIDTH-1:0] exp_ca    [0:5];
    logic signed [DATA_WIDTH-1:0] exp_cb_e  [0:5];
    logic signed [DATA_WIDTH-1:0] exp_cb_bz [0:5];

    task automatic check(
        input int            depth,
        input signed [15:0]  got_ca,
        input signed [15:0]  got_cb_e,
        input signed [15:0]  got_cb_bz,
        input signed [15:0]  exp_ca,
        input signed [15:0]  exp_cb_e,
        input signed [15:0]  exp_cb_bz
    );
        if (got_ca === exp_ca && got_cb_e === exp_cb_e && got_cb_bz === exp_cb_bz) begin
            $display("PASS  d=%0d  ca=%0d  cb_e=%0d  cb_bz=%0d", depth, got_ca, got_cb_e, got_cb_bz);
            pass_count++;
        end else begin
            $display("FAIL  d=%0d  ca=%0d(exp %0d)  cb_e=%0d(exp %0d)  cb_bz=%0d(exp %0d)",
                depth, got_ca, exp_ca, got_cb_e, exp_cb_e, got_cb_bz, exp_cb_bz);
            fail_count++;
        end
    endtask

    initial begin
        exp_ca[0]    =  16'sd8192; exp_cb_e[0]  = -16'sd717;  exp_cb_bz[0] = -16'sd2867;
        exp_ca[1]    =  16'sd7862; exp_cb_e[1]  = -16'sd703;  exp_cb_bz[1] = -16'sd2809;
        exp_ca[2]    =  16'sd6949; exp_cb_e[2]  = -16'sd663;  exp_cb_bz[2] = -16'sd2649;
        exp_ca[3]    =  16'sd5637; exp_cb_e[3]  = -16'sd605;  exp_cb_bz[3] = -16'sd2420;
        exp_ca[4]    =  16'sd4141; exp_cb_e[4]  = -16'sd540;  exp_cb_bz[4] = -16'sd2158;
        exp_ca[5]    =  16'sd2635; exp_cb_e[5]  = -16'sd474;  exp_cb_bz[5] = -16'sd1895;

        $display("=== tb_pml ===");

        for (int i = 0; i < PML_SIZE; i++) begin
            d = i;
            #1;
            check(i, ca, cb_e, cb_bz, exp_ca[i], exp_cb_e[i], exp_cb_bz[i]);
        end

        d = 6;
        #1;
        if (ca === 16'sd8192 && cb_e === -16'sd717 && cb_bz === -16'sd2867) begin
            $display("PASS  d=6 (default)  ca=%0d  cb_e=%0d  cb_bz=%0d", ca, cb_e, cb_bz);
            pass_count++;
        end else begin
            $display("FAIL  d=6 (default)  ca=%0d  cb_e=%0d  cb_bz=%0d", ca, cb_e, cb_bz);
            fail_count++;
        end

        $display("--- monotonicity ---");
        begin
            logic mono_ok;
            mono_ok = 1'b1;
            for (int i = 1; i < PML_SIZE; i++) begin
                if (!(exp_ca[i] < exp_ca[i-1])) begin
                    $display("FAIL  ca not decreasing at d=%0d", i);
                    mono_ok = 1'b0;
                    fail_count++;
                end
                if (!(exp_cb_e[i] > exp_cb_e[i-1])) begin
                    $display("FAIL  cb_e magnitude not decreasing at d=%0d", i);
                    mono_ok = 1'b0;
                    fail_count++;
                end
                if (!(exp_cb_bz[i] > exp_cb_bz[i-1])) begin
                    $display("FAIL  cb_bz magnitude not decreasing at d=%0d", i);
                    mono_ok = 1'b0;
                    fail_count++;
                end
            end
            if (mono_ok) begin
                $display("PASS  ca, cb_e, cb_bz all monotone with depth");
                pass_count++;
            end
        end

        $display("=== %0d passed, %0d failed ===", pass_count, fail_count);
        if (fail_count == 0) $display("ALL TESTS PASSED");
        else                 $display("TESTS FAILED");
        $finish;
    end

endmodule
