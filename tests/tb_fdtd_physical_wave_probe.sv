`timescale 1ns/1ps

module tb_fdtd_physical_wave_probe;
    localparam CELLS      = 192;
    localparam CELL_WIDTH = 8;
    localparam DATA_WIDTH = 16;
    localparam ADDR_WIDTH = 2 * CELL_WIDTH;
    localparam GRID       = CELLS * CELLS;
    localparam SRC_ROW    = 96;
    localparam SRC_COL    = 96;
    localparam PROBE_NEAR = 100;
    localparam PROBE_MID  = 108;
    localparam PROBE_FAR  = 120;
    localparam SOURCE_ADDR = SRC_ROW * CELLS + SRC_COL;
    localparam FRAMES     = 18;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;

    reg solver_enable = 1'b0;
    reg mag_mode = 1'b0;
    reg [DATA_WIDTH-1:0] source_q313 = {DATA_WIDTH{1'b0}};
    reg source_valid = 1'b0;

    wire source_latched;
    wire [31:0] solver_checksum;
    wire solver_done;
    wire mag_busy;
    wire mag_done;

    wire [ADDR_WIDTH-1:0] ey_addra;
    wire ey_ena;
    wire [0:0] ey_wea;
    wire [DATA_WIDTH-1:0] ey_dina;
    reg  [DATA_WIDTH-1:0] ey_douta;
    wire [ADDR_WIDTH-1:0] ey_addrb;
    wire ey_enb;
    wire [0:0] ey_web;
    wire [DATA_WIDTH-1:0] ey_dinb;
    reg  [DATA_WIDTH-1:0] ey_doutb;

    wire [ADDR_WIDTH-1:0] ex_addra;
    wire ex_ena;
    wire [0:0] ex_wea;
    wire [DATA_WIDTH-1:0] ex_dina;
    reg  [DATA_WIDTH-1:0] ex_douta;
    wire [ADDR_WIDTH-1:0] ex_addrb;
    wire ex_enb;
    wire [0:0] ex_web;
    wire [DATA_WIDTH-1:0] ex_dinb;

    wire [ADDR_WIDTH-1:0] bz_addra;
    wire bz_ena;
    wire [0:0] bz_wea;
    wire [DATA_WIDTH-1:0] bz_dina;
    reg  [DATA_WIDTH-1:0] bz_douta;
    wire [ADDR_WIDTH-1:0] bz_addrb;
    wire bz_enb;
    wire [0:0] bz_web;
    wire [DATA_WIDTH-1:0] bz_dinb;
    reg  [DATA_WIDTH-1:0] bz_doutb;

    wire [ADDR_WIDTH-1:0] s_mag_addra;
    wire s_mag_ena;
    wire [0:0] s_mag_wea;
    wire [DATA_WIDTH-1:0] s_mag_dina;
    reg  [DATA_WIDTH-1:0] s_mag_douta;
    wire [ADDR_WIDTH-1:0] s_mag_addrb;
    wire s_mag_enb;
    wire [0:0] s_mag_web;
    wire [DATA_WIDTH-1:0] s_mag_dinb;
    reg  [DATA_WIDTH-1:0] s_mag_doutb;

    reg signed [DATA_WIDTH-1:0] ey_mem [0:GRID-1];
    reg signed [DATA_WIDTH-1:0] ex_mem [0:GRID-1];
    reg signed [DATA_WIDTH-1:0] bz_mem [0:GRID-1];
    reg        [DATA_WIDTH-1:0] s_mag_mem [0:GRID-1];

    integer i;
    integer frame;
    integer mag_writes_this_frame;
    integer total_cycles;
    integer first_near_frame;
    integer first_mid_frame;
    integer near_peak;
    integer mid_peak;
    integer far_peak;
    integer max_field_seen;
    reg [31:0] e_checksum;
    reg [31:0] s_checksum;
    reg [31:0] mag_checksum_running;

    fdtd_solver_bd_adapter #(
        .CELLS(CELLS),
        .CELL_WIDTH(CELL_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .SOURCE_ADDR(SOURCE_ADDR)
    ) dut (
        .clk(clk),
        .rst(rst),
        .solver_enable(solver_enable),
        .mag_mode(mag_mode),
        .source_q313(source_q313),
        .source_valid(source_valid),
        .source_latched(source_latched),
        .solver_checksum(solver_checksum),
        .solver_done(solver_done),
        .ey_addra(ey_addra),
        .ey_ena(ey_ena),
        .ey_wea(ey_wea),
        .ey_dina(ey_dina),
        .ey_douta(ey_douta),
        .ey_addrb(ey_addrb),
        .ey_enb(ey_enb),
        .ey_web(ey_web),
        .ey_dinb(ey_dinb),
        .ey_doutb(ey_doutb),
        .ex_addra(ex_addra),
        .ex_ena(ex_ena),
        .ex_wea(ex_wea),
        .ex_dina(ex_dina),
        .ex_douta(ex_douta),
        .ex_addrb(ex_addrb),
        .ex_enb(ex_enb),
        .ex_web(ex_web),
        .ex_dinb(ex_dinb),
        .bz_addra(bz_addra),
        .bz_ena(bz_ena),
        .bz_wea(bz_wea),
        .bz_dina(bz_dina),
        .bz_douta(bz_douta),
        .bz_addrb(bz_addrb),
        .bz_enb(bz_enb),
        .bz_web(bz_web),
        .bz_dinb(bz_dinb),
        .bz_doutb(bz_doutb),
        .s_mag_addra(s_mag_addra),
        .s_mag_ena(s_mag_ena),
        .s_mag_wea(s_mag_wea),
        .s_mag_dina(s_mag_dina),
        .s_mag_douta(s_mag_douta),
        .s_mag_addrb(s_mag_addrb),
        .s_mag_enb(s_mag_enb),
        .s_mag_web(s_mag_web),
        .s_mag_dinb(s_mag_dinb),
        .s_mag_doutb(s_mag_doutb),
        .mag_busy(mag_busy),
        .mag_done(mag_done)
    );

    function automatic [ADDR_WIDTH-1:0] flat(input integer row, input integer col);
        flat = row * CELLS + col;
    endfunction

    function automatic integer abs16(input signed [DATA_WIDTH-1:0] value);
        begin
            if (value < 0)
                abs16 = -value;
            else
                abs16 = value;
        end
    endfunction

    function automatic integer max3_abs_at(input integer addr);
        integer ey_abs;
        integer ex_abs;
        integer bz_abs;
        begin
            ey_abs = abs16(ey_mem[addr]);
            ex_abs = abs16(ex_mem[addr]);
            bz_abs = abs16(bz_mem[addr]);
            max3_abs_at = ey_abs;
            if (ex_abs > max3_abs_at) max3_abs_at = ex_abs;
            if (bz_abs > max3_abs_at) max3_abs_at = bz_abs;
        end
    endfunction

    task automatic clear_memories;
        begin
            for (i = 0; i < GRID; i = i + 1) begin
                ey_mem[i] = '0;
                ex_mem[i] = '0;
                bz_mem[i] = '0;
                s_mag_mem[i] = '0;
            end
        end
    endtask

    task automatic run_frame;
        input signed [DATA_WIDTH-1:0] source_value;
        input mode_value;
        integer timeout;
        begin
            solver_enable = 1'b0;
            source_q313 = source_value;
            source_valid = 1'b1;
            mag_mode = mode_value;
            mag_writes_this_frame = 0;
            mag_checksum_running = 32'd0;
            repeat (2) @(posedge clk);

            solver_enable = 1'b1;
            @(posedge clk);
            source_valid = 1'b0;

            timeout = 0;
            while (!solver_done && timeout < (3 * GRID + 100)) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (!solver_done) begin
                $display("FAIL: solver_done timeout");
                $finish;
            end

            timeout = 0;
            while (!mag_done && timeout < (GRID + 100)) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (!mag_done) begin
                $display("FAIL: mag_done timeout");
                $finish;
            end
            if (mag_writes_this_frame != GRID) begin
                $display("FAIL: magnitude pass wrote %0d cells, expected %0d", mag_writes_this_frame, GRID);
                $finish;
            end

            solver_enable = 1'b0;
            repeat (2) @(posedge clk);
        end
    endtask

    task automatic update_observations;
        integer near_now;
        integer mid_now;
        integer far_now;
        integer local_max;
        begin
            near_now = max3_abs_at(flat(SRC_ROW, PROBE_NEAR));
            mid_now  = max3_abs_at(flat(SRC_ROW, PROBE_MID));
            far_now  = max3_abs_at(flat(SRC_ROW, PROBE_FAR));

            if (near_now > near_peak) near_peak = near_now;
            if (mid_now > mid_peak) mid_peak = mid_now;
            if (far_now > far_peak) far_peak = far_now;

            if (near_now > 0 && first_near_frame < 0) first_near_frame = frame;
            if (mid_now > 0 && first_mid_frame < 0) first_mid_frame = frame;

            local_max = 0;
            for (i = 0; i < GRID; i = i + 1) begin
                if (max3_abs_at(i) > local_max) local_max = max3_abs_at(i);
            end
            if (local_max > max_field_seen) max_field_seen = local_max;

            $display(
                "FRAME %0d: center=%0d near=%0d mid=%0d far=%0d max=%0d mag_checksum=%08x",
                frame,
                max3_abs_at(flat(SRC_ROW, SRC_COL)),
                near_now,
                mid_now,
                far_now,
                local_max,
                mag_checksum_running
            );
        end
    endtask

    always_ff @(posedge clk) begin
        if (ey_ena) ey_douta <= ey_mem[ey_addra];
        if (ey_enb) ey_doutb <= ey_mem[ey_addrb];
        if (ey_wea[0]) ey_mem[ey_addra] <= ey_dina;
        if (ey_web[0]) ey_mem[ey_addrb] <= ey_dinb;

        if (ex_ena) ex_douta <= ex_mem[ex_addra];
        if (ex_wea[0]) ex_mem[ex_addra] <= ex_dina;
        if (ex_web[0]) ex_mem[ex_addrb] <= ex_dinb;

        if (bz_ena) bz_douta <= bz_mem[bz_addra];
        if (bz_enb) bz_doutb <= bz_mem[bz_addrb];
        if (bz_wea[0]) bz_mem[bz_addra] <= bz_dina;
        if (bz_web[0]) bz_mem[bz_addrb] <= bz_dinb;

        if (s_mag_ena) s_mag_douta <= s_mag_mem[s_mag_addra];
        if (s_mag_enb) s_mag_doutb <= s_mag_mem[s_mag_addrb];
        if (s_mag_wea[0]) begin
            s_mag_mem[s_mag_addra] <= s_mag_dina;
            mag_writes_this_frame <= mag_writes_this_frame + 1;
            mag_checksum_running <= {mag_checksum_running[30:0], mag_checksum_running[31]}
                ^ {s_mag_addra, s_mag_dina};
        end
        if (s_mag_web[0]) s_mag_mem[s_mag_addrb] <= s_mag_dinb;
    end

    initial begin
        clear_memories();
        first_near_frame = -1;
        first_mid_frame = -1;
        near_peak = 0;
        mid_peak = 0;
        far_peak = 0;
        max_field_seen = 0;
        total_cycles = 0;

        repeat (5) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        $display("PHYSICAL_WAVE_PROBE: impulse source, |E| render buffer");
        for (frame = 0; frame < FRAMES; frame = frame + 1) begin
            run_frame((frame == 0) ? 16'sd8192 : 16'sd0, 1'b0);
            update_observations();
        end
        e_checksum = mag_checksum_running;

        if (near_peak == 0) begin
            $display("FAIL: near probe never responded");
            $finish;
        end
        if (mid_peak == 0) begin
            $display("FAIL: mid probe never responded");
            $finish;
        end
        if (first_near_frame < 0 || first_mid_frame < 0 || first_near_frame > first_mid_frame) begin
            $display(
                "FAIL: probe arrival order invalid near=%0d mid=%0d",
                first_near_frame,
                first_mid_frame
            );
            $finish;
        end
        if (max_field_seen >= 30000) begin
            $display("FAIL: fixed-point field nearly saturated, max=%0d", max_field_seen);
            $finish;
        end

        clear_memories();
        repeat (5) @(posedge clk);
        rst = 1'b1;
        repeat (3) @(posedge clk);
        rst = 1'b0;
        repeat (5) @(posedge clk);

        $display("PHYSICAL_WAVE_PROBE: impulse source, |S| render buffer");
        for (frame = 0; frame < FRAMES; frame = frame + 1) begin
            run_frame((frame == 0) ? 16'sd8192 : 16'sd0, 1'b1);
        end
        s_checksum = mag_checksum_running;

        $display("SUMMARY: near_peak=%0d mid_peak=%0d far_peak=%0d max_field=%0d", near_peak, mid_peak, far_peak, max_field_seen);
        $display("SUMMARY: first_near_frame=%0d first_mid_frame=%0d", first_near_frame, first_mid_frame);
        $display("SUMMARY: e_checksum=%08x s_checksum=%08x", e_checksum, s_checksum);

        if (e_checksum == 32'd0) begin
            $display("FAIL: |E| magnitude checksum is zero");
            $finish;
        end
        if (s_checksum == 32'd0) begin
            $display("FAIL: |S| magnitude checksum is zero");
            $finish;
        end
        if (e_checksum == s_checksum) begin
            $display("FAIL: |E| and |S| magnitude checksums match unexpectedly");
            $finish;
        end

        $display("PASS: physical wave probe simulation shows propagation, stability, and distinct |E|/|S| buffers");
        $finish;
    end
endmodule
