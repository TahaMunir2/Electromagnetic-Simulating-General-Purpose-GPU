`timescale 1ns/1ps

module top_fdtd_system #(
    parameter CELLS = 64,
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 6
)(
    input  logic                         clk,
    input  logic                         rst,
    input  logic                         start,
    input  logic [15:0]                  num_iterations,
    input  logic [15:0]                  phase_step,
    input  logic [2*ADDR_WIDTH-1:0]      source_addr,
    input  logic [2*ADDR_WIDTH-1:0]      probe_addr,
    input  logic signed [DATA_WIDTH-1:0] C_E,
    input  logic signed [DATA_WIDTH-1:0] C_B,
    output logic                         busy,
    output logic                         done,
    output logic [3:0]                   state_debug,
    output logic [15:0]                  iteration_count,
    output logic [2*ADDR_WIDTH-1:0]      cell_debug,
    output logic signed [DATA_WIDTH-1:0] source_sample,
    output logic                         source_sample_valid,
    output logic signed [DATA_WIDTH-1:0] ey_probe,
    output logic signed [DATA_WIDTH-1:0] ex_probe,
    output logic signed [DATA_WIDTH-1:0] bz_probe
);

    localparam logic [2*ADDR_WIDTH-1:0] ADDR_ONE       = {{(2*ADDR_WIDTH-1){1'b0}}, 1'b1};
    localparam logic [2*ADDR_WIDTH-1:0] LAST_INIT_ADDR = {(2*ADDR_WIDTH){1'b1}};

    typedef enum logic [1:0] {
        TOP_IDLE,
        TOP_INIT,
        TOP_RUN,
        TOP_DONE
    } top_state_t;

    top_state_t top_state;

    logic        fsm_rst;
    logic        fsm_start;
    logic        fsm_done;
    logic        cordic_enable;
    logic        solver_enable;
    logic        cordic_done;
    logic        solver_done;

    logic [15:0] phase_acc;
    logic [15:0] cordic_phase_in;
    logic        cordic_phase_valid;
    logic [15:0] cordic_sin;
    logic [15:0] cordic_cos;
    logic        cordic_out_valid;
    logic        cordic_inflight;

    logic [2*ADDR_WIDTH-1:0] init_idx;

    logic [2*ADDR_WIDTH-1:0] ey_rd_addr_0, ey_rd_addr_1;
    logic [DATA_WIDTH-1:0]   ey_rd_data_0, ey_rd_data_1;
    logic [2*ADDR_WIDTH-1:0] ex_rd_addr_0, ex_rd_addr_1;
    logic [DATA_WIDTH-1:0]   ex_rd_data_0, ex_rd_data_1;
    logic [2*ADDR_WIDTH-1:0] bz_rd_addr_0, bz_rd_addr_1;
    logic [DATA_WIDTH-1:0]   bz_rd_data_0, bz_rd_data_1;
    logic                    ey_we;
    logic [2*ADDR_WIDTH-1:0] ey_wr_addr;
    logic [DATA_WIDTH-1:0]   ey_wr_data;
    logic                    ex_we;
    logic [2*ADDR_WIDTH-1:0] ex_wr_addr;
    logic [DATA_WIDTH-1:0]   ex_wr_data;
    logic                    bz_we;
    logic [2*ADDR_WIDTH-1:0] bz_wr_addr;
    logic [DATA_WIDTH-1:0]   bz_wr_data;

    logic [2*ADDR_WIDTH-1:0] solver_ey_rd_addr, solver_ey_wr_addr;
    logic [DATA_WIDTH-1:0]   solver_ey_wr_data;
    logic                    solver_ey_we;
    logic [2*ADDR_WIDTH-1:0] solver_ex_rd_addr, solver_ex_wr_addr;
    logic [DATA_WIDTH-1:0]   solver_ex_wr_data;
    logic                    solver_ex_we;
    logic [2*ADDR_WIDTH-1:0] solver_bz_rd_addr, solver_bz_wr_addr;
    logic [DATA_WIDTH-1:0]   solver_bz_wr_data;
    logic                    solver_bz_we;
    logic [2*ADDR_WIDTH-1:0] solver_ey_adj_rd_addr;
    logic [2*ADDR_WIDTH-1:0] solver_bz_adj_rd_addr;

    assign fsm_rst     = rst || (top_state != TOP_RUN);
    assign busy        = (top_state == TOP_INIT) || (top_state == TOP_RUN);
    assign done        = (top_state == TOP_DONE);
    assign state_debug = {2'b0, top_state};
    assign cell_debug  = '0;
    assign ey_probe    = $signed(ey_rd_data_1);
    assign ex_probe    = $signed(ex_rd_data_1);
    assign bz_probe    = $signed(bz_rd_data_1);

    always_comb begin
        ey_rd_addr_0 = solver_ey_rd_addr;
        ex_rd_addr_0 = solver_ex_rd_addr;
        bz_rd_addr_0 = solver_bz_rd_addr;

        if (top_state == TOP_RUN) begin
            ey_rd_addr_1 = solver_ey_adj_rd_addr;
            bz_rd_addr_1 = solver_bz_adj_rd_addr;
            ex_rd_addr_1 = '0;
        end else begin
            ey_rd_addr_1 = probe_addr;
            ex_rd_addr_1 = probe_addr;
            bz_rd_addr_1 = probe_addr;
        end

        if (top_state == TOP_INIT) begin
            ey_we      = 1'b1;
            ey_wr_addr = init_idx;
            ey_wr_data = {DATA_WIDTH{1'b0}};
            ex_we      = 1'b1;
            ex_wr_addr = init_idx;
            ex_wr_data = {DATA_WIDTH{1'b0}};
            bz_we      = 1'b1;
            bz_wr_addr = init_idx;
            bz_wr_data = {DATA_WIDTH{1'b0}};
        end else begin
            ey_we      = solver_ey_we;
            ey_wr_addr = solver_ey_wr_addr;
            ey_wr_data = solver_ey_wr_data;
            ex_we      = solver_ex_we;
            ex_wr_addr = solver_ex_wr_addr;
            ex_wr_data = solver_ex_wr_data;
            bz_we      = solver_bz_we;
            bz_wr_addr = solver_bz_wr_addr;
            bz_wr_data = solver_bz_wr_data;
        end
    end

    fsm_controller u_fsm (
        .clk(clk),
        .rst(fsm_rst),
        .start(fsm_start),
        .num_iterations(num_iterations),
        .cordic_done(cordic_done),
        .solver_done(solver_done),
        .cordic_enable(cordic_enable),
        .solver_enable(solver_enable),
        .fsm_done(fsm_done)
    );

    cordic_generator u_cordic (
        .clk(clk),
        .rst(rst),
        .phase_in(cordic_phase_in),
        .phase_valid(cordic_phase_valid),
        .sin_out(cordic_sin),
        .cos_out(cordic_cos),
        .out_valid(cordic_out_valid)
    );

    bram_module #(
        .DEPTH(CELLS*CELLS),
        .WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(2*ADDR_WIDTH)
    ) u_bram (
        .clk(clk),
        .rst(rst),
        .ey_rd_addr_0(ey_rd_addr_0),
        .ey_rd_data_0(ey_rd_data_0),
        .ey_rd_addr_1(ey_rd_addr_1),
        .ey_rd_data_1(ey_rd_data_1),
        .ex_rd_addr_0(ex_rd_addr_0),
        .ex_rd_data_0(ex_rd_data_0),
        .ex_rd_addr_1(ex_rd_addr_1),
        .ex_rd_data_1(ex_rd_data_1),
        .bz_rd_addr_0(bz_rd_addr_0),
        .bz_rd_data_0(bz_rd_data_0),
        .bz_rd_addr_1(bz_rd_addr_1),
        .bz_rd_data_1(bz_rd_data_1),
        .ey_we(ey_we),
        .ey_wr_addr(ey_wr_addr),
        .ey_wr_data(ey_wr_data),
        .ex_we(ex_we),
        .ex_wr_addr(ex_wr_addr),
        .ex_wr_data(ex_wr_data),
        .bz_we(bz_we),
        .bz_wr_addr(bz_wr_addr),
        .bz_wr_data(bz_wr_data)
    );

    fdtd_solver #(
        .CELLS(CELLS),
        .CELL_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_solver (
        .clk(clk),
        .rst(rst),
        .C_E(C_E),
        .C_B(C_B),
        .source_in(source_sample),
        .source_valid(source_sample_valid),
        .source_addr(source_addr),
        .ey_rd_addr(solver_ey_rd_addr),
        .ey_rd_dout(ey_rd_data_0),
        .ey_wr_addr(solver_ey_wr_addr),
        .ey_wr_data(solver_ey_wr_data),
        .ey_we(solver_ey_we),
        .ex_rd_addr(solver_ex_rd_addr),
        .ex_rd_dout(ex_rd_data_0),
        .ex_wr_addr(solver_ex_wr_addr),
        .ex_wr_data(solver_ex_wr_data),
        .ex_we(solver_ex_we),
        .bz_rd_addr(solver_bz_rd_addr),
        .bz_rd_dout(bz_rd_data_0),
        .bz_wr_addr(solver_bz_wr_addr),
        .bz_wr_data(solver_bz_wr_data),
        .bz_we(solver_bz_we),
        .bz_adj_rd_addr(solver_bz_adj_rd_addr),
        .bz_adj_dout(bz_rd_data_1),
        .ey_adj_rd_addr(solver_ey_adj_rd_addr),
        .ey_adj_dout(ey_rd_data_1),
        .solver_enable(solver_enable),
        .solver_done(solver_done)
    );

    always_ff @(posedge clk) begin
        if (rst) begin
            top_state       <= TOP_IDLE;
            fsm_start       <= 1'b0;
            init_idx        <= {(2*ADDR_WIDTH){1'b0}};
            iteration_count <= 16'd0;
        end else begin
            fsm_start <= 1'b0;
            if (solver_done) iteration_count <= iteration_count + 1'b1;

            case (top_state)
                TOP_IDLE: begin
                    iteration_count <= 16'd0;
                    if (start) top_state <= TOP_INIT;
                end

                TOP_INIT: begin
                    if (init_idx == LAST_INIT_ADDR) begin
                        init_idx  <= {ADDR_WIDTH{1'b0}};
                        fsm_start <= 1'b1;
                        top_state <= TOP_RUN;
                    end else begin
                        init_idx <= init_idx + ADDR_ONE;
                    end
                end

                TOP_RUN: begin
                    if (fsm_done) top_state <= TOP_DONE;
                end

                TOP_DONE: begin
                    if (start) begin
                        init_idx        <= {(2*ADDR_WIDTH){1'b0}};
                        iteration_count <= 16'd0;
                        top_state       <= TOP_INIT;
                    end
                end
            endcase
        end
    end

    always_ff @(posedge clk) begin
        if (rst || (top_state == TOP_IDLE) || (top_state == TOP_INIT)) begin
            phase_acc           <= 16'd0;
            cordic_phase_in     <= 16'd0;
            cordic_phase_valid  <= 1'b0;
            cordic_done         <= 1'b0;
            cordic_inflight     <= 1'b0;
            source_sample       <= {DATA_WIDTH{1'b0}};
            source_sample_valid <= 1'b0;
        end else begin
            cordic_phase_valid <= 1'b0;
            cordic_done        <= 1'b0;

            if (cordic_enable && !cordic_inflight && !cordic_done) begin
                cordic_phase_in     <= phase_acc + phase_step;
                cordic_phase_valid  <= 1'b1;
                phase_acc           <= phase_acc + phase_step;
                cordic_inflight     <= 1'b1;
                source_sample_valid <= 1'b0;
            end

            if (cordic_out_valid) begin
                source_sample       <= $signed(cordic_sin);
                source_sample_valid <= 1'b1;
                cordic_done         <= 1'b1;
                cordic_inflight     <= 1'b0;
            end
        end
    end

endmodule
