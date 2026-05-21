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
    input  logic [ADDR_WIDTH-1:0]        source_addr,
    input  logic [ADDR_WIDTH-1:0]        probe_addr,
    input  logic signed [DATA_WIDTH-1:0] C_E,
    input  logic signed [DATA_WIDTH-1:0] C_B,
    output logic                         busy,
    output logic                         done,
    output logic [3:0]                   state_debug,
    output logic [15:0]                  iteration_count,
    output logic [ADDR_WIDTH-1:0]        cell_debug,
    output logic signed [DATA_WIDTH-1:0] source_sample,
    output logic                         source_sample_valid,
    output logic signed [DATA_WIDTH-1:0] ey_probe,
    output logic signed [DATA_WIDTH-1:0] bz_probe
);

    localparam logic [ADDR_WIDTH-1:0] ADDR_ONE          = {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
    localparam logic [ADDR_WIDTH-1:0] FIRST_ACTIVE_CELL = {{(ADDR_WIDTH-1){1'b0}}, 1'b1};
    localparam logic [ADDR_WIDTH-1:0] LAST_INIT_ADDR    = {ADDR_WIDTH{1'b1}};
    localparam logic [ADDR_WIDTH-1:0] LAST_ACTIVE_CELL  = LAST_INIT_ADDR - ADDR_ONE;

    typedef enum logic [1:0] {
        TOP_IDLE,
        TOP_INIT,
        TOP_RUN,
        TOP_DONE
    } top_state_t;

    typedef enum logic [2:0] {
        SOLVER_IDLE,
        SOLVER_READ,
        SOLVER_PIPE1,
        SOLVER_PIPE2,
        SOLVER_WRITE,
        SOLVER_DONE_WAIT
    } solver_state_t;

    typedef enum logic {
        FIELD_EY,
        FIELD_BZ
    } field_pass_t;

    top_state_t top_state;
    solver_state_t solver_state;
    field_pass_t field_pass;

    logic fsm_rst;
    logic fsm_start;
    logic fsm_done;
    logic cordic_enable;
    logic solver_enable;
    logic cordic_done;
    logic solver_done;

    logic [15:0] phase_acc;
    logic [15:0] cordic_phase_in;
    logic        cordic_phase_valid;
    logic [15:0] cordic_sin;
    logic [15:0] cordic_cos;
    logic        cordic_out_valid;
    logic        cordic_inflight;

    logic [ADDR_WIDTH-1:0] init_idx;
    logic [ADDR_WIDTH-1:0] cell_idx;

    logic [ADDR_WIDTH-1:0] ey_rd_addr_0;
    logic [ADDR_WIDTH-1:0] ey_rd_addr_1;
    logic [DATA_WIDTH-1:0] ey_rd_data_0;
    logic [DATA_WIDTH-1:0] ey_rd_data_1;
    logic [ADDR_WIDTH-1:0] bz_rd_addr_0;
    logic [ADDR_WIDTH-1:0] bz_rd_addr_1;
    logic [DATA_WIDTH-1:0] bz_rd_data_0;
    logic [DATA_WIDTH-1:0] bz_rd_data_1;
    logic                  ey_we;
    logic [ADDR_WIDTH-1:0] ey_wr_addr;
    logic [DATA_WIDTH-1:0] ey_wr_data;
    logic                  bz_we;
    logic [ADDR_WIDTH-1:0] bz_wr_addr;
    logic [DATA_WIDTH-1:0] bz_wr_data;

    logic signed [DATA_WIDTH-1:0] engine_ey_new;
    logic signed [DATA_WIDTH-1:0] engine_bz_new;
    logic signed [DATA_WIDTH-1:0] engine_ey_old;
    logic signed [DATA_WIDTH-1:0] engine_ey_right;
    logic signed [DATA_WIDTH-1:0] engine_bz_old;
    logic signed [DATA_WIDTH-1:0] engine_bz_left;
    logic signed [DATA_WIDTH-1:0] engine_bz_right;

    assign fsm_rst     = rst || (top_state != TOP_RUN);
    assign busy        = (top_state == TOP_INIT) || (top_state == TOP_RUN);
    assign done        = (top_state == TOP_DONE);
    assign state_debug = {top_state, solver_state[1:0]};
    assign cell_debug  = cell_idx;
    assign ey_probe    = $signed(ey_rd_data_0);
    assign bz_probe    = $signed(bz_rd_data_0);
    assign engine_ey_old   = ey_rd_data_0;
    assign engine_ey_right = ey_rd_data_1;
    assign engine_bz_old   = bz_rd_data_0;
    assign engine_bz_left  = bz_rd_data_0;
    assign engine_bz_right = bz_rd_data_1;

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
        .DEPTH(CELLS),
        .WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) u_bram (
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

    fdtd_engine #(
        .FP_WIDTH(DATA_WIDTH)
    ) u_engine (
        .clk(clk),
        .C_E(C_E),
        .C_B(C_B),
        .ey_old(engine_ey_old),
        .bz_left(engine_bz_left),
        .bz_right(engine_bz_right),
        .bz_old(engine_bz_old),
        .ey_left(engine_ey_old),
        .ey_right(engine_ey_right),
        .ey_new(engine_ey_new),
        .bz_new(engine_bz_new)
    );

    always_comb begin
        ey_rd_addr_0 = probe_addr;
        ey_rd_addr_1 = source_addr;
        bz_rd_addr_0 = probe_addr;
        bz_rd_addr_1 = source_addr;

        ey_we      = 1'b0;
        ey_wr_addr = init_idx;
        ey_wr_data = {DATA_WIDTH{1'b0}};
        bz_we      = 1'b0;
        bz_wr_addr = init_idx;
        bz_wr_data = {DATA_WIDTH{1'b0}};

        if ((solver_state == SOLVER_READ) ||
            (solver_state == SOLVER_PIPE1) ||
            (solver_state == SOLVER_PIPE2) ||
            (solver_state == SOLVER_WRITE)) begin
            ey_rd_addr_0 = cell_idx;
            ey_rd_addr_1 = cell_idx + ADDR_ONE;
            bz_rd_addr_0 = cell_idx;
            bz_rd_addr_1 = cell_idx + ADDR_ONE;
        end

        case (top_state)
            TOP_INIT: begin
                ey_we      = 1'b1;
                ey_wr_addr = init_idx;
                ey_wr_data = {DATA_WIDTH{1'b0}};
                bz_we      = 1'b1;
                bz_wr_addr = init_idx;
                bz_wr_data = {DATA_WIDTH{1'b0}};
            end

            default: begin
                if (solver_state == SOLVER_WRITE) begin
                    if (field_pass == FIELD_EY) begin
                        ey_we      = 1'b1;
                        ey_wr_addr = cell_idx;
                        ey_wr_data = (source_sample_valid && (cell_idx == source_addr))
                                   ? source_sample
                                   : engine_ey_new;
                    end else begin
                        bz_we      = 1'b1;
                        bz_wr_addr = cell_idx;
                        bz_wr_data = engine_bz_new;
                    end
                end
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            top_state       <= TOP_IDLE;
            fsm_start       <= 1'b0;
            init_idx        <= {ADDR_WIDTH{1'b0}};
            iteration_count <= 16'd0;
        end else begin
            fsm_start <= 1'b0;

            case (top_state)
                TOP_IDLE: begin
                    init_idx        <= {ADDR_WIDTH{1'b0}};
                    if (start) begin
                        iteration_count <= 16'd0;
                        top_state <= TOP_INIT;
                    end
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
                    if (fsm_done) begin
                        top_state <= TOP_DONE;
                    end
                end

                TOP_DONE: begin
                    if (start) begin
                        init_idx        <= {ADDR_WIDTH{1'b0}};
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

    always_ff @(posedge clk) begin
        if (rst || !solver_enable) begin
            solver_state <= SOLVER_IDLE;
            solver_done  <= 1'b0;
            cell_idx     <= FIRST_ACTIVE_CELL;
            field_pass   <= FIELD_EY;
        end else begin
            solver_done <= 1'b0;

            case (solver_state)
                SOLVER_IDLE: begin
                    cell_idx     <= FIRST_ACTIVE_CELL;
                    field_pass   <= FIELD_EY;
                    solver_state <= SOLVER_READ;
                end

                SOLVER_READ: begin
                    solver_state <= SOLVER_PIPE1;
                end

                SOLVER_PIPE1: begin
                    solver_state <= SOLVER_PIPE2;
                end

                SOLVER_PIPE2: begin
                    solver_state <= SOLVER_WRITE;
                end

                SOLVER_WRITE: begin
                    if (cell_idx == LAST_ACTIVE_CELL) begin
                        if (field_pass == FIELD_EY) begin
                            field_pass   <= FIELD_BZ;
                            cell_idx     <= FIRST_ACTIVE_CELL;
                            solver_state <= SOLVER_READ;
                        end else begin
                            iteration_count <= iteration_count + 16'd1;
                            solver_done     <= 1'b1;
                            solver_state    <= SOLVER_DONE_WAIT;
                        end
                    end else begin
                        cell_idx     <= cell_idx + ADDR_ONE;
                        solver_state <= SOLVER_READ;
                    end
                end

                SOLVER_DONE_WAIT: begin
                    solver_state <= SOLVER_DONE_WAIT;
                end

                default: begin
                    solver_state <= SOLVER_IDLE;
                end
            endcase
        end
    end

endmodule
