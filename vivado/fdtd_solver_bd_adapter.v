`timescale 1ns/1ps

module fdtd_solver_bd_adapter #(
    parameter CELLS = 192,
    parameter CELL_WIDTH = 8,
    parameter DATA_WIDTH = 16,
    parameter [2*CELL_WIDTH-1:0] SOURCE_ADDR = 16'd18528
)(
    input  wire clk,
    input  wire rst,
    input  wire solver_enable,
    input  wire mag_mode,

    input  wire [DATA_WIDTH-1:0] source_q313,
    input  wire                  source_valid,
    output wire                  source_latched,
    output wire [31:0]           solver_checksum,
    output wire                  solver_done,

    output wire [2*CELL_WIDTH-1:0] ey_addra,
    output wire                    ey_ena,
    output wire [0:0]              ey_wea,
    output wire [DATA_WIDTH-1:0]   ey_dina,
    input  wire [DATA_WIDTH-1:0]   ey_douta,
    output wire [2*CELL_WIDTH-1:0] ey_addrb,
    output wire                    ey_enb,
    output wire [0:0]              ey_web,
    output wire [DATA_WIDTH-1:0]   ey_dinb,
    input  wire [DATA_WIDTH-1:0]   ey_doutb,

    output wire [2*CELL_WIDTH-1:0] ex_addra,
    output wire                    ex_ena,
    output wire [0:0]              ex_wea,
    output wire [DATA_WIDTH-1:0]   ex_dina,
    input  wire [DATA_WIDTH-1:0]   ex_douta,
    output wire [2*CELL_WIDTH-1:0] ex_addrb,
    output wire                    ex_enb,
    output wire [0:0]              ex_web,
    output wire [DATA_WIDTH-1:0]   ex_dinb,

    output wire [2*CELL_WIDTH-1:0] bz_addra,
    output wire                    bz_ena,
    output wire [0:0]              bz_wea,
    output wire [DATA_WIDTH-1:0]   bz_dina,
    input  wire [DATA_WIDTH-1:0]   bz_douta,
    output wire [2*CELL_WIDTH-1:0] bz_addrb,
    output wire                    bz_enb,
    output wire [0:0]              bz_web,
    output wire [DATA_WIDTH-1:0]   bz_dinb,
    input  wire [DATA_WIDTH-1:0]   bz_doutb,

    output wire [2*CELL_WIDTH-1:0] s_mag_addra,
    output wire                    s_mag_ena,
    output wire [0:0]              s_mag_wea,
    output wire [DATA_WIDTH-1:0]   s_mag_dina,
    input  wire [DATA_WIDTH-1:0]   s_mag_douta,
    output wire [2*CELL_WIDTH-1:0] s_mag_addrb,
    output wire                    s_mag_enb,
    output wire [0:0]              s_mag_web,
    output wire [DATA_WIDTH-1:0]   s_mag_dinb,
    input  wire [DATA_WIDTH-1:0]   s_mag_doutb,

    output wire                    mag_busy,
    output wire                    mag_done
);

    localparam signed [DATA_WIDTH-1:0] C_E_Q313 = 16'sd717;
    localparam signed [DATA_WIDTH-1:0] C_B_Q313 = 16'sd2867;
    localparam integer MAG_PRODUCT_SHIFT = 13;
    localparam integer GRID_SIZE = CELLS * CELLS;
    localparam [2*CELL_WIDTH-1:0] LAST_ADDR = GRID_SIZE - 1;

    reg [DATA_WIDTH-1:0] held_source_q313;
    reg                  held_source_valid;
    reg [31:0]           checksum_reg;
    reg                  solver_done_d;

    wire [2*CELL_WIDTH-1:0] ey_rd_addr;
    wire [2*CELL_WIDTH-1:0] ey_wr_addr;
    wire [DATA_WIDTH-1:0]   ey_wr_data;
    wire                    ey_we;
    wire [2*CELL_WIDTH-1:0] ey_adj_rd_addr;

    wire [2*CELL_WIDTH-1:0] ex_rd_addr;
    wire [2*CELL_WIDTH-1:0] ex_wr_addr;
    wire [DATA_WIDTH-1:0]   ex_wr_data;
    wire                    ex_we;

    wire [2*CELL_WIDTH-1:0] bz_rd_addr;
    wire [2*CELL_WIDTH-1:0] bz_wr_addr;
    wire [DATA_WIDTH-1:0]   bz_wr_data;
    wire                    bz_we;
    wire [2*CELL_WIDTH-1:0] bz_adj_rd_addr;
    wire                    solver_write_event;
    wire [31:0]             solver_write_mix;

    reg [2*CELL_WIDTH-1:0] e_mag_rd_addr;
    reg [2*CELL_WIDTH-1:0] e_mag_rd_addr_d;
    reg [2*CELL_WIDTH-1:0] e_mag_wr_addr;
    reg [DATA_WIDTH-1:0]   e_mag_wr_data;
    reg                    e_mag_active;
    reg                    e_mag_data_valid;
    reg                    e_mag_we;
    reg                    e_mag_done_reg;
    reg                    e_mag_done_pending;
    reg                    mag_mode_latched;

    wire                   e_mag_start;
    wire [DATA_WIDTH-1:0]  e_mag_approx;
    wire [DATA_WIDTH-1:0]  mag_result;

    function [DATA_WIDTH-1:0] abs_unsigned;
        input signed [DATA_WIDTH-1:0] value;
        begin
            if (value[DATA_WIDTH-1])
                abs_unsigned = (~value) + {{(DATA_WIDTH-1){1'b0}}, 1'b1};
            else
                abs_unsigned = value;
        end
    endfunction

    function [DATA_WIDTH-1:0] e_mag_from_fields;
        input signed [DATA_WIDTH-1:0] ex_value;
        input signed [DATA_WIDTH-1:0] ey_value;
        reg [DATA_WIDTH-1:0] ex_abs;
        reg [DATA_WIDTH-1:0] ey_abs;
        reg [DATA_WIDTH-1:0] hi;
        reg [DATA_WIDTH-1:0] lo;
        reg [DATA_WIDTH:0]   sum;
        begin
            ex_abs = abs_unsigned(ex_value);
            ey_abs = abs_unsigned(ey_value);
            hi = (ex_abs >= ey_abs) ? ex_abs : ey_abs;
            lo = (ex_abs >= ey_abs) ? ey_abs : ex_abs;
            sum = {1'b0, hi} + {2'b00, lo[DATA_WIDTH-1:1]};
            e_mag_from_fields = sum[DATA_WIDTH] ? {DATA_WIDTH{1'b1}} : sum[DATA_WIDTH-1:0];
        end
    endfunction

    function [DATA_WIDTH-1:0] s_mag_from_fields;
        input signed [DATA_WIDTH-1:0] ex_value;
        input signed [DATA_WIDTH-1:0] ey_value;
        input signed [DATA_WIDTH-1:0] bz_value;
        reg [DATA_WIDTH-1:0] e_approx;
        reg [DATA_WIDTH-1:0] bz_abs;
        reg [(2*DATA_WIDTH)-1:0] product;
        reg [(2*DATA_WIDTH)-1:0] scaled;
        begin
            e_approx = e_mag_from_fields(ex_value, ey_value);
            bz_abs = abs_unsigned(bz_value);
            product = e_approx * bz_abs;
            scaled = product >> MAG_PRODUCT_SHIFT;
            s_mag_from_fields = |scaled[(2*DATA_WIDTH)-1:DATA_WIDTH] ?
                {DATA_WIDTH{1'b1}} : scaled[DATA_WIDTH-1:0];
        end
    endfunction

    assign e_mag_start = solver_done & ~solver_done_d;
    assign mag_busy = e_mag_active;
    assign mag_done = e_mag_done_reg;
    assign e_mag_approx = e_mag_from_fields($signed(ex_douta), $signed(ey_douta));
    assign mag_result = mag_mode_latched ?
        s_mag_from_fields($signed(ex_douta), $signed(ey_douta), $signed(bz_douta)) :
        e_mag_approx;

    always @(posedge clk) begin
        if (rst) begin
            held_source_q313  <= {DATA_WIDTH{1'b0}};
            held_source_valid <= 1'b0;
            checksum_reg      <= 32'd0;
            solver_done_d     <= 1'b0;
        end else begin
            solver_done_d <= solver_done;
            if (source_valid) begin
                held_source_q313  <= source_q313;
                held_source_valid <= 1'b1;
            end
            if (solver_write_event) begin
                checksum_reg <= {checksum_reg[30:0], checksum_reg[31]} ^ solver_write_mix;
            end
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            e_mag_rd_addr     <= {2*CELL_WIDTH{1'b0}};
            e_mag_rd_addr_d   <= {2*CELL_WIDTH{1'b0}};
            e_mag_wr_addr     <= {2*CELL_WIDTH{1'b0}};
            e_mag_wr_data     <= {DATA_WIDTH{1'b0}};
            e_mag_active      <= 1'b0;
            e_mag_data_valid  <= 1'b0;
            e_mag_we          <= 1'b0;
            e_mag_done_reg    <= 1'b0;
            e_mag_done_pending <= 1'b0;
            mag_mode_latched  <= 1'b0;
        end else begin
            e_mag_we           <= 1'b0;
            e_mag_done_reg     <= e_mag_done_pending;
            e_mag_done_pending <= 1'b0;

            if (e_mag_start && !e_mag_active) begin
                e_mag_rd_addr    <= {2*CELL_WIDTH{1'b0}};
                e_mag_rd_addr_d  <= {2*CELL_WIDTH{1'b0}};
                e_mag_active     <= 1'b1;
                e_mag_data_valid <= 1'b0;
                mag_mode_latched <= mag_mode;
            end else if (e_mag_active) begin
                e_mag_rd_addr_d <= e_mag_rd_addr;

                if (e_mag_data_valid) begin
                    e_mag_wr_addr <= e_mag_rd_addr_d;
                    e_mag_wr_data <= mag_result;
                    e_mag_we      <= 1'b1;

                    if (e_mag_rd_addr_d == LAST_ADDR) begin
                        e_mag_active     <= 1'b0;
                        e_mag_data_valid <= 1'b0;
                        e_mag_done_pending <= 1'b1;
                    end
                end else begin
                    e_mag_data_valid <= 1'b1;
                end

                if (e_mag_rd_addr != LAST_ADDR) begin
                    e_mag_rd_addr <= e_mag_rd_addr + {{(2*CELL_WIDTH-1){1'b0}}, 1'b1};
                end
            end
        end
    end

    assign source_latched = held_source_valid;
    assign solver_checksum = checksum_reg;

    assign solver_write_event = ey_we | ex_we | bz_we;
    assign solver_write_mix =
        (ey_we ? ({ey_wr_addr, ey_wr_data} ^ 32'h45590000) : 32'd0) ^
        (ex_we ? ({ex_wr_addr, ex_wr_data} ^ 32'h45580000) : 32'd0) ^
        (bz_we ? ({bz_wr_addr, bz_wr_data} ^ 32'h425a0000) : 32'd0);

    fdtd_solver #(
        .CELLS(CELLS),
        .CELL_WIDTH(CELL_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_solver (
        .clk(clk),
        .rst(rst),
        .C_E(C_E_Q313),
        .C_B(C_B_Q313),
        .source_in(held_source_q313),
        .source_valid(held_source_valid),
        .source_addr(SOURCE_ADDR),
        .ey_rd_addr(ey_rd_addr),
        .ey_rd_dout(ey_douta),
        .ey_wr_addr(ey_wr_addr),
        .ey_wr_data(ey_wr_data),
        .ey_we(ey_we),
        .ex_rd_addr(ex_rd_addr),
        .ex_rd_dout(ex_douta),
        .ex_wr_addr(ex_wr_addr),
        .ex_wr_data(ex_wr_data),
        .ex_we(ex_we),
        .bz_rd_addr(bz_rd_addr),
        .bz_rd_dout(bz_douta),
        .bz_wr_addr(bz_wr_addr),
        .bz_wr_data(bz_wr_data),
        .bz_we(bz_we),
        .solver_enable(solver_enable),
        .solver_done(solver_done),
        .bz_adj_rd_addr(bz_adj_rd_addr),
        .bz_adj_dout(bz_doutb),
        .ey_adj_rd_addr(ey_adj_rd_addr),
        .ey_adj_dout(ey_doutb)
    );

    assign ey_addra = e_mag_active ? e_mag_rd_addr : ey_rd_addr;
    assign ey_ena   = 1'b1;
    assign ey_wea   = 1'b0;
    assign ey_dina  = {DATA_WIDTH{1'b0}};
    assign ey_addrb = e_mag_active ? {2*CELL_WIDTH{1'b0}} : (ey_we ? ey_wr_addr : ey_adj_rd_addr);
    assign ey_enb   = 1'b1;
    assign ey_web   = e_mag_active ? 1'b0 : ey_we;
    assign ey_dinb  = ey_wr_data;

    assign ex_addra = e_mag_active ? e_mag_rd_addr : ex_rd_addr;
    assign ex_ena   = 1'b1;
    assign ex_wea   = 1'b0;
    assign ex_dina  = {DATA_WIDTH{1'b0}};
    assign ex_addrb = e_mag_active ? {2*CELL_WIDTH{1'b0}} : ex_wr_addr;
    assign ex_enb   = 1'b1;
    assign ex_web   = e_mag_active ? 1'b0 : ex_we;
    assign ex_dinb  = ex_wr_data;

    assign bz_addra = e_mag_active ? e_mag_rd_addr : bz_rd_addr;
    assign bz_ena   = 1'b1;
    assign bz_wea   = 1'b0;
    assign bz_dina  = {DATA_WIDTH{1'b0}};
    assign bz_addrb = e_mag_active ? {2*CELL_WIDTH{1'b0}} : (bz_we ? bz_wr_addr : bz_adj_rd_addr);
    assign bz_enb   = 1'b1;
    assign bz_web   = e_mag_active ? 1'b0 : bz_we;
    assign bz_dinb  = bz_wr_data;

    assign s_mag_addra = e_mag_wr_addr;
    assign s_mag_ena   = 1'b1;
    assign s_mag_wea   = e_mag_we;
    assign s_mag_dina  = e_mag_wr_data;
    assign s_mag_addrb = {2*CELL_WIDTH{1'b0}};
    assign s_mag_enb   = 1'b1;
    assign s_mag_web   = 1'b0;
    assign s_mag_dinb  = {DATA_WIDTH{1'b0}};

endmodule
