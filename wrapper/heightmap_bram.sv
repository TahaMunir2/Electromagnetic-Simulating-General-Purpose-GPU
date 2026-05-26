// Single-port read-only BRAM for heightmap data.
// Vivado infers RAMB18E2/RAMB36E2 from the (* ram_style = "block" *) attribute.
// With ADDR_W=12 (GRID_N=64): 4096 x 16-bit = 2 x RAMB36E2 per instance.

module heightmap_bram #(
    parameter int ADDR_W = 12,
    parameter int DATA_W = 16
)(
    input  logic                       clk,
    input  logic [ADDR_W-1:0]          addr,
    input  logic                       re,
    output logic signed [DATA_W-1:0]   dout
);
    localparam int DEPTH = 1 << ADDR_W;

    (* ram_style = "block" *)
    logic signed [DATA_W-1:0] mem [0:DEPTH-1];

    always_ff @(posedge clk) begin
        if (re)
            dout <= mem[addr];
    end

endmodule
