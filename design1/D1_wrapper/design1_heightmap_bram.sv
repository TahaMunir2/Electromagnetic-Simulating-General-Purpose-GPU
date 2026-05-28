// Single-port read-only BRAM for heightmap data.
// Vivado infers RAMB18E2/RAMB36E2 from the (* ram_style = "block" *) attribute.
// With ADDR_W=12 (GRID_N=64): 4096 x 16-bit = 2 x RAMB36E2 per instance.

module design1_heightmap_bram #(
    parameter int ADDR_W = 12,
    parameter int DATA_W = 16,
    parameter bit USE_INIT_FILE = 1'b0,
    parameter string INIT_FILE = "heightmap_64x64.hex",
    parameter bit USE_MOCK_DATA = 1'b1
)(
    input  logic                       clk,
    input  logic [ADDR_W-1:0]          addr,
    input  logic                       re,
    output logic signed [DATA_W-1:0]   dout
);
    localparam int DEPTH = 1 << ADDR_W;

    (* ram_style = "block" *)
    logic signed [DATA_W-1:0] mem [0:DEPTH-1];

    initial begin
        int x;
        int y;
        int n;
        int cx;
        int cy;
        int ax;
        int ay;
        int half;
        int maxd;
        int height;
        int peak;

        if (USE_INIT_FILE) begin
            $readmemh(INIT_FILE, mem);
        end else if (USE_MOCK_DATA) begin
            n = 1 << (ADDR_W / 2);
            half = n / 2;
            peak = 3072;   // 0.375 in Q2.13
            for (int addr_i = 0; addr_i < DEPTH; addr_i++) begin
                x = addr_i & (n - 1);
                y = addr_i >> (ADDR_W / 2);
                cx = x - half;
                cy = y - half;
                ax = (cx < 0) ? -cx : cx;
                ay = (cy < 0) ? -cy : cy;
                maxd = (ax > ay) ? ax : ay;

                if (maxd >= half) begin
                    mem[addr_i] = '0;
                end else begin
                    height = (peak * (half - maxd)) / half;
                    mem[addr_i] = height;
                end
            end
        end else begin
            for (int addr_i = 0; addr_i < DEPTH; addr_i++)
                mem[addr_i] = '0;
        end
    end

    always_ff @(posedge clk) begin
        if (re)
            dout <= mem[addr];
    end

endmodule
