
module design1_ray_gen #(
    //Image geometry
    parameter int W              = 640,
    parameter int H              = 480,
    parameter int PX_W           = $clog2(W),       // bits to hold px
    parameter int PY_W           = $clog2(H),       // bits to hold py

    //Fixed-point formats
    // Direction components D and basis vectors: signed Q(I_D).(F_D)
    parameter int DIR_W          = 16,              // total bits per dir component
    parameter int DIR_I          = 2,               // integer bits  (+1 sign)
    parameter int DIR_F          = DIR_W - 1 - DIR_I, // = 13

    // u, v: signed Q(I_UV).(F_UV)
    parameter int UV_W           = 16,
    parameter int UV_I           = 1,
    parameter int UV_F           = UV_W - 1 - UV_I,   // = 14

    // K_U, K_V: signed Q(I_K).(F_K)
    parameter int K_W            = 16,
    parameter int K_I            = 0,
    parameter int K_F            = K_W - 1 - K_I,     // = 15

    //Constants (set externally) 
    // Default values assume W=640, H=480, FOV=90 deg, aspect=W/H.
    // K_U = (2/W) * tan(FOV/2) * (W/H)  = (2/H) * tan(45) = 2/480 = 0.004166...
    // K_V = (2/H) * tan(FOV/2)          = 2/480           = 0.004166...
    // Stored in Q0.15: round(0.004166... * 2^15) = 137
    parameter logic signed [K_W-1:0] K_U = 16'sd137,
    parameter logic signed [K_W-1:0] K_V = 16'sd137,

    //Pipeline pixel-coord forwarding
    // The downstream stages need (px, py) for sof/eol generation.
    // We delay-line them through the 4-cycle pipeline.
    parameter int N_DELAY        = 4
)(
    input  logic                       clk,
    input  logic                       rst_n,

    // Pipeline upstream (from pixel counter) 
    input  logic [PX_W-1:0]            px_in,
    input  logic [PY_W-1:0]            py_in,
    input  logic                       valid_in,

    //Camera basis vectors (per-frame constant) 
    // Q(DIR_I).(DIR_F).  Written by CPU via AXI-Lite once per frame.
    input  logic signed [DIR_W-1:0]    fwd_x,
    input  logic signed [DIR_W-1:0]    fwd_y,
    input  logic signed [DIR_W-1:0]    fwd_z,
    input  logic signed [DIR_W-1:0]    right_x,
    input  logic signed [DIR_W-1:0]    right_y,
    input  logic signed [DIR_W-1:0]    right_z,
    input  logic signed [DIR_W-1:0]    up_x,
    input  logic signed [DIR_W-1:0]    up_y,
    input  logic signed [DIR_W-1:0]    up_z,

    //Back-pressure (from downstream)
    input  logic                       stall,    // freeze pipeline when high

    //Pipeline downstream (to marcher)
    output logic signed [DIR_W-1:0]    Dx,
    output logic signed [DIR_W-1:0]    Dy,
    output logic signed [DIR_W-1:0]    Dz,
    output logic [PX_W-1:0]            px_out,
    output logic [PY_W-1:0]            py_out,
    output logic                       valid_out
);


    //  Pipeline-enable: advance unless downstream is stalling us.

    logic en;
    assign en = ~stall;


    //  STAGE 0 -- centre the pixel coordinates
    //
    //    px_c =  px - W/2     (signed, can be negative on the left half)
    //    py_c =  H/2 - py     (signed, positive at top of screen)
    //
    //  Need one extra bit because the result is signed.

    localparam int PXC_W = PX_W + 1;   // signed, one extra bit
    localparam int PYC_W = PY_W + 1;

    logic signed [PXC_W-1:0]   px_c_s0;
    logic signed [PYC_W-1:0]   py_c_s0;
    logic                      valid_s0;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_s0 <= 1'b0;
        end else if (en) begin
            px_c_s0  <= $signed({1'b0, px_in}) - PXC_W'(W/2);
            py_c_s0  <= PYC_W'(H/2) - $signed({1'b0, py_in});
            valid_s0 <= valid_in;
        end
    end

    //  STAGE 1 -- multiply by the (constant) screen-scale factors.
    //
    //    u = px_c * K_U       (Q(PXC_I).0  *  Q0.K_F   ->  Q?.?  shifted to Q1.14)
    //    v = py_c * K_V
    //
    //  Implementation note: this is a CONSTANT-coefficient multiply, so the
    //  synthesiser will collapse it into a tiny shift/add network OR a single
    //  DSP, depending on the value of K.  Cheap.

    localparam int RAW_UV_W = PXC_W + K_W;   // full product width

    logic signed [RAW_UV_W-1:0] u_raw_s1, v_raw_s1;
    logic signed [UV_W-1:0]     u_s1,     v_s1;
    logic                       valid_s1;

    // Shift amount to convert raw product into target Q(UV_I).(UV_F) format.
    //   raw is Q(PXC_W-1).K_F  =>  shift right by (K_F - UV_F)
    //   (px_c is an integer, hence 0 fractional bits)
    localparam int UV_SHIFT = K_F - UV_F;   // = 15 - 14 = 1

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
        end else if (en) begin
            u_raw_s1 <= px_c_s0 * K_U;
            v_raw_s1 <= py_c_s0 * K_V;
            valid_s1 <= valid_s0;
        end
    end

    // Truncate (with rounding optional -- here just truncate) to UV format.
    // NOTE: combinational slice; the *registered* values feeding stage 2 are
    // u_raw_s1 / v_raw_s1.  We slice on the way into stage 2.
    assign u_s1 = u_raw_s1[UV_SHIFT + UV_W - 1 -: UV_W];
    assign v_s1 = v_raw_s1[UV_SHIFT + UV_W - 1 -: UV_W];


    //  STAGE 2 -- six variable-by-variable multiplies.
    //
    //    ur_x = u * right_x          vu_x = v * up_x
    //    ur_y = u * right_y          vu_y = v * up_y
    //    ur_z = u * right_z          vu_z = v * up_z
    //
    //  Each consumes 1 DSP slice.  Total: 6 DSPs per ray_gen instance.
    //  Full product width = UV_W + DIR_W = 32 bits, signed.

    localparam int PROD_W = UV_W + DIR_W;

    logic signed [PROD_W-1:0]  ur_x_s2, ur_y_s2, ur_z_s2;
    logic signed [PROD_W-1:0]  vu_x_s2, vu_y_s2, vu_z_s2;
    logic                      valid_s2;

    // Forward the basis vectors and fwd through the pipeline (they're
    // constant per frame, but we need them coincident with the data).
    logic signed [DIR_W-1:0]   fwd_x_s2, fwd_y_s2, fwd_z_s2;

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_s2 <= 1'b0;
        end else if (en) begin
            ur_x_s2  <= u_s1 * right_x;
            ur_y_s2  <= u_s1 * right_y;
            ur_z_s2  <= u_s1 * right_z;
            vu_x_s2  <= v_s1 * up_x;
            vu_y_s2  <= v_s1 * up_y;
            vu_z_s2  <= v_s1 * up_z;
            fwd_x_s2 <= fwd_x;
            fwd_y_s2 <= fwd_y;
            fwd_z_s2 <= fwd_z;
            valid_s2 <= valid_s1;
        end
    end


    //  STAGE 3 -- sum the three terms into the final direction vector.
    //
    //    Dx = fwd_x + (ur_x >> S) + (vu_x >> S)
    //    Dy = fwd_y + (ur_y >> S) + (vu_y >> S)
    //    Dz = fwd_z + (ur_z >> S) + (vu_z >> S)
    //
    //  The shift S aligns the product Q(UV_I+DIR_I).(UV_F+DIR_F) back into
    //  the direction's Q(DIR_I).(DIR_F) format.  S = UV_F = 14.

    localparam int SUM_SHIFT = UV_F;   // = 14

    // Each shifted product is sliced back to DIR_W bits.
    logic signed [DIR_W-1:0]   ur_x_aln, ur_y_aln, ur_z_aln;
    logic signed [DIR_W-1:0]   vu_x_aln, vu_y_aln, vu_z_aln;

    assign ur_x_aln = ur_x_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];
    assign ur_y_aln = ur_y_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];
    assign ur_z_aln = ur_z_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];
    assign vu_x_aln = vu_x_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];
    assign vu_y_aln = vu_y_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];
    assign vu_z_aln = vu_z_s2[SUM_SHIFT + DIR_W - 1 -: DIR_W];

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
        end else if (en) begin
            Dx        <= fwd_x_s2 + ur_x_aln + vu_x_aln;
            Dy        <= fwd_y_s2 + ur_y_aln + vu_y_aln;
            Dz        <= fwd_z_s2 + ur_z_aln + vu_z_aln;
            valid_out <= valid_s2;
        end
    end


    //  Pixel-coordinate delay line.  px_in / py_in must travel alongside
    //  the data so the marcher (and downstream) know which pixel they
    //  belong to, especially for sof/tlast generation at the EMIT stage.

    logic [PX_W-1:0]  px_pipe [N_DELAY-1:0];
    logic [PY_W-1:0]  py_pipe [N_DELAY-1:0];

    always_ff @(posedge clk) begin
        if (en) begin
            px_pipe[0] <= px_in;
            py_pipe[0] <= py_in;
            for (int i = 1; i < N_DELAY; i++) begin
                px_pipe[i] <= px_pipe[i-1];
                py_pipe[i] <= py_pipe[i-1];
            end
        end
    end

    assign px_out = px_pipe[N_DELAY-1];
    assign py_out = py_pipe[N_DELAY-1];

endmodule
