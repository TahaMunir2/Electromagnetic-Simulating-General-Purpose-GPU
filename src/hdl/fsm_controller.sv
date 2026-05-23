`timescale 1ns/1ps

module fsm_controller (
    input  wire clk,
    input  wire rst,
    input  wire start,
    input  wire [15:0] num_iterations,
    input  logic cordic_done,
    input  logic solver_done,
    output logic cordic_enable,
    output logic solver_enable,
    output logic fsm_done
);

    typedef enum {IDLE, SOURCE_GEN, SOLVE, DONE} states;
    states my_state, next_state;
    logic [15:0] iteration_count;

    always_ff @(posedge clk) begin
       if(rst) begin
        iteration_count <= 0;
        my_state <= IDLE;
       end
       else begin
        if(solver_done) iteration_count <= iteration_count + 1;
        my_state <= next_state;
       end
    end

    always_comb begin
        next_state    = my_state;
        cordic_enable = 0;
        solver_enable = 0;
        fsm_done      = 0;
        case(my_state) 
            IDLE: if(start) next_state = SOURCE_GEN;
            SOURCE_GEN: begin
                cordic_enable = 1;
                if(cordic_done) next_state = SOLVE;
            end
            SOLVE: begin 
                solver_enable = 1;
                cordic_enable = 0;
                if(solver_done) next_state = DONE;
            end
            // need to add state called Poynting
            DONE: begin
                solver_enable = 0;
                if(iteration_count < num_iterations) next_state = SOURCE_GEN;
                else fsm_done = 1;
            end
            default: begin
                solver_enable = 0;
                cordic_enable = 0;
                next_state = IDLE;
            end
        endcase
    end


endmodule
