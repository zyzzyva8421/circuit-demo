// Complex Processor Core Test Netlist
// Features: Buses, Multiple Instances, Feedback loops, Control Logic
// Non-ANSI style for compatibility with simple parser

module processor_core (clk, rst_n, instr, data_in, data_out, status_z, status_o);
    // Port declarations
    input clk;
    input rst_n;
    input [7:0] instr;
    input [7:0] data_in;
    output [7:0] data_out;
    output status_z;
    output status_o;

    // Internal wires
    wire [7:0] reg_a_out;
    wire [7:0] reg_b_out;
    wire [7:0] alu_res;
    wire [7:0] mux_out;
    
    // Control wires
    wire load_a;
    wire load_b;
    wire [1:0] alu_op;
    
    // 1. Decoder
    // Takes instruction and generates control signals
    decoder dec_inst (
        .instr(instr),
        .load_a(load_a),
        .load_b(load_b),
        .op(alu_op)
    );
    
    // 2. Registers
    // Register A loads from data_in
    dff_8bit reg_a (
        .clk(clk),
        .rst_n(rst_n),
        .en(load_a),
        .d(data_in),
        .q(reg_a_out)
    );
    
    // Register B loads from Mux Output
    dff_8bit reg_b (
        .clk(clk),
        .rst_n(rst_n),
        .en(load_b),
        .d(mux_out),
        .q(reg_b_out)
    );
    
    // 3. Multiplexer
    // Selects between data_in and ALU result (Accumulator loop)
    mux2_8bit mux_b (
        .sel(load_b),
        .a(data_in),
        .b(alu_res),
        .y(mux_out)
    );
    
    // 4. ALU
    // Performs operation on Reg A and Reg B
    alu_8bit alu_inst (
        .op(alu_op),
        .a(reg_a_out),
        .b(reg_b_out),
        .res(alu_res),
        .z(status_z),
        .v(status_o)
    );
    
    // 5. Output Buffer
    // Drives the result to output
    buf_8bit out_buf (
        .in(alu_res),
        .out(data_out)
    );

endmodule

// --- Submodule Definitions ---

module decoder (instr, load_a, load_b, op);
    input [7:0] instr;
    output load_a, load_b;
    output [1:0] op;
endmodule

module dff_8bit (clk, rst_n, en, d, q);
    input clk, rst_n, en;
    input [7:0] d;
    output [7:0] q;
endmodule

module mux2_8bit (sel, a, b, y);
    input sel;
    input [7:0] a, b;
    output [7:0] y;
endmodule

module alu_8bit (op, a, b, res, z, v);
    input [1:0] op;
    input [7:0] a, b;
    output [7:0] res;
    output z, v;
endmodule

module buf_8bit (in, out);
    input [7:0] in;
    output [7:0] out;
endmodule
