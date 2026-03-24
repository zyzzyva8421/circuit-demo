// Simple Verilog Netlist for Circuit Testing
// A simple inverter chain with some combinational logic

module test_circuit (
    input in,
    output out
);
    wire n1, n2, n3;
    
    // Inverter chain
    inv u1 (.A(in), .Y(n1));
    inv u2 (.A(n1), .Y(n2));
    inv u3 (.A(n2), .Y(n3));
    inv u4 (.A(n3), .Y(out));
    
endmodule

// Simple NAND gate module
module nand2 (A, B, Y);
    input A, B;
    output Y;
    assign Y = ~(A & B);
endmodule

// Simple NOR gate module  
module nor2 (A, B, Y);
    input A, B;
    output Y;
    assign Y = ~(A | B);
endmodule

// Simple inverter
module inv (A, Y);
    input A;
    output Y;
    assign Y = ~A;
endmodule

// D Flip-Flop
module dff (CLK, D, Q);
    input CLK, D;
    output reg Q;
    always @(posedge CLK) begin
        Q <= D;
    end
endmodule
