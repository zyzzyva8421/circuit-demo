// Deep Hierarchy Test for Schematic Viewer
// Represents a simplified SoC structure with 4 levels of depth

module soc_top (
    input clk,
    input rst_n,
    input [31:0] external_data_in,
    output [31:0] external_data_out,
    input uart_rx,
    output uart_tx
);
    wire [31:0] cpu_mem_addr;
    wire [31:0] cpu_mem_wdata;
    wire [31:0] cpu_mem_rdata;
    wire cpu_mem_we;
    wire [7:0] periph_addr;
    wire [7:0] periph_wdata;
    wire [7:0] periph_rdata;
    wire periph_we;
    wire irq;

    cpu_core cpu (
        .clk(clk),
        .rst_n(rst_n),
        .mem_addr(cpu_mem_addr),
        .mem_wdata(cpu_mem_wdata),
        .mem_rdata(cpu_mem_rdata),
        .mem_we(cpu_mem_we),
        .periph_addr(periph_addr),
        .periph_wdata(periph_wdata),
        .periph_rdata(periph_rdata),
        .periph_we(periph_we),
        .irq(irq)
    );

    memory_subsystem mem (
        .clk(clk),
        .addr(cpu_mem_addr),
        .wdata(cpu_mem_wdata),
        .rdata(cpu_mem_rdata),
        .we(cpu_mem_we)
    );

    peripherals periph (
        .clk(clk),
        .rst_n(rst_n),
        .addr(periph_addr),
        .wdata(periph_wdata),
        .rdata(periph_rdata),
        .we(periph_we),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .irq(irq)
    );

endmodule

// --- Level 2 ---

module cpu_core (clk, rst_n, mem_addr, mem_wdata, mem_rdata, mem_we, periph_addr, periph_wdata, periph_rdata, periph_we, irq);
    input clk, rst_n, irq;
    input [31:0] mem_rdata;
    input [7:0] periph_rdata;
    output [31:0] mem_addr, mem_wdata;
    output mem_we;
    output [7:0] periph_addr, periph_wdata;
    output periph_we;

    wire [31:0] alu_result;
    wire [31:0] reg_a, reg_b;
    wire [3:0] alu_op;
    wire reg_we;

    control_unit cu (
        .clk(clk), 
        .rst_n(rst_n),
        .irq(irq), 
        .op_out(alu_op), 
        .mem_we(mem_we),
        .reg_we(reg_we)
    );

    register_file rf (
        .clk(clk),
        .rst_n(rst_n),
        .we(reg_we),
        .wdata(mem_rdata),
        .rdata_a(reg_a),
        .rdata_b(reg_b)
    );

    alu alu_inst (
        .a(reg_a), 
        .b(reg_b), 
        .op(alu_op), 
        .result(alu_result)
    );

    // Some simple logic for address mapping
    assign mem_addr = alu_result;
    assign mem_wdata = reg_b;

endmodule

module memory_subsystem (clk, addr, wdata, rdata, we);
    input clk, we;
    input [31:0] addr, wdata;
    output [31:0] rdata;
    
    wire [31:0] cache_hit_data;
    wire cache_hit;

    cache_controller cache (
        .clk(clk),
        .addr(addr),
        .hit(cache_hit),
        .data(cache_hit_data)
    );

    sram_array sram (
        .clk(clk),
        .addr(addr),
        .wdata(wdata),
        .we(we),
        .rdata(rdata)
    );

endmodule

module peripherals (clk, rst_n, addr, wdata, rdata, we, uart_rx, uart_tx, irq);
    input clk, rst_n, we;
    input [7:0] addr, wdata;
    output [7:0] rdata;
    input uart_rx;
    output uart_tx, irq;

    wire uart_irq, timer_irq;

    uart_block uart (
        .clk(clk),
        .rst_n(rst_n),
        .rx(uart_rx),
        .tx(uart_tx),
        .irq(uart_irq)
    );

    timer_block timer (
        .clk(clk),
        .rst_n(rst_n),
        .enable(we),
        .irq(timer_irq)
    );

    assign irq = uart_irq | timer_irq;
endmodule

// --- Level 3 ---

module alu (a, b, op, result);
    input [31:0] a, b;
    input [3:0] op;
    output [31:0] result;
    
    wire [31:0] add_res, shift_res;

    adder_32bit add_inst (.a(a), .b(b), .sum(add_res));
    shifter_32bit shift_inst (.in(a), .amt(b), .out(shift_res));
    
    // Muxing result
    assign result = (op == 0) ? add_res : shift_res;
endmodule

module register_file (clk, rst_n, we, wdata, rdata_a, rdata_b);
    input clk, rst_n, we;
    input [31:0] wdata;
    output [31:0] rdata_a, rdata_b;

    // Banked registers
    reg_bank bank0 (.clk(clk), .wdata(wdata));
    reg_bank bank1 (.clk(clk), .wdata(wdata));
    reg_bank bank2 (.clk(clk), .wdata(wdata));
    reg_bank bank3 (.clk(clk), .wdata(wdata));
endmodule

module control_unit (clk, rst_n, irq, op_out, mem_we, reg_we);
    input clk, rst_n, irq;
    output [3:0] op_out;
    output mem_we, reg_we;
endmodule

module uart_block (clk, rst_n, rx, tx, irq);
    input clk, rst_n, rx;
    output tx, irq;
    
    uart_tx_mod tx_inst (.clk(clk), .out(tx));
    uart_rx_mod rx_inst (.clk(clk), .in(rx));
endmodule

// --- Leaves ---
module adder_32bit(a, b, sum); input [31:0] a,b; output [31:0] sum; endmodule
module shifter_32bit(in, amt, out); input [31:0] in, amt; output [31:0] out; endmodule
module reg_bank(clk, wdata); input clk; input [31:0] wdata; endmodule
module cache_controller(clk, addr, hit, data); input clk; input [31:0] addr; output hit; output [31:0] data; endmodule
module sram_array(clk, addr, wdata, we, rdata); input clk, we; input [31:0] addr, wdata; output [31:0] rdata; endmodule
module timer_block(clk, rst_n, enable, irq); input clk, rst_n, enable; output irq; endmodule
module uart_tx_mod(clk, out); input clk; output out; endmodule
module uart_rx_mod(clk, in); input clk; input in; endmodule
