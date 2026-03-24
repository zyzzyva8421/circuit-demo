import sys

def generate_high_fanout_verilog(filename, fanout_count=100):
    with open(filename, 'w') as f:
        f.write("// High Fanout Test Case\n")
        f.write(f"// Fanout: {fanout_count}\n\n")
        
        # Define the high fanout top module first so it's picked as top
        f.write("module high_fanout_top (input clk, input rst_n, output done);\n")
        f.write("    wire driver_net;\n")
        f.write("    wire [31:0] dummy_out;\n\n")
        
        # Driver
        f.write("    buf_gate driver_inst (.in(clk), .out(driver_net));\n\n")
        
        # Fanout Loads
        for i in range(fanout_count):
            f.write(f"    buf_gate load_{i} (.in(driver_net), .out(dummy_out[{i%32}]));\n")
            
        f.write("\nendmodule\n\n")

        # Define the buffer module
        f.write("module buf_gate (input in, output out);\n")
        f.write("endmodule\n")

if __name__ == "__main__":
    count = 100
    if len(sys.argv) > 1:
        count = int(sys.argv[1])
    generate_high_fanout_verilog("high_fanout.v", count)
    print(f"Generated high_fanout.v with {count} loads")
