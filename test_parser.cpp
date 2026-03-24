#include "netlistparser.h"
#include <iostream>

int main() {
    std::string filename = "../deep_hierarchy.v";
    Netlist netlist = NetlistParser::parse(filename);
    
    std::cout << "Top Module: " << netlist.topModule << "\n";
    if (netlist.modules.count("soc_top")) {
        Module& m = netlist.modules["soc_top"];
        for(const auto& p : m.ports) {
            std::cout << "Port: " << p.name << " Dir: '" << p.direction << "'\n";
        }
    } else {
        std::cout << "Module soc_top not found\n";
    }
    return 0;
}
