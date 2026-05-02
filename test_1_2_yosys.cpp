#include <iostream>
#include <fstream>
#include <cmath>
#include "circuitgraph.h"
#include "netlistparser.h"

int main() {
    CircuitGraph graph;
    std::string filename = "1_2_yosys.v";
    
    std::cout << "Testing: " << filename << std::endl;
    Netlist netlist = NetlistParser::parse(filename);
    
    if (netlist.modules.empty()) {
        std::cerr << "Failed to parse modules\n";
        return 1;
    }
    
    std::string topModule = netlist.topModule;
    if (topModule.empty() && !netlist.modules.empty()) {
        topModule = netlist.modules.begin()->first;
    }
    
    std::cout << "Building graph for: " << topModule << std::endl;
    graph.buildHierarchical(netlist, topModule);
    
    std::cout << "Nodes: " << graph.nodes.size() << std::endl;
    std::cout << "Applying layout...\n";
    graph.applyLayout();
    
    // Check for overlaps
    int overlapCount = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        CNode* n1 = graph.nodes[i];
        
        // Only check ModuleInstance nodes
        if (n1->data.type != CircuitNodeType::ModuleInstance) continue;
        
        for (size_t j = i + 1; j < graph.nodes.size(); ++j) {
            CNode* n2 = graph.nodes[j];
            if (n2->data.type != CircuitNodeType::ModuleInstance) continue;
            
            bool overlap = !(n1->x + n1->width <= n2->x || 
                             n2->x + n2->width <= n1->x || 
                             n1->y + n1->height <= n2->y || 
                             n2->y + n2->height <= n1->y);
            
            if (overlap) {
                overlapCount++;
                std::cout << "OVERLAP: " << n1->data.label 
                         << " [" << n1->x << "," << n1->y << " " << n1->width << "x" << n1->height << "]"
                         << " <-> " << n2->data.label 
                         << " [" << n2->x << "," << n2->y << " " << n2->width << "x" << n2->height << "]"
                         << std::endl;
            }
        }
    }
    
    if (overlapCount == 0) {
        std::cout << "✓ No overlaps detected\n";
        return 0;
    } else {
        std::cout << "✗ " << overlapCount << " overlaps detected\n";
        return 1;
    }
}
