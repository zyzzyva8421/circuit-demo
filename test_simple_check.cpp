#include <iostream>
#include <fstream>
#include <cmath>
#include "circuitgraph.h"
#include "netlistparser.h"

int main() {
    CircuitGraph graph;
    std::string filename = "1_2_yosys.v";
    
    std::cout << "Parsing: " << filename << "\n";
    Netlist netlist = NetlistParser::parse(filename);
    
    if (netlist.modules.empty()) {
        std::cerr << "Failed to parse modules\n";
        return 1;
    }
    
    std::string topModule = netlist.topModule;
    if (topModule.empty() && !netlist.modules.empty()) {
        topModule = netlist.modules.begin()->first;
    }
    
    std::cout << "Building graph for: " << topModule << "\n";
    graph.buildHierarchical(netlist, topModule);
    std::cout << "Nodes: " << graph.nodes.size() << "\n";
    std::cout << "Edges: " << graph.edges.size() << "\n";
    
    std::cout << "Applying layout...\n";
    graph.applyLayout();
    
    // Simple statistics
    int collisionsWithMargin2 = 0;
    int collisionsWithMargin4 = 0;
    int collisionsWithMargin8 = 0;
    
    auto checkCollisions = [&](double margin) -> int {
        int count = 0;
        for (const auto* edge : graph.edges) {
            if (edge->points.size() < 2) continue;
            
            for (const auto* node : graph.nodes) {
                if (node->data.type != CircuitNodeType::ModuleInstance) continue;
                if (node == edge->source || node == edge->target) continue;
                
                // Expand node by margin
                double nx0 = node->x - margin;
                double ny0 = node->y - margin;
                double nx1 = node->x + node->width + margin;
                double ny1 = node->y + node->height + margin;
                
                // Check each segment
                for (size_t i = 0; i + 1 < edge->points.size(); ++i) {
                    const auto& p1 = edge->points[i];
                    const auto& p2 = edge->points[i + 1];
                    
                    // AABB intersection for line segment and rectangle
                    double minX = std::min(p1.x(), p2.x());
                    double maxX = std::max(p1.x(), p2.x());
                    double minY = std::min(p1.y(), p2.y());
                    double maxY = std::max(p1.y(), p2.y());
                    
                    if (!(maxX < nx0 || minX > nx1 || maxY < ny0 || minY > ny1)) {
                        count++;
                        break;
                    }
                }
            }
        }
        return count;
    };
    
    collisionsWithMargin2 = checkCollisions(2.0);
    collisionsWithMargin4 = checkCollisions(4.0);
    collisionsWithMargin8 = checkCollisions(8.0);
    
    int collisionsWithMargin0 = checkCollisions(0.0);
    
    std::cout << "\nCollision check with different margins:\n";
    std::cout << "  Margin 0.0: " << collisionsWithMargin0 << " edges\n";
    std::cout << "  Margin 2.0: " << collisionsWithMargin2 << " edges\n";
    std::cout << "  Margin 4.0: " << collisionsWithMargin4 << " edges\n";
    std::cout << "  Margin 8.0: " << collisionsWithMargin8 << " edges\n";
    
    // Instance overlaps
    int instanceOverlaps = 0;
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        CNode* n1 = graph.nodes[i];
        if (n1->data.type != CircuitNodeType::ModuleInstance) continue;
        
        for (size_t j = i + 1; j < graph.nodes.size(); ++j) {
            CNode* n2 = graph.nodes[j];
            if (n2->data.type != CircuitNodeType::ModuleInstance) continue;
            
            bool overlap = !(n1->x + n1->width <= n2->x || 
                             n2->x + n2->width <= n1->x || 
                             n1->y + n1->height <= n2->y || 
                             n2->y + n2->height <= n1->y);
            if (overlap) instanceOverlaps++;
        }
    }
    
    std::cout << "\nInstance Overlaps: " << instanceOverlaps << "\n";
    
    if (instanceOverlaps > 0 || collisionsWithMargin0 > 0) {
        std::cout << "\n❌ Layout has issues\n";
        return 1;
    }
    std::cout << "\n✅ Layout is clean\n";
    return 0;
}
