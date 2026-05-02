#include <QApplication>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include "mainwindow.h"
#include "circuitgraph.h"
#include "netlistparser.h"

// Forward declaration
void selfTest();
void perfTest(std::string filename);

int main(int argc, char *argv[]) {
    // Check arguments for self-test FIRST
    if (argc > 1 && std::string(argv[1]) == "--self-test") {
        selfTest();
        return 0;
    }
    
    if (argc > 1 && std::string(argv[1]) == "--perf-test") {
        std::string filename = "high_fanout.v";
        if (argc > 2) filename = argv[2];
        perfTest(filename);
        return 0;
    }

    QApplication app(argc, argv);
    app.setApplicationName("Circuit Schematic Viewer");
    app.setApplicationVersion("1.0");

    MainWindow window;
    if (argc > 2 && std::string(argv[1]) == "--open") {
        window.loadNetlistFile(QString::fromStdString(argv[2]));
    }
    window.show();
    
    return app.exec();
}

void selfTest() {
    CircuitGraph graph;
    
    // Load the netlist
    std::string filename = "../deep_hierarchy.v";
    std::ifstream f(filename.c_str());
    if (!f.good()) {
        filename = "deep_hierarchy.v"; // Try current directory
        std::ifstream f2(filename.c_str());
        if (!f2.good()) {
             std::cerr << "Failed to find deep_hierarchy.v" << std::endl;
             return;
        }
    }
    
    std::cout << "Parsing netlist: " << filename << std::endl;
    Netlist netlist = NetlistParser::parse(filename);

    if (netlist.modules.empty()) {
        std::cout << "Failed to parse modules from " << filename << std::endl;
        return;
    }

    std::string topModule = netlist.topModule;
    if (topModule.empty()) {
         if (!netlist.modules.empty()) topModule = netlist.modules.begin()->first;
    }

    std::cout << "Building graph for module: " << topModule << std::endl;

    try {
        graph.buildHierarchical(netlist, topModule);
        
        if (graph.nodes.empty()) {
            std::cout << "Graph is empty after buildHierarchical.\n";
            return;
        }

        std::cout << "Graph built with " << graph.nodes.size() << " nodes.\n";
        
        std::cout << "Applying layout...\n";
        graph.applyLayout();
        std::cout << "Layout applied.\n";

    } catch (const std::exception& e) {
        std::cout << "Exception during graph build or layout: " << e.what() << std::endl;
        return;
    }

    // Check layout correctness
    bool layoutCorrect = true;

    // Check port sides
    for (const auto* n : graph.nodes) {
        if (n->data.type == CircuitNodeType::ModuleInstance) {
             const Instance* inst = n->data.instancePtr;
             if (!inst) continue;
             if (netlist.modules.count(inst->type) == 0) continue;
             const Module& m = netlist.modules.at(inst->type);
             
             for (const auto& p : n->ports) {
                 // Find direction in module definition
                 std::string dir = "";
                 for(const auto& mp : m.ports) {
                     if (mp.name == p.name) {
                         dir = mp.direction;
                         break;
                     }
                 }
                 
                 if (dir == "input") {
                     // Should be on Left (near x=0)
                     if (p.x > n->width / 2) {
                         layoutCorrect = false;
                         std::cout << "Port Misplacement: " << n->data.label << ":" << p.name 
                                   << " (input) is at x=" << p.x << " (width=" << n->width << ")\n";
                     }
                 } else if (dir == "output") {
                     // Should be on Right (near x=width)
                     if (p.x < n->width / 2) {
                         layoutCorrect = false;
                         std::cout << "Port Misplacement: " << n->data.label << ":" << p.name 
                                   << " (output) is at x=" << p.x << " (width=" << n->width << ")\n";
                     }
                 }
             }
        }
    }
    
    for (size_t i = 0; i < graph.nodes.size(); ++i) {
        CNode* n1 = graph.nodes[i];

        for (size_t j = i + 1; j < graph.nodes.size(); ++j) {
            CNode* n2 = graph.nodes[j];
            
            // Simple AABB overlap check
            bool overlap = !(n1->x + n1->width <= n2->x || 
                             n2->x + n2->width <= n1->x || 
                             n1->y + n1->height <= n2->y || 
                             n2->y + n2->height <= n1->y);
                             
            if (overlap) {
                 if (n1->data.type == CircuitNodeType::ModuleInstance && n2->data.type == CircuitNodeType::ModuleInstance) {
                     layoutCorrect = false;
                     std::cout << "Modules " << n1->data.label << " and " << n2->data.label << " overlap.\n";
                 }
            }
        }
    }

    if (layoutCorrect) {
        std::cout << "Layout verification passed (no module overlap).\n";
    } else {
        std::cout << "Layout verification failed (overlaps detected).\n";
    }

    // New Check: Top Level Port Placement
    bool portsPlacedCorrectly = true;
    double minX = 1e9, maxX = -1e9;
    std::vector<CNode*> inputs, outputs;
    
    // Find bounds among all nodes (approx)
    for(auto n : graph.nodes) {
        if(n->x < minX) minX = n->x;
        if(n->x + n->width > maxX) maxX = n->x + n->width;
        if(n->data.type == CircuitNodeType::Port) {
            if(n->data.direction == "input") inputs.push_back(n);
            if(n->data.direction == "output") outputs.push_back(n);
        }
    }
    
    // Check Inputs at Left
    for(auto n : inputs) {
        if (n->x > minX + 100) { // Allow some margin
            portsPlacedCorrectly = false;
            std::cout << "Input Port Misplacement: " << n->data.label << " is at x=" << n->x << " which is far from left edge\n";
        }
    }
    // Check Outputs at Right
    for(auto n : outputs) {
        if (n->x < maxX - 100) {
             portsPlacedCorrectly = false;
             std::cout << "Output Port Misplacement: " << n->data.label << " is at x=" << n->x << " which is far from right edge\n";
        }
    }
    
    if(portsPlacedCorrectly) std::cout << "Port Placement Verification Passed.\n";
    else std::cout << "Port Placement Verification Failed.\n";

    // Check connection points alignment
    bool connectionsAligned = true;
    for (const auto& edge : graph.edges) {
        if (edge->points.empty()) continue;

        auto getGlobalPos = [](CNode* n) -> QPointF {
            double x = 0, y = 0;
            while (n) {
                x += n->x;
                y += n->y;
                n = n->parent;
            }
            return QPointF(x, y);
        };

        // Source Check
        QPointF sourceGlobal = getGlobalPos(edge->source);
        QPointF expectedSource = sourceGlobal + QPointF(edge->source->width/2, edge->source->height/2);
        
        bool checkedSource = false;
        if (!edge->sourcePort.empty()) {
            bool found = false;
            for (const auto& p : edge->source->ports) {
                if (p.name == edge->sourcePort) {
                    expectedSource = sourceGlobal + QPointF(p.x, p.y);
                    
                    double ex = expectedSource.x();
                    double ey = expectedSource.y() + p.height / 2.0;

                    if (p.side == "EAST") ex += p.width;
                    else if (p.side == "SOUTH") { ex += p.width / 2.0; ey = expectedSource.y() + p.height; }
                    else if (p.side == "NORTH") { ex += p.width / 2.0; ey = expectedSource.y(); }

                    expectedSource.setX(ex);
                    expectedSource.setY(ey);
                    
                    found = true;
                    break;
                }
            }
            checkedSource = true;
        } else if (edge->source->data.type == CircuitNodeType::Port) {
             // Input Port: Expect EAST side
             expectedSource = sourceGlobal + QPointF(edge->source->width, edge->source->height / 2.0);
             checkedSource = true;
        } else if (edge->source->data.type == CircuitNodeType::NetJoint) {
             // NetJoint: Expect Center
             expectedSource = sourceGlobal + QPointF(edge->source->width / 2.0, edge->source->height / 2.0);
             checkedSource = true;
        }
        
        QPointF actualSource = edge->points.front();
        double dS = std::hypot(actualSource.x() - expectedSource.x(), actualSource.y() - expectedSource.y());
        if (dS > 5.0) {
             if (checkedSource) {
                  connectionsAligned = false;
                  std::cout << "Edge Source Misaligned: " << edge->source->data.label << (edge->sourcePort.empty()?"":":"+edge->sourcePort)
                           << " Expected: " << expectedSource.x() << "," << expectedSource.y()
                           << " (Node: " << sourceGlobal.x() << "," << sourceGlobal.y() << ")"
                           << " Actual: " << actualSource.x() << "," << actualSource.y() 
                           << " Diff: " << dS << std::endl;
             } else {
                 // Relaxed check: Accept connection anywhere on/near the node boundary for port-less nodes
                 bool onBoundary = false;
                 double x = actualSource.x(); double y = actualSource.y();
                 double nx = sourceGlobal.x(); double ny = sourceGlobal.y();
                 double nw = edge->source->width; double nh = edge->source->height;
                 
                 // Check if point is roughly within the bounding box (expanded by tolerance)
                 if (x >= nx - 2 && x <= nx + nw + 2 && y >= ny - 2 && y <= ny + nh + 2) {
                     onBoundary = true;
                 }

                 if (!onBoundary) {
                     connectionsAligned = false; 
                     std::cout << "Edge Source Misaligned (Generic): " << edge->source->data.label 
                           << " Expected: " << expectedSource.x() << "," << expectedSource.y()
                           << " (Node: " << sourceGlobal.x() << "," << sourceGlobal.y() << ")"
                           << " Actual: " << actualSource.x() << "," << actualSource.y() 
                           << " Diff: " << dS << std::endl;
                 }
             }
        }

        // Target Check
        QPointF targetGlobal = getGlobalPos(edge->target);
        QPointF expectedTarget = targetGlobal + QPointF(edge->target->width/2, edge->target->height/2);
        
        bool checkedTarget = false;
        if (!edge->targetPort.empty()) {
            bool found = false;
            for (const auto& p : edge->target->ports) {
                 if (p.name == edge->targetPort) {
                    expectedTarget = targetGlobal + QPointF(p.x, p.y);
                    
                    double ex = expectedTarget.x();
                    double ey = expectedTarget.y() + p.height / 2.0;

                    if (p.side == "EAST") ex += p.width;
                    else if (p.side == "SOUTH") { ex += p.width / 2.0; ey = expectedTarget.y() + p.height; }
                    else if (p.side == "NORTH") { ex += p.width / 2.0; ey = expectedTarget.y(); }

                    expectedTarget.setX(ex);
                    expectedTarget.setY(ey);
                    
                    found = true;
                    break;
                 }
            }
            checkedTarget = true;
        } else if (edge->target->data.type == CircuitNodeType::Port) {
             // Output Port: Expect WEST side
             expectedTarget = targetGlobal + QPointF(0, edge->target->height / 2.0);
             checkedTarget = true;
        } else if (edge->target->data.type == CircuitNodeType::NetJoint) {
             // NetJoint: Expect Center
             expectedTarget = targetGlobal + QPointF(edge->target->width / 2.0, edge->target->height / 2.0);
             checkedTarget = true;
        }
        
        QPointF actualTarget = edge->points.back();
        double dT = std::hypot(actualTarget.x() - expectedTarget.x(), actualTarget.y() - expectedTarget.y());
        if (dT > 5.0) {
             if (checkedTarget) {
                  connectionsAligned = false;
                  std::cout << "Edge Target Misaligned: " << edge->target->data.label << (edge->targetPort.empty()?"":":"+edge->targetPort) 
                           << " Expected: " << expectedTarget.x() << "," << expectedTarget.y()
                           << " (Node: " << targetGlobal.x() << "," << targetGlobal.y() << ")"
                           << " Actual: " << actualTarget.x() << "," << actualTarget.y() 
                           << " Diff: " << dT << std::endl;
             } else {
                 // Relaxed check: Accept connection anywhere on/near the node boundary for port-less nodes
                 bool onBoundary = false;
                 double x = actualTarget.x(); double y = actualTarget.y();
                 double nx = targetGlobal.x(); double ny = targetGlobal.y();
                 double nw = edge->target->width; double nh = edge->target->height;
                 
                 if (x >= nx - 2 && x <= nx + nw + 2 && y >= ny - 2 && y <= ny + nh + 2) {
                     onBoundary = true;
                 }
                
                 if (!onBoundary) {
                     connectionsAligned = false;
                     std::cout << "Edge Target Misaligned (Generic): " << edge->target->data.label 
                           << " Expected: " << expectedTarget.x() << "," << expectedTarget.y()
                           << " (Node: " << targetGlobal.x() << "," << targetGlobal.y() << ")"
                           << " Actual: " << actualTarget.x() << "," << actualTarget.y() 
                           << " Diff: " << dT << std::endl;
                 }
             }
        }
    }

    // Check Port Sides
    bool portsSideCorrect = true;
    for (const auto& n : graph.nodes) {
        if (n->data.type == CircuitNodeType::ModuleInstance && !n->ports.empty()) {
             // We need to know which are inputs and outputs.
             // Usually determined by buildRecursive from netlist. 
             // We can check strictly against 'x' coordinate relative to width.
             
             // BUT, we don't have easy access to module definition here to check direction without looking up netlist.
             // Instead, we can check if x is either ~0 or ~width.
             
             for(const auto& p : n->ports) {
                 // Check if centered on boundary
                 double portCenterX = p.x + p.width/2.0;

                 // Check for overlap with other ports on the same module
                 for(const auto& otherP : n->ports) {
                     if (&p == &otherP) continue;
                     
                     // Simple overlap check (assuming same size)
                     // If centers are too close
                     if (std::abs(p.x - otherP.x) < 1.0 && std::abs(p.y - otherP.y) < 1.0) {
                         portsSideCorrect = false;
                         std::cout << "Port Overlap Error: " << n->data.label << ":" << p.name << " overlaps with " << otherP.name 
                                   << " at (" << p.x << "," << p.y << ")\n";
                     }
                 }

                 // Allow center to be at 0 (left edge) or n->width (right edge)
                 // Or allow if the port box is strictly inside (e.g. x=0) vs strictly outside (x=-w/2 which leads center=0)
                 
                 // Case 1: Left Edge
                 // If p.x = 0 (and w=4), center is 2. This is inside.
                 // If p.x = -2 (and w=4), center is 0. This is on boundary.
                 
                 // Our previous fix sets p.x = 0 for LEFT, and p.x = width for RIGHT.
                 // This puts the port INSIDE the module, attached to the edge.
                 // So center is width/2 (e.g. 2).
                 
                 bool onLeft = (p.x == 0) || (std::abs(p.x + p.width) < 0.001);
                 bool onRight = (std::abs(p.x - (n->width - p.width)) < 0.001) || (std::abs(p.x - n->width) < 0.001); 
                 
                 if (!onLeft && !onRight) {
                     portsSideCorrect = false;
                     std::cout << "Port Error: " << n->data.label << ":" << p.name 
                               << " X=" << p.x << " W=" << p.width << " (Center=" << portCenterX << ")"
                               << " Module W=" << n->width 
                               << " -> Expected 0 or " << (n->width - p.width) << " or " << n->width << "\n";
                 }
             }
        }
    }

    if (connectionsAligned && portsSideCorrect) {
        std::cout << "Connection endpoints alignment verification passed.\n";
    } else {
        std::cout << "Connection endpoints alignment verification failed.\n";
    }

    // Check Orthogonality
    bool orthoCheck = true;
    for (const auto& edge : graph.edges) {
        if (edge->points.size() < 2) continue;
        for (size_t i = 1; i < edge->points.size(); ++i) {
             const QPointF& p1 = edge->points[i-1];
             const QPointF& p2 = edge->points[i];
             
             bool isHorz = std::abs(p1.y() - p2.y()) < 2.0; // Tolerance 2 px
             bool isVert = std::abs(p1.x() - p2.x()) < 2.0; // Tolerance 2 px
             
             if (!isHorz && !isVert) {
                  orthoCheck = false;
                  std::cout << "Edge Segment Not Orthogonal: " 
                            << "(" << p1.x() << "," << p1.y() << ") -> (" 
                            << p2.x() << "," << p2.y() << ")\n";
             }
        }
    }
    if (orthoCheck) std::cout << "Edge Orthogonality Passed.\n"; else std::cout << "Edge Orthogonality FAILED.\n";
}

void perfTest(std::string filename) {
    CircuitGraph graph;
    std::cout << "Parsing netlist: " << filename << std::endl;
    
    // Resolve path - check current dir, then build dir, then parent dir
    std::string resolvedPath = filename;
    std::ifstream test(resolvedPath);
    if (!test.good()) {
        resolvedPath = "../" + filename;
        test.open(resolvedPath);
        if (!test.good()) {
            resolvedPath = filename; // Fall back to original
        }
    }
    
    Netlist netlist = NetlistParser::parse(resolvedPath);

    if (netlist.modules.empty()) {
        std::cout << "Failed to parse modules from " << filename << std::endl;
        return;
    }

    std::string topModule = netlist.topModule;
    if (topModule.empty()) {
         if (!netlist.modules.empty()) topModule = netlist.modules.begin()->first;
    }

    std::cout << "Building graph for module: " << topModule << std::endl;
    auto startBuild = std::chrono::high_resolution_clock::now();
    
    try {
        graph.buildHierarchical(netlist, topModule);
        std::cout << "Graph build completed\n";
    } catch (const std::exception& e) {
        std::cout << "Build Exception: " << e.what() << std::endl;
        return;
    }
    
    auto endBuild = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> buildTime = endBuild - startBuild;
    std::cout << "Graph built with " << graph.nodes.size() << " nodes in " 
              << buildTime.count() << " seconds.\n";
    
    std::cout << "Applying ELK layout (Place & Route)...\n";
    auto startLayout = std::chrono::high_resolution_clock::now();
    
    graph.applyLayout();
    
    auto endLayout = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> layoutTime = endLayout - startLayout;
    std::cout << "Layout applied in " << layoutTime.count() << " seconds.\n";
    
    std::cout << "Performance Test Complete.\n";
}
