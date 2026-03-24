#ifndef CIRCUIT_GRAPH_H
#define CIRCUIT_GRAPH_H

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/cluster/ClusterGraph.h>
#include <ogdf/cluster/ClusterGraphAttributes.h>
#include <ogdf/layered/SugiyamaLayout.h>
#include <ogdf/layered/LongestPathRanking.h>
#include <ogdf/layered/MedianHeuristic.h>
#include <ogdf/layered/OptimalHierarchyLayout.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>

#include "netlistparser.h"

// Types of nodes in our graph
enum class CircuitNodeType {
    ModuleInstance, // A sub-module instance (standard box)
    Port,           // Input/Output port of the current module
    NetJoint,       // A junction for a net
    ExpandedInstance // An instance that has been expanded (acts as a container)
};

struct GraphNodeData {
    CircuitNodeType type;
    std::string id;          // Instance name or Port name (fully qualified if nested)
    std::string label;       // Label to display
    std::string moduleType;  // For instances: what module is it?
    
    // For ports, layout hint
    std::string direction;   // "input", "output"
    
    // Hierarchy path
    std::string path; 
    
    // Back-reference
    const Instance* instancePtr = nullptr;
};

struct EdgePinData {
    std::string sourcePin;
    std::string targetPin;
};

class CircuitGraph {
public:
    ogdf::Graph g;
    ogdf::ClusterGraph clusterG;
    ogdf::GraphAttributes* ga;
    ogdf::ClusterGraphAttributes* cga;
    
    // Map internal node IDs to OGDF nodes
    std::map<std::string, ogdf::node> nodeMap;
    std::map<ogdf::node, GraphNodeData> nodeDataMap;
    std::map<ogdf::edge, EdgePinData> edgePinMap;
    
    // Keep track of which instances are expanded (by full path name)
    std::set<std::string> expandedInstances;
    
    CircuitGraph() : clusterG(g), ga(nullptr), cga(nullptr) {}
    
    ~CircuitGraph() {
        if (cga) delete cga; 
        if (ga) delete ga;
    }
    
    // Flatten hierarchy
    void buildHierarchical(const Netlist& netlist, const std::string& topModuleName) {
        // Clear previous graph
        g.clear(); // This clears clusters in clusterG too
        edgePinMap.clear();
        nodeMap.clear();
        nodeDataMap.clear();

        if (ga) delete ga;
        if (cga) delete cga;
        
        ga = new ogdf::GraphAttributes(g, 
            ogdf::GraphAttributes::nodeGraphics | 
            ogdf::GraphAttributes::edgeGraphics | 
            ogdf::GraphAttributes::nodeLabel | 
            ogdf::GraphAttributes::edgeLabel |
            ogdf::GraphAttributes::nodeStyle |
            ogdf::GraphAttributes::edgeStyle);
            
        cga = new ogdf::ClusterGraphAttributes(clusterG, 
             ogdf::ClusterGraphAttributes::nodeGraphics |
             ogdf::ClusterGraphAttributes::clusterGraphics |
             ogdf::ClusterGraphAttributes::clusterLabel |
             ogdf::ClusterGraphAttributes::clusterStyle);

        // Recursive build
        const Module& topModule = netlist.modules.at(topModuleName);
        buildRecursive(netlist, topModule, "", nullptr); // Start at root
    }
    
    void buildRecursive(const Netlist& netlist, const Module& module, std::string pathPrefix, ogdf::cluster parentCluster) {
        // 1. Create nodes for all Instances
        for (const auto& inst : module.instances) {
            std::string fullPath = pathPrefix + inst.name;
            
            // Check if expanded
            if (expandedInstances.count(fullPath)) {
                // If expanded, create a Cluster
                ogdf::cluster c = clusterG.newCluster(parentCluster);
                cga->label(c) = inst.name + " (" + inst.type + ")";
                cga->fillColor(c) = ogdf::Color::Name::Ivory;
                cga->strokeColor(c) = ogdf::Color::Name::Darkgrey;
                
                // Recurse into the sub-module
                if (netlist.modules.count(inst.type)) {
                    const Module& subMod = netlist.modules.at(inst.type);
                    buildRecursive(netlist, subMod, fullPath + ".", c);
                }
            } else {
                // Normal Instance Node
                ogdf::node n = g.newNode();
                if (parentCluster) {
                    clusterG.reassignNode(n, parentCluster);
                }
                
                std::string id = "inst_" + fullPath;
                nodeMap[id] = n;
                
                GraphNodeData data;
                data.type = CircuitNodeType::ModuleInstance;
                data.id = inst.name;
                data.path = fullPath;
                data.label = inst.name + "\n(" + inst.type + ")";
                data.moduleType = inst.type;
                data.instancePtr = &inst;
                nodeDataMap[n] = data;
                
                ga->label(n) = data.label;
                ga->width(n) = 80;
                ga->height(n) = 60;
                ga->shape(n) = ogdf::Shape::Rect;
                ga->fillColor(n) = ogdf::Color::Name::White;
            }
        }
        
        // 2. Create nodes for Ports
        // For sub-modules (inside a cluster), they are nodes INSIDE the cluster that act as interface.
        
        for (const auto& port : module.ports) {
            ogdf::node n = g.newNode();
            if (parentCluster) {
                clusterG.reassignNode(n, parentCluster);
            }
            
            std::string id = "port_" + pathPrefix + port.name;
            nodeMap[id] = n;
            
            GraphNodeData data;
            data.type = CircuitNodeType::Port;
            data.id = port.name;
            data.path = pathPrefix + port.name; // Unique path
            data.label = port.name;
            data.direction = port.direction;
            nodeDataMap[n] = data;
            
            ga->label(n) = data.label;
            ga->width(n) = 15;
            ga->height(n) = 15;
            if (port.direction == "input") ga->shape(n) = ogdf::Shape::Triangle;
            else ga->shape(n) = ogdf::Shape::Rect; // Output usually rect or point
            ga->fillColor(n) = ogdf::Color::Name::Lightgrey;
        }
        
        // 3. Connect Local Nets
        struct Conn { ogdf::node n; std::string pin; bool isDriver; };
        std::map<std::string, std::vector<Conn>> netMap;
        
        // A. Instances connections
        for (const auto& inst : module.instances) {
            std::string fullInstName = pathPrefix + inst.name;
            bool isExpanded = expandedInstances.count(fullInstName);

            if (isExpanded) {
                 if (netlist.modules.count(inst.type)) {
                    const Module& subMod = netlist.modules.at(inst.type);
                    for (const auto& pair : inst.portMap) {
                        std::string pinName = pair.first;
                        std::string netName = pair.second;
                        
                        // Find the port node inside the child
                        std::string portNodeId = "port_" + fullInstName + "." + pinName;
                        if (nodeMap.count(portNodeId)) {
                             ogdf::node pNode = nodeMap[portNodeId];
                             bool isDriver = false;
                             for(const auto& p : subMod.ports) {
                                 if(p.name == pinName && p.direction == "input") isDriver = false; // Input of child is Load for parent net
                                 if(p.name == pinName && p.direction == "output") isDriver = true; // Output of child is Driver for parent net
                             }
                             netMap[netName].push_back({pNode, "", isDriver});
                        }
                    }
                 }
            } else {
                std::string instNodeId = "inst_" + fullInstName;
                if (nodeMap.count(instNodeId)) {
                    ogdf::node instNode = nodeMap[instNodeId];
                    const Module* subMod = nullptr;
                    if(netlist.modules.count(inst.type)) subMod = &netlist.modules.at(inst.type);
                    
                    for (const auto& pair : inst.portMap) {
                        std::string pinName = pair.first;
                        std::string netName = pair.second;
                        bool isDriver = false;
                        if (subMod) {
                             for(const auto& p : subMod->ports) {
                                 if(p.name == pinName && p.direction == "output") isDriver = true;   
                             }
                        }
                        netMap[netName].push_back({instNode, pinName, isDriver});
                    }
                }
            }
        }
        
        // B. Module Ports (Self)
        for (const auto& port : module.ports) {
            std::string portNodeId = "port_" + pathPrefix + port.name;
            if (nodeMap.count(portNodeId)) {
                ogdf::node pNode = nodeMap[portNodeId];
                // Self Ports: Input is Driver (from outside), Output is Sink (to outside)
                // BUT internally: Input drives internal nets, Output is driven by internal nets.
                bool isDriver = (port.direction == "input");
                netMap[port.name].push_back({pNode, "", isDriver});
            }
        }
        
        // C. Create Edges for this scope
        for (const auto& pair : netMap) {
             const auto& conns = pair.second;
             if (conns.size() < 2) continue;
             
             std::vector<Conn> drivers, sinks;
             for(auto c : conns) { if(c.isDriver) drivers.push_back(c); else sinks.push_back(c); }
             
             // Handle basic cases: 1 driver N sinks, or Float to Float
             if (drivers.empty() && !sinks.empty()) { 
                 // All sinks, treat one as driver just for visual connectivity? Or floating?
                 // Let's pick first as pseudo-driver
                 drivers.push_back(sinks[0]); sinks.erase(sinks.begin()); 
             }
             
             if (conns.size() == 2 && drivers.size() == 1 && sinks.size() == 1) {
                // Direct connection
                ogdf::edge e = g.newEdge(drivers[0].n, sinks[0].n);
                edgePinMap[e] = {drivers[0].pin, sinks[0].pin};
             } else {
                // Use a joint node
                ogdf::node joint = g.newNode();
                if (parentCluster) {
                     clusterG.reassignNode(joint, parentCluster);
                }
                
                GraphNodeData data; data.type = CircuitNodeType::NetJoint; data.label = ".";
                nodeDataMap[joint] = data;
                ga->width(joint) = 5; ga->height(joint) = 5; ga->shape(joint) = ogdf::Shape::Ellipse; ga->fillColor(joint) = ogdf::Color::Name::Black;
                
                for(auto d : drivers) {
                    ogdf::edge e = g.newEdge(d.n, joint);
                    edgePinMap[e] = {d.pin, ""};
                }
                for(auto s : sinks) {
                    ogdf::edge e = g.newEdge(joint, s.n);
                    edgePinMap[e] = {"", s.pin};
                }
             }
        }
    }
    
    void applyLayout() {
        if (g.numberOfNodes() == 0) return;
        
        ogdf::SugiyamaLayout layout;
        layout.setRanking(new ogdf::LongestPathRanking);
        layout.setCrossMin(new ogdf::MedianHeuristic);
        
        // Use default hierarchy layout which is safer (FastHierarchyLayout typically)
        // ogdf::OptimalHierarchyLayout* ohl = new ogdf::OptimalHierarchyLayout;
        // ohl->layerDistance(50.0);
        // ohl->nodeDistance(40.0);
        // ohl->weightBalancing(0.8);
        // layout.setLayout(ohl);
        
        // Use standard graph attributes call, ignore explicit cluster constraints during layout for now to avoid solver issues
        // We can recompute cluster bounds later if needed
        layout.call(*ga);
        
        // Rotate layout from Top-Down to Left-to-Right
        // Sugiyama layout defaults to vertical layers. We swap X and Y to make it horizontal.
        
        // 1. Rotate Nodes
        for(ogdf::node n : g.nodes) {
            double x = ga->x(n);
            double y = ga->y(n);
            double w = ga->width(n);
            double h = ga->height(n);
            
            ga->x(n) = y;
            ga->y(n) = x;
            ga->width(n) = h;
            ga->height(n) = w;
        }
        
        // 2. Rotate Edge Bends
        for(ogdf::edge e : g.edges) {
            ogdf::DPolyline& bends = ga->bends(e);
            for(auto& p : bends) {
                double bx = p.m_x;
                double by = p.m_y;
                p.m_x = by;
                p.m_y = bx;
            }
        }
        
        // Compute cluster bounds based on children (using the new rotated coordinates)
        updateClusterBounds();
    }
    
    void updateClusterBounds() {
        // Initialize cluster bounds to "empty"
        // Iterate all clusters? No easy way to iterate all. use recursion from root?
        // Or assume we can access them.
        
        // Simple map to store bounds during computation
        struct BBox { double x1=1e9, y1=1e9, x2=-1e9, y2=-1e9; bool set=false; };
        std::map<ogdf::cluster, BBox> bounds;
        
        // 1. Process Nodes
        for(ogdf::node n : g.nodes) {
            double x = ga->x(n);
            double y = ga->y(n);
            double w = ga->width(n);
            double h = ga->height(n);
            
            ogdf::cluster c = clusterG.clusterOf(n);
            while(c != nullptr) {
                BBox& b = bounds[c];
                b.x1 = std::min(b.x1, x);
                b.y1 = std::min(b.y1, y);
                b.x2 = std::max(b.x2, x + w);
                b.y2 = std::max(b.y2, y + h);
                b.set = true;
                c = c->parent(); // Move up
            }
        }
        
        // 2. Apply to CGA
        // Need to traverse map or clusters.
        // Since we populated map based on nodes, we might miss empty clusters but that's fine.
        for(auto& pair : bounds) {
            ogdf::cluster c = pair.first;
            BBox& b = pair.second;
            if (b.set) {
                // Add some padding
                double pad = 10.0;
                cga->x(c) = b.x1 - pad;
                cga->y(c) = b.y1 - pad;
                cga->width(c) = (b.x2 - b.x1) + 2*pad;
                cga->height(c) = (b.y2 - b.y1) + 2*pad;
            } else {
                cga->x(c) = 0; cga->y(c) = 0; cga->width(c) = 0; cga->height(c) = 0;
            }
        }
    }
    
    // Kept for compatibility but redirects
    void buildFromModule(const Netlist& netlist, const std::string& moduleName) {
         buildHierarchical(netlist, moduleName);
    }
};

#endif // CIRCUIT_GRAPH_H