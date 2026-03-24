#ifndef CIRCUIT_GRAPH_H
#define CIRCUIT_GRAPH_H

#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>
#include <functional>
#include <cmath>
#include <algorithm>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QProcess>
#include <QDebug>
#include <QFile>
#include <QPointF>

#include "netlistparser.h"

enum class CircuitNodeType {
    ModuleInstance, 
    Port,           
    NetJoint,       
    ExpandedInstance 
};

struct GraphNodeData {
    CircuitNodeType type;
    std::string id;          
    std::string label;       
    std::string moduleType;  
    std::string direction;   
    std::string path; 
    const Instance* instancePtr = nullptr;
};

struct CPort {
    std::string id;
    std::string name;
    double x = 0, y = 0; // Relative center
    double width = 0, height = 0; // if port has dimensions
    std::string side; // "WEST", "EAST", etc.
};

struct CNode {
    std::string id;
    double x = 0, y = 0, width = 0, height = 0;
    std::vector<CPort> ports;
    std::vector<CNode*> children;
    CNode* parent = nullptr;
    GraphNodeData data;
};

struct CEdge {
    CNode* source;
    CNode* target;
    std::string sourcePort; 
    std::string targetPort;
    std::vector<QPointF> points; // Contains Start -> Bends -> End
};

class CircuitGraph {
public:
    std::vector<CNode*> nodes; 
    std::vector<CEdge*> edges;
    std::map<std::string, CNode*> nodeMap;
    std::vector<CNode*> rootNodes;
    std::set<std::string> expandedInstances;

    CircuitGraph() {}
    ~CircuitGraph() { clear(); }

    void clear() {
        for(auto n : nodes) delete n;
        nodes.clear();
        for(auto e : edges) delete e;
        edges.clear();
        nodeMap.clear();
        rootNodes.clear();
    }
    
    void buildFromModule(const Netlist& netlist, const std::string& moduleName) {
        buildHierarchical(netlist, moduleName);
    }
    
    void buildHierarchical(const Netlist& netlist, const std::string& topModuleName) {
        clear();
        if (netlist.modules.count(topModuleName) == 0) return;
        const Module& topModule = netlist.modules.at(topModuleName);
        buildRecursive(netlist, topModule, "", nullptr);
    }
    
    void buildRecursive(const Netlist& netlist, const Module& module, std::string pathPrefix, CNode* parent) {
        // 1. Create nodes for Instances
        for (const auto& inst : module.instances) {
            std::string fullPath = pathPrefix + inst.name;
            
            CNode* n = new CNode();
            n->id = "inst_" + fullPath;
            n->data.type = CircuitNodeType::ModuleInstance;
            n->data.id = inst.name;
            n->data.path = fullPath;
            n->data.label = inst.name + "\n(" + inst.type + ")";
            n->data.moduleType = inst.type;
            n->data.instancePtr = &inst;
            n->width = 80; n->height = 60; // Default, will be updated

            // Generate Ports for Layout
            if (netlist.modules.count(inst.type)) {
                const Module& subMod = netlist.modules.at(inst.type);
                std::vector<std::string> inputs, outputs;
                for (const auto& p : subMod.ports) {
                    if (p.direction == "input") inputs.push_back(p.name);
                    else outputs.push_back(p.name);
                }
                
                double PIN_SPACING = 20.0;
                double MARGIN_Y = 10.0;
                size_t maxPins = std::max(inputs.size(), outputs.size());
                
                n->height = std::max(60.0, maxPins * PIN_SPACING + MARGIN_Y * 2);
                
                // Calculate Start Y for Inputs (Centered, but Snapped to Grid)
                double idealInputY = (n->height - (inputs.size() * PIN_SPACING)) / 2.0;
                int kIn = std::round((idealInputY - MARGIN_Y) / PIN_SPACING);
                double inputStartY = MARGIN_Y + kIn * PIN_SPACING;

                for (size_t i = 0; i < inputs.size(); ++i) {
                     CPort p;
                     p.name = inputs[i];
                     p.id = n->id + "_" + p.name;
                     p.side = "WEST";
                     p.width = 4; p.height = 4;
                     p.x = 0; // Inside edge
                     p.y = inputStartY + i * PIN_SPACING + PIN_SPACING/2.0 - p.height/2.0; 
                     n->ports.push_back(p);
                }
                
                // Calculate Start Y for Outputs (Centered, Snapped)
                double idealOutputY = (n->height - (outputs.size() * PIN_SPACING)) / 2.0;
                int kOut = std::round((idealOutputY - MARGIN_Y) / PIN_SPACING);
                double outputStartY = MARGIN_Y + kOut * PIN_SPACING;

                for (size_t i = 0; i < outputs.size(); ++i) {
                     CPort p;
                     p.name = outputs[i];
                     p.id = n->id + "_" + p.name;
                     p.side = "EAST";
                     p.width = 4; p.height = 4;
                     p.x = n->width - p.width; // Inside edge
                     p.y = outputStartY + i * PIN_SPACING + PIN_SPACING/2.0 - p.height/2.0;
                     n->ports.push_back(p);
                }
            } else {
                 if (nodes.back() == n) { 
                      nodes.pop_back(); 
                      if(parent && !parent->children.empty() && parent->children.back()==n) parent->children.pop_back();
                      else if (!rootNodes.empty() && rootNodes.back()==n) rootNodes.pop_back();
                      delete n;
                      continue; 
                 }
            }
            
            nodes.push_back(n);
            nodeMap[n->id] = n;
            if (parent) parent->children.push_back(n); else rootNodes.push_back(n);
            n->parent = parent;
            
            // Re-add to logic if needed (already added above? Wait, original code added n BEFORE ports)
            // Original code:
            // nodes.push_back(n);
            // nodeMap... 
            // Generate Ports...

            
            if (expandedInstances.count(fullPath)) {
                n->data.type = CircuitNodeType::ExpandedInstance;
                if (netlist.modules.count(inst.type)) {
                    const Module& subMod = netlist.modules.at(inst.type);
                    buildRecursive(netlist, subMod, fullPath + ".", n);
                }
            }
        }
        
        // 2. Nodes for Ports
        for (const auto& port : module.ports) {
            CNode* n = new CNode();
            n->id = "port_" + pathPrefix + port.name;
            n->data.type = CircuitNodeType::Port;
            n->data.id = port.name;
            n->data.path = pathPrefix + port.name;
            n->data.label = port.name;
            n->data.direction = port.direction;
            n->width = 15; n->height = 15;
            
            nodes.push_back(n);
            nodeMap[n->id] = n;
            if (parent) parent->children.push_back(n); else rootNodes.push_back(n);
            n->parent = parent;
        }
        
        // 3. Connect Local Nets
        struct Conn { CNode* n; std::string pin; bool isDriver; };
        std::map<std::string, std::vector<Conn>> netMap;
        
        // A. Instance connections
        for (const auto& inst : module.instances) {
            std::string fullInstName = pathPrefix + inst.name;
            bool isExpanded = expandedInstances.count(fullInstName);

            if (isExpanded) {
                 if (netlist.modules.count(inst.type)) {
                    const Module& subMod = netlist.modules.at(inst.type);
                    for (const auto& pair : inst.portMap) {
                        std::string pinName = pair.first;
                        std::string netName = pair.second;
                        std::string portNodeId = "port_" + fullInstName + "." + pinName;
                        if (nodeMap.count(portNodeId)) {
                             CNode* pNode = nodeMap[portNodeId];
                             bool isDriver = false;
                             for(const auto& p : subMod.ports) {
                                 if(p.name == pinName && p.direction == "input") isDriver = false; 
                                 if(p.name == pinName && p.direction == "output") isDriver = true; 
                             }
                             netMap[netName].push_back({pNode, "", isDriver});
                        }
                    }
                 }
            } else {
                std::string instNodeId = "inst_" + fullInstName;
                if (nodeMap.count(instNodeId)) {
                    CNode* instNode = nodeMap[instNodeId];
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
                CNode* pNode = nodeMap[portNodeId];
                bool isDriver = (port.direction == "input");
                netMap[port.name].push_back({pNode, "", isDriver});
            }
        }
        
        // C. Create Edges
        for(const auto& pair : netMap) {
             const auto& conns = pair.second;
             if (conns.size() < 2) continue;
             
             std::vector<Conn> drivers, sinks;
             for(auto c : conns) { if(c.isDriver) drivers.push_back(c); else sinks.push_back(c); }
             
             if (drivers.empty() && !sinks.empty()) { 
                 drivers.push_back(sinks[0]); sinks.erase(sinks.begin()); 
             }
             
             if (conns.size() == 2 && drivers.size() == 1 && sinks.size() == 1) {
                CEdge* e = new CEdge{drivers[0].n, sinks[0].n, drivers[0].pin, sinks[0].pin};
                edges.push_back(e);
             } else {
                
                // Use a joint node
                CNode* joint = new CNode();
                joint->id = "net_" + pathPrefix + pair.first + "_" + std::to_string(nodes.size()); 
                joint->data.type = CircuitNodeType::NetJoint; 
                joint->data.label = ".";
                joint->width = 5; joint->height = 5;
                nodes.push_back(joint);
                nodeMap[joint->id] = joint;
                if(parent) parent->children.push_back(joint); else rootNodes.push_back(joint);
                joint->parent = parent;
                
                for(auto d : drivers) {
                    edges.push_back(new CEdge{d.n, joint, d.pin, ""});
                }
                for(auto s : sinks) {
                    edges.push_back(new CEdge{joint, s.n, "", s.pin});
                }
             }
        }
    }

    void applyLayout() {
        QJsonObject root = toElkJson();
        
        QProcess process;
        process.setProcessChannelMode(QProcess::MergedChannels);
        process.start("node", QStringList() << "layout_script.js");
        
        if (!process.waitForStarted()) {
            std::cerr << "Failed to start node process\n";
            return;
        }
        
        QJsonDocument doc(root);
        // Debug JSON
        std::cerr << "--- ELK JSON Input ---" << std::endl;
        std::cerr << doc.toJson(QJsonDocument::Indented).data() << std::endl;
        std::cerr << "----------------------" << std::endl;
        
        process.write(doc.toJson(QJsonDocument::Compact));
        process.closeWriteChannel();
        
        if (!process.waitForFinished(10000)) {
            std::cerr << "Layout process timed out: " << process.readAll().data() << "\n";
            return;
        }
        
        QByteArray output = process.readAll();
        QJsonDocument outDoc = QJsonDocument::fromJson(output);
        if (outDoc.isNull()) {
            std::cerr << "Failed to parse layout output: " << output.data() << "\n";
            return;
        }
        
        // Debug Output JSON
        // std::cerr << "--- ELK JSON Output ---" << std::endl;
        // std::cerr << outDoc.toJson(QJsonDocument::Indented).data() << std::endl;
        // std::cerr << "-----------------------" << std::endl;
        
        applyElkJson(outDoc.object());
    }

private:
   QJsonObject toElkJson() {
       QJsonObject root;
       root["id"] = "root";
       QJsonObject layoutOptions;
       layoutOptions["elk.algorithm"] = "layered";
       layoutOptions["elk.direction"] = "RIGHT"; 
       layoutOptions["elk.spacing.nodeNode"] = 60; // Increased
       layoutOptions["elk.layered.spacing.nodeNodeBetweenLayers"] = 60; // Increased
       // layoutOptions["elk.edgeRouting"] = "ORTHOGONAL"; // Let default happen
       layoutOptions["elk.layered.nodePlacement.strategy"] = "BRANDES_KOEPF";
       
       // Try setting port constraints on root to see if it helps child port recognition
       // layoutOptions["elk.portConstraints"] = "FIXED_POS"; 
       // layoutOptions["org.eclipse.elk.portConstraints"] = "FIXED_POS";
       layoutOptions["org.eclipse.elk.layered.nodePlacement.bk.fixedAlignment"] = "TERMINAL_ALIGNED";
       layoutOptions["org.eclipse.elk.port.borderOffset"] = 0.0;
       // Ensure port constraints are applied per node type, not globally
       layoutOptions["elk.spacing.portPort"] = 10;
       root["layoutOptions"] = layoutOptions;
       
       QJsonArray children;
       // Sort nodes by Port Status to help ELK understand flow
       std::vector<CNode*> sortedNodes = rootNodes;
       std::sort(sortedNodes.begin(), sortedNodes.end(), [](CNode* a, CNode* b) {
            if (a->data.type == CircuitNodeType::Port && b->data.type != CircuitNodeType::Port) return true;
            if (a->data.type == CircuitNodeType::Port && b->data.type == CircuitNodeType::Port) {
                if (a->data.direction == "input" && b->data.direction == "output") return true;
            }
            return false;
       });

       for(auto n : sortedNodes) {
           children.append(nodeToJson(n));
       }
       root["children"] = children;
       
       QJsonArray edgesJson;
       for(size_t i=0; i<edges.size(); ++i) {
           CEdge* e = edges[i];
           QJsonObject edgeObj;
           edgeObj["id"] = "e" + QString::number(i);
           
           
           // Correctly use Port IDs in sources/targets for ELK to respect port connections
           std::string srcId = QString::fromStdString(e->source->id).toStdString();
           if (!e->sourcePort.empty() && !e->source->ports.empty()) {
                for(const auto& p : e->source->ports) {
                    if (p.name == e->sourcePort) {
                        srcId = p.id;
                        break;
                    }
                }
           }
           QJsonArray sources; sources.append(QString::fromStdString(srcId));
           edgeObj["sources"] = sources;

           std::string tgtId = QString::fromStdString(e->target->id).toStdString();
           if (!e->targetPort.empty() && !e->target->ports.empty()) {
                for(const auto& p : e->target->ports) {
                    if (p.name == e->targetPort) {
                        tgtId = p.id;
                        break;
                    }
                }
           }
           QJsonArray targets; targets.append(QString::fromStdString(tgtId));
           edgeObj["targets"] = targets;
           
           // Remove sourcePort/targetPort as we are now linking directly to ports via IDs
           
           QJsonObject eOpts;
           eOpts["elk.edgeRouting"] = "ORTHOGONAL";
           edgeObj["layoutOptions"] = eOpts;
           
           edgesJson.append(edgeObj);
       }
       root["edges"] = edgesJson;
       
       return root;
   }
   
   QJsonObject nodeToJson(CNode* n) {
       QJsonObject obj;
       obj["id"] = QString::fromStdString(n->id);
       obj["width"] = n->width;
       obj["height"] = n->height;
       
       if (!n->ports.empty()) {
           QJsonArray portsArr;
           std::map<std::string, int> sideIndices;
           bool fixedPos = (n->data.type == CircuitNodeType::ModuleInstance);

           for(const auto& p : n->ports) {
               QJsonObject pObj;
               pObj["id"] = QString::fromStdString(p.id); 
               pObj["width"] = p.width;
               pObj["height"] = p.height;
               pObj["x"] = p.x;
               pObj["y"] = p.y;
               
               QJsonObject lo;
               lo["elk.port.side"] = QString::fromStdString(p.side);
               if (!fixedPos) {
                   lo["elk.port.index"] = sideIndices[p.side]++;
               }
               
               pObj["layoutOptions"] = lo;
               portsArr.append(pObj);
           }
           obj["ports"] = portsArr;
       }
       
       // Common layout options
       QJsonObject lOpts;
       lOpts["org.eclipse.elk.port.borderOffset"] = 0.0;
       
       if (!n->children.empty()) {
            lOpts["elk.algorithm"] = "layered";
            lOpts["elk.direction"] = "RIGHT";
            // lOpts["elk.edgeRouting"] = "ORTHOGONAL"; // Set on edges now
            
            lOpts["elk.layered.spacing.nodeNodeBetweenLayers"] = 40.0;
            lOpts["elk.spacing.portPort"] = 10.0;
            lOpts["elk.nodeSize.constraints"] = "MINIMUM_SIZE"; 
            lOpts["org.eclipse.elk.nodeSize.constraints"] = "MINIMUM_SIZE";
            
            lOpts["elk.layered.nodePlacement.bk.fixedAlignment"] = "TERMINAL_ALIGNED";
            lOpts["org.eclipse.elk.layered.nodePlacement.bk.fixedAlignment"] = "TERMINAL_ALIGNED";
            lOpts["elk.layered.unnecessaryBending"] = false;
            lOpts["org.eclipse.elk.layered.unnecessaryBending"] = false;

            QJsonArray kids;
            for(auto c : n->children) kids.append(nodeToJson(c));
            obj["children"] = kids;
       } else {
            // Leaf node options
            lOpts["elk.nodeSize.constraints"] = "FIXED"; 
            
            if (n->data.type == CircuitNodeType::ModuleInstance) {
                 lOpts["elk.portConstraints"] = "FIXED_POS"; 
                 lOpts["org.eclipse.elk.portConstraints"] = "FIXED_POS";
                 lOpts["elk.layered.nodePlacement.bk.fixedAlignment"] = "TERMINAL_ALIGNED";
                 lOpts["org.eclipse.elk.layered.nodePlacement.bk.fixedAlignment"] = "TERMINAL_ALIGNED";
            }
            
            if (n->data.type == CircuitNodeType::Port) {
                if (n->data.direction == "input") {
                     lOpts["elk.layered.layering.layerConstraint"] = "FIRST";
                     lOpts["org.eclipse.elk.layered.layering.layerConstraint"] = "FIRST";
                     lOpts["elk.priority"] = 100;
                     lOpts["org.eclipse.elk.priority"] = 100;
                } else if (n->data.direction == "output") {
                     lOpts["elk.layered.layering.layerConstraint"] = "LAST";
                     lOpts["org.eclipse.elk.layered.layering.layerConstraint"] = "LAST";
                     lOpts["elk.priority"] = 100;
                     lOpts["org.eclipse.elk.priority"] = 100;
                }
            } else if (n->data.type == CircuitNodeType::NetJoint) {
                lOpts["elk.priority"] = 1;
                lOpts["org.eclipse.elk.priority"] = 1;
            }
       }
       
       obj["layoutOptions"] = lOpts;
       return obj;
   }

   void applyElkJson(const QJsonObject& obj) {
       if (obj.contains("id") && obj.contains("x") && obj.contains("y")) {
           std::string id = obj["id"].toString().toStdString();
           if (nodeMap.count(id)) {
               CNode* n = nodeMap[id];
               n->x = obj["x"].toDouble();
               n->y = obj["y"].toDouble();
               n->width = obj["width"].toDouble();
               n->height = obj["height"].toDouble();
               
               if (obj.contains("ports")) {
                   QJsonArray portsArr = obj["ports"].toArray();
                   for(auto pVal : portsArr) {
                       QJsonObject pObj = pVal.toObject();
                       std::string pId = pObj["id"].toString().toStdString();
                       for(auto& p : n->ports) {
                           if (p.id == pId) {
                               p.x = pObj["x"].toDouble();
                               p.y = pObj["y"].toDouble();
                               p.width = pObj["width"].toDouble();
                               p.height = pObj["height"].toDouble();
                               break;
                           }
                       }
                   }
               }
           }
       }
       
       if (obj.contains("children")) {
           QJsonArray children = obj["children"].toArray();
           for(auto val : children) applyElkJson(val.toObject());
       }
       
       if (obj.contains("edges")) {
           QJsonArray jsonEdges = obj["edges"].toArray();
           for(auto val : jsonEdges) {
                QJsonObject eObj = val.toObject();
                std::string eid = eObj["id"].toString().toStdString();
                // Assumes edge IDs are "e" + index
                if (eid.size() > 1 && eid[0] == 'e') {
                    try {
                        int idx = std::stoi(eid.substr(1));
                        if (idx >= 0 && idx < (int)edges.size()) {
                            CEdge* edge = edges[idx];
                            edge->points.clear();
                            
                            if (eObj.contains("sections")) {
                                QJsonArray sections = eObj["sections"].toArray();
                                for(auto s : sections) {
                                    QJsonObject sect = s.toObject();
                                    
                                    if(sect.contains("startPoint")) {
                                         QJsonObject p = sect["startPoint"].toObject();
                                         edge->points.push_back(QPointF(p["x"].toDouble(), p["y"].toDouble()));
                                    }
                                    
                                    if(sect.contains("bendPoints")) {
                                        QJsonArray pts = sect["bendPoints"].toArray();
                                        for(auto p : pts) {
                                             QJsonObject po = p.toObject();
                                             edge->points.push_back(QPointF(po["x"].toDouble(), po["y"].toDouble()));
                                        }
                                    }
                                    
                                    if(sect.contains("endPoint")) {
                                         QJsonObject p = sect["endPoint"].toObject();
                                         edge->points.push_back(QPointF(p["x"].toDouble(), p["y"].toDouble()));
                                    }
                                }
                            }

                            // Snap points to ports with orthogonality (Removed to keep ELK router result)
                            /*
                            if(!edge->points.empty()) {
                                // Source Update
                                if (edge->source) {
                                    if (!edge->sourcePort.empty()) {
                                        for(const auto& p : edge->source->ports) {
                                             if(p.name == edge->sourcePort) {
                                                 double px = edge->source->x + p.x;
                                                 double py = edge->source->y + p.y + p.height / 2.0;

                                                 if (p.side == "EAST") px += p.width;
                                                 else if (p.side == "SOUTH") { px += p.width / 2.0; py = edge->source->y + p.y + p.height; }
                                                 else if (p.side == "NORTH") { px += p.width / 2.0; py = edge->source->y + p.y; }

                                                 edge->points[0].setX(px);
                                                 edge->points[0].setY(py);

                                                 if (edge->points.size() > 1) {
                                                     QPointF& next = edge->points[1];
                                                     if (p.side == "WEST" || p.side == "EAST") {
                                                         next.setY(py);
                                                     } else if (p.side == "NORTH" || p.side == "SOUTH") {
                                                         next.setX(px);
                                                     } else {
                                                         if (std::abs(next.x() - px) > std::abs(next.y() - py)) {
                                                              next.setY(py); 
                                                         } else {
                                                              next.setX(px);
                                                         }
                                                     }
                                                 }
                                                 break;
                                             }
                                        }
                                    } else if (edge->source->data.type == CircuitNodeType::Port) {
                                         // Input Port Node: Connect to EAST side
                                         double px = edge->source->x + edge->source->width;
                                         double py = edge->source->y + edge->source->height / 2.0;
                                         
                                         edge->points[0].setX(px);
                                         edge->points[0].setY(py);

                                         if (edge->points.size() > 1) {
                                              edge->points[1].setY(py); // Force horizontal exit
                                         }
                                    } else if (edge->source->data.type == CircuitNodeType::NetJoint) {
                                         // Snap to Center
                                         double px = edge->source->x + edge->source->width / 2.0;
                                         double py = edge->source->y + edge->source->height / 2.0;
                                         
                                         edge->points[0].setX(px);
                                         edge->points[0].setY(py);
                                         
                                         if (edge->points.size() > 1 && std::hypot(edge->points[1].x()-px, edge->points[1].y()-py) < 20) {
                                             QPointF& next = edge->points[1];
                                             if (std::abs(next.x() - px) > std::abs(next.y() - py)) {
                                                  next.setY(py); // Horizontal
                                             } else {
                                                  next.setX(px); // Vertical
                                             }
                                         }
                                    }
                                }

                                // Target Update
                                if (edge->target) {
                                    if (!edge->targetPort.empty()) {
                                        for(const auto& p : edge->target->ports) {
                                             if(p.name == edge->targetPort) {
                                                 double px = edge->target->x + p.x;
                                                 double py = edge->target->y + p.y + p.height / 2.0;
                                                 
                                                 if (p.side == "EAST") px += p.width;
                                                 else if (p.side == "SOUTH") { px += p.width / 2.0; py = edge->target->y + p.y + p.height; }
                                                 else if (p.side == "NORTH") { px += p.width / 2.0; py = edge->target->y + p.y; }

                                                 edge->points.back().setX(px);
                                                 edge->points.back().setY(py);

                                                 if (edge->points.size() > 1) {
                                                     // If previous point is attached to a Source Port, DO NOT MOVE IT
                                                     bool sourceIsPort = !edge->sourcePort.empty() || (edge->source && edge->source->data.type == CircuitNodeType::Port);
                                                     if (!sourceIsPort) {
                                                         QPointF& prev = edge->points[edge->points.size()-2];
                                                         if (p.side == "WEST" || p.side == "EAST") {
                                                             prev.setY(py);
                                                         } else if (p.side == "NORTH" || p.side == "SOUTH") {
                                                             prev.setX(px);
                                                         } else {
                                                             if (std::abs(prev.x() - px) > std::abs(prev.y() - py)) {
                                                                  prev.setY(py);
                                                             } else {
                                                                  prev.setX(px);
                                                             }
                                                         }
                                                     }
                                                 }
                                                 break;
                                             }
                                        }
                                    } else if (edge->target->data.type == CircuitNodeType::Port) {
                                         // Output Port Node: Connect to WEST side
                                         double px = edge->target->x;
                                         double py = edge->target->y + edge->target->height / 2.0;

                                         edge->points.back().setX(px);
                                         edge->points.back().setY(py);

                                         if (edge->points.size() > 1) {
                                              bool sourceIsPort = !edge->sourcePort.empty() || (edge->source && edge->source->data.type == CircuitNodeType::Port);
                                              if (!sourceIsPort) {
                                                  edge->points[edge->points.size()-2].setY(py); // Force horizontal entry
                                              }
                                         }
                                    } else if (edge->target->data.type == CircuitNodeType::NetJoint) {
                                         // Snap to Center
                                         double px = edge->target->x + edge->target->width / 2.0;
                                         double py = edge->target->y + edge->target->height / 2.0;
                                         
                                         edge->points.back().setX(px);
                                         edge->points.back().setY(py);
                                         
                                         if (edge->points.size() > 1 && std::hypot(edge->points[edge->points.size()-2].x()-px, edge->points[edge->points.size()-2].y()-py) < 20) {
                                             bool sourceIsPort = !edge->sourcePort.empty() || (edge->source && edge->source->data.type == CircuitNodeType::Port);
                                             
                                             // DEBUG
                                             if (edge->source->data.id == "periph") {
                                                 std::cout << "DEBUG TargetNetJoint: Source=" << edge->source->data.id 
                                                           << " Port=" << edge->sourcePort 
                                                           << " IsPort=" << sourceIsPort 
                                                           << " PointsSize=" << edge->points.size() 
                                                           << " Prev=" << edge->points[edge->points.size()-2].x() << "," << edge->points[edge->points.size()-2].y() 
                                                           << " PxPy=" << px << "," << py << std::endl;
                                             }

                                             if (!sourceIsPort) {
                                                 QPointF& prev = edge->points[edge->points.size()-2];
                                                 if (std::abs(prev.x() - px) > std::abs(prev.y() - py)) {
                                                      prev.setY(py); // Horizontal
                                                 } else {
                                                      prev.setX(px); // Vertical
                                                 }
                                             }
                                         }
                                    }
                                }
                                
                                // Post-process: Insert bends for diagonal segments
                                bool changed = true;
                                while(changed) {
                                    changed = false;
                                    for (size_t k = 0; k < edge->points.size() - 1; ++k) {
                                        QPointF p1 = edge->points[k];
                                        QPointF p2 = edge->points[k+1];
                                        if (std::abs(p1.x() - p2.x()) > 2.0 && std::abs(p1.y() - p2.y()) > 2.0) {
                                             double midX = p2.x();
                                             double midY = p1.y();
                                             
                                             auto it = edge->points.begin() + k + 1;
                                             edge->points.insert(it, QPointF(midX, midY));
                                             changed = true; // Restart to be safe
                                             break;
                                        }
                                    }
                                }
                            }
                            */
                        }

                    } catch (...) {}
                }
           }
       }
   }
};

#endif // CIRCUIT_GRAPH_H