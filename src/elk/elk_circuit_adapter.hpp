/**
 * ELK C++ Port - CircuitGraph Adapter
 * Converts CircuitGraph to ELK and back
 */

#ifndef ELK_CIRCUIT_ADAPTER_HPP
#define ELK_CIRCUIT_ADAPTER_HPP

#include "elk.hpp"

// Forward declarations
class CircuitGraph;
class CNode;

// Adapter to convert between CircuitGraph and ELK
class ElkCircuitAdapter {
public:
    // Convert CircuitGraph to ElkGraph
    static elk::ElkGraph* toElk(CircuitGraph& cg) {
        auto* graph = new elk::ElkGraph("circuit");
        
        // Copy nodes
        for (auto* n : cg.nodes) {
            auto* elkNode = graph->createNode(n->id);
            elkNode->setSize(n->width, n->height);
            elkNode->setLocation(n->x, n->y);
            
            // Copy ports
            for (const auto& p : n->ports) {
                auto* elkPort = graph->createPort(elkNode, p.id);
                elkPort->setSize(p.width, p.height);
                elkPort->setLocation(p.x, p.y);
                
                // Convert side
                if (p.side == "WEST") elkPort->setSide(elk::PortSide::WEST);
                else if (p.side == "EAST") elkPort->setSide(elk::PortSide::EAST);
                else if (p.side == "NORTH") elkPort->setSide(elk::PortSide::NORTH);
                else if (p.side == "SOUTH") elkPort->setSide(elk::PortSide::SOUTH);
            }
        }
        
        // Copy edges
        for (auto* e : cg.edges) {
            auto* src = graph->getNode(e->source->id);
            auto* tgt = graph->getNode(e->target->id);
            if (src && tgt) {
                graph->createEdge(src, tgt);
            }
        }
        
        return graph;
    }
    
    // Copy layout results back to CircuitGraph
    static void fromElk(CircuitGraph& cg, elk::ElkGraph* elkGraph) {
        // Copy node positions
        for (auto* n : cg.nodes) {
            auto* elkNode = elkGraph->getNode(n->id);
            if (elkNode) {
                n->x = elkNode->x();
                n->y = elkNode->y();
                n->width = elkNode->width();
                n->height = elkNode->height();
                
                // Copy port positions
                for (auto& p : n->ports) {
                    for (const auto& ep : elkNode->ports()) {
                        if (ep->id() == p.id) {
                            p.x = ep->x();
                            p.y = ep->y();
                            // Store side back
                            switch (ep->side()) {
                                case elk::PortSide::WEST: p.side = "WEST"; break;
                                case elk::PortSide::EAST: p.side = "EAST"; break;
                                case elk::PortSide::NORTH: p.side = "NORTH"; break;
                                case elk::PortSide::SOUTH: p.side = "SOUTH"; break;
                                default: break;
                            }
                            break;
                        }
                    }
                }
            }
        }
        
        // Copy edge bend points
        for (auto* e : cg.edges) {
            auto* elkEdge = findElkEdge(elkGraph, e->source, e->target);
            if (elkEdge) {
                e->points.clear();
                for (const auto& bp : elkEdge->bendPoints()) {
                    e->points.push_back(QPointF(bp.x(), bp.y()));
                }
            }
        }
    }
    
    // Run layout on CircuitGraph
    static void layout(CircuitGraph& cg) {
        auto* elkGraph = toElk(cg);
        
        // Configure options
        elkGraph->options().setNodePlacement(elk::NodePlacementStrategy::BRANDES_KOEPF);
        elkGraph->options().spacing().nodeNode = 60;
        elkGraph->options().spacing().nodeNodeBetweenLayers = 60;
        elkGraph->options().spacing().portPort = 10;
        
        // Run layout
        elk::layout(elkGraph);
        
        // Copy results
        fromElk(cg, elkGraph);
        
        delete elkGraph;
    }
    
private:
    static elk::ElkEdge* findElkEdge(elk::ElkGraph* g, CNode* src, CNode* tgt) {
        for (auto& e : g->root()->edges()) {
            if (e->source() && e->target()) {
                if (e->source()->id() == src->id && e->target()->id() == tgt->id) {
                    return e.get();
                }
            }
        }
        return nullptr;
    }
};

#endif // ELK_CIRCUIT_ADAPTER_HPP