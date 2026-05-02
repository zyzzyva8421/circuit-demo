/**
 * ELK C++ Port - Implementation
 */

#include "elk_graph.hpp"
#include "elk_node.hpp"

namespace elk {

// Find lowest common ancestor
ElkNode* ElkGraph::findLCA(ElkConnectableShape* a, ElkConnectableShape* b) const {
    // Get containing nodes
    ElkNode* nodeA = dynamic_cast<ElkNode*>(a);
    ElkNode* nodeB = dynamic_cast<ElkNode*>(b);
    
    if (a->outgoingEdges().empty() == false) {
        if (auto* edge = a->outgoingEdges().front()) {
            nodeA = edge->containingNode();
        }
    }
    if (b->incomingEdges().empty() == false) {
        if (auto* edge = b->incomingEdges().front()) {
            nodeB = edge->containingNode();
        }
    }
    
    if (!nodeA || !nodeB) return root_.get();
    
    // Collect ancestors of A
    std::vector<ElkNode*> ancestorsA;
    for (ElkNode* p = nodeA; p; p = p->parent()) {
        ancestorsA.push_back(p);
    }
    
    // Find first common ancestor
    for (ElkNode* p = nodeB; p; p = p->parent()) {
        for (auto* a : ancestorsA) {
            if (p == a) return p;
        }
    }
    
    return root_.get();
}

} // namespace elk