/**
 * ELK C++ Port - Node Data Structure
 * Migration from org.eclipse.elk.graph.ElkNode
 */

#ifndef ELK_NODE_HPP
#define ELK_NODE_HPP

#include <vector>
#include <memory>
#include <string>
#include "elk_port.hpp"
#include "elk_edge.hpp"
#include "elk_types.hpp"

namespace elk {

// Forward declaration
class ElkNode;

// Node - can contain ports, children, and edges
// Supports hierarchical nesting
class ElkNode : public ElkConnectableShape {
public:
    ElkNode() : ElkConnectableShape(), parent_(nullptr) {}
    explicit ElkNode(const std::string& id) : ElkConnectableShape(), id_(id), parent_(nullptr) {}
    virtual ~ElkNode() = default;
    
    // Id
    const std::string& id() const { return id_; }
    void setId(const std::string& id) { id_ = id; }
    
    // Per-node layout options (like JSON layoutOptions)
    void setLayeringConstraint(LayeringConstraint c) { layeringConstraint_ = c; }
    LayeringConstraint layeringConstraint() const { return layeringConstraint_; }
    
    void setNodeSizeConstraints(NodeSizeConstraints c) { nodeSizeConstraints_ = c; }
    NodeSizeConstraints nodeSizeConstraints() const { return nodeSizeConstraints_; }
    
    void setPortConstraints(PortConstraints c) { portConstraints_ = c; }
    PortConstraints portConstraints() const { return portConstraints_; }
    
    void setPriority(int p) { priority_ = p; }
    int priority() const { return priority_; }
    
    void setPortBorderOffset(double d) { portBorderOffset_ = d; }
    double portBorderOffset() const { return portBorderOffset_; }
    
    // Parent node (for hierarchy)
    ElkNode* parent() const { return parent_; }
    void setParent(ElkNode* node) { parent_ = node; }
    bool hasParent() const { return parent_ != nullptr; }
    bool isRoot() const { return parent_ == nullptr; }
    
    // Ports
    void addPort(std::unique_ptr<ElkPort> port) {
        port->setParent(this);
        ports_.push_back(std::move(port));
    }
    
    ElkPort* getPort(const std::string& portId) const {
        for (const auto& p : ports_) {
            if (p->id() == portId) return p.get();
        }
        return nullptr;
    }
    
    const std::vector<std::unique_ptr<ElkPort>>& ports() const { return ports_; }
    std::vector<std::unique_ptr<ElkPort>>& ports() { return ports_; }
    
    bool hasPorts() const { return !ports_.empty(); }
    size_t portCount() const { return ports_.size(); }
    
    // Children (hierarchical nodes)
    void addChild(std::unique_ptr<ElkNode> child) {
        child->setParent(this);
        children_.push_back(std::move(child));
    }
    
    ElkNode* getChild(const std::string& childId) const {
        for (const auto& c : children_) {
            if (c->id() == childId) return c.get();
        }
        return nullptr;
    }
    
    const std::vector<std::unique_ptr<ElkNode>>& children() const { return children_; }
    std::vector<std::unique_ptr<ElkNode>>& children() { return children_; }
    
    bool hasChildren() const { return !children_.empty(); }
    size_t childCount() const { return children_.size(); }
    
    // Contained edges
    void addEdge(std::unique_ptr<ElkEdge> edge) {
        edge->setContainingNode(this);
        edges_.push_back(std::move(edge));
    }
    
    const std::vector<std::unique_ptr<ElkEdge>>& edges() const { return edges_; }
    std::vector<std::unique_ptr<ElkEdge>>& edges() { return edges_; }
    
    // Global position (accounting for hierarchy)
    double globalX() const;
    double globalY() const;
    
    // Port with specific side
    std::vector<ElkPort*> getPortsOnSide(PortSide side) const {
        std::vector<ElkPort*> result;
        for (const auto& p : ports_) {
            if (p->side() == side) result.push_back(p.get());
        }
        return result;
    }
    
private:
    std::string id_;
    ElkNode* parent_;  // non-owning
    std::vector<std::unique_ptr<ElkPort>> ports_;
    std::vector<std::unique_ptr<ElkNode>> children_;
    std::vector<std::unique_ptr<ElkEdge>> edges_;
    
    // Per-node layout options (like JSON layoutOptions)
    LayeringConstraint layeringConstraint_ = LayeringConstraint::ANY;
    NodeSizeConstraints nodeSizeConstraints_ = NodeSizeConstraints::NONE;
    PortConstraints portConstraints_ = PortConstraints::FREE;
    int priority_ = 0;
    double portBorderOffset_ = 0.0;
};

// Implementation of global position
inline double ElkNode::globalX() const {
    double x = ElkShape::x();
    const ElkNode* p = parent_;
    while (p) {
        x += p->x();
        p = p->parent_;
    }
    return x;
}

inline double ElkNode::globalY() const {
    double y = ElkShape::y();
    const ElkNode* p = parent_;
    while (p) {
        y += p->y();
        p = p->parent_;
    }
    return y;
}

} // namespace elk

#endif // ELK_NODE_HPP