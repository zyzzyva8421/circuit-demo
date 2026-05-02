/**
 * ELK C++ Port - Edge and BendPoint Data Structures
 * Migration from org.eclipse.elk.graph.ElkEdge and ElkBendPoint
 */

#ifndef ELK_EDGE_HPP
#define ELK_EDGE_HPP

#include <vector>
#include <memory>
#include <string>
#include "elk_shape.hpp"

namespace elk {

// Forward declarations
class ElkNode;
class ElkPort;

// Bend Point - routing point on an edge
class ElkBendPoint {
public:
    ElkBendPoint() : x_(0), y_(0) {}
    ElkBendPoint(double x, double y) : x_(x), y_(y) {}
    
    double x() const { return x_; }
    double y() const { return y_; }
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void set(double x, double y) { x_ = x; y_ = y; }
    
private:
    double x_;
    double y_;
};

// Edge - connects source(s) to target(s)
// Supports both regular edges and hyperedges
class ElkEdge : public ElkShape {
public:
    ElkEdge() : ElkShape(), containingNode_(nullptr) {}
    explicit ElkEdge(const std::string& id) : ElkShape(), id_(id), containingNode_(nullptr) {}
    virtual ~ElkEdge() = default;
    
    // Id
    const std::string& id() const { return id_; }
    void setId(const std::string& id) { id_ = id; }
    
    // Containing node
    ElkNode* containingNode() const { return containingNode_; }
    void setContainingNode(ElkNode* node) { containingNode_ = node; }
    
    // Sources and Targets (can be nodes or ports)
    void addSource(ElkConnectableShape* src) { sources_.push_back(src); }
    void addTarget(ElkConnectableShape* tgt) { targets_.push_back(tgt); }
    void setSource(ElkConnectableShape* src) { sources_ = {src}; }
    void setTarget(ElkConnectableShape* tgt) { targets_ = {tgt}; }
    
    void clearSources() { sources_.clear(); }
    void clearTargets() { targets_.clear(); }
    
    const std::vector<ElkConnectableShape*>& sources() const { return sources_; }
    const std::vector<ElkConnectableShape*>& targets() const { return targets_; }
    
    // Edge type checks
    bool isHyperedge() const { return sources_.size() > 1 || targets_.size() > 1; }
    bool isSimple() const { return sources_.size() == 1 && targets_.size() == 1; }
    bool isSelfLoop() const;
    bool isConnected() const { return !sources_.empty() && !targets_.empty(); }
    
    // Single source/target access (for simple edges)
    ElkConnectableShape* source() const { 
        return sources_.empty() ? nullptr : sources_.front(); 
    }
    ElkConnectableShape* target() const { 
        return targets_.empty() ? nullptr : targets_.front(); 
    }
    
    // Bend points
    void addBendPoint(double x, double y) {
        bendPoints_.emplace_back(x, y);
    }
    void clearBendPoints() { bendPoints_.clear(); }
    
    const std::vector<ElkBendPoint>& bendPoints() const { return bendPoints_; }
    std::vector<ElkBendPoint>& bendPoints() { return bendPoints_; }
    
    // Convenience: get all routing points (source -> bends -> target)
    std::vector<std::pair<double, double>> getRoutingPoints() const;
    
private:
    std::string id_;
    ElkNode* containingNode_;  // non-owning
    std::vector<ElkConnectableShape*> sources_;
    std::vector<ElkConnectableShape*> targets_;
    std::vector<ElkBendPoint> bendPoints_;
};

} // namespace elk

#endif // ELK_EDGE_HPP