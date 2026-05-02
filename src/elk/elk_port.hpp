/**
 * ELK C++ Port - Port Data Structure
 * Migration from org.eclipse.elk.graph.ElkPort
 */

#ifndef ELK_PORT_HPP
#define ELK_PORT_HPP

#include <string>
#include <vector>
#include <memory>
#include "elk_shape.hpp"
#include "elk_types.hpp"

namespace elk {

// Forward declaration
class ElkNode;

// Port - connection point on a node
class ElkPort : public ElkConnectableShape {
public:
    ElkPort() : ElkConnectableShape(), parent_(nullptr), side_(PortSide::UNDEFINED), index_(0) {}
    explicit ElkPort(const std::string& id) : ElkConnectableShape(), id_(id), parent_(nullptr), side_(PortSide::UNDEFINED), index_(0) {}
    virtual ~ElkPort() = default;
    
    // Id
    const std::string& id() const { return id_; }
    void setId(const std::string& id) { id_ = id; }
    
    // Parent node
    ElkNode* parent() const { return parent_; }
    void setParent(ElkNode* node) { parent_ = node; }
    
    // Port side (NORTH/SOUTH/EAST/WEST)
    PortSide side() const { return side_; }
    void setSide(PortSide s) { side_ = s; }
    
    // Index for ordering ports on same side
    int index() const { return index_; }
    void setIndex(int i) { index_ = i; }
    
    // Copy with new id
    std::unique_ptr<ElkPort> clone(const std::string& newId) const {
        auto p = std::make_unique<ElkPort>(newId);
        p->setSize(width(), height());
        p->setLocation(x(), y());
        p->setSide(side_);
        p->setIndex(index_);
        return p;
    }
    
private:
    std::string id_;
    ElkNode* parent_;  // non-owning
    PortSide side_;
    int index_;
};

} // namespace elk

#endif // ELK_PORT_HPP