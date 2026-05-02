/**
 * ELK C++ Port - Shape Base Class
 * Migration from org.eclipse.elk.graph.ElkShape
 */

#ifndef ELK_SHAPE_HPP
#define ELK_SHAPE_HPP

#include <string>
#include <vector>

namespace elk {

// Forward declarations
class ElkEdge;
class ElkNode;

// Base Shape class - position and dimensions
class ElkShape {
public:
    ElkShape() : x_(0), y_(0), width_(0), height_(0) {}
    virtual ~ElkShape() = default;
    
    // Getters
    double x() const { return x_; }
    double y() const { return y_; }
    double width() const { return width_; }
    double height() const { return height_; }
    
    // Setters
    void setX(double x) { x_ = x; }
    void setY(double y) { y_ = y; }
    void setWidth(double w) { width_ = w; }
    void setHeight(double h) { height_ = h; }
    
    void setLocation(double x, double y) { x_ = x; y_ = y; }
    void setSize(double w, double h) { width_ = w; height_ = h; }
    void setBounds(double x, double y, double w, double h) {
        x_ = x; y_ = y; width_ = w; height_ = h;
    }
    
    // Center point
    double centerX() const { return x_ + width_ / 2.0; }
    double centerY() const { return y_ + height_ / 2.0; }
    
    // Right/Bottom edge
    double right() const { return x_ + width_; }
    double bottom() const { return y_ + height_; }
    
    // Check if point is inside
    bool contains(double px, double py) const {
        return px >= x_ && px <= x_ + width_ &&
               py >= y_ && py <= y_ + height_;
    }
    
protected:
    double x_;
    double y_;
    double width_;
    double height_;
};

// Connectable Shape - can be endpoint of edge (extends ElkShape)
class ElkConnectableShape : public ElkShape {
public:
    ElkConnectableShape() : ElkShape(), incomingEdges_(), outgoingEdges_() {}
    virtual ~ElkConnectableShape() = default;
    
    // Edge management
    void addIncoming(ElkEdge* edge) { incomingEdges_.push_back(edge); }
    void addOutgoing(ElkEdge* edge) { outgoingEdges_.push_back(edge); }
    void removeIncoming(ElkEdge* edge);
    void removeOutgoing(ElkEdge* edge);
    
    const std::vector<ElkEdge*>& incomingEdges() const { return incomingEdges_; }
    const std::vector<ElkEdge*>& outgoingEdges() const { return outgoingEdges_; }
    
    // Utility
    bool isSource() const { return !outgoingEdges_.empty(); }
    bool isSink() const { return !incomingEdges_.empty(); }
    bool isConnected() const { return !incomingEdges_.empty() || !outgoingEdges_.empty(); }
    
private:
    std::vector<ElkEdge*> incomingEdges_;
    std::vector<ElkEdge*> outgoingEdges_;
};

} // namespace elk

#endif // ELK_SHAPE_HPP