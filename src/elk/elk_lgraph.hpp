/**
 * ELK C++ Port - Internal Layered Graph Structures
 * Migration from org.eclipse.elk.alg.layered.graph
 */

#ifndef ELK_LGRAPH_HPP
#define ELK_LGRAPH_HPP

#include <vector>
#include <memory>
#include <map>
#include <set>
#include <limits>
#include "elk_graph.hpp"

namespace elk {

// Forward declarations
class LNode;
class LPort;
class LEdge;
class Layer;
class LGraph;

// ============== LPort ==============
class LPort {
public:
    LPort(ElkPort* port, LNode* node) 
        : elkPort_(port), node_(node), side_(PortSide::UNDEFINED), index_(0),
          originalSide_(port ? port->side() : PortSide::UNDEFINED),
          anchorX_(0), anchorY_(0) {}
    
    ~LPort() = default;
    
    ElkPort* elkPort() const { return elkPort_; }
    LNode* node() const { return node_; }
    
    PortSide side() const { return side_; }
    void setSide(PortSide s) { side_ = s; }
    
    int index() const { return index_; }
    void setIndex(int i) { index_ = i; }
    
    PortSide originalSide() const { return originalSide_; }
    void setOriginalSide(PortSide s) { originalSide_ = s; }
    
    double anchorX() const { return anchorX_; }
    double anchorY() const { return anchorY_; }
    void setAnchor(double x, double y) { anchorX_ = x; anchorY_ = y; }
    
private:
    ElkPort* elkPort_;
    LNode* node_;
    PortSide side_;
    int index_;
    PortSide originalSide_;
    double anchorX_;
    double anchorY_;
};

// ============== LNode ==============
class LNode {
public:
    LNode(ElkNode* node) 
        : elkNode_(node), layerIndex_(-1), position_(0),
          width_(node->width()), height_(node->height()) {}
    
    ~LNode() = default;
    
    ElkNode* elkNode() const { return elkNode_; }
    
    int layer() const { return layerIndex_; }
    void setLayer(int layer) { layerIndex_ = layer; }
    bool hasLayer() const { return layerIndex_ >= 0; }
    
    int position() const { return position_; }
    void setPosition(int pos) { position_ = pos; }
    
    double width() const { return width_; }
    double height() const { return height_; }
    void setSize(double w, double h) { width_ = w; height_ = h; }
    
    void addPort(LPort* port) { ports_.push_back(port); }
    const std::vector<LPort*>& ports() const { return ports_; }
    
    int longestPathHeight() const { return longestPathHeight_; }
    void setLongestPathHeight(int h) { longestPathHeight_ = h; }
    
    double barycenter() const { return barycenter_; }
    void setBarycenter(double b) { barycenter_ = b; }
    
    int columnSpan() const { return columnSpan_; }
    void setColumnSpan(int span) { columnSpan_ = span; }
    
private:
    ElkNode* elkNode_;
    int layerIndex_;
    int position_;
    double width_;
    double height_;
    std::vector<LPort*> ports_;
    int longestPathHeight_ = 1;
    double barycenter_ = 0;
    int columnSpan_ = 1;
};

// ============== LEdge ==============
class LEdge {
public:
    LEdge(ElkEdge* edge, LNode* source, LNode* target)
        : elkEdge_(edge), source_(source), target_(target),
          sourcePort_(nullptr), targetPort_(nullptr) {}
    
    ~LEdge() = default;
    
    ElkEdge* elkEdge() const { return elkEdge_; }
    LNode* source() const { return source_; }
    LNode* target() const { return target_; }
    
    LPort* sourcePort() const { return sourcePort_; }
    void setSourcePort(LPort* p) { sourcePort_ = p; }
    
    LPort* targetPort() const { return targetPort_; }
    void setTargetPort(LPort* p) { targetPort_ = p; }
    
    bool isInverted() const { return isInverted_; }
    void setInverted(bool inv) { isInverted_ = inv; }
    
    int weight() const { return weight_; }
    void setWeight(int w) { weight_ = w; }
    
private:
    ElkEdge* elkEdge_;
    LNode* source_;
    LNode* target_;
    LPort* sourcePort_;
    LPort* targetPort_;
    bool isInverted_ = false;
    int weight_ = 1;
};

// ============== Layer ==============
class Layer {
public:
    Layer(int index) : index_(index) {}
    ~Layer() = default;
    
    int index() const { return index_; }
    
    void addNode(LNode* node) { nodes_.push_back(node); }
    
    const std::vector<LNode*>& nodes() const { return nodes_; }
    std::vector<LNode*>& nodes() { return nodes_; }
    
    double maxHeight() const {
        double maxH = 0;
        for (auto* n : nodes_) {
            maxH = std::max(maxH, n->height());
        }
        return maxH;
    }
    
private:
    int index_;
    std::vector<LNode*> nodes_;
};

// ============== LGraph ==============
class LGraph {
public:
    explicit LGraph(ElkGraph* graph) : elkGraph_(graph) {}
    ~LGraph() = default;
    
    ElkGraph* elkGraph() const { return elkGraph_; }
    
    void addLayer(std::unique_ptr<Layer> layer) {
        layers_.push_back(std::move(layer));
    }
    
    const std::vector<std::unique_ptr<Layer>>& layers() const { return layers_; }
    std::vector<std::unique_ptr<Layer>>& layers() { return layers_; }
    
    void addNode(std::unique_ptr<LNode> node) {
        lnodes_.push_back(std::move(node));
    }
    
    const std::vector<std::unique_ptr<LNode>>& nodes() const { return lnodes_; }
    std::vector<std::unique_ptr<LNode>>& nodes() { return lnodes_; }
    
    int layerCount() const { return static_cast<int>(layers_.size()); }
    
    void build() {
        // Build internal graph from ElkGraph
        for (auto& node : elkGraph_->children()) {
            addNode(std::make_unique<LNode>(node.get()));
        }
    }
    
    void clear() {
        layers_.clear();
        lnodes_.clear();
        ledges_.clear();
    }
    
private:
    ElkGraph* elkGraph_;
    std::vector<std::unique_ptr<Layer>> layers_;
    std::vector<std::unique_ptr<LNode>> lnodes_;
    std::vector<std::unique_ptr<LEdge>> ledges_;
};

} // namespace elk

#endif // ELK_LGRAPH_HPP