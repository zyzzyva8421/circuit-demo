/**
 * ELK C++ Port - Main Graph Container
 * Migration from org.eclipse.elk.graph.ElkGraph
 */

#ifndef ELK_GRAPH_HPP
#define ELK_GRAPH_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <optional>
#include <variant>
#include "elk_node.hpp"
#include "elk_edge.hpp"
#include "elk_types.hpp"

namespace elk {

// Layout Options - matches ELK configuration
class LayoutOptions {
public:
    LayoutOptions() {
        // Default ELK layered options
        algorithm_ = Algorithm::LAYERED;
        spacing_.nodeNode = 60.0;
        spacing_.nodeNodeBetweenLayers = 60.0;
        spacing_.portPort = 10.0;
        nodePlacement_ = NodePlacementStrategy::BRANDES_KOEPF;
    }
    
    // Algorithm
    Algorithm algorithm() const { return algorithm_; }
    void setAlgorithm(Algorithm a) { algorithm_ = a; }
    
    // Spacing
    struct Spacing {
        double nodeNode = 60.0;
        double nodeNodeBetweenLayers = 60.0;
        double portPort = 10.0;
    };
    const Spacing& spacing() const { return spacing_; }
    Spacing& spacing() { return spacing_; }
    
    // Node placement
    NodePlacementStrategy nodePlacement() const { return nodePlacement_; }
    void setNodePlacement(NodePlacementStrategy s) { nodePlacement_ = s; }
    
    // Direction
    Direction direction() const { return direction_; }
    void setDirection(Direction d) { direction_ = d; }
    
    // Fixed alignment
    FixedAlignment fixedAlignment() const { return fixedAlignment_; }
    void setFixedAlignment(FixedAlignment a) { fixedAlignment_ = a; }
    
    // Port border offset
    double portBorderOffset() const { return portBorderOffset_; }
    void setPortBorderOffset(double d) { portBorderOffset_ = d; }
    
    // Layering constraint (per node)
    void setLayeringConstraint(LayeringConstraint c) { layeringConstraint_ = c; }
    LayeringConstraint layeringConstraint() const { return layeringConstraint_; }
    
    // Node size constraints (per node)
    void setNodeSizeConstraints(NodeSizeConstraints c) { nodeSizeConstraints_ = c; }
    NodeSizeConstraints nodeSizeConstraints() const { return nodeSizeConstraints_; }
    
    // Port constraints (per node)
    void setPortConstraints(PortConstraints c) { portConstraints_ = c; }
    PortConstraints portConstraints() const { return portConstraints_; }
    
    // Priority
    void setPriority(int p) { priority_ = p; }
    int priority() const { return priority_; }
    
    // Generic property access
    template<typename T>
    std::optional<T> getOption(const std::string& key) const {
        auto it = properties_.find(key);
        if (it != properties_.end()) {
            return std::get<T>(it->second);
        }
        return std::nullopt;
    }
    
    template<typename T>
    void setOption(const std::string& key, T value) {
        properties_[key] = value;
    }
    
private:
    Algorithm algorithm_;
    Direction direction_ = Direction::RIGHT;
    Spacing spacing_;
    NodePlacementStrategy nodePlacement_;
    FixedAlignment fixedAlignment_ = FixedAlignment::TERMINAL_ALIGNED;
    double portBorderOffset_ = 0.0;
    
    // Per-node options
    LayeringConstraint layeringConstraint_ = LayeringConstraint::ANY;
    NodeSizeConstraints nodeSizeConstraints_ = NodeSizeConstraints::NONE;
    PortConstraints portConstraints_ = PortConstraints::FREE;
    int priority_ = 0;
    
    std::map<std::string, std::variant<int, double, std::string, bool>> properties_;
};

// Main Graph container
class ElkGraph {
public:
    ElkGraph() : root_(std::make_unique<ElkNode>("root")) {}
    explicit ElkGraph(const std::string& name) : name_(name), root_(std::make_unique<ElkNode>("root")) {}
    virtual ~ElkGraph() = default;
    
    // Name
    const std::string& name() const { return name_; }
    void setName(const std::string& n) { name_ = n; }
    
    // Root node (represents the graph itself)
    ElkNode* root() const { return root_.get(); }
    
    // Layout options
    const LayoutOptions& options() const { return options_; }
    LayoutOptions& options() { return options_; }
    
    // Children of root (actual nodes in graph)
    void addChild(std::unique_ptr<ElkNode> node) {
        root_->addChild(std::move(node));
    }
    
    const std::vector<std::unique_ptr<ElkNode>>& children() const { 
        return root_->children(); 
    }
    
    // All nodes (recursive)
    std::vector<ElkNode*> getAllNodes() const {
        std::vector<ElkNode*> result;
        collectNodes(root_.get(), result);
        return result;
    }
    
    // All edges
    std::vector<ElkEdge*> getAllEdges() const {
        std::vector<ElkEdge*> result;
        collectEdges(root_.get(), result);
        return result;
    }
    
    // Find node by id
    ElkNode* getNode(const std::string& id) const {
        return findNode(root_.get(), id);
    }
    
    // Convenience: create node and add to root
    ElkNode* createNode(const std::string& id) {
        auto node = std::make_unique<ElkNode>(id);
        ElkNode* ptr = node.get();
        root_->addChild(std::move(node));
        return ptr;
    }
    
    // Convenience: create port on node
    ElkPort* createPort(ElkNode* node, const std::string& portId) {
        auto port = std::make_unique<ElkPort>(portId);
        ElkPort* ptr = port.get();
        node->addPort(std::move(port));
        return ptr;
    }
    
    // Convenience: create edge
    ElkEdge* createEdge(ElkConnectableShape* source, ElkConnectableShape* target) {
        auto edge = std::make_unique<ElkEdge>("e" + std::to_string(edgeCount_++));
        edge->setSource(source);
        edge->setTarget(target);
        source->addOutgoing(edge.get());
        target->addIncoming(edge.get());
        
        // Find containing node (lowest common ancestor of source and target)
        ElkNode* container = findLCA(source, target);
        edge->setContainingNode(container);
        
        ElkEdge* ptr = edge.get();
        container->addEdge(std::move(edge));
        return ptr;
    }
    
private:
    void collectNodes(ElkNode* node, std::vector<ElkNode*>& result) const {
        for (const auto& child : node->children()) {
            result.push_back(child.get());
            collectNodes(child.get(), result);
        }
    }
    
    void collectEdges(ElkNode* node, std::vector<ElkEdge*>& result) const {
        for (const auto& edge : node->edges()) {
            result.push_back(edge.get());
        }
        for (const auto& child : node->children()) {
            collectEdges(child.get(), result);
        }
    }
    
    ElkNode* findNode(ElkNode* node, const std::string& id) const {
        if (node->id() == id) return node;
        for (const auto& child : node->children()) {
            if (auto* found = findNode(child.get(), id)) return found;
        }
        return nullptr;
    }
    
    ElkNode* findLCA(ElkConnectableShape* a, ElkConnectableShape* b) const;
    
    std::string name_;
    std::unique_ptr<ElkNode> root_;
    LayoutOptions options_;
    size_t edgeCount_ = 0;
};

} // namespace elk

#endif // ELK_GRAPH_HPP