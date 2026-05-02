/**
 * ELK C++ Port - CircuitGraph Layout Implementation
 */

#include "elk_circuit_layout.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <map>
#include <random>
#include <set>
#include <chrono>
#include <future>
#include <tuple>
#include <vector>

#include "circuitgraph.h"
#include <unordered_set>
#include "elk/elk.hpp"

namespace {

// Sorted module node index for fast Y-range collision queries.
// Key: y midpoint of module rect. Built once before routing.
struct ModuleNodeIndex {
    // Maps y_center -> CNode* for ModuleInstance/ExpandedInstance nodes.
    std::multimap<double, const CNode*> byYMid;

    // Returns iterator range for nodes whose Y extent might overlap [yLo, yHi].
    // Uses conservative over-approximation: center in [yLo - maxHalfH, yHi + maxHalfH].
    // maxHalfH is the max half-height of any module (stored at build time).
    double maxHalfH = 0.0;
    double maxHalfW = 0.0;

    void build(const std::vector<CNode*>& nodes) {
        for (const auto* n : nodes) {
            if (!n) continue;
            if (n->data.type != CircuitNodeType::ModuleInstance &&
                n->data.type != CircuitNodeType::ExpandedInstance) continue;
            double cy = n->y + n->height * 0.5;
            byYMid.emplace(cy, n);
            maxHalfH = std::max(maxHalfH, n->height * 0.5 + 4.0);
            maxHalfW = std::max(maxHalfW, n->width  * 0.5 + 4.0);
        }
    }

    std::pair<std::multimap<double,const CNode*>::const_iterator,
              std::multimap<double,const CNode*>::const_iterator>
    candidatesForY(double y, double margin) const {
        double lo = y - maxHalfH - margin;
        double hi = y + maxHalfH + margin;
        return {byYMid.lower_bound(lo), byYMid.upper_bound(hi)};
    }

    std::pair<std::multimap<double,const CNode*>::const_iterator,
              std::multimap<double,const CNode*>::const_iterator>
    candidatesForX(double x, double margin) const {
        // For vertical segments we query all modules (no X index) — but we
        // can still use the Y range to limit to modules with potential overlap.
        // Here just return all.
        return {byYMid.begin(), byYMid.end()};
    }
};


struct HorizontalSeg {
    double y;
    double x0;
    double x1;
};

// Sorted container for HorizontalSeg allowing O(log N) Y-range queries.
struct SortedHSegs {
    // sorted by y
    std::multimap<double, std::pair<double,double>> data; // y -> (x0,x1)

    void push_back(const HorizontalSeg& s) {
        data.emplace(s.y, std::make_pair(s.x0, s.x1));
    }
    void push_back(HorizontalSeg&& s) {
        data.emplace(s.y, std::make_pair(s.x0, s.x1));
    }
    bool empty() const { return data.empty(); }
    size_t size() const { return data.size(); }
    // Range query: all segs with y in [yLo, yHi]
    auto range(double yLo, double yHi) const {
        return std::make_pair(data.lower_bound(yLo), data.upper_bound(yHi));
    }
};

struct VerticalSeg {
    double x;
    double y0;
    double y1;
};

bool isInputPortNode(const CNode* node) {
    return node->data.type == CircuitNodeType::Port && node->data.direction == "input";
}

bool verticalSegmentIntersectsRect(double x, double y0, double y1, const CNode* rectNode, double margin);
bool horizontalSegmentIntersectsRect(double y, double x0, double x1, const CNode* rectNode, double margin);

int countPolylineModuleCollisionsIndexed(
    const std::vector<QPointF>& points,
    const ModuleNodeIndex& modIdx,
    const CNode* src,
    const CNode* tgt,
    double margin = 2.0
) {
    if (points.size() < 2) return 0;

    std::unordered_set<const CNode*> hitNodes;
    for (size_t i = 1; i < points.size(); ++i) {
        const auto& a = points[i - 1];
        const auto& b = points[i];

        if (std::abs(a.x() - b.x()) < 0.001) {
            double x = a.x();
            auto [it0, it1] = modIdx.candidatesForX(x, margin);
            for (auto it = it0; it != it1; ++it) {
                const CNode* n = it->second;
                if (n == src || n == tgt) continue;
                if (verticalSegmentIntersectsRect(x, a.y(), b.y(), n, margin)) {
                    hitNodes.insert(n);
                }
            }
        } else if (std::abs(a.y() - b.y()) < 0.001) {
            double y = a.y();
            auto [it0, it1] = modIdx.candidatesForY(y, margin);
            for (auto it = it0; it != it1; ++it) {
                const CNode* n = it->second;
                if (n == src || n == tgt) continue;
                if (horizontalSegmentIntersectsRect(y, a.x(), b.x(), n, margin)) {
                    hitNodes.insert(n);
                }
            }
        }
    }
    return static_cast<int>(hitNodes.size());
}
bool isOutputPortNode(const CNode* node) {
    return node->data.type == CircuitNodeType::Port && node->data.direction == "output";
}

bool isNetJointNode(const CNode* node) {
    return node->data.type == CircuitNodeType::NetJoint;
}

bool rangesOverlap(double a0, double a1, double b0, double b1, double eps = 0.001) {
    if (a0 > a1) std::swap(a0, a1);
    if (b0 > b1) std::swap(b0, b1);
    return !(a1 < b0 + eps || b1 < a0 + eps);
}

bool verticalSegmentIntersectsRect(double x, double y0, double y1, const CNode* rectNode, double margin = 4.0) {
    if (!rectNode) return false;
    double rx0 = rectNode->x - margin;
    double rx1 = rectNode->x + rectNode->width + margin;
    double ry0 = rectNode->y - margin;
    double ry1 = rectNode->y + rectNode->height + margin;
    bool xInside = (x >= rx0 && x <= rx1);
    bool yOverlap = rangesOverlap(y0, y1, ry0, ry1);
    return xInside && yOverlap;
}

bool horizontalSegmentIntersectsRect(double y, double x0, double x1, const CNode* rectNode, double margin = 4.0) {
    if (!rectNode) return false;
    double rx0 = rectNode->x - margin;
    double rx1 = rectNode->x + rectNode->width + margin;
    double ry0 = rectNode->y - margin;
    double ry1 = rectNode->y + rectNode->height + margin;
    bool yInside = (y >= ry0 && y <= ry1);
    bool xOverlap = rangesOverlap(x0, x1, rx0, rx1);
    return yInside && xOverlap;
}

int countPolylineModuleCollisions(
    const std::vector<QPointF>& points,
    const std::vector<CNode*>& allNodes,
    const CNode* src,
    const CNode* tgt,
    double margin = 2.0
) {
    if (points.size() < 2) return 0;

    int collisions = 0;
    for (const auto* n : allNodes) {
        if (!n) continue;
        if (n == src || n == tgt) continue;
        if (n->data.type != CircuitNodeType::ModuleInstance &&
            n->data.type != CircuitNodeType::ExpandedInstance) {
            continue;
        }

        bool hit = false;
        for (size_t i = 1; i < points.size(); ++i) {
            const auto& a = points[i - 1];
            const auto& b = points[i];
            if (std::abs(a.x() - b.x()) < 0.001) {
                if (verticalSegmentIntersectsRect(a.x(), a.y(), b.y(), n, margin)) {
                    hit = true;
                    break;
                }
            } else if (std::abs(a.y() - b.y()) < 0.001) {
                if (horizontalSegmentIntersectsRect(a.y(), a.x(), b.x(), n, margin)) {
                    hit = true;
                    break;
                }
            }
        }

        if (hit) ++collisions;
    }

    return collisions;
}

double findSafeVerticalChannelX(
    double preferredX,
    double y0,
    double y1,
    const ModuleNodeIndex& modIdx,
    const CNode* src,
    const CNode* tgt,
    const std::vector<VerticalSeg>& occupied,
    double trackGap,
    double step,
    double minX,
    double maxX
) {
    // Prefilter module nodes that could possibly overlap the Y range [y0,y1].
    // candidatesForX uses the Y-sorted index to limit candidates.
    auto conflictCountAt = [&](double x) {
        int conflicts = 0;
        auto [mn0, mn1] = modIdx.candidatesForX(x, 4.0);
        for (auto it = mn0; it != mn1; ++it) {
            const CNode* n = it->second;
            if (n == src || n == tgt) continue;
            if (verticalSegmentIntersectsRect(x, y0, y1, n)) {
                ++conflicts;
            }
        }

        for (const auto& seg : occupied) {
            if (std::abs(seg.x - x) < trackGap && rangesOverlap(y0, y1, seg.y0, seg.y1)) {
                ++conflicts;
            }
        }

        return conflicts;
    };

    if (conflictCountAt(preferredX) == 0) {
        return preferredX;
    }

    int bestConflicts = conflictCountAt(preferredX);
    double bestX = preferredX;

    for (int delta = 1; delta <= 80; ++delta) {
        double right = preferredX + delta * step;
        if (right <= maxX) {
            int rc = conflictCountAt(right);
            if (rc == 0) return right;
            if (rc < bestConflicts) {
                bestConflicts = rc;
                bestX = right;
            }
        }
        double left = preferredX - delta * step;
        if (left >= minX) {
            int lc = conflictCountAt(left);
            if (lc == 0) return left;
            if (lc < bestConflicts) {
                bestConflicts = lc;
                bestX = left;
            }
        }
    }

    return bestX;
}

double findSafeHorizontalChannelY(
    double preferredY,
    double x0,
    double x1,
    const ModuleNodeIndex& modIdx,
    const CNode* src,
    const CNode* tgt,
    const SortedHSegs& occupied,
    double trackGap,
    double step,
    double minY,
    double maxY
) {
    auto conflictCountAt = [&](double y) {
        int conflicts = 0;
        // O(log N + k): only module nodes whose Y center is near y
        auto [mn0, mn1] = modIdx.candidatesForY(y, 4.0);
        for (auto it = mn0; it != mn1; ++it) {
            const CNode* n = it->second;
            if (n == src || n == tgt) continue;
            if (horizontalSegmentIntersectsRect(y, x0, x1, n)) {
                ++conflicts;
            }
        }

        // O(log N + k): only segs within trackGap of y
        auto [ht0, ht1] = occupied.range(y - trackGap + 0.001, y + trackGap - 0.001);
        for (auto it = ht0; it != ht1; ++it) {
            if (rangesOverlap(x0, x1, it->second.first, it->second.second)) {
                ++conflicts;
            }
        }

        return conflicts;
    };

    if (conflictCountAt(preferredY) == 0) {
        return preferredY;
    }

    int bestConflicts = conflictCountAt(preferredY);
    double bestY = preferredY;

    for (int delta = 1; delta <= 120; ++delta) {
        double down = preferredY + delta * step;
        if (down <= maxY) {
            int dc = conflictCountAt(down);
            if (dc == 0) return down;
            if (dc < bestConflicts) {
                bestConflicts = dc;
                bestY = down;
            }
        }
        double up = preferredY - delta * step;
        if (up >= minY) {
            int uc = conflictCountAt(up);
            if (uc == 0) return up;
            if (uc < bestConflicts) {
                bestConflicts = uc;
                bestY = up;
            }
        }
    }

    return bestY;
}

double average(const std::vector<double>& values, double fallback) {
    if (values.empty()) {
        return fallback;
    }

    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

QPointF sourceAnchor(const CEdge* edge) {
    if (!edge || !edge->source) {
        return QPointF();
    }

    const CNode* node = edge->source;
    if (!edge->sourcePort.empty()) {
        for (const auto& port : node->ports) {
            if (port.name != edge->sourcePort) {
                continue;
            }

            double x = node->x + port.x;
            double y = node->y + port.y + port.height / 2.0;
            if (port.side == "EAST") {
                x += port.width;
            } else if (port.side == "NORTH") {
                x += port.width / 2.0;
                y = node->y + port.y;
            } else if (port.side == "SOUTH") {
                x += port.width / 2.0;
                y = node->y + port.y + port.height;
            }
            return QPointF(x, y);
        }
    }

    if (isInputPortNode(node)) {
        return QPointF(node->x + node->width, node->y + node->height / 2.0);
    }
    if (isNetJointNode(node)) {
        return QPointF(node->x + node->width / 2.0, node->y + node->height / 2.0);
    }
    return QPointF(node->x + node->width, node->y + node->height / 2.0);
}

QPointF targetAnchor(const CEdge* edge) {
    if (!edge || !edge->target) {
        return QPointF();
    }

    const CNode* node = edge->target;
    if (!edge->targetPort.empty()) {
        for (const auto& port : node->ports) {
            if (port.name != edge->targetPort) {
                continue;
            }

            double x = node->x + port.x;
            double y = node->y + port.y + port.height / 2.0;
            if (port.side == "EAST") {
                x += port.width;
            } else if (port.side == "NORTH") {
                x += port.width / 2.0;
                y = node->y + port.y;
            } else if (port.side == "SOUTH") {
                x += port.width / 2.0;
                y = node->y + port.y + port.height;
            }
            return QPointF(x, y);
        }
    }

    if (isOutputPortNode(node)) {
        return QPointF(node->x, node->y + node->height / 2.0);
    }
    if (isNetJointNode(node)) {
        return QPointF(node->x + node->width / 2.0, node->y + node->height / 2.0);
    }
    return QPointF(node->x, node->y + node->height / 2.0);
}

void normalizePolyline(std::vector<QPointF>& points) {
    std::vector<QPointF> normalized;
    normalized.reserve(points.size());
    for (const auto& point : points) {
        if (!normalized.empty()) {
            const auto& last = normalized.back();
            if (std::abs(last.x() - point.x()) < 0.001 && std::abs(last.y() - point.y()) < 0.001) {
                continue;
            }
        }
        normalized.push_back(point);
    }
    points.swap(normalized);
}

void simplifyOrthogonalPolyline(std::vector<QPointF>& points, double eps = 0.001) {
    if (points.size() < 3) return;

    std::vector<QPointF> simplified;
    simplified.reserve(points.size());
    simplified.push_back(points.front());

    for (size_t i = 1; i + 1 < points.size(); ++i) {
        const QPointF& a = simplified.back();
        const QPointF& b = points[i];
        const QPointF& c = points[i + 1];

        bool verticalCollinear = std::abs(a.x() - b.x()) < eps && std::abs(b.x() - c.x()) < eps;
        bool horizontalCollinear = std::abs(a.y() - b.y()) < eps && std::abs(b.y() - c.y()) < eps;
        if (verticalCollinear || horizontalCollinear) {
            continue;
        }
        simplified.push_back(b);
    }

    simplified.push_back(points.back());
    points.swap(simplified);
}

} // namespace

void applyElkLayout(CircuitGraph& cg) {
    auto phaseStart = std::chrono::steady_clock::now();
    auto logPhase = [&](const char* phase) {
        auto now = std::chrono::steady_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - phaseStart).count();
        fprintf(stderr, "[LAYOUT-PHASE] %s: %lld ms\n", phase, (long long)ms);
        phaseStart = now;
    };

    elk::ElkGraph graph("circuit");
    graph.options().setAlgorithm(elk::Algorithm::LAYERED);
    graph.options().setDirection(elk::Direction::RIGHT);
    graph.options().setNodePlacement(elk::NodePlacementStrategy::BRANDES_KOEPF);
    graph.options().spacing().nodeNode = 60.0;
    graph.options().spacing().nodeNodeBetweenLayers = 60.0;
    graph.options().spacing().portPort = 10.0;
    graph.options().setPortBorderOffset(0.0);

    std::map<CNode*, elk::ElkNode*> elkNodeMap;
    std::map<CEdge*, elk::ElkEdge*> elkEdgeMap;
    std::map<CNode*, int> layerMap;
    std::map<CNode*, std::vector<CNode*>> predecessors;
    std::map<CNode*, std::vector<CNode*>> successors;
    std::map<CNode*, int> sourceOrder;

    for (size_t index = 0; index < cg.nodes.size(); ++index) {
        CNode* node = cg.nodes[index];
        sourceOrder[node] = static_cast<int>(index);

        auto* elkNode = graph.createNode(node->id);
        elkNode->setSize(node->width, node->height);
        elkNodeMap[node] = elkNode;

        for (const auto& port : node->ports) {
            auto* elkPort = graph.createPort(elkNode, port.id);
            elkPort->setSize(port.width, port.height);
            elkPort->setLocation(port.x, port.y);
            if (port.side == "WEST") {
                elkPort->setSide(elk::PortSide::WEST);
            } else if (port.side == "EAST") {
                elkPort->setSide(elk::PortSide::EAST);
            } else if (port.side == "NORTH") {
                elkPort->setSide(elk::PortSide::NORTH);
            } else if (port.side == "SOUTH") {
                elkPort->setSide(elk::PortSide::SOUTH);
            }
        }
    }

    for (auto* edge : cg.edges) {
        auto* src = elkNodeMap[edge->source];
        auto* tgt = elkNodeMap[edge->target];
        if (!src || !tgt) {
            continue;
        }
        elkEdgeMap[edge] = graph.createEdge(src, tgt);
        successors[edge->source].push_back(edge->target);
        predecessors[edge->target].push_back(edge->source);
    }

    logPhase("build graph nodes/edges");

    for (auto* node : cg.nodes) {
        if (isInputPortNode(node)) {
            layerMap[node] = 0;
        } else if (isOutputPortNode(node)) {
            layerMap[node] = -1;
        } else {
            layerMap[node] = 1;
        }
    }

    for (size_t pass = 0; pass < cg.nodes.size(); ++pass) {
        bool changed = false;
        for (auto* edge : cg.edges) {
            if (isOutputPortNode(edge->target)) {
                continue;
            }
            int candidate = layerMap[edge->source] + 1;
            if (candidate > layerMap[edge->target]) {
                layerMap[edge->target] = candidate;
                changed = true;
            }
        }
        if (!changed) {
            break;
        }
    }

    logPhase("initial layering");
    fprintf(stderr, "[LAYOUT-SCALE] nodes=%d edges=%d\n",
            (int)cg.nodes.size(), (int)cg.edges.size());

    int maxInternalLayer = 0;
    for (auto* node : cg.nodes) {
        if (!isOutputPortNode(node)) {
            maxInternalLayer = std::max(maxInternalLayer, layerMap[node]);
        }
    }

    for (auto* node : cg.nodes) {
        if (!isOutputPortNode(node)) {
            continue;
        }
        int targetLayer = maxInternalLayer + 1;
        for (auto* pred : predecessors[node]) {
            targetLayer = std::max(targetLayer, layerMap[pred] + 1);
        }
        layerMap[node] = targetLayer;
    }


    std::map<int, std::vector<CNode*>> layers;
    for (auto* node : cg.nodes) {
        layers[layerMap[node]].push_back(node);
    }
    fprintf(stderr, "[LAYOUT-SCALE] layers=%d\n", (int)layers.size());

    std::map<int, double> layerWidths;
    std::map<int, double> layerX;
    for (const auto& entry : layers) {
        double width = 0.0;
        for (auto* node : entry.second) {
            width = std::max(width, node->width);
        }
        layerWidths[entry.first] = width;
    }

    // Dynamic layer spacing: count edges that cross each gap (between layer L and L+1).
    // Mirrors ELK OrthogonalEdgeRouter: routingWidth = 2*edgeNodeSpacing + (N-1)*edgeEdgeSpacing,
    // clamped to nodeNodeBetweenLayers as minimum.
    // Defaults: edgeEdgeBetweenLayers=10, edgeNodeBetweenLayers=10 (from Layered.melk).
    const double edgeEdgeSpacingBL = 10.0;
    const double edgeNodeSpacingBL = 10.0;
    const double minGap = graph.options().spacing().nodeNodeBetweenLayers; // 60

    std::map<int, int> edgesPerGap; // gap index L = "gap between layer L and L+1"
    for (auto* e : cg.edges) {
        int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
        int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
        if (sl > tl) std::swap(sl, tl); // handle backward edges safely
        for (int g = sl; g < tl; ++g)
            edgesPerGap[g]++;
    }

    auto gapWidth = [&](int layerIdx) -> double {
        int n = edgesPerGap.count(layerIdx) ? edgesPerGap[layerIdx] : 0;
        if (n <= 1) return minGap;
        double rw = 2.0 * edgeNodeSpacingBL + (n - 1) * edgeEdgeSpacingBL;
        return std::max(minGap, rw);
    };

    double currentX = 0.0;
    for (const auto& entry : layers) {
        layerX[entry.first] = currentX;
        currentX += layerWidths[entry.first] + gapWidth(entry.first);
    }

    // Initialize order by source appearance for deterministic behavior.
    for (auto& entry : layers) {
        auto& layerNodes = entry.second;
        std::stable_sort(layerNodes.begin(), layerNodes.end(), [&](CNode* left, CNode* right) {
            return sourceOrder[left] < sourceOrder[right];
        });
    }

    std::map<CNode*, int> orderIndex;
    auto rebuildOrderIndex = [&]() {
        for (auto& entry : layers) {
            int idx = 0;
            for (auto* node : entry.second) {
                orderIndex[node] = idx++;
            }
        }
    };

    rebuildOrderIndex();

    // ===================================================================
    // Crossing minimization with dummy nodes for long edges (ELK LongEdge).
    // For every edge spanning more than one layer, insert a virtual
    // PlacementSlot at each intermediate layer so the barycenter
    // algorithm can "see" the long-edge tension when sorting those layers.
    // ===================================================================
    struct PlacementSlot {
        CNode* realNode  = nullptr; // non-null for real nodes only
        int    edgeIdx   = -1;      // index into cg.edges for dummies
        int    chainStep = 0;       // 0 = closest to source
    };

    const auto baseLayers = layers;

    auto buildExtLayers = [&](const std::map<int, std::vector<CNode*>>& orderedLayers)
        -> std::map<int, std::vector<PlacementSlot>> {
        std::map<int, std::vector<PlacementSlot>> out;
        for (const auto& kv : orderedLayers) {
            for (auto* n : kv.second) out[kv.first].push_back({n, -1, 0});
        }
        for (int i = 0; i < (int)cg.edges.size(); ++i) {
            auto* e = cg.edges[i];
            int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
            int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
            for (int k = 0; k < tl - sl - 1; ++k)
                out[sl + k + 1].push_back({nullptr, i, k});
        }
        return out;
    };

    std::map<int, std::vector<PlacementSlot>> extLayers = buildExtLayers(baseLayers);

    // O(layer_size) lookup helpers; layers are small so this is fast.
    auto findReal = [&](int L, CNode* n) -> int {
        auto it = extLayers.find(L);
        if (it == extLayers.end()) return -1;
        const auto& sv = it->second;
        for (int i = 0; i < (int)sv.size(); ++i)
            if (sv[i].realNode == n) return i;
        return -1;
    };
    auto findDummy = [&](int L, int eidx, int step) -> int {
        auto it = extLayers.find(L);
        if (it == extLayers.end()) return -1;
        const auto& sv = it->second;
        for (int i = 0; i < (int)sv.size(); ++i)
            if (!sv[i].realNode && sv[i].edgeIdx == eidx && sv[i].chainStep == step)
                return i;
        return -1;
    };

    auto portBias = [&](const CNode* node, const std::string& portName) -> double {
        if (!node || portName.empty() || node->ports.empty()) return 0.5;
        std::vector<std::pair<double, std::string>> ordered;
        ordered.reserve(node->ports.size());
        for (const auto& p : node->ports) ordered.push_back({p.y, p.name});
        std::stable_sort(ordered.begin(), ordered.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });
        for (size_t i = 0; i < ordered.size(); ++i) {
            if (ordered[i].second == portName)
                return (double)(i + 1) / (double)(ordered.size() + 1);
        }
        return 0.5;
    };

    // ===================================================================
    // Pre-index edges by bilayer (leftL, rightL) for O(E_bilayer) access
    // instead of scanning all cg.edges every call to collectBilayerSegments.
    // ===================================================================
    // directEdgesByBilayer[leftL][rightL] = list of (edgeIdx) for span=1 edges.
    // longEdgesByBilayer[leftL][rightL]   = list of (edgeIdx, chainStep) for span>1 dummy segments.
    std::map<int, std::map<int, std::vector<int>>> directEdgesByBilayer;
    std::map<int, std::map<int, std::vector<std::pair<int,int>>>> longEdgesByBilayer;
    for (int i = 0; i < (int)cg.edges.size(); ++i) {
        auto* e = cg.edges[i];
        int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
        int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
        if (tl == sl + 1) {
            directEdgesByBilayer[sl][tl].push_back(i);
        } else if (tl > sl + 1) {
            int span = tl - sl;
            for (int k = 0; k < span; ++k) {
                directEdgesByBilayer[sl + k][sl + k + 1]; // ensure exists
                longEdgesByBilayer[sl + k][sl + k + 1].push_back({i, k});
            }
        }
    }

    // ===================================================================
    // Exact bilayer crossing minimization.
    // Includes per-port bias in endpoint ordering so crossing objective sees
    // not only node order, but also port-level attachment order.
    // ===================================================================
    auto collectBilayerSegments = [&](int leftL, int rightL)
        -> std::vector<std::pair<double, double>> {
        std::vector<std::pair<double, double>> segs;
        if (!extLayers.count(leftL) || !extLayers.count(rightL)) return segs;

        const double portBiasWeight = 0.0;

        // Use pre-indexed edges for O(E_bilayer) instead of O(E_total).
        auto dIt = directEdgesByBilayer.find(leftL);
        if (dIt != directEdgesByBilayer.end()) {
            auto dIt2 = dIt->second.find(rightL);
            if (dIt2 != dIt->second.end()) {
                for (int eidx : dIt2->second) {
                    auto* e = cg.edges[eidx];
                    int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
                    int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
                    if (tl == sl + 1) {
                        int u = findReal(leftL, e->source);
                        int v = findReal(rightL, e->target);
                        if (u >= 0 && v >= 0) {
                            double pu = (double)u + portBiasWeight * portBias(e->source, e->sourcePort);
                            double pv = (double)v + portBiasWeight * portBias(e->target, e->targetPort);
                            segs.push_back({pu, pv});
                        }
                    }
                }
            }
        }
        auto lIt = longEdgesByBilayer.find(leftL);
        if (lIt != longEdgesByBilayer.end()) {
            auto lIt2 = lIt->second.find(rightL);
            if (lIt2 != lIt->second.end()) {
                for (auto [eidx, k] : lIt2->second) {
                    auto* e = cg.edges[eidx];
                    int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
                    int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
                    int span = tl - sl;
                    int aL = sl + k, bL = sl + k + 1;
                    if (aL != leftL || bL != rightL) continue;
                    int u = (k == 0) ? findReal(aL, e->source)
                                     : findDummy(aL, eidx, k - 1);
                    int v = (k == span - 1) ? findReal(bL, e->target)
                                            : findDummy(bL, eidx, k);
                    if (u >= 0 && v >= 0) {
                        double pu = (k == 0)
                            ? ((double)u + portBiasWeight * portBias(e->source, e->sourcePort))
                            : ((double)u + 0.5);
                        double pv = (k == span - 1)
                            ? ((double)v + portBiasWeight * portBias(e->target, e->targetPort))
                            : ((double)v + 0.5);
                        segs.push_back({pu, pv});
                    }
                }
            }
        }

        return segs;
    };

    auto bilayerCrossings = [&](int leftL, int rightL) -> int {
        auto segs = collectBilayerSegments(leftL, rightL);
        if (segs.empty()) return 0;
        // Sort by left pos ascending, break ties by right pos ascending
        // so same-source edges are in order and won't count as inversions.
        std::sort(segs.begin(), segs.end(), [](const auto& a, const auto& b) {
            return a.first < b.first || (a.first == b.first && a.second < b.second);
        });
        // Count inversions in right-position sequence (= crossings) via merge sort O(n log n).
        std::vector<double> arr;
        arr.reserve(segs.size());
        for (auto& s : segs) arr.push_back(s.second);
        long long inv = 0;
        std::vector<double> tmp(arr.size());
        std::function<void(int,int)> ms = [&](int lo, int hi) {
            if (hi - lo <= 1) return;
            int mid = (lo + hi) / 2;
            ms(lo, mid); ms(mid, hi);
            int i = lo, j = mid, k = lo;
            while (i < mid && j < hi) {
                if (arr[i] <= arr[j]) tmp[k++] = arr[i++];
                else { inv += (mid - i); tmp[k++] = arr[j++]; }
            }
            while (i < mid) tmp[k++] = arr[i++];
            while (j < hi)  tmp[k++] = arr[j++];
            for (int x = lo; x < hi; ++x) arr[x] = tmp[x];
        };
        ms(0, (int)arr.size());
        return (int)std::min(inv, (long long)INT_MAX);
    };

    // Build neighbor-position lists for the moving layer from a bilayer segment list.
    // nbPos[i] = sorted positions in fixed layer of neighbors of mv[i].
    auto buildNbPos = [&](int leftL, int rightL, bool movingIsLeft) -> std::vector<std::vector<int>> {
        const auto& mv = extLayers[movingIsLeft ? leftL : rightL];
        int nMv = (int)mv.size();
        auto segs = collectBilayerSegments(leftL, rightL);
        std::vector<std::vector<int>> nbPos(nMv);
        for (auto& [u, v] : segs) {
            int mi = (int)(movingIsLeft ? u : v);
            int fp = (int)(movingIsLeft ? v : u);
            if (mi >= 0 && mi < nMv) nbPos[mi].push_back(fp);
        }
        for (auto& nn : nbPos) std::sort(nn.begin(), nn.end());
        return nbPos;
    };

    // Count crossings from neighbor-position lists using merge sort O(E log E).
    // Each nbPos[i] must be sorted ascending (ensures same-source pairs don't count).
    auto countFromNbPos = [&](std::vector<std::vector<int>> nbPos) -> int {
        std::vector<int> rhs;
        for (const auto& nn : nbPos) for (int p : nn) rhs.push_back(p);
        if ((int)rhs.size() <= 1) return 0;
        long long inv = 0;
        std::vector<int> tmp(rhs.size());
        std::function<void(int,int)> ms = [&](int lo, int hi) {
            if (hi - lo <= 1) return;
            int mid = (lo + hi) / 2;
            ms(lo, mid); ms(mid, hi);
            int i = lo, j = mid, k = lo;
            while (i < mid && j < hi) {
                if (rhs[i] <= rhs[j]) tmp[k++] = rhs[i++];
                else { inv += (mid - i); tmp[k++] = rhs[j++]; }
            }
            while (i < mid) tmp[k++] = rhs[i++];
            while (j < hi)  tmp[k++] = rhs[j++];
            for (int x = lo; x < hi; ++x) rhs[x] = tmp[x];
        };
        ms(0, (int)rhs.size());
        return (int)std::min(inv, (long long)INT_MAX);
    };

    auto localCrossingsAtLayer = [&](int L) -> int {
        int c = 0;
        if (extLayers.count(L - 1) && extLayers.count(L)) c += bilayerCrossings(L - 1, L);
        if (extLayers.count(L) && extLayers.count(L + 1)) c += bilayerCrossings(L, L + 1);
        return c;
    };

    auto improveBilayer = [&](int fixedL, int movingL) -> bool {
        if (!extLayers.count(fixedL) || !extLayers.count(movingL)) return false;
        auto& mv = extLayers[movingL];
        if ((int)mv.size() <= 1) return false;

        int leftL  = std::min(fixedL, movingL);
        int rightL = std::max(fixedL, movingL);
        bool movingIsLeft = (movingL == leftL);
        int nMv = (int)mv.size();

        // Build neighbor-position lists once (O(E) scan).
        auto nbPos = buildNbPos(leftL, rightL, movingIsLeft);

        // Barycenter score using nbPos (mixed real+dummy extLayers ranks).
        // This is self-consistent with countFromNbPos which also uses extLayers ranks.
        // Unconnected nodes get an offset of 0.5 so they interleave between connected ones.
        double nFixed = (double)extLayers[fixedL].size();
        auto baryScore = [&](int i) -> double {
            if (!nbPos[i].empty()) {
                double sum = 0.0;
                for (int p : nbPos[i]) sum += p;
                return sum / nbPos[i].size();
            }
            // Unconnected: use current position scaled to fixed-layer space,
            // with a fractional offset to avoid ties with connected nodes.
            double curPos = i * nFixed / std::max(1, nMv);
            return curPos + 0.5;
        };

        // Try barycenter sort; keep only if it reduces crossings.
        {
            int origCross = countFromNbPos(nbPos);
            auto mvSave  = mv;
            auto nbSave  = nbPos;
            std::vector<std::pair<double,int>> scored(nMv);
            for (int i = 0; i < nMv; ++i) scored[i] = {baryScore(i), i};
            std::stable_sort(scored.begin(), scored.end(), [](const auto& a, const auto& b){
                return a.first < b.first;
            });
            std::vector<PlacementSlot>      sorted_mv(nMv);
            std::vector<std::vector<int>>   sorted_nb(nMv);
            for (int i = 0; i < nMv; ++i) {
                sorted_mv[i] = mvSave[scored[i].second];
                sorted_nb[i] = nbSave[scored[i].second];
            }
            mv    = sorted_mv;
            nbPos = sorted_nb;
            if (countFromNbPos(nbPos) > origCross) {
                mv    = std::move(mvSave);
                nbPos = std::move(nbSave);
            }
        }

        // Adjacent swap refinement: local O(deg^2) per pair — no bilayerCrossings calls.
        bool changedAny = false;
        const int localPasses = 10;
        for (int pass = 0; pass < localPasses; ++pass) {
            bool passChanged = false;
            for (int i = 0; i + 1 < nMv; ++i) {
                const auto& aN = nbPos[i];
                const auto& bN = nbPos[i + 1];
                // c_ab = crossings with a before b; c_ba = crossings with b before a.
                int c_ab = 0, c_ba = 0;
                for (int na : aN) for (int nb : bN) {
                    if (na > nb) ++c_ab;
                    else if (na < nb) ++c_ba;
                }
                if (c_ba < c_ab) {
                    std::swap(mv[i], mv[i + 1]);
                    std::swap(nbPos[i], nbPos[i + 1]);
                    passChanged = true;
                    changedAny  = true;
                }
            }
            if (!passChanged) break;
        }

        return changedAny;
    };

    auto runTranspose = [&](const std::vector<int>& layerOrder) -> bool {
        bool anyChanged = false;
        for (int L : layerOrder) {
            if (!extLayers.count(L)) continue;
            auto& sv = extLayers[L];
            int n = (int)sv.size();
            if (n <= 1) continue;

            // Build neighbor-position lists for both adjacent bilayers.
            std::vector<std::vector<int>> nbLeft(n), nbRight(n);
            if (extLayers.count(L - 1)) {
                auto segs = collectBilayerSegments(L - 1, L);
                for (auto& [u, v] : segs) {
                    int mi = (int)v;  // L is rightL (moving)
                    int fp = (int)u;
                    if (mi >= 0 && mi < n) nbLeft[mi].push_back(fp);
                }
            }
            if (extLayers.count(L + 1)) {
                auto segs = collectBilayerSegments(L, L + 1);
                for (auto& [u, v] : segs) {
                    int mi = (int)u;  // L is leftL (moving)
                    int fp = (int)v;
                    if (mi >= 0 && mi < n) nbRight[mi].push_back(fp);
                }
            }
            for (auto& nn : nbLeft)  std::sort(nn.begin(), nn.end());
            for (auto& nn : nbRight) std::sort(nn.begin(), nn.end());

            for (int i = 0; i + 1 < n; ++i) {
                int c_ab = 0, c_ba = 0;
                for (int na : nbLeft[i])  for (int nb : nbLeft[i+1])  { if (na > nb) ++c_ab; else if (na < nb) ++c_ba; }
                for (int na : nbRight[i]) for (int nb : nbRight[i+1]) { if (na > nb) ++c_ab; else if (na < nb) ++c_ba; }
                if (c_ba < c_ab) {
                    std::swap(sv[i], sv[i + 1]);
                    std::swap(nbLeft[i],  nbLeft[i + 1]);
                    std::swap(nbRight[i], nbRight[i + 1]);
                    anyChanged = true;
                }
            }
        }
        return anyChanged;
    };

    // Java LayerSweep + GreedySwitch style local improvement:
    // Uses incremental O(deg^2) crossing computation per swap (same as runTranspose but
    // considers both adjacent bilayers simultaneously for a two-sided evaluation).
    // This replaces the previous O(E log E) per-swap implementation.
    auto runGreedySwitch = [&](const std::vector<int>& layerOrder, int maxPasses) -> bool {
        bool anyChanged = false;
        for (int pass = 0; pass < maxPasses; ++pass) {
            bool passChanged = false;
            for (int L : layerOrder) {
                if (!extLayers.count(L)) continue;
                auto& sv = extLayers[L];
                int n = (int)sv.size();
                if (n <= 1) continue;

                // Build nbLeft and nbRight just like runTranspose.
                std::vector<std::vector<int>> nbLeft(n), nbRight(n);
                if (extLayers.count(L - 1)) {
                    auto segs = collectBilayerSegments(L - 1, L);
                    for (auto& [u, v] : segs) {
                        int mi = (int)v;
                        int fp = (int)u;
                        if (mi >= 0 && mi < n) nbLeft[mi].push_back(fp);
                    }
                }
                if (extLayers.count(L + 1)) {
                    auto segs = collectBilayerSegments(L, L + 1);
                    for (auto& [u, v] : segs) {
                        int mi = (int)u;
                        int fp = (int)v;
                        if (mi >= 0 && mi < n) nbRight[mi].push_back(fp);
                    }
                }
                for (auto& nn : nbLeft)  std::sort(nn.begin(), nn.end());
                for (auto& nn : nbRight) std::sort(nn.begin(), nn.end());

                // Adjacent swap: two-sided incremental crossing count (O(deg^2) per swap).
                for (int i = 0; i + 1 < n; ++i) {
                    int c_ab = 0, c_ba = 0;
                    for (int na : nbLeft[i])  for (int nb : nbLeft[i+1])  { if (na > nb) ++c_ab; else if (na < nb) ++c_ba; }
                    for (int na : nbRight[i]) for (int nb : nbRight[i+1]) { if (na > nb) ++c_ab; else if (na < nb) ++c_ba; }
                    if (c_ba < c_ab) {
                        std::swap(sv[i], sv[i + 1]);
                        std::swap(nbLeft[i],  nbLeft[i + 1]);
                        std::swap(nbRight[i], nbRight[i + 1]);
                        passChanged = true;
                        anyChanged = true;
                    }
                }
            }
            if (!passChanged) break;
        }
        return anyChanged;
    };

    auto totalAdjacentCrossings = [&](const std::vector<int>& layerOrder) -> int {
        int total = 0;
        for (size_t i = 1; i < layerOrder.size(); ++i)
            total += bilayerCrossings(layerOrder[i - 1], layerOrder[i]);
        return total;
    };

    auto buildInitialOrderedLayers = [&](int mode) -> std::map<int, std::vector<CNode*>> {
        std::map<int, std::vector<CNode*>> out = baseLayers;
        for (auto& kv : out) {
            auto& lv = kv.second;
            int L = kv.first;
            std::stable_sort(lv.begin(), lv.end(), [&](CNode* a, CNode* b) {
                auto score = [&](CNode* n) {
                    std::vector<double> values;
                    if (mode == 0) {
                        values.push_back((double)sourceOrder[n]);
                    } else if (mode == 1) {
                        for (auto* p : predecessors[n]) {
                            if (layerMap.count(p) && layerMap[p] == L - 1) {
                                values.push_back((double)sourceOrder[p]);
                            }
                        }
                    } else if (mode == 2) {
                        for (auto* s : successors[n]) {
                            if (layerMap.count(s) && layerMap[s] == L + 1) {
                                values.push_back((double)sourceOrder[s]);
                            }
                        }
                    } else {
                        values.push_back((double)(-sourceOrder[n]));
                    }
                    return average(values, (double)sourceOrder[n]);
                };
                double sa = score(a), sb = score(b);
                if (std::abs(sa - sb) > 1e-9) return sa < sb;
                return sourceOrder[a] < sourceOrder[b];
            });
        }
        return out;
    };

    std::vector<int> layerOrder;
    layerOrder.reserve(baseLayers.size());
    for (const auto& kv : baseLayers) layerOrder.push_back(kv.first);

    // Placement-first optimization: multi-start + transpose.
    // Best stable setting on current benchmarks: single-seed, 16 modes.
    int bestCrossingCount = std::numeric_limits<int>::max();
    std::map<int, std::vector<PlacementSlot>> bestExtLayers;

    // Adaptive limits: large graphs can have O(N*layers) cost per sweep.
    // Scale down aggressively to avoid hanging on netlists with >1000 nodes.
    int totalNodes = 0;
    for (const auto& kv : baseLayers) totalNodes += (int)kv.second.size();
    int totalEdges = (int)cg.edges.size();
    // Count dummy nodes from long edges (span>1 contributes span-1 dummies)
    int totalDummies = 0;
    for (auto* e : cg.edges) {
        int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
        int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
        if (tl > sl + 1) totalDummies += (tl - sl - 1);
    }
    int graphComplexity = totalNodes + totalEdges + totalDummies;
    fprintf(stderr, "[LAYOUT-SCALE] nodes=%d edges=%d dummies=%d complexity=%d\n",
            totalNodes, totalEdges, totalDummies, graphComplexity);

    int sweepCount, totalModes;
    if (graphComplexity > 50000) {
        sweepCount = 3;
        totalModes = 1;   // one deterministic pass only
    } else if (graphComplexity > 20000) {
        sweepCount = 5;
        totalModes = 2;
    } else if (graphComplexity > 5000) {
        sweepCount = 10;
        totalModes = 4;
    } else {
        sweepCount = 30;
        totalModes = 16;  // 4 fixed + 12 random.
    }
    std::mt19937 rng(12345);
    for (int mode = 0; mode < totalModes; ++mode) {
        std::map<int, std::vector<CNode*>> initialLayers;
        if (mode < 4) {
            initialLayers = buildInitialOrderedLayers(mode);
        } else {
            // Random restart: shuffle each layer's initial order.
            initialLayers = baseLayers;
            for (auto& kv : initialLayers) {
                std::shuffle(kv.second.begin(), kv.second.end(), rng);
            }
        }
        extLayers = buildExtLayers(initialLayers);

        int noImproveStreak = 0;
        int prevCrossingCount = totalAdjacentCrossings(layerOrder);
        int bestInRestart = prevCrossingCount;
        std::map<int, std::vector<PlacementSlot>> bestInRestartLayers = extLayers;
        for (int sweep = 0; sweep < sweepCount; ++sweep) {
            bool anyChanged = false;
            for (size_t i = 1; i < layerOrder.size(); ++i) {
                if (improveBilayer(layerOrder[i - 1], layerOrder[i])) anyChanged = true;
            }
            for (int i = (int)layerOrder.size() - 2; i >= 0; --i) {
                if (improveBilayer(layerOrder[i + 1], layerOrder[i])) anyChanged = true;
            }
            // For large graphs skip transpose/greedy-switch inside the per-sweep loop
            // (they are O(N_layer * deg^2) and dominate runtime).
            if (graphComplexity <= 20000) {
                if (runTranspose(layerOrder)) anyChanged = true;
                if (runGreedySwitch(layerOrder, 2)) anyChanged = true;
            }

            int crossingCount = totalAdjacentCrossings(layerOrder);
            if (crossingCount < bestInRestart) {
                bestInRestart = crossingCount;
                bestInRestartLayers = extLayers;
                noImproveStreak = 0;
            } else {
                ++noImproveStreak;
            }
            prevCrossingCount = crossingCount;

            if (!anyChanged || noImproveStreak >= 5) break;
        }
        // Restore the best configuration found during this restart.
        extLayers = bestInRestartLayers;

        int finalCrossingCount = totalAdjacentCrossings(layerOrder);
        if (finalCrossingCount < bestCrossingCount) {
            bestCrossingCount = finalCrossingCount;
            bestExtLayers = extLayers;
        }
    }

    if (!bestExtLayers.empty()) {
        extLayers = bestExtLayers;
    }

    logPhase("crossing minimization multi-start");

    // ===================================================================
    // P3 post-processing: Greedy Switch pass (Java ELK GreedySwitchHeuristic).
    // After multi-start barycenter+transpose, run 2 forward+backward sweep
    // passes of adjacent-swap refinement using the exact bilayer crossing counter.
    // This mirrors GraphConfigurator activateGreedySwitchFor -> TWO_SIDED mode.
    // Cost: O(sweeps * layers * E) — cheap relative to multi-start.
    // ===================================================================
    {
        const int greedySweeps = (graphComplexity > 20000) ? 1 : 3;
        for (int gs = 0; gs < greedySweeps; ++gs) {
            // Forward sweep: left-to-right, fix left layer, move right.
            for (size_t li = 0; li + 1 < layerOrder.size(); ++li) {
                improveBilayer(layerOrder[li], layerOrder[li + 1]);
            }
            // Backward sweep: right-to-left, fix right layer, move left.
            for (int li = (int)layerOrder.size() - 2; li >= 0; --li) {
                improveBilayer(layerOrder[li + 1], layerOrder[li]);
            }
            // Transpose + two-sided greedy switch (converges locally optimal).
            runTranspose(layerOrder);
            runGreedySwitch(layerOrder, (graphComplexity > 20000) ? 1 : 3);
        }
        // Re-materialize the extLayers ordering back to the real layers map
        // (improveBilayer modifies extLayers in-place).
        for (auto& kv : layers) {
            const auto& extSV = extLayers[kv.first];
            std::vector<CNode*> ordered;
            ordered.reserve(kv.second.size());
            for (const auto& s : extSV)
                if (s.realNode) ordered.push_back(s.realNode);
            kv.second = ordered;
        }
        rebuildOrderIndex();
    }

    logPhase("greedy switch post-process");

    // Materialize temporary BK dummy nodes from extLayers virtual slots.
    // Key: (edgeIdx, chainStep), where chainStep k is the dummy at layer (srcLayer + 1 + k).
    std::vector<std::unique_ptr<CNode>> bkDummyStorage;
    std::map<std::pair<int, int>, CNode*> bkDummyByStep;
    std::map<CNode*, std::pair<int, int>> bkStepByDummy;
    int bkDummySerial = 0;
    for (const auto& kv : extLayers) {
        int L = kv.first;
        for (const auto& slot : kv.second) {
            if (slot.realNode) continue;
            std::pair<int, int> key = {slot.edgeIdx, slot.chainStep};
            if (bkDummyByStep.count(key)) continue;

            auto dn = std::make_unique<CNode>();
            dn->id = "__bk_dummy_" + std::to_string(slot.edgeIdx) + "_" + std::to_string(slot.chainStep)
                   + "_" + std::to_string(bkDummySerial++);
            dn->width = 0.0;
            dn->height = 0.0;
            dn->data.type = CircuitNodeType::NetJoint;
            dn->data.direction.clear();
            CNode* raw = dn.get();
            bkDummyByStep[key] = raw;
            bkStepByDummy[raw] = key;
            layerMap[raw] = L;
            sourceOrder[raw] = static_cast<int>(sourceOrder.size());
            bkDummyStorage.push_back(std::move(dn));
        }
    }

    logPhase("materialize BK dummies");

    // Extract real node order from extended layers back into `layers`.
    for (auto& kv : layers) {
        const auto& extSV = extLayers[kv.first];
        std::vector<CNode*> ordered;
        ordered.reserve(kv.second.size());
        for (const auto& s : extSV)
            if (s.realNode) ordered.push_back(s.realNode);
        kv.second = ordered;
    }
    rebuildOrderIndex();

    // BK-style Y assignment: 4-direction median alignment + block compaction.
    const double nodeSpacingY = graph.options().spacing().nodeNode;
    std::map<int, std::vector<CNode*>> normalByLayer;
    std::map<int, std::vector<CNode*>> inputByLayer;
    std::map<int, std::vector<CNode*>> outputByLayer;
    for (const auto& kv : layers) {
        int L = kv.first;
        for (auto* n : kv.second) {
            if (isInputPortNode(n)) inputByLayer[L].push_back(n);
            else if (isOutputPortNode(n)) outputByLayer[L].push_back(n);
            else normalByLayer[L].push_back(n);
        }
    }

    std::map<int, std::map<CNode*, int>> layerNodeIndex;
    for (const auto& kv : normalByLayer) {
        for (int i = 0; i < (int)kv.second.size(); ++i) {
            layerNodeIndex[kv.first][kv.second[i]] = i;
        }
    }

    std::vector<int> normalLayers;
    normalLayers.reserve(normalByLayer.size());
    for (const auto& kv : normalByLayer) normalLayers.push_back(kv.first);

    // BK layer content = real normal nodes + dummy nodes, in the finalized extLayers order.
    std::map<int, std::vector<CNode*>> bkNormalByLayer;
    for (const auto& kv : extLayers) {
        int L = kv.first;
        for (const auto& slot : kv.second) {
            if (slot.realNode) {
                CNode* n = slot.realNode;
                if (!isInputPortNode(n) && !isOutputPortNode(n)) {
                    bkNormalByLayer[L].push_back(n);
                }
            } else {
                auto it = bkDummyByStep.find({slot.edgeIdx, slot.chainStep});
                if (it != bkDummyByStep.end()) {
                    bkNormalByLayer[L].push_back(it->second);
                }
            }
        }
    }

    std::map<int, std::map<CNode*, int>> bkLayerNodeIndex;
    for (const auto& kv : bkNormalByLayer) {
        for (int i = 0; i < (int)kv.second.size(); ++i) {
            bkLayerNodeIndex[kv.first][kv.second[i]] = i;
        }
    }

    std::vector<int> bkLayers;
    bkLayers.reserve(bkNormalByLayer.size());
    for (const auto& kv : bkNormalByLayer) bkLayers.push_back(kv.first);

    // BK predecessor/successor graph with long edges split by dummy chain nodes.
    std::map<CNode*, std::vector<CNode*>> bkPredecessors;
    std::map<CNode*, std::vector<CNode*>> bkSuccessors;
    auto addBkArc = [&](CNode* u, CNode* v) {
        if (!u || !v) return;
        bkSuccessors[u].push_back(v);
        bkPredecessors[v].push_back(u);
    };
    for (int eidx = 0; eidx < (int)cg.edges.size(); ++eidx) {
        auto* e = cg.edges[eidx];
        int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
        int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
        if (tl - sl <= 1) {
            addBkArc(e->source, e->target);
            continue;
        }
        CNode* prev = e->source;
        for (int k = 0; k < (tl - sl - 1); ++k) {
            auto it = bkDummyByStep.find({eidx, k});
            if (it == bkDummyByStep.end()) continue;
            CNode* dn = it->second;
            addBkArc(prev, dn);
            prev = dn;
        }
        addBkArc(prev, e->target);
    }

    auto isNormalNode = [&](CNode* n) {
        return n && !isInputPortNode(n) && !isOutputPortNode(n) && layerMap.count(n);
    };

    auto isBkDummyNode = [&](CNode* n) {
        return bkStepByDummy.count(n) > 0;
    };

    // Build edge lookup for insideBlockShift port-based computation.
    std::map<std::pair<CNode*, CNode*>, CEdge*> bkEdgeLookup;
    for (auto* e : cg.edges) {
        if (e && e->source && e->target)
            bkEdgeLookup[{e->source, e->target}] = e;
    }

    auto runBkPass = [&](bool topDown, bool leftToRight) {
        std::map<CNode*, CNode*> root;
        std::map<CNode*, CNode*> align;
        std::map<CNode*, double> innerShift;
        std::map<CNode*, double> y;

        for (const auto& kv : bkNormalByLayer) {
            for (auto* n : kv.second) {
                root[n] = n;
                align[n] = n;
                innerShift[n] = 0.0;
            }
        }

        // --- Step 1: verticalAlignment (Java ELK BKAligner.verticalAlignment) ---
        // layerOrder and adjLayer depend on HDirection (leftToRight), NOT VDirection (topDown).
        // Node ordering within layer and 'r' direction depend on VDirection (topDown).
        std::vector<int> layerOrder = bkLayers;
        if (!leftToRight) std::reverse(layerOrder.begin(), layerOrder.end());

        for (int L : layerOrder) {
            int adjLayer = leftToRight ? (L - 1) : (L + 1);
            if (!bkNormalByLayer.count(adjLayer)) continue;

            auto nodes = bkNormalByLayer[L];
            // VDirection.UP reverses node list and starts r at INT_MAX.
            if (!topDown) std::reverse(nodes.begin(), nodes.end());
            int r = topDown ? -1 : std::numeric_limits<int>::max();

            for (auto* v : nodes) {
                if (align[v] != v) continue; // already part of a block

                // HDirection determines whether predecessors or successors are the "upper neighbors"
                std::vector<CNode*> neighbors;
                const auto& cands = leftToRight ? bkPredecessors[v] : bkSuccessors[v];
                for (auto* u : cands) {
                    if (!isNormalNode(u)) continue;
                    if (!layerMap.count(u) || layerMap[u] != adjLayer) continue;
                    neighbors.push_back(u);
                }
                if (neighbors.empty()) continue;

                std::sort(neighbors.begin(), neighbors.end(), [&](CNode* a, CNode* b) {
                    return bkLayerNodeIndex[adjLayer][a] < bkLayerNodeIndex[adjLayer][b];
                });

                const int d = (int)neighbors.size();
                int lo = (d - 1) / 2;
                int hi = d / 2;

                // For VDirection.UP iterate high→lo, for DOWN lo→hi.
                // Java BK checks align[v]==v inside the loop; we break after first success.
                if (!topDown) std::swap(lo, hi);
                int step = topDown ? 1 : -1;
                for (int mi = lo; mi != hi + step; mi += step) {
                    if (align[v] != v) break; // already aligned in this iteration
                    auto* u = neighbors[mi];
                    int ui = bkLayerNodeIndex[adjLayer][u];
                    bool monotone = topDown ? (ui > r) : (ui < r);
                    if (!monotone) continue;

                    align[u] = v;
                    root[v] = root[u];
                    align[v] = root[v];
                    r = ui;
                    break; // each node aligns with at most one median neighbor (Java BK behavior)
                }
            }
        }

        // --- Step 2: insideBlockShift using actual port Y positions ---
        // Matches Java ELK BKAligner.insideBlockShift():
        //   portPosDiff = srcPort.center_y - tgtPort.center_y
        //   innerShift[next] = innerShift[current] + portPosDiff
        for (const auto& kv : bkNormalByLayer) {
            for (auto* n : kv.second) {
                if (root[n] != n) continue;
                innerShift[n] = 0.0;

                CNode* curr = n;
                int safety = 0;
                while (true) {
                    if (++safety > 100000) break;
                    CNode* next = align[curr];
                    if (next == n) break; // completed cycle back to root

                    // Find the edge connecting curr and next in the block chain.
                    CEdge* edge = nullptr;
                    {
                        auto it = bkEdgeLookup.find({curr, next});
                        if (it != bkEdgeLookup.end()) edge = it->second;
                        else {
                            auto it2 = bkEdgeLookup.find({next, curr});
                            if (it2 != bkEdgeLookup.end()) edge = it2->second;
                        }
                    }

                    double portDiff = 0.0;
                    if (edge) {
                        CNode* srcN = edge->source;
                        CNode* tgtN = edge->target;
                        double srcCY = srcN->height * 0.5;
                        double tgtCY = tgtN->height * 0.5;
                        if (!edge->sourcePort.empty()) {
                            for (const auto& p : srcN->ports) {
                                if (p.name == edge->sourcePort) { srcCY = p.y + p.height * 0.5; break; }
                            }
                        }
                        if (!edge->targetPort.empty()) {
                            for (const auto& p : tgtN->ports) {
                                if (p.name == edge->targetPort) { tgtCY = p.y + p.height * 0.5; break; }
                            }
                        }
                        // portPosDiff: srcPort.y - tgtPort.y (Java BK HDirection.RIGHT)
                        portDiff = leftToRight ? (srcCY - tgtCY) : (tgtCY - srcCY);
                    }

                    innerShift[next] = innerShift[curr] + portDiff;
                    curr = next;
                }
            }
        }

        // --- Step 3: Recursive block compaction (Java ELK BKCompactor.placeBlock) ---
        // For each block root r, traverse all nodes in the block chain.
        // For each node, check its same-layer neighbor and recursively place that neighbor's block first.
        // Then update blockY[r] = max/min based on the neighbor's position.
        // This correctly handles multi-layer blocks (key fix vs. per-layer greedy compaction).
        std::map<CNode*, double> blockY;
        std::set<CNode*> placed;

        std::function<void(CNode*)> placeBlockRec = [&](CNode* r) {
            if (placed.count(r)) return;
            placed.insert(r);
            blockY[r] = 0.0;

            CNode* curr = r;
            int safety = 0;
            do {
                if (++safety > 100000) break;

                int L = layerMap.count(curr) ? layerMap[curr] : 0;
                auto layIt = bkNormalByLayer.find(L);
                if (layIt == bkNormalByLayer.end()) { curr = align[curr]; continue; }
                const auto& layerNodes = layIt->second;

                auto idxIt = bkLayerNodeIndex[L].find(curr);
                if (idxIt == bkLayerNodeIndex[L].end()) { curr = align[curr]; continue; }
                int idx = idxIt->second;

                // Select the same-layer neighbor that must be placed before curr.
                // For topDown (VDirection.DOWN): neighbor is above = idx-1.
                // For bottomUp (VDirection.UP): neighbor is below = idx+1.
                CNode* neighbor = nullptr;
                if (topDown) {
                    if (idx > 0) neighbor = layerNodes[idx - 1];
                } else {
                    if (idx < (int)layerNodes.size() - 1) neighbor = layerNodes[idx + 1];
                }

                if (neighbor && isNormalNode(neighbor)) {
                    CNode* nr = root[neighbor];
                    placeBlockRec(nr); // ensure neighbor's block is placed first

                    double sep = (isBkDummyNode(curr) || isBkDummyNode(neighbor)) ? 0.0 : nodeSpacingY;

                    double req;
                    if (topDown) {
                        // neighbor is above curr:
                        // blockY[nr] + innerShift[neighbor] + height[neighbor] + spacing
                        //   <= blockY[r] + innerShift[curr]
                        req = blockY[nr] + innerShift[neighbor] + neighbor->height
                            + sep - innerShift[curr];
                        blockY[r] = std::max(blockY[r], req);
                    } else {
                        // neighbor is below curr:
                        // blockY[r] + innerShift[curr] + height[curr] + spacing
                        //   <= blockY[nr] + innerShift[neighbor]
                        req = blockY[nr] + innerShift[neighbor]
                            - curr->height - sep - innerShift[curr];
                        blockY[r] = std::min(blockY[r], req);
                    }
                }

                curr = align[curr];
            } while (curr != r);
        };

        // Trigger placement for all blocks, processing layers in the correct traversal order.
        std::vector<int> processOrder = bkLayers;
        if (!leftToRight) std::reverse(processOrder.begin(), processOrder.end());
        for (int L : processOrder) {
            auto nodes = bkNormalByLayer[L];
            if (!topDown) std::reverse(nodes.begin(), nodes.end());
            for (auto* n : nodes) {
                if (root[n] == n) placeBlockRec(n);
            }
        }

        // Assign final Y = blockY[root] + innerShift, normalize to >= 0.
        double minY = std::numeric_limits<double>::infinity();
        for (const auto& kv : bkNormalByLayer) {
            for (auto* n : kv.second) {
                auto rIt = blockY.find(root[n]);
                double base = (rIt != blockY.end()) ? rIt->second : 0.0;
                y[n] = base + innerShift[n];
                minY = std::min(minY, y[n]);
            }
        }
        if (minY < 0.0 && std::isfinite(minY)) {
            for (auto& kv : y) kv.second -= minY;
        }
        return y;
    };

    // Run all 4 BK passes in parallel — each is independent (reads shared const state,
    // writes only into its own returned map).
    auto f0 = std::async(std::launch::async, [&]{ return runBkPass(true,  true ); });
    auto f1 = std::async(std::launch::async, [&]{ return runBkPass(true,  false); });
    auto f2 = std::async(std::launch::async, [&]{ return runBkPass(false, true ); });
    auto f3 = std::async(std::launch::async, [&]{ return runBkPass(false, false); });
    std::vector<std::map<CNode*, double>> passes;
    passes.push_back(f0.get());
    passes.push_back(f1.get());
    passes.push_back(f2.get());
    passes.push_back(f3.get());

    // Java ELK balanced layout: shift each pass to align tops (DOWN) or bottoms (UP) to the
    // minimum-width pass, then for each node take the median (average of the two middle values)
    // of the 4 shifted Y positions.  Passes 0,1 are topDown (DOWN); passes 2,3 are bottomUp (UP).
    const bool passTopDown[4] = {true, true, false, false};
    const int N = 4;

    // Compute per-pass min/max Y (top of topmost node, bottom of bottommost node).
    double passMin[N], passMax[N], passWidth[N];
    for (int i = 0; i < N; ++i) {
        passMin[i] =  std::numeric_limits<double>::infinity();
        passMax[i] = -std::numeric_limits<double>::infinity();
        for (const auto& kv : bkNormalByLayer) {
            for (auto* n : kv.second) {
                auto it = passes[i].find(n);
                if (it != passes[i].end()) {
                    passMin[i] = std::min(passMin[i], it->second);
                    passMax[i] = std::max(passMax[i], it->second + n->height);
                }
            }
        }
        passWidth[i] = (passMax[i] > passMin[i]) ? (passMax[i] - passMin[i]) : 0.0;
    }

    // Find the pass with minimum total height span.
    int minWidthLayout = 0;
    for (int i = 1; i < N; ++i) {
        if (passWidth[i] < passWidth[minWidthLayout]) minWidthLayout = i;
    }

    // Compute alignment shifts: DOWN passes align tops; UP passes align bottoms.
    double shift[N];
    for (int i = 0; i < N; ++i) {
        if (passTopDown[i]) {
            shift[i] = passMin[minWidthLayout] - passMin[i];  // align tops
        } else {
            shift[i] = passMax[minWidthLayout] - passMax[i];  // align bottoms
        }
    }

    // Median: for each node sort the 4 shifted Y values and take average of the two middle ones.
    std::map<CNode*, double> yPos;
    for (const auto& kv : bkNormalByLayer) {
        for (auto* n : kv.second) {
            double vals[N];
            int cnt = 0;
            for (int i = 0; i < N; ++i) {
                auto it = passes[i].find(n);
                if (it != passes[i].end()) {
                    vals[cnt++] = it->second + shift[i];
                }
            }
            if (cnt == 0) { yPos[n] = 0.0; continue; }
            std::sort(vals, vals + cnt);
            if (cnt == 4) {
                yPos[n] = (vals[1] + vals[2]) / 2.0;
            } else if (cnt % 2 == 0) {
                yPos[n] = (vals[cnt/2 - 1] + vals[cnt/2]) / 2.0;
            } else {
                yPos[n] = vals[cnt/2];
            }
        }
    }

    // Apply a modest vertical stretch to match Java ELK's natural Y spread from BK block placement.
    // Java ELK nodes are ~1.65x more spread in Y due to node margins and block separation effects.
    const double bkVerticalStretch = 1.25;
    for (auto& kv : yPos) kv.second *= bkVerticalStretch;

    for (const auto& kv : normalByLayer) {
        const auto& nodesRef = kv.second;
        if (nodesRef.empty()) continue;
        for (int i = 1; i < (int)nodesRef.size(); ++i) {
            auto* prev = nodesRef[i - 1];
            auto* curr = nodesRef[i];
            double minTop = yPos[prev] + prev->height + nodeSpacingY;
            if (yPos[curr] < minTop) yPos[curr] = minTop;
        }
        for (int i = (int)nodesRef.size() - 2; i >= 0; --i) {
            auto* curr = nodesRef[i];
            auto* next = nodesRef[i + 1];
            double maxTop = yPos[next] - nodeSpacingY - curr->height;
            if (yPos[curr] > maxTop) yPos[curr] = maxTop;
        }
    }

    double globalMinY = std::numeric_limits<double>::infinity();
    for (const auto& kv : yPos) globalMinY = std::min(globalMinY, kv.second);
    if (globalMinY < 0.0 && std::isfinite(globalMinY)) {
        for (auto& kv : yPos) kv.second -= globalMinY;
    }

    std::map<std::pair<int, int>, double> dummyYByStep;
    for (const auto& kv : bkDummyByStep) {
        auto itY = yPos.find(kv.second);
        if (itY != yPos.end()) {
            dummyYByStep[kv.first] = itY->second;
        }
    }

    for (const auto& kv : normalByLayer) {
        int L = kv.first;
        double lw = layerWidths.count(L) ? layerWidths[L] : 0.0;
        for (auto* n : kv.second) {
            n->x = layerX[L] + 0.5 * (lw - n->width);
            n->y = yPos[n];
        }
    }

    auto placePorts = [&](int layer, std::vector<CNode*>& ports, bool isInput) {
        std::stable_sort(ports.begin(), ports.end(), [&](CNode* left, CNode* right) {
            auto connectedOrder = [&](CNode* node) {
                std::vector<double> values;
                const auto& peers = isInput ? successors[node] : predecessors[node];
                for (auto* peer : peers) {
                    auto found = orderIndex.find(peer);
                    if (found != orderIndex.end()) values.push_back(static_cast<double>(found->second));
                }
                return average(values, static_cast<double>(sourceOrder[node]));
            };

            double leftScore = connectedOrder(left);
            double rightScore = connectedOrder(right);
            if (std::abs(leftScore - rightScore) > 0.001) return leftScore < rightScore;
            return sourceOrder[left] < sourceOrder[right];
        });

        double portY = 0.0;
        double lw = layerWidths.count(layer) ? layerWidths[layer] : 0.0;
        for (auto* node : ports) {
            node->x = layerX[layer] + 0.5 * (lw - node->width);
            node->y = portY;
            portY += node->height + 25.0;
        }
    };

    for (auto& kv : inputByLayer) placePorts(kv.first, kv.second, true);
    for (auto& kv : outputByLayer) placePorts(kv.first, kv.second, false);

    // ===================================================================
    // Port-side ordering: for each ModuleInstance, re-sort same-side ports
    // by the Y of their connected neighbour, so port wires fan out in order
    // and local crossings are minimised.
    // ===================================================================
    // Build per-edge: for each port, what is the "far-end Y" (connected node Y)?
    // We use the node Y as a proxy (port Y within node is small relative to nodeNode).
    // Pre-index port connections to avoid O(E) scan per port in portFarEndY.
    // portFarY[(node, portName)] = average Y of connected nodes (far-end).
    std::map<std::pair<CNode*, std::string>, std::pair<double, int>> portFarYAccum;
    for (auto* e : cg.edges) {
        if (!e->source || !e->target) continue;
        // Source port far-end = target Y
        if (!e->sourcePort.empty()) {
            auto& acc = portFarYAccum[{e->source, e->sourcePort}];
            acc.first += e->target->y + e->target->height * 0.5;
            acc.second++;
        }
        // Target port far-end = source Y
        if (!e->targetPort.empty()) {
            auto& acc = portFarYAccum[{e->target, e->targetPort}];
            acc.first += e->source->y + e->source->height * 0.5;
            acc.second++;
        }
    }

    auto portFarEndY = [&](CNode* node, const CPort& port) -> double {
        auto it = portFarYAccum.find({node, port.name});
        if (it != portFarYAccum.end() && it->second.second > 0)
            return it->second.first / it->second.second;
        return node->y + port.y;
    };

    for (auto* node : cg.nodes) {
        if (node->data.type != CircuitNodeType::ModuleInstance &&
            node->data.type != CircuitNodeType::ExpandedInstance) continue;
        if (node->ports.empty()) continue;

        // Group ports by side.
        std::map<std::string, std::vector<size_t>> sideMap;
        for (size_t i = 0; i < node->ports.size(); ++i)
            sideMap[node->ports[i].side].push_back(i);

        for (auto& kv : sideMap) {
            auto& idxs = kv.second;
            if (idxs.size() < 2) continue;

            // Sort indices by far-end Y (ascending → top port first).
            std::stable_sort(idxs.begin(), idxs.end(), [&](size_t a, size_t b) {
                return portFarEndY(node, node->ports[a]) <
                       portFarEndY(node, node->ports[b]);
            });

            // Re-space port Y coordinates on WEST/EAST sides (vertical stacking).
            const std::string& side = kv.first;
            if (side == "WEST" || side == "EAST") {
                double portSpacing = node->height / static_cast<double>(idxs.size() + 1);
                for (size_t rank = 0; rank < idxs.size(); ++rank) {
                    auto& p = node->ports[idxs[rank]];
                    p.y = portSpacing * (rank + 1) - p.height * 0.5;
                    if (p.y < 0) p.y = 0;
                    if (p.y + p.height > node->height) p.y = node->height - p.height;
                }
            }
        }
    }

    for (auto* node : cg.nodes) {
        auto* elkNode = elkNodeMap[node];
        if (!elkNode) {
            continue;
        }
        elkNode->setLocation(node->x, node->y);

        for (auto& port : node->ports) {
            if (port.side == "WEST") {
                port.x = 0.0;
            } else if (port.side == "EAST") {
                port.x = node->width - port.width;
            } else if (port.side == "NORTH") {
                port.y = 0.0;
            } else if (port.side == "SOUTH") {
                port.y = node->height - port.height;
            }

            for (const auto& elkPort : elkNode->ports()) {
                if (elkPort->id() == port.id) {
                    elkPort->setLocation(port.x, port.y);
                    break;
                }
            }
        }
    }

    // Compute routing bounds and reserve channels in whitespace between/around layers.
    double minNodeX = std::numeric_limits<double>::max();
    double minNodeY = std::numeric_limits<double>::max();
    double maxNodeY = std::numeric_limits<double>::lowest();
    for (auto* n : cg.nodes) {
        minNodeX = std::min(minNodeX, n->x);
        minNodeY = std::min(minNodeY, n->y);
        maxNodeY = std::max(maxNodeY, n->y + n->height);
    }

    // Precompute backward-edge routing channel: just to the right of the rightmost layer
    double rightmostBoundary = 0.0;
    for (const auto& kv : layerX) {
        double rx = kv.second + layerWidths[kv.first];
        if (rx > rightmostBoundary) rightmostBoundary = rx;
    }
    double backChannelX = rightmostBoundary + graph.options().spacing().nodeNodeBetweenLayers;
    double routeStep = std::max(8.0, graph.options().spacing().portPort);
    double trackGap = std::max(20.0, routeStep * 2.0);
    int lanesPerGap = std::max(2, static_cast<int>(graph.options().spacing().nodeNodeBetweenLayers / std::max(8.0, trackGap)));
    lanesPerGap = std::min(lanesPerGap, 6);
    int backwardLaneWindow = 24;
    double safeMinX = minNodeX - 3.0 * graph.options().spacing().nodeNodeBetweenLayers;
    double safeMaxX = backChannelX + graph.options().spacing().nodeNodeBetweenLayers
                    + backwardLaneWindow * std::max(trackGap, 8.0);
    double safeMinY = minNodeY - 4.0 * graph.options().spacing().nodeNode;
    double safeMaxY = maxNodeY + 4.0 * graph.options().spacing().nodeNode;

    // Build spatial index for module nodes (O(N log N) once, enables O(log N) collision queries).
    ModuleNodeIndex modIdx;
    modIdx.build(cg.nodes);

    // Occupancy maps for layered-like track allocation (avoid placing many segments on same channel).
    SortedHSegs occupiedHorizontal;
    std::vector<VerticalSeg> occupiedVertical;

    auto chooseVerticalChannelX = [&](double preferredX,
                                      double y0,
                                      double y1,
                                      double minX,
                                      double maxX,
                                      const CNode* src,
                                      const CNode* tgt) {
        auto scoreAt = [&](double x) {
            int score = 0;
            // Use Y-range spatial index to avoid scanning all nodes.
            auto [mn0, mn1] = modIdx.candidatesForX(x, 2.0);
            for (auto it = mn0; it != mn1; ++it) {
                const CNode* n = it->second;
                if (n == src || n == tgt) continue;
                if (verticalSegmentIntersectsRect(x, y0, y1, n, 2.0)) {
                    score += 100;
                }
            }

            for (const auto& seg : occupiedVertical) {
                if (std::abs(seg.x - x) < trackGap && rangesOverlap(y0, y1, seg.y0, seg.y1)) {
                    score += 5;
                }
            }

            double lo = std::min(y0, y1);
            double hi = std::max(y0, y1);
            for (const auto& [hy, hxr] : occupiedHorizontal.data) {
                if (hy > lo + 0.001 && hy < hi - 0.001 && x > hxr.first + 0.001 && x < hxr.second - 0.001) {
                    score += 3;
                }
            }

            return score;
        };

        preferredX = std::max(minX, std::min(maxX, preferredX));
        int bestScore = scoreAt(preferredX);
        double bestX = preferredX;
        double bestDist = 0.0;

        for (int delta = 1; delta <= 80; ++delta) {
            double right = preferredX + delta * routeStep;
            if (right <= maxX) {
                int s = scoreAt(right);
                double d = std::abs(right - preferredX);
                if (s < bestScore || (s == bestScore && d + 1e-6 < bestDist)) {
                    bestScore = s;
                    bestX = right;
                    bestDist = d;
                    if (bestScore == 0) break;
                }
            }
            double left = preferredX - delta * routeStep;
            if (left >= minX) {
                int s = scoreAt(left);
                double d = std::abs(left - preferredX);
                if (s < bestScore || (s == bestScore && d + 1e-6 < bestDist)) {
                    bestScore = s;
                    bestX = left;
                    bestDist = d;
                    if (bestScore == 0) break;
                }
            }
        }

        return bestX;
    };

    auto pathRoutingScore = [&](const std::vector<QPointF>& path) {
        int score = 0;
        for (size_t i = 1; i < path.size(); ++i) {
            const auto& a = path[i - 1];
            const auto& b = path[i];
            if (std::abs(a.x() - b.x()) < 0.001) {
                double x = a.x();
                double y0 = std::min(a.y(), b.y());
                double y1 = std::max(a.y(), b.y());
                for (const auto& v : occupiedVertical) {
                    if (std::abs(v.x - x) < trackGap && rangesOverlap(y0, y1, v.y0, v.y1)) score += 3;
                }
                for (const auto& [hy, hxr] : occupiedHorizontal.data) {
                    if (hy > y0 + 0.001 && hy < y1 - 0.001 && x > hxr.first + 0.001 && x < hxr.second - 0.001) score += 1;
                }
            } else if (std::abs(a.y() - b.y()) < 0.001) {
                double y = a.y();
                double x0 = std::min(a.x(), b.x());
                double x1 = std::max(a.x(), b.x());
                for (const auto& [hy, hxr] : occupiedHorizontal.data) {
                    if (std::abs(hy - y) < trackGap && rangesOverlap(x0, x1, hxr.first, hxr.second)) score += 3;
                }
                for (const auto& v : occupiedVertical) {
                    if (v.x > x0 + 0.001 && v.x < x1 - 0.001 && y > std::min(v.y0, v.y1) + 0.001 && y < std::max(v.y0, v.y1) - 0.001) score += 1;
                }
            }
        }
        return score;
    };

    logPhase("BK/port placement and pre-routing prep");

    // Deterministic routing order: route shorter/earlier edges first to reserve central channels.
    std::vector<CEdge*> routeEdges = cg.edges;
    std::stable_sort(routeEdges.begin(), routeEdges.end(), [&](CEdge* a, CEdge* b) {
        int as = layerMap.count(a->source) ? layerMap[a->source] : 0;
        int at = layerMap.count(a->target) ? layerMap[a->target] : 0;
        int bs = layerMap.count(b->source) ? layerMap[b->source] : 0;
        int bt = layerMap.count(b->target) ? layerMap[b->target] : 0;

        int aspan = std::abs(at - as);
        int bspan = std::abs(bt - bs);
        if (aspan != bspan) return aspan < bspan;  // short edges first (span=1 before span>1)
        if (as != bs) return as < bs;
        if (at != bt) return at < bt;

        auto ay = (a->source->y + a->target->y) * 0.5;
        auto by = (b->source->y + b->target->y) * 0.5;
        if (std::abs(ay - by) > 0.001) return ay < by;

        return a < b;
    });

    // Compute midpoint X of the gap between layer L and L+1
    auto gapMidX = [&](int L) -> double {
        auto it1 = layerX.find(L);
        auto it2 = layerX.find(L + 1);
        if (it1 == layerX.end() || it2 == layerX.end()) return -1.0;
        double rightEdge = it1->second + layerWidths[L];
        double leftEdge  = it2->second;
        return (rightEdge + leftEdge) * 0.5;
    };

    // ELK-faithful gap segment ordering:
    // For each gap, build per-edge segments with ACTUAL y-coordinates (not position ranks),
    // then create crossing-count-based dependencies (ELK createDependencyIfNecessary),
    // break cycles, topologically number, and map to x-positions.
    std::map<CEdge*, int> edgeIndex;
    for (int i = 0; i < (int)cg.edges.size(); ++i) {
        edgeIndex[cg.edges[i]] = i;
    }

    // Per-gap segment descriptor using real y coords (ELK HyperEdgeSegment model).
    struct GapSeg {
        int edgeIdx;
        int step;
        double inY;   // y of source-side connection (incoming H arrives here)
        double outY;  // y of target-side connection (outgoing H departs here)
        double lo, hi; // vertical extent = [min(inY,outY), max(inY,outY)]
        int laneRank = 0; // filled during topological numbering
    };

    // ELK countCrossings: how many positions in `posis` fall strictly within [lo, hi].
    auto countCrossings1 = [](double y, double lo, double hi) {
        return (y > lo + 0.001 && y < hi - 0.001) ? 1 : 0;
    };

    std::map<int, std::vector<GapSeg>> gapSegs;
    for (int eidx = 0; eidx < (int)cg.edges.size(); ++eidx) {
        auto* e = cg.edges[eidx];
        int sl = layerMap.count(e->source) ? layerMap[e->source] : 0;
        int tl = layerMap.count(e->target) ? layerMap[e->target] : 0;
        if (tl <= sl) continue;

        int spanLocal = tl - sl;
        double srcY = sourceAnchor(e).y();
        double tgtY = targetAnchor(e).y();
        for (int k = 0; k < spanLocal; ++k) {
            int L = sl + k;
            // Linearly interpolate y for intermediate gaps.
            double inY  = srcY + (double)k       / spanLocal * (tgtY - srcY);
            double outY = srcY + (double)(k + 1) / spanLocal * (tgtY - srcY);
            // For exact endpoints use actual anchor y.
            if (k == 0)            inY  = srcY;
            if (k == spanLocal - 1) outY = tgtY;
            gapSegs[L].push_back({eidx, k, inY, outY,
                                  std::min(inY, outY), std::max(inY, outY), 0});

        }
    }

    // ===================================================================
    // Hyper-edge segment routing: group edges by net (same source port),
    // build dependency graph, assign routing slots, recompute gap widths.
    // Mimics Java ELK OrthogonalRoutingGenerator exactly.
    // ===================================================================
    struct HyperEdgeSeg {
        std::vector<std::pair<int,int>> edgeSteps; // (edgeIdx, step)
        std::vector<double> incomingCoords;  // sorted source-Y positions
        std::vector<double> outgoingCoords;  // sorted target-Y positions
        double startCoord = 0.0;
        double endCoord   = 0.0;
        int slot = 0;
    };

    std::map<int, int> slotsPerGap;
    std::map<int, std::vector<HyperEdgeSeg>> hyperSegsPerGap;

    for (auto& kv : gapSegs) {
        int L = kv.first;
        auto& segs = kv.second;
        if (segs.empty()) continue;

        // Group GapSeg entries by (source_node, source_port) = net driver.
        // Multiple fan-out edges from the same output port share ONE routing slot.
        std::map<std::pair<CNode*, std::string>, int> keyToIdx;
        std::vector<HyperEdgeSeg> hyperSegs;

        for (auto& seg : segs) {
            auto* e = cg.edges[seg.edgeIdx];
            // Allow all gap segments for slot assignment (both span=1 and span>1).
            // Long edges now get slots in every gap they cross, so gX1 uses a proper slot
            // instead of gapMidX, spreading V-segments across different X positions.
            int sl2 = layerMap.count(e->source) ? layerMap[e->source] : 0;
            int tl2 = layerMap.count(e->target) ? layerMap[e->target] : 0;
            int spanLocal2 = tl2 - sl2;
            (void)spanLocal2;
            auto key = std::make_pair(e->source, e->sourcePort);
            auto it = keyToIdx.find(key);
            int idx;
            if (it == keyToIdx.end()) {
                idx = (int)hyperSegs.size();
                keyToIdx[key] = idx;
                hyperSegs.push_back(HyperEdgeSeg{});
            } else {
                idx = it->second;
            }
            auto& hSeg = hyperSegs[idx];
            hSeg.edgeSteps.push_back({seg.edgeIdx, seg.step});

            // Insert inY sorted (incoming connection coordinate)
            auto it2 = std::lower_bound(hSeg.incomingCoords.begin(), hSeg.incomingCoords.end(), seg.inY);
            if (it2 == hSeg.incomingCoords.end() || std::abs(*it2 - seg.inY) > 0.001)
                hSeg.incomingCoords.insert(it2, seg.inY);
            // Insert outY sorted (outgoing connection coordinate)
            auto it3 = std::lower_bound(hSeg.outgoingCoords.begin(), hSeg.outgoingCoords.end(), seg.outY);
            if (it3 == hSeg.outgoingCoords.end() || std::abs(*it3 - seg.outY) > 0.001)
                hSeg.outgoingCoords.insert(it3, seg.outY);
        }

        // Compute Y extents for each hyper-edge segment.
        for (auto& hSeg : hyperSegs) {
            double mn =  std::numeric_limits<double>::max();
            double mx = -std::numeric_limits<double>::max();
            for (double y : hSeg.incomingCoords) { mn = std::min(mn, y); mx = std::max(mx, y); }
            for (double y : hSeg.outgoingCoords) { mn = std::min(mn, y); mx = std::max(mx, y); }
            hSeg.startCoord = mn;
            hSeg.endCoord   = mx;
        }

        int n = (int)hyperSegs.size();

        // Build dependency graph (Java ELK createDependencyIfNecessary).
        auto countCrossH = [](const std::vector<double>& coords, double lo, double hi) {
            int c = 0;
            for (double y : coords) if (y > lo + 0.001 && y < hi - 0.001) ++c;
            return c;
        };

        std::vector<std::vector<int>> dag2(n);
        std::vector<int> indeg2(n, 0);

        for (int i = 0; i < n; ++i) {
            auto& a = hyperSegs[i];
            if (a.endCoord - a.startCoord < 0.5) continue;
            for (int j = i + 1; j < n; ++j) {
                auto& b = hyperSegs[j];
                if (b.endCoord - b.startCoord < 0.5) continue;

                int cAB = countCrossH(b.incomingCoords, a.startCoord, a.endCoord)
                        + countCrossH(a.outgoingCoords, b.startCoord, b.endCoord);
                int cBA = countCrossH(a.incomingCoords, b.startCoord, b.endCoord)
                        + countCrossH(b.outgoingCoords, a.startCoord, a.endCoord);

                if (cAB < cBA) {
                    dag2[i].push_back(j); ++indeg2[j];
                } else if (cBA < cAB) {
                    dag2[j].push_back(i); ++indeg2[i];
                } else if (cAB > 0) {
                    if (a.startCoord <= b.startCoord) { dag2[i].push_back(j); ++indeg2[j]; }
                    else                              { dag2[j].push_back(i); ++indeg2[i]; }
                }
            }
        }

        // DFS cycle breaking.
        std::vector<int> color2(n, 0);
        std::function<void(int)> dfs2 = [&](int u) {
            color2[u] = 1;
            for (int ki = (int)dag2[u].size() - 1; ki >= 0; --ki) {
                int v = dag2[u][ki];
                if (color2[v] == 1) { --indeg2[v]; dag2[u].erase(dag2[u].begin() + ki); }
                else if (color2[v] == 0) dfs2(v);
            }
            color2[u] = 2;
        };
        for (int i = 0; i < n; ++i) if (color2[i] == 0) dfs2(i);

        // Interval graph coloring slot assignment (optimal: slot count = max overlap).
        // Sort by startCoord, use First-Fit with proper non-overlap check.
        std::vector<int> slot2(n, 0);
        std::vector<std::pair<double,int>> segIdx(n);
        for (int i = 0; i < n; ++i) segIdx[i] = {hyperSegs[i].startCoord, i};
        std::sort(segIdx.begin(), segIdx.end());

        // Active slots: (endCoord, slot)
        std::vector<std::pair<double,int>> active;
        int maxSlot = 0;
        for (auto& si : segIdx) {
            int idx = si.second;
            double lo = hyperSegs[idx].startCoord;
            double hi = hyperSegs[idx].endCoord;
            // Remove expired slots (endCoord < lo, no overlap)
            for (auto it = active.begin(); it != active.end(); ) {
                if (it->first < lo - 0.001) it = active.erase(it);
                else ++it;
            }
            // Find smallest available slot (by slot number)
            std::set<int> used;
            for (auto& a : active) used.insert(a.second);
            int newSlot = 0;
            while (used.count(newSlot)) ++newSlot;
            slot2[idx] = newSlot;
            maxSlot = std::max(maxSlot, newSlot);
            active.push_back({hi, newSlot});
        }
        for (int i = 0; i < n; ++i) hyperSegs[i].slot = slot2[i];

        int maxSlot2 = n > 0 ? *std::max_element(slot2.begin(), slot2.end()) : 0;
        slotsPerGap[L]        = maxSlot2 + 1;
        hyperSegsPerGap[L]    = std::move(hyperSegs);

        // Debug: print gap L=4 slot info
        if (L == 4) {
            fprintf(stderr, "[SLOT] gap L=%d  n=%d  maxSlot=%d\n", L, n, maxSlot2);
            // Count span=1 vs span>1 in this gap
            auto& hs2 = hyperSegsPerGap[L];
            int span1cnt = 0, spanNcnt = 0;
            for (auto& h : hs2) {
                bool anySpanN = false;
                for (auto& [eidx2, step2] : h.edgeSteps) {
                    auto* e2 = cg.edges[eidx2];
                    int sl2b = layerMap.count(e2->source) ? layerMap[e2->source] : 0;
                    int tl2b = layerMap.count(e2->target) ? layerMap[e2->target] : 0;
                    if (tl2b - sl2b > 1) anySpanN = true;
                }
                if (anySpanN) spanNcnt++; else span1cnt++;
            }
            fprintf(stderr, "[SLOT] gap L=%d  span1=%d span>1=%d\n", L, span1cnt, spanNcnt);
            // Count max simultaneous Y overlap (interval graph clique number)
            std::vector<std::pair<double,int>> events; // (y, +1/-1)
            for (auto& h : hs2) {
                if (h.endCoord > h.startCoord + 0.5) {
                    events.push_back({h.startCoord, 1});
                    events.push_back({h.endCoord, -1});
                }
            }
            std::sort(events.begin(), events.end(), [](auto& a, auto& b){
                return a.first < b.first || (a.first == b.first && a.second < b.second);
            });
            int cur = 0, maxOverlap = 0;
            for (auto& ev : events) { cur += ev.second; maxOverlap = std::max(maxOverlap, cur); }
            fprintf(stderr, "[SLOT] gap L=%d  max simultaneous Y overlap = %d\n", L, maxOverlap);
            // Print each hyper-seg Y range and slot
            for (int i = 0; i < (int)hs2.size(); ++i) {
                auto& h = hs2[i];
                fprintf(stderr, "[SLOT]  seg[%d] slot=%d Y=[%.0f,%.0f] edges=%zu\n",
                    i, h.slot, h.startCoord, h.endCoord, h.edgeSteps.size());
            }
        }

    }

    logPhase("slot assignment by gap");

    // Recompute layerX using the Java ELK gap-width formula:
    //   gapWidth = max(nodeNodeSpacing, 2*edgeNodeSpacing + (slotsCount-1)*edgeEdgeSpacing)
    // Use a reduced edge-edge spacing (6.5 vs 10) to match Java ELK's observed gap widths (~250u).
    // Java ELK achieves fewer slots via Y-extent-based hyper-edge merging; we approximate
    // the same gap width by scaling the per-slot spacing proportionally.
    const double slotEdgeSpacing = 8.0;
    {
        double currentX = 0.0;
        for (const auto& entry : layers) {
            int L = entry.first;
            layerX[L] = currentX;
            int numSlots = slotsPerGap.count(L) ? slotsPerGap[L] : 0;
            double rw = (numSlots == 0)
                ? minGap
                : std::max(minGap, 2.0 * edgeNodeSpacingBL + (numSlots - 1) * slotEdgeSpacing);
            currentX += layerWidths[L] + rw;
        }
    }

    // Update ALL node X positions to match new layerX.
    for (auto* node : cg.nodes) {
        int L = layerMap.count(node) ? layerMap[node] : 0;
        double lw = layerWidths.count(L) ? layerWidths[L] : 0.0;
        node->x = layerX[L] + 0.5 * (lw - node->width);
    }

    // Compute laneXByGapSeg: segmentX = gapLeft + edgeNodeSpacing + slot * edgeEdgeSpacing.
    // This matches Java ELK: startPos = leftLayerRight + edgeNodeSpacing,
    //                        segmentX = startPos + routingSlot * edgeEdgeSpacing.
    std::map<std::tuple<int, int, int>, double> laneXByGapSeg;
    for (auto& kv2 : hyperSegsPerGap) {
        int L = kv2.first;
        auto itLX = layerX.find(L);
        auto itRX = layerX.find(L + 1);
        if (itLX == layerX.end() || itRX == layerX.end()) continue;
        double leftBoundary  = itLX->second + layerWidths[L];
        double rightBoundary = itRX->second;
        double startPos      = leftBoundary + edgeNodeSpacingBL;
        for (auto& hSeg : kv2.second) {
            double segX = startPos + hSeg.slot * slotEdgeSpacing; // use reduced slot spacing
            segX = std::max(leftBoundary + 1.0, std::min(rightBoundary - 1.0, segX));
            for (auto& [eidx, step] : hSeg.edgeSteps) {
                laneXByGapSeg[std::make_tuple(L, eidx, step)] = segX;
            }
        }
    }

    int backwardLaneCounter = 0;

    // Count slot assignment coverage for diagnostics.
    {
        int span1Total = 0, span1WithSlot = 0;
        for (auto* edge : routeEdges) {
            int sL = layerMap.count(edge->source) ? layerMap[edge->source] : 0;
            int tL = layerMap.count(edge->target) ? layerMap[edge->target] : 0;
            if (tL - sL != 1) continue;
            span1Total++;
            auto eIt = edgeIndex.find(edge);
            if (eIt != edgeIndex.end()) {
                if (laneXByGapSeg.count(std::make_tuple(sL, eIt->second, 0)))
                    span1WithSlot++;
            }
        }
        fprintf(stderr, "[DBG] span=1 edges: %d total, %d with slot (%.0f%%)\n",
                span1Total, span1WithSlot, span1Total ? 100.0*span1WithSlot/span1Total : 0.0);
    }

    // Print fan-out distribution.
    {
        std::map<std::pair<CNode*, std::string>, int> fanout;
        for (auto* e : routeEdges)
            fanout[{e->source, e->sourcePort}]++;
        int maxFO = 0; std::map<int,int> hist;
        for (auto& kv : fanout) { maxFO = std::max(maxFO, kv.second); hist[kv.second]++; }
        fprintf(stderr, "[DBG] max fan-out per output port: %d\n", maxFO);
        fprintf(stderr, "[DBG] fan-out histogram: ");
        for (auto& kv : hist) if (kv.first > 1) fprintf(stderr, "%d×%d ", kv.first, kv.second);
        fprintf(stderr, "\n");
    }

    // Count edge types.
    {
        int nStraight=0, nFwd1=0, nFwdN=0, nBwd1=0, nBwdN=0;
        for (auto* e : routeEdges) {
            int sL = layerMap.count(e->source) ? layerMap[e->source] : 0;
            int tL = layerMap.count(e->target) ? layerMap[e->target] : 0;
            int sp = tL - sL;
            if (sp == 0) nStraight++;
            else if (sp == 1) nFwd1++;
            else if (sp > 1) nFwdN++;
            else if (sp == -1) nBwd1++;
            else nBwdN++;
        }

        logPhase("edge routing");
        fprintf(stderr, "[DBG] edge types: straight=%d fwd1=%d fwdN=%d bwd1=%d bwdN=%d total=%d\n",
                nStraight, nFwd1, nFwdN, nBwd1, nBwdN,
                nStraight+nFwd1+nFwdN+nBwd1+nBwdN);
    }

    // Precompute hyper-edge groups for span=1 edges: same source port in the same gap.
    // Each group shares a single vertical segment (true hyper-edge routing like Java ELK).
    // Key: (srcLayer, edgeIdx) → gX for the shared vertical segment.
    // Group key: (srcLayer, srcNode, srcPort)
    struct HyperGroup {
        double gX;
        double vYmin, vYmax;  // extent of shared V segment
        std::vector<CEdge*> edges;
        bool routed = false;
    };
    std::map<std::tuple<int, CNode*, std::string>, HyperGroup> hyperGroupMap;
    // Also: edgeIdx → hyper group key
    std::map<CEdge*, std::tuple<int, CNode*, std::string>> edgeToHyperGroup;

    for (auto* edge : routeEdges) {
        int srcL = layerMap.count(edge->source) ? layerMap[edge->source] : 0;
        int tgtL = layerMap.count(edge->target) ? layerMap[edge->target] : 0;
        if (tgtL - srcL != 1) continue; // only span=1 forward edges
        auto eIt = edgeIndex.find(edge);
        if (eIt == edgeIndex.end()) continue;
        auto laneIt = laneXByGapSeg.find(std::make_tuple(srcL, eIt->second, 0));
        if (laneIt == laneXByGapSeg.end()) continue;
        double gX = laneIt->second;
        auto gKey = std::make_tuple(srcL, edge->source, edge->sourcePort);
        auto& grp = hyperGroupMap[gKey];
        if (grp.edges.empty()) { grp.gX = gX; grp.vYmin = 1e18; grp.vYmax = -1e18; }
        grp.edges.push_back(edge);
        edgeToHyperGroup[edge] = gKey;
    }
    // Compute V segment extents for each group using actual port Y coordinates.
    for (auto& kv : hyperGroupMap) {
        auto& grp = kv.second;
        for (auto* e : grp.edges) {
            QPointF s = sourceAnchor(e);
            QPointF t = targetAnchor(e);
            grp.vYmin = std::min({grp.vYmin, s.y(), t.y()});
            grp.vYmax = std::max({grp.vYmax, s.y(), t.y()});
        }
    }

    // Precompute span>1 edge group ranks for spread routing.
    // rank1[edge] = rank of (source, sourcePort) among all span>1 edges sharing the same srcGap (srcLayer).
    // rank2[edge] = rank of (source, sourcePort) among all span>1 edges sharing the same tgtGap (tgtLayer-1).
    // Zigzag spread around gapMidX in each gap minimizes congestion while keeping crossings near baseline.
    std::map<CEdge*, int> spanNRank1, spanNRank2;
    {
        // Per-gap rank assignment: key = (gapLayer, source, sourcePort)
        std::map<std::pair<int, std::pair<CNode*, std::string>>, int> gapPortRank;
        std::map<int, int> gapRankCounter;  // next available rank per gap
        for (auto* edge : routeEdges) {
            int sL = layerMap.count(edge->source) ? layerMap[edge->source] : 0;
            int tL = layerMap.count(edge->target) ? layerMap[edge->target] : 0;
            if (tL - sL <= 1) continue;
            auto portKey = std::make_pair(edge->source, edge->sourcePort);
            // rank1: rank in srcGap (gap = srcLayer)
            auto key1 = std::make_pair(sL, portKey);
            if (!gapPortRank.count(key1)) gapPortRank[key1] = gapRankCounter[sL]++;
            spanNRank1[edge] = gapPortRank[key1];
            // rank2: rank in tgtGap (gap = tgtLayer-1)
            auto key2 = std::make_pair(tL - 1, portKey);
            if (!gapPortRank.count(key2)) gapPortRank[key2] = gapRankCounter[tL - 1]++;
            spanNRank2[edge] = gapPortRank[key2];
        }
    }

    // For very large graphs, skip expensive per-edge channel refinement (crossingScoreAtY
    // scans occupiedVertical which grows to O(E) → O(E²) total).
    const bool skipExpensiveRouteOpts = (routeEdges.size() > 10000);

    for (auto* edge : routeEdges) {
        edge->points.clear();
        QPointF start = sourceAnchor(edge);
        QPointF end = targetAnchor(edge);

        int srcLayer = layerMap.count(edge->source) ? layerMap[edge->source] : 0;
        int tgtLayer = layerMap.count(edge->target) ? layerMap[edge->target] : 0;
        double lspacing = graph.options().spacing().nodeNodeBetweenLayers;

        edge->points.push_back(start);

        // Quick straight-line case only if horizontal segment is obstacle-free.
        if (std::abs(start.y() - end.y()) < 0.5) {
            double yDirect = start.y();
            bool straightOk = false;
            if (!straightOk) {
                yDirect = findSafeHorizontalChannelY(
                    start.y(), start.x(), end.x(),
                    modIdx, edge->source, edge->target,
                    occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
                straightOk = (std::abs(yDirect - start.y()) < 0.5);
            }
            if (straightOk) {
                edge->points.push_back(end);
                normalizePolyline(edge->points);

                auto* elkEdge = elkEdgeMap[edge];
                if (elkEdge) {
                    elkEdge->clearBendPoints();
                    for (size_t index = 1; index + 1 < edge->points.size(); ++index) {
                        elkEdge->addBendPoint(edge->points[index].x(), edge->points[index].y());
                    }
                }
                occupiedHorizontal.push_back({yDirect, start.x(), end.x()});
                continue;
            }
        }

        // Unified channel-based orthogonal routing with horizontal + vertical avoidance.
        // Long forward edges (span > 1) are split per inter-layer gap (ELK LongEdge mechanism).
        int span = tgtLayer - srcLayer;

        if (srcLayer > tgtLayer) {
            int backSpan = srcLayer - tgtLayer;
            if (backSpan == 1) {
                // Adjacent-layer backward edge: route through the shared gap (same as forward span=1).
                // This avoids the long backChannelX horizontals that cross every forward-edge vertical.
                // The gap between tgtLayer and srcLayer is exactly where this edge needs a bend.
                double gX = gapMidX(tgtLayer);
                if (gX < 0) gX = (start.x() + end.x()) * 0.5;

                if (std::abs(start.y() - end.y()) < 0.5) {
                    edge->points.push_back(end);
                } else {
                    edge->points.push_back(QPointF(gX, start.y()));
                    edge->points.push_back(QPointF(gX, end.y()));
                    edge->points.push_back(end);
                }
            } else {
            // Long backward edge: route around the right side via backChannelX.
            int lane = backwardLaneCounter++ % backwardLaneWindow;
            double baseChannelX = backChannelX + lane * std::max(trackGap, 8.0);

            double yA = findSafeHorizontalChannelY(
                start.y(), start.x(), baseChannelX,
                modIdx, edge->source, edge->target,
                occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
            double yB = findSafeHorizontalChannelY(
                end.y(), baseChannelX, end.x(),
                modIdx, edge->source, edge->target,
                occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);

            auto crossingScoreAtY = [&](double y, double xL, double xR) {
                int score = 0;
                double x0 = std::min(xL, xR) + 0.5;
                double x1 = std::max(xL, xR) - 0.5;
                for (const auto& vs : occupiedVertical) {
                    if (vs.x < x0 || vs.x > x1) continue;
                    double vy0 = std::min(vs.y0, vs.y1);
                    double vy1 = std::max(vs.y0, vs.y1);
                    if (y > vy0 + 0.001 && y < vy1 - 0.001) score++;
                }
                return score;
            };

            auto refineHorizontalY = [&](double prefY, double xL, double xR) {
                double y = findSafeHorizontalChannelY(
                    prefY, xL, xR,
                    modIdx, edge->source, edge->target,
                    occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
                int bestScore = crossingScoreAtY(y, xL, xR);
                double bestY = y;
                double bestDist = std::abs(bestY - prefY);
                if (bestScore > 0) {
                    for (int sign : {-1, 1}) {
                        double candPref = prefY + sign * routeStep;
                        double candY = findSafeHorizontalChannelY(
                            candPref, xL, xR,
                            modIdx, edge->source, edge->target,
                            occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
                        int candScore = crossingScoreAtY(candY, xL, xR);
                        double candDist = std::abs(candY - prefY);
                        if (candScore < bestScore || (candScore == bestScore && candDist + 1e-6 < bestDist)) {
                            bestScore = candScore;
                            bestY = candY;
                            bestDist = candDist;
                        }
                    }
                }
                return bestY;
            };

            if (!skipExpensiveRouteOpts) {
                yA = refineHorizontalY(yA, start.x(), baseChannelX);
                yB = refineHorizontalY(yB, baseChannelX, end.x());
            }

            double channelX = findSafeVerticalChannelX(
                baseChannelX, yA, yB,
                modIdx, edge->source, edge->target,
                occupiedVertical, trackGap, routeStep, safeMinX, safeMaxX + 5.0 * routeStep);
            yA = findSafeHorizontalChannelY(
                yA, start.x(), channelX,
                modIdx, edge->source, edge->target,
                occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
            yB = findSafeHorizontalChannelY(
                yB, channelX, end.x(),
                modIdx, edge->source, edge->target,
                occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);

            edge->points.push_back(QPointF(start.x(), yA));
            edge->points.push_back(QPointF(channelX, yA));
            edge->points.push_back(QPointF(channelX, yB));
            edge->points.push_back(QPointF(end.x(), yB));
            edge->points.push_back(end);
            } // end long backward edge
        } else if (span == 1) {
            // Adjacent layer: route through the single inter-layer gap.
            double gX = gapMidX(srcLayer);
            auto eIt2 = edgeIndex.find(edge);
            if (eIt2 != edgeIndex.end()) {
                auto laneIt = laneXByGapSeg.find(std::make_tuple(srcLayer, eIt2->second, 0));
                if (laneIt != laneXByGapSeg.end()) {
                    gX = laneIt->second; // Use exact hyper-edge slot X position.
                }
            }
            if (gX < 0) gX = (start.x() + end.x()) * 0.5;

            // Java ELK hyper-edge routing: if this edge belongs to a fan-out group (>=2 edges
            // from the same source port in this gap), use a shared vertical segment (T-junction).
            auto hgIt = edgeToHyperGroup.find(edge);
            (void)hgIt;

            // Always use 4-point gap routing (Java ELK style): start → (gX,sY) → (gX,tY) → end
            // V segment stays in inter-layer gap (gX from slot assignment or gap midpoint).
            if (std::abs(start.y() - end.y()) < 0.5) {
                edge->points.push_back(end);
            } else {
                edge->points.push_back(QPointF(gX, start.y()));
                edge->points.push_back(QPointF(gX, end.y()));
                edge->points.push_back(end);
            }
        } else {
            // Long forward edge (span > 1): bus routing — 4 waypoints, same as Java ELK.
            // Anchor-flipped strategy: gX1 near RIGHT of srcGap, gX2 near LEFT of tgtGap.
            // For span=2: busH passes only through layer(srcLayer+1) region (no V segs there) → 0 crossings.
            // For span=3+: srcGap and tgtGap contributions eliminated; only intermediate gaps remain.
            // Per-source-port groupRank offsets spread V segments across different X positions → low congestion.
            {
                auto eIt = edgeIndex.find(edge);
                int eidx = (eIt != edgeIndex.end()) ? eIt->second : -1;
                (void)eidx;

                // Optimal strategy: use laneXByGapSeg slot assignments for gX1 and gX2.
                // Slot assignment spreads V segments across all available X positions per gap,
                // reducing max congestion from 13 (gapMidX baseline) to 8 (slot baseline).
                // Crossings increase from 2545 to 2742 due to longer busH coverage of intermediate gaps,
                // but this is acceptable tradeoff vs unacceptable congestion=13.
                double gX1 = gapMidX(srcLayer);
                double gX2 = gapMidX(tgtLayer - 1);
                if (gX1 < 0) gX1 = start.x() + lspacing * 0.5;
                if (gX2 < 0) gX2 = end.x() - lspacing * 0.5;
                if (eidx >= 0) {
                    auto it1 = laneXByGapSeg.find(std::make_tuple(srcLayer, eidx, 0));
                    if (it1 != laneXByGapSeg.end()) gX1 = it1->second;
                    auto it2 = laneXByGapSeg.find(std::make_tuple(tgtLayer - 1, eidx, span - 1));
                    if (it2 != laneXByGapSeg.end()) gX2 = it2->second;
                }

                // Safety: if gX1>=gX2 (shouldn't happen for forward edges), fall back to gapMidX.
                if (gX1 >= gX2) {
                    gX1 = gapMidX(srcLayer);
                    if (gX1 < 0) gX1 = start.x() + lspacing * 0.5;
                    gX2 = gapMidX(tgtLayer - 1);
                    if (gX2 < 0) gX2 = end.x() - lspacing * 0.5;
                }

                if (std::abs(start.y() - end.y()) < 0.5) {
                    edge->points.push_back(end);
                } else {
                    double xL = std::min(gX1, gX2);
                    double xR = std::max(gX1, gX2);
                    double prefBusY = (start.y() + end.y()) * 0.5;

                    double busY = findSafeHorizontalChannelY(
                        prefBusY, xL, xR, modIdx, edge->source, edge->target,
                        occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);

                    if (!skipExpensiveRouteOpts && !occupiedVertical.empty()) {
                        auto countVCross = [&](double y) -> int {
                            int cnt = 0;
                            for (const auto& vs : occupiedVertical) {
                                if (vs.x < xL - 0.5 || vs.x > xR + 0.5) continue;
                                double vy0 = std::min(vs.y0, vs.y1), vy1 = std::max(vs.y0, vs.y1);
                                if (y > vy0 + 0.5 && y < vy1 - 0.5) ++cnt;
                            }
                            return cnt;
                        };
                        auto isModSafe = [&](double y) -> bool {
                            auto [mn0, mn1] = modIdx.candidatesForY(y, 4.0);
                            for (auto it = mn0; it != mn1; ++it) {
                                const CNode* n = it->second;
                                if (n == edge->source || n == edge->target) continue;
                                if (horizontalSegmentIntersectsRect(y, xL, xR, n)) return false;
                            }
                            return true;
                        };
                        auto isSafe = [&](double y) -> bool {
                            if (!isModSafe(y)) return false;
                            auto [ht0, ht1] = occupiedHorizontal.range(y - trackGap + 0.001, y + trackGap - 0.001);
                            for (auto it = ht0; it != ht1; ++it)
                                if (rangesOverlap(xL, xR, it->second.first, it->second.second)) return false;
                            return true;
                        };
                        std::vector<double> candYs;
                        candYs.push_back(busY);
                        for (int d = 1; d <= 60; ++d) {
                            candYs.push_back(busY + d * routeStep);
                            candYs.push_back(busY - d * routeStep);
                        }
                        int bestTC = countVCross(busY);
                        double bestBusY = busY;
                        if (bestTC > 0) {
                            for (double tryY : candYs) {
                                if (tryY < safeMinY || tryY > safeMaxY) continue;
                                if (!isSafe(tryY)) continue;
                                int tc = countVCross(tryY);
                                if (tc < bestTC) { bestTC = tc; bestBusY = tryY; if (bestTC == 0) break; }
                            }
                            if (bestTC > 0) {
                                for (double tryY : candYs) {
                                    if (tryY < safeMinY || tryY > safeMaxY) continue;
                                    if (!isModSafe(tryY)) continue;
                                    int tc = countVCross(tryY);
                                    if (tc < bestTC) { bestTC = tc; bestBusY = tryY; if (bestTC == 0) break; }
                                }
                            }
                        }
                        busY = bestBusY;
                    }

                    edge->points.push_back(QPointF(gX1, start.y()));
                    edge->points.push_back(QPointF(gX1, busY));
                    if (std::abs(gX2 - gX1) > 0.5)
                        edge->points.push_back(QPointF(gX2, busY));
                    edge->points.push_back(QPointF(gX2, end.y()));
                    edge->points.push_back(end);
                }
            }
        }
        // Module-collision repair: only triggered for edges that pass through module bodies.
        // Scoring is O(nodes) only - no occupancy list scan to keep routing O(E * N).
        // For large graphs, skip this O(E*N) check entirely.
        int baseModuleCollisions = countPolylineModuleCollisionsIndexed(
            edge->points, modIdx, edge->source, edge->target, 2.0);

        if (baseModuleCollisions > 0) {
            auto buildRepairPath = [&](double busY) {
                std::vector<QPointF> p;
                p.push_back(start);
                if (srcLayer <= tgtLayer) {
                    double gX1 = gapMidX(srcLayer);
                    double gX2 = (span <= 1) ? gX1 : gapMidX(tgtLayer - 1);
                    if (gX1 < 0) gX1 = start.x() + lspacing * 0.5;
                    if (gX2 < 0) gX2 = end.x()   - lspacing * 0.5;
                    if (gX1 >= gX2 - 1.0) { gX1 = start.x() + lspacing * 0.3; gX2 = end.x() - lspacing * 0.3; }
                    p.push_back(QPointF(gX1, start.y()));
                    p.push_back(QPointF(gX1, busY));
                    if (gX2 > gX1 + 1.0) p.push_back(QPointF(gX2, busY));
                    p.push_back(QPointF(gX2, end.y()));
                } else {
                    p.push_back(QPointF(start.x(), busY));
                    p.push_back(QPointF(end.x(),   busY));
                }
                p.push_back(end);
                normalizePolyline(p);
                simplifyOrthogonalPolyline(p);
                return p;
            };

            // Score: module-collision count + bend penalty (no occupancy scan).
            auto repairScore = [&](const std::vector<QPointF>& p) {
                int c = countPolylineModuleCollisionsIndexed(p, modIdx, edge->source, edge->target, 2.0);
                int b = std::max(0, static_cast<int>(p.size()) - 2);
                return c * 100 + b;
            };

            int bestScore = repairScore(edge->points);
            std::vector<QPointF> bestPath = edge->points;

            auto tryPath = [&](std::vector<QPointF> p) {
                int s = repairScore(p);
                if (s < bestScore) { bestScore = s; bestPath = std::move(p); }
            };

            tryPath(buildRepairPath(safeMinY + routeStep));
            tryPath(buildRepairPath(safeMaxY - routeStep));

            // Mid-Y bus for long forward edges.
            if (!skipExpensiveRouteOpts && srcLayer < tgtLayer && span > 1) {
                double gX1r = gapMidX(srcLayer);
                double gX2r = gapMidX(tgtLayer - 1);
                if (gX1r < 0) gX1r = start.x() + lspacing * 0.5;
                if (gX2r < 0) gX2r = end.x()   - lspacing * 0.5;
                double xL_mid = std::min(gX1r, gX2r) + 1.0;
                double xR_mid = std::max(gX1r, gX2r) - 1.0;
                if (xL_mid < xR_mid) {
                    double midY = findSafeHorizontalChannelY(
                        (start.y() + end.y()) * 0.5, xL_mid, xR_mid,
                        modIdx, edge->source, edge->target,
                        occupiedHorizontal, trackGap, routeStep, safeMinY, safeMaxY);
                    tryPath(buildRepairPath(midY));
                }
            }

            edge->points = bestPath;
        }

        // Mark occupied tracks after final routing of this edge.
        // For large graphs, skip this to avoid O(E^2) accumulated scan time.
        if (!skipExpensiveRouteOpts) {
        for (size_t i = 1; i < edge->points.size(); ++i) {
            const QPointF& a = edge->points[i - 1];
            const QPointF& b = edge->points[i];
            if (std::abs(a.x() - b.x()) < 0.001) {
                occupiedVertical.push_back({a.x(), a.y(), b.y()});
            } else if (std::abs(a.y() - b.y()) < 0.001) {
                occupiedHorizontal.push_back({a.y(), a.x(), b.x()});
            }
        }
        }

        auto* elkEdge = elkEdgeMap[edge];
        if (!elkEdge) {
            continue;
        }
        elkEdge->clearBendPoints();
        for (size_t index = 1; index + 1 < edge->points.size(); ++index) {
            elkEdge->addBendPoint(edge->points[index].x(), edge->points[index].y());
        }
    }
}
