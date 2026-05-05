/**
 * ELK C++ Port - Layered Layout Implementation
 * Migration from org.eclipse.elk.alg.layered
 *
 * Implements the standard ELK Layered layout pipeline:
 * P1: Cycle breaking (already done in applyElkLayout)
 * P2: Layer assignment (longestPathLayering)
 * P3: Node ordering (already done in applyElkLayout)
 * P4: Node placement (brandesKoepfPlace)
 * P5: Edge routing (orthogonalRoute)
 *
 * Reference: org.eclipse.elk.alg.layered.p2layers.LongestPathLayerer
 */

#include "elk_layered.hpp"
#include "elk_graph.hpp"
#include "elk_port.hpp"
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <limits>
#include <cmath>
#include <stack>

namespace elk {

// Forward declarations
class LGraph;
class LNode;
class LEdge;
class Layer;
class ElkGraph;
class LayoutOptions;

namespace {

// Helper: collect all layerless nodes from LGraph
std::vector<LNode*> getLayerlessNodes(LGraph* lgraph) {
    std::vector<LNode*> nodes;
    for (const auto& n : lgraph->nodes()) {
        nodes.push_back(n.get());
    }
    return nodes;
}

// Helper: get outgoing LEdges from a node via its ports
// LPort wraps an ElkPort, which inherits from ElkConnectableShape with outgoingEdges()
std::vector<ElkEdge*> getOutgoingElkEdges(LNode* node, LGraph* lgraph) {
    std::vector<ElkEdge*> edges;
    ElkNode* elkNode = node->elkNode();
    if (!elkNode) return edges;
    ElkGraph* graph = lgraph->elkGraph();
    if (!graph) return edges;

    // Get all edges and filter by source
    auto allEdges = graph->getAllEdges();
    for (auto* edge : allEdges) {
        if (edge->source() == elkNode) {
            edges.push_back(edge);
        }
    }
    return edges;
}

// Helper: get target LNode from an ElkEdge
// Note: LNode wraps ElkNode, so we need to find the LNode whose elkNode() matches
static LNode* getTargetLNode(ElkEdge* edge, const std::map<ElkNode*, LNode*>& elkToLNode) {
    if (!edge) return nullptr;
    ElkConnectableShape* tgt = edge->target();
    if (!tgt) return nullptr;
    // tgt is an ElkNode (inherits from ElkConnectableShape)
    ElkNode* elkNode = dynamic_cast<ElkNode*>(tgt);
    if (!elkNode) return nullptr;
    // Look up the LNode that wraps this ElkNode
    auto it = elkToLNode.find(elkNode);
    if (it != elkToLNode.end()) return it->second;
    return nullptr;
}

} // anonymous namespace

// ============================================================
// P2: Longest Path Layering
// Matches: org.eclipse.elk.alg.layered.p2layers.LongestPathLayerer
//
// Java algorithm (DFS post-order with memoization):
//   visit(node):
//     if visited: return height
//     maxHeight = 1
//     for each outgoing edge:
//       targetHeight = visit(target)
//       maxHeight = max(maxHeight, targetHeight + 1)
//     putNode(node, maxHeight)
//     return maxHeight
//
// Key behavior:
// - Ignores self-loops
// - Nodes with no outgoing edges get height=1 (bottom layer)
// - Layers stored with index 0 = top (layer count - height)
// ============================================================

void longestPathLayering(LGraph* lgraph) {
    if (!lgraph) return;

    auto nodes = getLayerlessNodes(lgraph);
    if (nodes.empty()) return;

    int n = (int)nodes.size();

    // nodeHeights: -1 = unvisited, >=0 = layer height (1-indexed)
    std::vector<int> nodeHeights(n, -1);

    // Build index map
    std::map<LNode*, int> nodeIndex;
    for (int i = 0; i < n; ++i) {
        nodeIndex[nodes[i]] = i;
    }

    // Build ElkNode* -> LNode* lookup
    std::map<ElkNode*, LNode*> elkToLNode;
    for (int i = 0; i < n; ++i) {
        elkToLNode[nodes[i]->elkNode()] = nodes[i];
    }

    // Recursive DFS post-order with memoization
    std::function<int(LNode*)> visit = [&](LNode* node) -> int {
        int idx = nodeIndex[node];
        if (nodeHeights[idx] >= 0) {
            return nodeHeights[idx];  // already visited
        }
        if (nodeHeights[idx] == -2) {
            return 0;  // cycle: break recursion
        }
        nodeHeights[idx] = -2;  // mark as in-progress to detect cycles

        int maxHeight = 1;

        // Check all outgoing edges (following Java LongestPathLayerer logic)
        auto outEdges = getOutgoingElkEdges(node, lgraph);

        for (auto* elkEdge : outEdges) {
            LNode* target = getTargetLNode(elkEdge, elkToLNode);
            if (!target || target == node) {
                continue;  // skip self-loops like Java
            }

            auto it = nodeIndex.find(target);
            if (it == nodeIndex.end()) {
                continue;
            }

            int targetHeight = visit(target);
            maxHeight = std::max(maxHeight, targetHeight + 1);
        }

        nodeHeights[idx] = maxHeight;
        return maxHeight;
    };

    // Visit all nodes to compute heights
    for (auto* node : nodes) {
        visit(node);
    }

    // Find max height to determine number of layers
    int maxHeight = 0;
    for (int h : nodeHeights) {
        maxHeight = std::max(maxHeight, h);
    }

    // Create layers: layer 0 = top (index 0), layer maxHeight-1 = bottom
    // Java stores layers in "reverse" order: layer 0 = bottom-most
    // But we store layer[0] = top, so maxHeight-node gets layer 0
    auto& layers = lgraph->layers();
    layers.clear();

    // Create maxHeight layers: layer index 0 = top, index maxHeight-1 = bottom
    // A node with height=1 goes to layer (maxHeight - 1) which is the bottom
    // A node with height=maxHeight goes to layer 0 which is the top
    // This is correct for input-to-output flow (LEFT-to-RIGHT layout)
    for (int i = 0; i < maxHeight; ++i) {
        layers.push_back(std::make_unique<Layer>(i));
    }

    // Assign each node to its layer: height=h → layer index = maxHeight - h
    for (int i = 0; i < n; ++i) {
        int height = nodeHeights[i];
        int layerIdx = maxHeight - height;  // height 1 → maxHeight-1 (bottom), height maxHeight → 0 (top)
        if (layerIdx >= 0 && layerIdx < (int)layers.size()) {
            layers[layerIdx]->addNode(nodes[i]);
            nodes[i]->setLayer(layerIdx);
        }
    }
}

// ============================================================
// P4: Brandes-Koepf Node Placement
// Matches: org.eclipse.elk.alg.layered.p4nodes.bk.BKNodePlacer
//
// Algorithm steps:
// 1. markConflicts() - Mark type-1 conflicts (short edge crosses long edge)
// 2. verticalAlignment() x4 - Build blocks in 4 directions (RD, RU, LD, LU)
// 3. insideBlockShift() x4 - Calculate inner shifts using port anchors
// 4. horizontalCompaction() x4 - Place blocks
// 5. chooseLayout() - Select best or balanced (median) layout
//
// Reference: BKNodePlacer.java, BKAligner.java, BKCompactor.java
// ============================================================

namespace bk {

// Direction enums (matching Java BKAlignedLayout.VDirection/HDirection)
enum class VDirection { DOWN, UP };
enum class HDirection { RIGHT, LEFT };

// Block alignment state for one pass
struct BKLayout {
    std::vector<LNode*> roots;          // root[nodeIndex] = block root
    std::vector<LNode*> align;          // align[nodeIndex] = next node in block (ring)
    std::vector<double> innerShift;     // innerShift[nodeIndex] = Y offset within block
    std::vector<double> blockSize;       // blockSize[rootIndex] = total block height
    std::vector<LNode*> sink;           // sink[rootIndex] = class root
    std::vector<double> shift;          // shift[rootIndex] = block shift for compaction
    std::vector<double> y;               // y[nodeIndex] = final Y coordinate
    std::vector<bool> su;                // su[rootIndex] = block is straightened
    std::vector<bool> od;                // od[rootIndex] = block is only-dummy nodes
    VDirection vdir;
    HDirection hdir;
    double layoutSize_ = 0.0;

    BKLayout(size_t nodeCount, VDirection v, HDirection h)
        : roots(nodeCount, nullptr), align(nodeCount, nullptr),
          innerShift(nodeCount, 0.0), blockSize(nodeCount, 0.0),
          sink(nodeCount, nullptr), shift(nodeCount, 0.0),
          y(nodeCount, 0.0), su(nodeCount, false), od(nodeCount, true),
          vdir(v), hdir(h) {
        for (size_t i = 0; i < nodeCount; ++i) {
            roots[i] = nullptr;
            align[i] = nullptr;
            sink[i] = nullptr;
        }
    }
};

// Helper: get node index in its layer
int getNodeIndexInLayer(LNode* node) {
    (void)node;
    // Simplified: actual implementation tracks node position in layer
    return 0;
}

// Neighborhood information (pre-computed like Java NeighborhoodInformation)
struct NeighborhoodInfo {
    std::map<LNode*, std::vector<std::pair<LNode*, ElkEdge*>>> leftNeighbors;
    std::map<LNode*, std::vector<std::pair<LNode*, ElkEdge*>>> rightNeighbors;
    std::map<LNode*, int> nodeIndex;
    int nodeCount = 0;
};

// Mark type-1 conflicts: edges where a non-inner segment crosses an inner segment
// Type 1 = non-inner (regular edge endpoint) crosses inner (long edge dummy node)
// These marked edges should NOT be used for alignment (prevents kinks in long edges)
void markConflicts(LGraph* lgraph, std::set<ElkEdge*>& markedEdges,
                   NeighborhoodInfo& ni) {
    markedEdges.clear();

    auto& layers = lgraph->layers();
    if (layers.size() < 3) return;  // Need at least 3 layers for conflicts

    int numberOfLayers = (int)layers.size();

    // Count nodes per layer
    std::vector<int> layerSize(numberOfLayers);
    for (int i = 0; i < numberOfLayers; ++i) {
        layerSize[i] = (int)layers[i]->nodes().size();
    }

    // Iterate through internal layers (not first or last)
    // For each node in layer i, check if it's incident to an inner segment
    // between layer i+1 and layer i
    for (int i = 1; i < numberOfLayers - 1; ++i) {
        const auto& currentLayerNodes = layers[i]->nodes();

        int k_0 = 0;

        for (int l_1 = 0; l_1 < layerSize[i + 1]; ++l_1) {
            // Node in layer i+1 at position l_1
            LNode* v_l_i = layers[i + 1]->nodes()[l_1];

            // Check if v_l_i is incident to an inner segment between layer i+1 and i
            bool isInner = false;
            int k_1 = layerSize[i] - 1;

            // Check left neighbors of v_l_i for inner segment
            auto leftNbrIt = ni.leftNeighbors.find(v_l_i);
            if (leftNbrIt != ni.leftNeighbors.end() && !leftNbrIt->second.empty()) {
                LNode* neighbor = leftNbrIt->second[0].first;
                auto it = ni.nodeIndex.find(neighbor);
                if (it != ni.nodeIndex.end()) {
                    k_1 = it->second;
                    isInner = true;
                }
            }

            // Process nodes in layer i from l to l_1
            for (int l = 0; l <= l_1 && l < layerSize[i]; ++l) {
                LNode* v_l = layers[i]->nodes()[l];

                // Check if v_l is NOT incident to inner segment
                // If not inner, mark all its left neighbors that are outside [k_0, k_1]
                auto leftNbrIt2 = ni.leftNeighbors.find(v_l);
                if (leftNbrIt2 != ni.leftNeighbors.end()) {
                    for (auto& p : leftNbrIt2->second) {
                        LNode* upperNeighbor = p.first;
                        ElkEdge* edge = p.second;

                        auto it = ni.nodeIndex.find(upperNeighbor);
                        if (it != ni.nodeIndex.end()) {
                            int k = it->second;
                            if (k < k_0 || k > k_1) {
                                markedEdges.insert(edge);
                            }
                        }
                    }
                }
            }

            k_0 = k_1;
        }
    }
}

// Vertical alignment: group nodes into blocks
// markedEdges contains type-1 conflicts - these prevent alignment
void verticalAlignment(BKLayout& bal, LGraph* lgraph,
                      const std::set<ElkEdge*>& markedEdges,
                      NeighborhoodInfo& ni) {
    size_t n = bal.roots.size();

    // Initialize: each node is its own block
    for (size_t i = 0; i < n; ++i) {
        bal.roots[i] = nullptr;  // Will be set below
        bal.align[i] = nullptr;   // Will be set below
        bal.innerShift[i] = 0.0;
    }

    // Get layer count
    auto& layers = lgraph->layers();
    int layerCount = (int)layers.size();

    // Build layer node list for indexing
    std::vector<std::vector<LNode*>> layerNodes(layerCount);
    for (int L = 0; L < layerCount; ++L) {
        for (LNode* node : layers[L]->nodes()) {
            layerNodes[L].push_back(node);
        }
    }

    // Create node to layer+index mapping
    std::map<LNode*, std::pair<int, int>> nodePosition;
    for (int L = 0; L < layerCount; ++L) {
        for (int idx = 0; idx < (int)layerNodes[L].size(); ++idx) {
            nodePosition[layerNodes[L][idx]] = {L, idx};
        }
    }

    // Initialize roots and align for all nodes in layered graph
    for (int L = 0; L < layerCount; ++L) {
        for (LNode* node : layerNodes[L]) {
            auto it = ni.nodeIndex.find(node);
            if (it != ni.nodeIndex.end()) {
                int idx = it->second;
                bal.roots[idx] = node;
                bal.align[idx] = node;
            }
        }
    }

    // Traverse layers in direction order
    // hdir = RIGHT means left-to-right (successors are "upper")
    // hdir = LEFT means right-to-left (predecessors are "upper")
    std::vector<int> layerOrder;
    for (int L = 0; L < layerCount; ++L) layerOrder.push_back(L);
    if (bal.hdir == HDirection::LEFT) {
        std::reverse(layerOrder.begin(), layerOrder.end());
    }

    for (int L : layerOrder) {
        int adjLayer = (bal.hdir == HDirection::RIGHT) ? L - 1 : L + 1;
        if (adjLayer < 0 || adjLayer >= layerCount) continue;

        auto& nodes = layerNodes[L];
        int r;

        // VDirection.UP means bottom-to-top traversal (reverse node order)
        if (bal.vdir == VDirection::UP) {
            r = std::numeric_limits<int>::max();
            std::reverse(nodes.begin(), nodes.end());
        } else {
            r = -1;
        }

        for (LNode* v : nodes) {
            auto itV = ni.nodeIndex.find(v);
            if (itV == ni.nodeIndex.end()) continue;
            int vIdx = itV->second;

            // Only process nodes that are still their own block root
            if (bal.align[vIdx] != v) continue;

            // Get neighbors: RIGHT means successors (left neighbors in Java)
            // LEFT means predecessors (right neighbors in Java)
            const auto& neighbors = (bal.hdir == HDirection::RIGHT)
                ? ni.rightNeighbors[v] : ni.leftNeighbors[v];

            if (neighbors.empty()) {
                // Debug: count how many nodes have no neighbors
                continue;
            }

            // Sort neighbors by their position in the adjacent layer
            std::vector<std::pair<LNode*, ElkEdge*>> sortedNeighbors = neighbors;
            std::sort(sortedNeighbors.begin(), sortedNeighbors.end(),
                [&](const auto& a, const auto& b) {
                    auto itA = nodePosition.find(a.first);
                    auto itB = nodePosition.find(b.first);
                    if (itA == nodePosition.end() || itB == nodePosition.end()) return false;
                    return itA->second.second < itB->second.second;
                });

            int d = (int)sortedNeighbors.size();
            int lo = (d - 1) / 2;   // floor((d+1)/2) - 1, then +1 = floor((d+1)/2)
            int hi = d / 2;

            // VDirection.UP: iterate high→lo, VDirection.DOWN: iterate lo→hi
            if (bal.vdir == VDirection::UP) {
                std::swap(lo, hi);
            }
            int step = (bal.vdir == VDirection::DOWN) ? 1 : -1;

            for (int mi = lo; ; mi += step) {
                if (bal.vdir == VDirection::UP && mi < 0) break;
                if (bal.vdir == VDirection::DOWN && mi >= d) break;

                if (bal.align[vIdx] != v) break;  // Already aligned

                auto& u_m_pair = sortedNeighbors[mi];
                LNode* u_m = u_m_pair.first;
                ElkEdge* edge = u_m_pair.second;

                auto itU = ni.nodeIndex.find(u_m);
                if (itU == ni.nodeIndex.end()) continue;
                int uIdx = itU->second;

                // Skip if edge is marked as type-1 conflict
                if (markedEdges.count(edge)) continue;

                // Check monotonicity constraint
                int ui = -1;
                auto posIt = nodePosition.find(u_m);
                if (posIt != nodePosition.end()) {
                    ui = posIt->second.second;
                }

                bool monotone;
                if (bal.vdir == VDirection::DOWN) {
                    monotone = (ui > r);
                } else {
                    monotone = (ui < r);
                }
                if (!monotone) continue;

                // Align u_m to v
                bal.align[uIdx] = v;
                bal.roots[vIdx] = bal.roots[uIdx];
                bal.align[vIdx] = bal.roots[vIdx];
                r = ui;
                break;
            }
        }
    }
}

// Helper: build edge lookup from (source, target) -> LEdge*
static std::map<std::pair<LNode*, LNode*>, LEdge*> buildEdgeLookup(LGraph* lgraph) {
    std::map<std::pair<LNode*, LNode*>, LEdge*> edgeLookup;
    for (auto& edge : lgraph->edges()) {
        edgeLookup[{edge->source(), edge->target()}] = edge.get();
    }
    return edgeLookup;
}

// Helper: find node's index in nodeIndex map
static int findNodeIdx(LNode* node, const std::map<LNode*, int>& nodeIndex) {
    auto it = nodeIndex.find(node);
    return (it != nodeIndex.end()) ? it->second : -1;
}

// Helper: find root index for a node
static int findRootIdx(LNode* node, const std::vector<LNode*>& roots, const std::map<LNode*, int>& nodeIndex) {
    int nodeIdx = findNodeIdx(node, nodeIndex);
    if (nodeIdx < 0) return -1;
    LNode* root = roots[nodeIdx];
    if (!root) return -1;
    // Find which index in roots[] contains this root
    for (size_t i = 0; i < roots.size(); ++i) {
        if (roots[i] == root) return (int)i;
    }
    return -1;
}

// Helper: find the next node in the block ring by following align[]
// Returns the node that align[current] points to
static LNode* findAlignedNode(LNode* current, const std::vector<LNode*>& align,
                               const std::map<LNode*, int>& nodeIndex) {
    int currIdx = findNodeIdx(current, nodeIndex);
    if (currIdx < 0) return nullptr;
    return align[currIdx];
}

// Debug: print alignment state for a BKLayout
static void debugPrintAlignment(BKLayout& bal, LGraph* lgraph, const char* label) {
    fprintf(stderr, "[BK-ALIGN-%s] START: roots=%d layers=%d nodes=%d\n",
            label, (int)bal.roots.size(), (int)lgraph->layers().size(), (int)lgraph->nodes().size());
    if (bal.roots.empty() || lgraph->layers().empty()) {
        fprintf(stderr, "[BK-ALIGN-%s] EARLY EXIT: no roots or no layers\n", label);
        return;
    }

    fprintf(stderr, "[BK-ALIGN-%s] nodes=%d\n", label, (int)bal.roots.size());

    // Count blocks and their sizes
    std::map<LNode*, int> blockSizes;
    std::map<LNode*, int> blockRoots;  // root -> count of nodes pointing to it as root
    for (size_t i = 0; i < bal.roots.size(); ++i) {
        if (bal.roots[i]) {
            blockSizes[bal.roots[i]]++;
            blockRoots[bal.roots[i]]++;
        }
    }
    fprintf(stderr, "[BK-ALIGN-%s] blocks=%d\n", label, (int)blockSizes.size());

    // Count nodes where align[i] != i (aligned to different node)
    int alignedCount = 0;
    for (size_t i = 0; i < bal.align.size(); ++i) {
        if (bal.align[i] != lgraph->nodes()[i].get()) {
            alignedCount++;
        }
    }
    fprintf(stderr, "[BK-ALIGN-%s] aligned nodes=%d / %d\n", label, alignedCount, (int)bal.roots.size());

    // For first 5 blocks, print sizes
    int count = 0;
    for (auto& kv : blockSizes) {
        fprintf(stderr, "[BK-ALIGN-%s] block root=%s size=%d\n",
                label, kv.first->elkNode()->id().c_str(), kv.second);
        if (++count >= 5) { fprintf(stderr, "[BK-ALIGN-%s] ...\n", label); break; }
    }

    // Print innerShift range
    double minS = std::numeric_limits<double>::infinity();
    double maxS = -std::numeric_limits<double>::infinity();
    int cntS = 0;
    for (size_t i = 0; i < bal.innerShift.size(); ++i) {
        if (bal.roots[i]) {
            minS = std::min(minS, bal.innerShift[i]);
            maxS = std::max(maxS, bal.innerShift[i]);
            cntS++;
        }
    }
    fprintf(stderr, "[BK-ALIGN-%s] innerShift: min=%.1f max=%.1f cnt=%d\n", label, minS, maxS, cntS);

    // Print Y range
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    int cntY = 0;
    for (size_t i = 0; i < bal.y.size(); ++i) {
        if (bal.y[i] != 0.0 || bal.roots[i]) {
            minY = std::min(minY, bal.y[i]);
            maxY = std::max(maxY, bal.y[i]);
            cntY++;
        }
    }
    fprintf(stderr, "[BK-ALIGN-%s] Y: min=%.1f max=%.1f cnt=%d\n", label, minY, maxY, cntY);

    // Print blockY range (for debugging horizontalCompaction)
    fprintf(stderr, "[BK-ALIGN-%s] blockSize: min=%.1f max=%.1f\n", label,
            minS, maxS);  // Reusing minS/maxS for blockSize
}

// Inside block shift: calculate Y offsets within blocks using port positions
// Matches Java ELK BKAligner.insideBlockShift()
// For each block root, traverse the block ring following align[] pointers,
// and compute cumulative port position differences as inner shifts.
void insideBlockShift(BKLayout& bal, LGraph* lgraph) {
    auto& layers = lgraph->layers();
    int layerCount = (int)layers.size();
    if (layerCount == 0) return;

    // Build layer node list
    std::vector<std::vector<LNode*>> layerNodes(layerCount);
    for (int L = 0; L < layerCount; ++L) {
        for (LNode* node : layers[L]->nodes()) {
            layerNodes[L].push_back(node);
        }
    }

    // Build node to layer+index mapping
    std::map<LNode*, std::pair<int, int>> nodePosition;
    for (int L = 0; L < layerCount; ++L) {
        for (int idx = 0; idx < (int)layerNodes[L].size(); ++idx) {
            nodePosition[layerNodes[L][idx]] = {L, idx};
        }
    }

    // Build node index map
    std::map<LNode*, int> nodeIndex;
    for (int L = 0; L < layerCount; ++L) {
        for (int idx = 0; idx < (int)layerNodes[L].size(); ++idx) {
            nodeIndex[layerNodes[L][idx]] = idx;
        }
    }

    // Build edge lookup for block edges
    auto edgeLookup = buildEdgeLookup(lgraph);

    size_t n = bal.roots.size();

    // Initialize innerShift for all nodes
    for (size_t i = 0; i < n; ++i) {
        bal.innerShift[i] = 0.0;
    }

    // For each block root, traverse the block ring and calculate inner shifts
    // A block is identified by a root node. The ring is traversed by following align[].
    // For each edge curr->next in the ring, innerShift[next] = innerShift[curr] + portDiff
    // where portDiff = srcPort.y - tgtPort.y (for HDirection.RIGHT)

    // Process each node as a potential block root
    for (auto& nodePtr : lgraph->nodes()) {
        LNode* node = nodePtr.get();
        int nodeIdx = findNodeIdx(node, nodeIndex);
        if (nodeIdx < 0) continue;

        LNode* root = bal.roots[nodeIdx];
        if (!root || root != node) continue;  // Only process block roots (node is its own root)

        // Start at this root, traverse the ring
        LNode* curr = node;
        bal.innerShift[nodeIdx] = 0.0;  // Root's innerShift is 0

        int safety = 0;
        while (true) {
            if (++safety > 100000) break;

            // Find next node in ring (what curr points to via align[])
            LNode* next = findAlignedNode(curr, bal.align, nodeIndex);
            if (!next || next == node) break;  // Back to root = ring complete

            int currIdx = findNodeIdx(curr, nodeIndex);
            int nextIdx = findNodeIdx(next, nodeIndex);
            if (currIdx < 0 || nextIdx < 0) break;

            // Find edge connecting curr -> next (or reverse)
            LEdge* edge = nullptr;
            auto edgeIt = edgeLookup.find({curr, next});
            if (edgeIt != edgeLookup.end()) {
                edge = edgeIt->second;
            } else {
                edgeIt = edgeLookup.find({next, curr});
                if (edgeIt != edgeLookup.end()) edge = edgeIt->second;
            }

            double portDiff = 0.0;
            if (edge) {
                // Get port center Y positions for src and tgt
                double srcCY = curr->height() / 2.0;
                double tgtCY = next->height() / 2.0;

                LPort* srcPort = edge->sourcePort();
                LPort* tgtPort = edge->targetPort();
                if (srcPort) {
                    srcCY = srcPort->anchorY();
                }
                if (tgtPort) {
                    tgtCY = tgtPort->anchorY();
                }

                // portPosDiff: srcPort.y - tgtPort.y
                portDiff = srcCY - tgtCY;
            }

            // Accumulate: innerShift[next] = innerShift[curr] + portDiff
            bal.innerShift[nextIdx] = bal.innerShift[currIdx] + portDiff;
            curr = next;
        }
    }
}

// Helper: find same-layer neighbor (prev=true means idx-1, prev=false means idx+1)
static LNode* findSameLayerNeighbor(LNode* node, bool prev,
                                    const std::map<LNode*, std::pair<int, int>>& nodePosition,
                                    const std::vector<std::vector<LNode*>>& layerNodes) {
    auto it = nodePosition.find(node);
    if (it == nodePosition.end()) return nullptr;

    int L = it->second.first;
    int idx = it->second.second;
    int newIdx = prev ? idx - 1 : idx + 1;

    if (L >= 0 && L < (int)layerNodes.size()) {
        if (newIdx >= 0 && newIdx < (int)layerNodes[L].size()) {
            return layerNodes[L][newIdx];
        }
    }
    return nullptr;
}

// Helper: check if node is a "dummy" (long edge endpoint marker)
// In the C++ port, we identify dummies by checking if the node has no ports
// or has zero dimensions (marker for hyperedge segments)
static bool isDummyNode(LNode* node) {
    if (!node) return false;
    // Dummies are typically created for long edges and have specific properties
    // Check if it's a BK dummy: nodes with very small or zero dimensions
    return node->width() < 1.0 || node->height() < 1.0;
}

// Horizontal compaction: place blocks with minimal y coordinates
// Matches Java ELK BKCompactor.placeBlock():
// For each block, recursively place neighbor blocks first, then compute
// block position based on same-layer neighbor spacing constraints.
void horizontalCompaction(BKLayout& bal, LGraph* lgraph) {
    auto& layers = lgraph->layers();
    int layerCount = (int)layers.size();
    if (layerCount == 0) return;

    // Build layer node list and position map
    std::vector<std::vector<LNode*>> layerNodes(layerCount);
    std::map<LNode*, std::pair<int, int>> nodePosition;
    for (int L = 0; L < layerCount; ++L) {
        for (int idx = 0; idx < (int)layers[L]->nodes().size(); ++idx) {
            LNode* node = layers[L]->nodes()[idx];
            layerNodes[L].push_back(node);
            nodePosition[node] = {L, idx};
        }
    }

    size_t n = bal.roots.size();
    if (n == 0) return;

    // Build node index map (index in lgraph->nodes())
    std::map<LNode*, int> graphNodeIndex;
    int gIdx = 0;
    for (auto& nodePtr : lgraph->nodes()) {
        graphNodeIndex[nodePtr.get()] = gIdx++;
    }

    // Initialize blockSize, shift, sink, su, od
    for (size_t i = 0; i < n; ++i) {
        bal.blockSize[i] = 0.0;
        bal.shift[i] = 0.0;
        bal.sink[i] = bal.roots[i];
        bal.su[i] = false;
        bal.od[i] = true;
    }

    // Compute block size for each block (sum of node heights in the block)
    for (int L = 0; L < layerCount; ++L) {
        for (LNode* node : layerNodes[L]) {
            auto it = nodePosition.find(node);
            if (it == nodePosition.end()) continue;

            // Find node's index in graph nodes
            int nodeGraphIdx = -1;
            auto git = graphNodeIndex.find(node);
            if (git != graphNodeIndex.end()) nodeGraphIdx = git->second;

            LNode* root = (nodeGraphIdx >= 0 && nodeGraphIdx < (int)n) ? bal.roots[nodeGraphIdx] : nullptr;
            if (!root) continue;

            // Find root index
            int rootIdx = -1;
            for (size_t j = 0; j < n; ++j) {
                if (bal.roots[j] == root) {
                    rootIdx = (int)j;
                    break;
                }
            }
            if (rootIdx < 0) continue;

            bal.blockSize[rootIdx] += node->height();

            // Check if this block contains only dummy nodes
            if (!isDummyNode(node)) {
                bal.od[rootIdx] = false;
            }
        }
    }

    // blockY maps block root index -> Y coordinate
    std::map<int, double> blockY;
    std::set<int> placed;

    const double nodeSpacingY = 10.0;

    // Recursive block placement function
    std::function<double(int)> placeBlockRec = [&](int rootIdx) -> double {
        if (!bal.roots[rootIdx]) return 0.0;
        if (placed.count(rootIdx)) return blockY[rootIdx];
        placed.insert(rootIdx);

        LNode* rootNode = bal.roots[rootIdx];
        bool initialized = false;
        double myBlockY = 0.0;

        fprintf(stderr, "[COMPACT] placeBlockRec rootIdx=%d root=%s\n",
                rootIdx, rootNode->elkNode()->id().c_str());

        // Traverse the block ring starting from root
        LNode* curr = rootNode;
        int safety = 0;
        do {
            if (++safety > 100000) break;

            auto currPosIt = nodePosition.find(curr);
            if (currPosIt == nodePosition.end()) {
                curr = findAlignedNode(curr, bal.align, graphNodeIndex);
                if (!curr) break;
                continue;
            }
            int currL = currPosIt->second.first;

            // Find same-layer neighbor that must be placed before curr
            // For VDirection.DOWN: neighbor is above (idx-1)
            // For VDirection.UP: neighbor is below (idx+1)
            bool topDown = (bal.vdir == bk::VDirection::DOWN);
            LNode* neighbor = nullptr;
            int neighborIdx = currPosIt->second.second;
            if (topDown) {
                if (neighborIdx > 0) neighbor = layerNodes[currL][neighborIdx - 1];
            } else {
                if (neighborIdx < (int)layerNodes[currL].size() - 1) {
                    neighbor = layerNodes[currL][neighborIdx + 1];
                }
            }

            if (neighbor) {
                // Find neighbor's index in graph nodes
                int neighborGraphIdx = -1;
                auto ngit = graphNodeIndex.find(neighbor);
                if (ngit != graphNodeIndex.end()) neighborGraphIdx = ngit->second;

                if (neighborGraphIdx >= 0 && neighborGraphIdx < (int)n) {
                    LNode* neighborRoot = bal.roots[neighborGraphIdx];
                    if (neighborRoot) {
                        // Find neighbor root index
                        int neighborRootIdx = -1;
                        for (size_t j = 0; j < n; ++j) {
                            if (bal.roots[j] == neighborRoot) {
                                neighborRootIdx = (int)j;
                                break;
                            }
                        }

                        if (neighborRootIdx >= 0 && neighborRootIdx != rootIdx) {
                            // Recursively place neighbor's block first
                            double neighborBlockY = placeBlockRec(neighborRootIdx);

                            // Get neighbor's inner shift
                            int neighborShiftIdx = neighborGraphIdx;
                            double neighborInnerShift = (neighborShiftIdx >= 0 && neighborShiftIdx < (int)n)
                                ? bal.innerShift[neighborShiftIdx] : 0.0;

                            // Get curr's inner shift
                            int currGraphIdx = -1;
                            auto cgit = graphNodeIndex.find(curr);
                            if (cgit != graphNodeIndex.end()) currGraphIdx = cgit->second;
                            int currShiftIdx = currGraphIdx;
                            double currInnerShift = (currShiftIdx >= 0 && currShiftIdx < (int)n)
                                ? bal.innerShift[currShiftIdx] : 0.0;

                            double sep = (isDummyNode(curr) || isDummyNode(neighbor)) ? 0.0 : nodeSpacingY;

                            double req;
                            if (topDown) {
                                // neighbor is above curr:
                                // blockY[neighborRootIdx] + neighborInnerShift + neighborHeight + sep
                                //   <= blockY[rootIdx] + currInnerShift
                                req = neighborBlockY + neighborInnerShift + neighbor->height() + sep - currInnerShift;
                            } else {
                                // neighbor is below curr:
                                // blockY[rootIdx] + currInnerShift + currHeight + sep
                                //   <= blockY[neighborRootIdx] + neighborInnerShift
                                req = neighborBlockY + neighborInnerShift
                                    - curr->height() - sep - currInnerShift;
                            }

                            if (!initialized) {
                                myBlockY = req;
                                initialized = true;
                            } else {
                                if (topDown) {
                                    myBlockY = std::max(myBlockY, req);
                                } else {
                                    myBlockY = std::min(myBlockY, req);
                                }
                            }
                        }
                    }
                }
            }

            // Move to next node in block ring
            curr = findAlignedNode(curr, bal.align, graphNodeIndex);
            if (!curr || curr == rootNode) break;

        } while (true);

        // If no constraints found, default to 0
        if (!initialized) {
            myBlockY = 0.0;
        }
        blockY[rootIdx] = myBlockY;
        fprintf(stderr, "[COMPACT] placeBlockRec rootIdx=%d returning blockY=%.1f\n", rootIdx, myBlockY);
        return myBlockY;
    };

    // Process all blocks: trigger placement for each block root
    // Process layers in direction order
    std::vector<int> layerOrder;
    for (int L = 0; L < layerCount; ++L) layerOrder.push_back(L);
    if (bal.hdir == bk::HDirection::LEFT) {
        std::reverse(layerOrder.begin(), layerOrder.end());
    }

    for (int L : layerOrder) {
        bool topDown = (bal.vdir == bk::VDirection::DOWN);
        auto& nodes = layerNodes[L];
        if (!topDown) {
            // Reverse for UP direction
            std::vector<LNode*> reversedNodes = nodes;
            std::reverse(reversedNodes.begin(), reversedNodes.end());
            for (LNode* node : reversedNodes) {
                int nodeGraphIdx = -1;
                auto git = graphNodeIndex.find(node);
                if (git != graphNodeIndex.end()) nodeGraphIdx = git->second;
                if (nodeGraphIdx < 0) continue;

                LNode* root = bal.roots[nodeGraphIdx];
                if (!root) continue;

                int rootIdx = -1;
                for (size_t j = 0; j < n; ++j) {
                    if (bal.roots[j] == root) {
                        rootIdx = (int)j;
                        break;
                    }
                }
                if (rootIdx >= 0 && bal.roots[rootIdx] == node) {
                    placeBlockRec(rootIdx);
                }
            }
        } else {
            for (LNode* node : nodes) {
                int nodeGraphIdx = -1;
                auto git = graphNodeIndex.find(node);
                if (git != graphNodeIndex.end()) nodeGraphIdx = git->second;
                if (nodeGraphIdx < 0) continue;

                LNode* root = bal.roots[nodeGraphIdx];
                if (!root) continue;

                int rootIdx = -1;
                for (size_t j = 0; j < n; ++j) {
                    if (bal.roots[j] == root) {
                        rootIdx = (int)j;
                        break;
                    }
                }
                if (rootIdx >= 0 && bal.roots[rootIdx] == node) {
                    placeBlockRec(rootIdx);
                }
            }
        }
    }

    // Assign final Y = blockY[root] + innerShift, normalize to >= 0
    double minY = std::numeric_limits<double>::infinity();
    for (int L = 0; L < layerCount; ++L) {
        for (LNode* node : layerNodes[L]) {
            int nodeGraphIdx = -1;
            auto git = graphNodeIndex.find(node);
            if (git != graphNodeIndex.end()) nodeGraphIdx = git->second;
            if (nodeGraphIdx < 0 || nodeGraphIdx >= (int)n) continue;

            LNode* root = bal.roots[nodeGraphIdx];
            if (!root) continue;

            // Find root index
            int rootIdx = -1;
            for (size_t j = 0; j < n; ++j) {
                if (bal.roots[j] == root) {
                    rootIdx = (int)j;
                    break;
                }
            }
            if (rootIdx < 0) continue;

            double baseY = blockY.count(rootIdx) ? blockY[rootIdx] : 0.0;
            bal.y[nodeGraphIdx] = baseY + bal.innerShift[nodeGraphIdx];
            minY = std::min(minY, bal.y[nodeGraphIdx]);
        }
    }

    // Normalize: shift all Y so minY >= 0
    if (minY < 0) {
        for (size_t i = 0; i < n; ++i) {
            bal.y[i] -= minY;
        }
    }

    // Compute layout size (height)
    double maxY = -std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < n; ++i) {
        maxY = std::max(maxY, bal.y[i]);
    }
    bal.layoutSize_ = (maxY > minY) ? (maxY - minY) : 0.0;
}

// Calculate layout size (height) for comparison
double calculateLayoutSize(BKLayout& bal, LGraph* lgraph) {
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();

    for (int L = 0; L < (int)lgraph->layers().size(); ++L) {
        for (LNode* node : lgraph->layers()[L]->nodes()) {
            auto it = lgraph->nodes().begin();
            // This is simplified - need proper node indexing
        }
    }
    return bal.layoutSize_;
}

// Create balanced layout: median Y of 4 passes
BKLayout createBalancedLayout(std::vector<BKLayout*>& layouts, LGraph* lgraph) {
    if (layouts.empty()) return BKLayout(0, VDirection::DOWN, HDirection::RIGHT);

    int nodeCount = (int)layouts[0]->roots.size();
    BKLayout balanced(nodeCount, VDirection::DOWN, HDirection::RIGHT);

    // Compute minY/maxY for each layout pass
    std::vector<double> width(layouts.size());
    std::vector<double> minY(layouts.size()), maxY(layouts.size());

    for (size_t i = 0; i < layouts.size(); ++i) {
        minY[i] = std::numeric_limits<double>::infinity();
        maxY[i] = -std::numeric_limits<double>::infinity();
        for (int idx = 0; idx < nodeCount; ++idx) {
            if (layouts[i]->y[idx] < minY[i]) minY[i] = layouts[i]->y[idx];
            if (layouts[i]->y[idx] > maxY[i]) maxY[i] = layouts[i]->y[idx];
        }
        width[i] = maxY[i] - minY[i];
    }

    // Find layout with minimum width (height span)
    int minWidthLayout = 0;
    for (size_t i = 1; i < layouts.size(); ++i) {
        if (width[i] < width[minWidthLayout]) minWidthLayout = (int)i;
    }

    // Calculate shifts for alignment: DOWN aligns tops, UP aligns bottoms
    std::vector<double> shift(layouts.size());
    for (size_t i = 0; i < layouts.size(); ++i) {
        if (layouts[i]->vdir == VDirection::DOWN) {
            shift[i] = minY[minWidthLayout] - minY[i];  // align tops
        } else {
            shift[i] = maxY[minWidthLayout] - maxY[i];  // align bottoms
        }
    }

    // For each node, take median of 4 shifted Y values
    std::vector<double> calculatedYs(layouts.size());
    for (int idx = 0; idx < nodeCount; ++idx) {
        for (size_t i = 0; i < layouts.size(); ++i) {
            calculatedYs[i] = layouts[i]->y[idx] + shift[i];
        }
        std::sort(calculatedYs.begin(), calculatedYs.end());
        balanced.y[idx] = (calculatedYs[1] + calculatedYs[2]) / 2.0;
        balanced.innerShift[idx] = 0.0;  // Inner shifts absorbed into Y
    }

    return balanced;
}

} // namespace bk

void brandesKoepfPlace(LGraph* lgraph, const LayoutOptions& options) {
    if (!lgraph) return;

    // Get node count
    int nodeCount = (int)lgraph->nodes().size();
    if (nodeCount == 0) return;

    // Build neighborhood information (left/right neighbors for each node)
    bk::NeighborhoodInfo ni;
    ni.nodeCount = nodeCount;

    // Build node index map and ElkNode->LNode lookup
    int idx = 0;
    std::map<ElkNode*, LNode*> elkToLNode;
    for (const auto& nodePtr : lgraph->nodes()) {
        LNode* node = nodePtr.get();
        ni.nodeIndex[node] = idx++;
        if (node->elkNode()) {
            elkToLNode[node->elkNode()] = node;
        }
    }

    // Helper lambda to get target LNode from edge
    auto getTargetLNode = [&](ElkEdge* edge) -> LNode* {
        if (!edge) return nullptr;
        ElkConnectableShape* tgt = edge->target();
        if (!tgt) return nullptr;
        ElkNode* elkNode = dynamic_cast<ElkNode*>(tgt);
        if (!elkNode) return nullptr;
        auto it = elkToLNode.find(elkNode);
        if (it != elkToLNode.end()) return it->second;
        return nullptr;
    };

    // Helper lambda to get source LNode from edge
    auto getSourceLNode = [&](ElkEdge* edge) -> LNode* {
        if (!edge) return nullptr;
        ElkConnectableShape* src = edge->source();
        if (!src) return nullptr;
        ElkNode* elkNode = dynamic_cast<ElkNode*>(src);
        if (!elkNode) return nullptr;
        auto it = elkToLNode.find(elkNode);
        if (it != elkToLNode.end()) return it->second;
        return nullptr;
    };

    // For each node, collect left and right neighbors
    for (const auto& nodePtr : lgraph->nodes()) {
        LNode* node = nodePtr.get();

        // Check all ports and their edges
        for (auto* port : node->ports()) {
            ElkPort* elkPort = port->elkPort();
            if (!elkPort) continue;

            // Right neighbors = successors (edges going right/to later layers)
            for (auto* edge : elkPort->outgoingEdges()) {
                LNode* target = getTargetLNode(edge);
                if (target && ni.nodeIndex.count(target)) {
                    ni.rightNeighbors[node].push_back({target, edge});
                    ni.leftNeighbors[target].push_back({node, edge});
                }
            }

            // Left neighbors = predecessors (edges coming from left/from earlier layers)
            for (auto* edge : elkPort->incomingEdges()) {
                LNode* source = getSourceLNode(edge);
                if (source && ni.nodeIndex.count(source)) {
                    ni.leftNeighbors[node].push_back({source, edge});
                    ni.rightNeighbors[source].push_back({node, edge});
                }
            }
        }
    }

    // Mark type-1 conflicts
    std::set<ElkEdge*> markedEdges;
    bk::markConflicts(lgraph, markedEdges, ni);
    fprintf(stderr, "[BK] marked conflicts: %d\n", (int)markedEdges.size());

    // Create 4 layout passes: RIGHTDOWN, RIGHTUP, LEFTDOWN, LEFTUP
    std::vector<bk::BKLayout*> layouts;
    layouts.push_back(new bk::BKLayout(nodeCount, bk::VDirection::DOWN, bk::HDirection::RIGHT));
    layouts.push_back(new bk::BKLayout(nodeCount, bk::VDirection::UP, bk::HDirection::RIGHT));
    layouts.push_back(new bk::BKLayout(nodeCount, bk::VDirection::DOWN, bk::HDirection::LEFT));
    layouts.push_back(new bk::BKLayout(nodeCount, bk::VDirection::UP, bk::HDirection::LEFT));

    // Run vertical alignment for each pass
    for (size_t i = 0; i < layouts.size(); ++i) {
        bk::verticalAlignment(*layouts[i], lgraph, markedEdges, ni);
        // Debug: count how many nodes are aligned (not their own root)
        int alignedCount = 0;
        int totalNodes = (int)layouts[i]->align.size();
        for (size_t j = 0; j < layouts[i]->align.size(); ++j) {
            if (layouts[i]->align[j] != lgraph->nodes()[j].get()) {
                alignedCount++;
            }
        }
        fprintf(stderr, "[BK] pass %zu after verticalAlignment: aligned=%d/%d\n",
                i, alignedCount, totalNodes);

        // Also check how many distinct roots there are
        std::map<LNode*, int> rootCount;
        for (size_t j = 0; j < layouts[i]->roots.size(); ++j) {
            if (layouts[i]->roots[j]) {
                rootCount[layouts[i]->roots[j]]++;
            }
        }
        fprintf(stderr, "[BK] pass %zu distinct roots=%d\n", i, (int)rootCount.size());
    }

    // Run inside block shift for each pass
    for (size_t i = 0; i < layouts.size(); ++i) {
        bk::insideBlockShift(*layouts[i], lgraph);
        // Debug: innerShift stats
        double minIS = std::numeric_limits<double>::infinity();
        double maxIS = -std::numeric_limits<double>::infinity();
        int cntIS = 0;
        for (size_t j = 0; j < layouts[i]->innerShift.size(); ++j) {
            if (layouts[i]->roots[j]) {
                double is = layouts[i]->innerShift[j];
                minIS = std::min(minIS, is);
                maxIS = std::max(maxIS, is);
                cntIS++;
            }
        }
        fprintf(stderr, "[BK] pass %zu after insideBlockShift: innerShift [%.1f, %.1f] cnt=%d\n",
                i, minIS, maxIS, cntIS);
    }

    // Run horizontal compaction for each pass
    for (size_t i = 0; i < layouts.size(); ++i) {
        bk::horizontalCompaction(*layouts[i], lgraph);
        // Debug: print blockY and innerShift statistics
        double minBY = std::numeric_limits<double>::infinity();
        double maxBY = -std::numeric_limits<double>::infinity();
        int cntBY = 0;
        double minIS = std::numeric_limits<double>::infinity();
        double maxIS = -std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < layouts[i]->y.size(); ++j) {
            if (layouts[i]->roots[j]) {
                double y = layouts[i]->y[j];
                double is = layouts[i]->innerShift[j];
                minBY = std::min(minBY, y);
                maxBY = std::max(maxBY, y);
                minIS = std::min(minIS, is);
                maxIS = std::max(maxIS, is);
                cntBY++;
            }
        }
        fprintf(stderr, "[BK] pass %zu after horizontalCompaction: Y [%.1f, %.1f] innerShift [%.1f, %.1f]\n",
                i, minBY, maxBY, minIS, maxIS);
    }

    // Choose layout: use balanced (median) or best single layout
    bk::BKLayout* chosenLayout = layouts[0];

    // If balanced layout requested, compute median of 4
    bool produceBalancedLayout = true;  // Use balanced layout like Java ELK
    if (produceBalancedLayout) {
        chosenLayout = new bk::BKLayout(bk::createBalancedLayout(layouts, lgraph));
    } else {
        // Choose layout with minimum height
        double bestSize = layouts[0]->layoutSize_;
        for (bk::BKLayout* bal : layouts) {
            if (bal->layoutSize_ < bestSize) {
                bestSize = bal->layoutSize_;
                chosenLayout = bal;
            }
        }
    }

    // Apply chosen layout Y coordinates to LGraph nodes
    // Write bal.y[i] to LNode at index i in lgraph->nodes()
    int lnodeIdx = 0;
    for (auto& nodePtr : lgraph->nodes()) {
        if (lnodeIdx < (int)chosenLayout->y.size()) {
            nodePtr->setY(chosenLayout->y[lnodeIdx]);
        }
        ++lnodeIdx;
    }

    // Cleanup
    for (bk::BKLayout* bal : layouts) {
        delete bal;
    }
    if (produceBalancedLayout) {
        delete chosenLayout;
    }

    (void)options;
}

// ============================================================
// P5: Orthogonal Edge Routing
// Matches: org.eclipse.elk.alg.layered.p5edges.orthogonal.OrthogonalRoutingGenerator
// ============================================================

void orthogonalRoute(ElkGraph* graph, const LayoutOptions& options) {
    if (!graph) return;
    // Placeholder - orthogonal routing is currently implemented inline
    // in applyElkLayout via slot assignment and hyperedge grouping
    (void)graph;
    (void)options;
}

// ============================================================
// Main Entry Point: elk::layout()
// ============================================================

void layout(ElkGraph* graph) {
    if (!graph) return;

    // Build LGraph from ElkGraph (internal layered representation)
    auto* lgraph = new LGraph(graph);
    lgraph->build();

    // P2: Layer assignment using longest path
    longestPathLayering(lgraph);

    // P4: Brandes-Koepf node placement
    LayoutOptions options;
    brandesKoepfPlace(lgraph, options);

    // Write Y coordinates back to ElkNode objects via LNodes
    for (auto& nodePtr : lgraph->nodes()) {
        LNode* lnode = nodePtr.get();
        ElkNode* elkNode = lnode->elkNode();
        if (elkNode) {
            elkNode->setY(lnode->y());
        }
    }

    // Note: P3 (node ordering) and P5 (edge routing) are
    // handled inline in applyElkLayout
}
} // namespace elk
