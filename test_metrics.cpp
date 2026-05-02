#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <map>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <future>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include "circuitgraph.h"
#include "netlistparser.h"

struct LayoutMetrics {
    int nodeCount = 0;
    int edgeCount = 0;

    // Overlap metrics
    int instanceOverlapCount = 0;

    // Edge-Instance collision metrics
    int edgeInstanceCollisionPairs = 0;   // edge-node collision pairs
    int edgesWithInstanceCollision = 0;   // unique edges that collide with any instance

    // Edge-Edge crossings
    int edgeEdgeCrossings = 0;

    // Total edge length
    double totalEdgeLength = 0.0;

    // Average edge waypoints
    double avgWaypoints = 0.0;

    // Topological (straight-line) crossings – measures placement quality only
    int topologicalCrossings = 0;

    // Derived KPIs
    double edgeCollisionRate = 0.0;       // edgesWithInstanceCollision / edgeCount
    double crossingsPerEdge = 0.0;        // edgeEdgeCrossings / edgeCount
    double edgeClarity = 0.0;             // 1 - edgeCollisionRate

    // Routing geometry (H/V segment ratio)
    double hSegLength = 0.0;       // total horizontal segment length
    double vSegLength = 0.0;       // total vertical segment length
    double hvRatio    = 0.0;       // H/(H+V): 0=all-vertical  1=all-horizontal

    // Routing efficiency (detour ratio)
    double avgDetourRatio = 0.0;   // avg (actual - manhattan) / manhattan per routed edge

    // Routing congestion (grid cell = 20 units)
    int    maxCongestion      = 0;   // max edges through any single cell
    double avgCongestion      = 0.0; // avg over occupied cells
    int    congestionHotspots = 0;   // cells with >= 4 edges

    // Distribution metrics (tail quality)
    double detourP50 = 0.0;
    double detourP90 = 0.0;
    double detourP95 = 0.0;
    double detourMax = 0.0;

    double waypointsP90 = 0.0;
    double waypointsP95 = 0.0;
    double waypointsMax = 0.0;

    // Size-normalized metrics for cross-design comparison
    double crossingsPer1kEdges = 0.0;
    double collisionPairsPer1kEdges = 0.0;
    double topoCrossingsPer1kEdges = 0.0;
};

double percentile(std::vector<double> values, double p) {
    if (values.empty()) {
        return 0.0;
    }
    if (p <= 0.0) {
        return *std::min_element(values.begin(), values.end());
    }
    if (p >= 100.0) {
        return *std::max_element(values.begin(), values.end());
    }
    std::sort(values.begin(), values.end());
    double rank = (p / 100.0) * (values.size() - 1);
    size_t lo = static_cast<size_t>(std::floor(rank));
    size_t hi = static_cast<size_t>(std::ceil(rank));
    if (lo == hi) {
        return values[lo];
    }
    double t = rank - lo;
    return values[lo] * (1.0 - t) + values[hi] * t;
}

// Check if point P lies inside or near rectangle (with margin)
bool pointInRect(double px, double py, double rx, double ry, double rw, double rh, double margin = 4.0) {
    return px >= rx - margin && px <= rx + rw + margin &&
           py >= ry - margin && py <= ry + rh + margin;
}

// Check if line segment (x1,y1)-(x2,y2) intersects rectangle
bool lineSegmentIntersectsRect(double x1, double y1, double x2, double y2,
                                double rx, double ry, double rw, double rh, double margin = 4.0) {
    // Expand rectangle by margin
    rx -= margin;
    ry -= margin;
    rw += 2 * margin;
    rh += 2 * margin;
    
    // Check if either endpoint is inside
    if (pointInRect(x1, y1, rx, ry, rw, rh, 0)) return true;
    if (pointInRect(x2, y2, rx, ry, rw, rh, 0)) return true;
    
    // Check if segment intersects rectangle edges
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    if (std::abs(dx) < 0.001) {
        // Vertical line
        double x = x1;
        if (x >= rx && x <= rx + rw) {
            double minY = std::min(y1, y2);
            double maxY = std::max(y1, y2);
            return !(maxY < ry || minY > ry + rh);
        }
    } else if (std::abs(dy) < 0.001) {
        // Horizontal line
        double y = y1;
        if (y >= ry && y <= ry + rh) {
            double minX = std::min(x1, x2);
            double maxX = std::max(x1, x2);
            return !(maxX < rx || minX > rx + rw);
        }
    }
    
    return false;
}

// Check if two line segments intersect (strict interior only, excludes T-junctions at endpoints).
// This matches the predicate used in test_metrics_elkjs.cpp for apples-to-apples comparison.
bool segmentsIntersect(double x1, double y1, double x2, double y2,
                       double x3, double y3, double x4, double y4) {
    auto cross2d = [](double ux, double uy, double vx, double vy) {
        return ux * vy - uy * vx;
    };
    double d1x = x2 - x1, d1y = y2 - y1;
    double d2x = x4 - x3, d2y = y4 - y3;
    double cross = cross2d(d1x, d1y, d2x, d2y);
    if (std::abs(cross) < 1e-9) return false;
    double t = cross2d(x3 - x1, y3 - y1, d2x, d2y) / cross;
    double u = cross2d(x3 - x1, y3 - y1, d1x, d1y) / cross;
    return t > 0.001 && t < 0.999 && u > 0.001 && u < 0.999;
}

LayoutMetrics computeMetrics(CircuitGraph& graph) {
    LayoutMetrics m;
    m.nodeCount = graph.nodes.size();
    m.edgeCount = graph.edges.size();
    
    // For large graphs (>2000 nodes), O(N²) overlap and O(E×N) collision checks are too slow.
    const bool skipQuadraticChecks = (graph.nodes.size() > 2000 || graph.edges.size() > 5000);

    // 1. Check instance overlaps
    if (!skipQuadraticChecks) {
        for (size_t i = 0; i < graph.nodes.size(); ++i) {
            CNode* n1 = graph.nodes[i];
            if (n1->data.type != CircuitNodeType::ModuleInstance) continue;
            
            for (size_t j = i + 1; j < graph.nodes.size(); ++j) {
                CNode* n2 = graph.nodes[j];
                if (n2->data.type != CircuitNodeType::ModuleInstance) continue;
                
                bool overlap = !(n1->x + n1->width <= n2->x || 
                                 n2->x + n2->width <= n1->x || 
                                 n1->y + n1->height <= n2->y || 
                                 n2->y + n2->height <= n1->y);
                if (overlap) m.instanceOverlapCount++;
            }
        }
    } else {
        m.instanceOverlapCount = -1;
    }
    
    // 2. Check edge-instance collisions
    if (!skipQuadraticChecks) {
        for (const auto* edge : graph.edges) {
            if (edge->points.size() < 2) continue;

            bool edgeCollides = false;
            for (const auto* node : graph.nodes) {
                if (node->data.type != CircuitNodeType::ModuleInstance) continue;
                if (node == edge->source || node == edge->target) continue;

                for (size_t i = 0; i + 1 < edge->points.size(); ++i) {
                    const auto& p1 = edge->points[i];
                    const auto& p2 = edge->points[i + 1];

                    if (lineSegmentIntersectsRect(p1.x(), p1.y(), p2.x(), p2.y(),
                                                  node->x, node->y, node->width, node->height, 2.0)) {
                        m.edgeInstanceCollisionPairs++;
                        edgeCollides = true;
                        break;
                    }
                }
            }
            if (edgeCollides) {
                m.edgesWithInstanceCollision++;
            }
        }
    } else {
        m.edgeInstanceCollisionPairs  = -1;
        m.edgesWithInstanceCollision  = -1;
    }
    
    // 3. Check edge-edge crossings
    // Group edges by (source, sourcePort) to form hyper-edges (fan-out groups share a virtual net).
    // For large graphs (>5000 edges), skip expensive O(N^2) geometric crossing checks.
    const bool skipExpensiveCrossings = skipQuadraticChecks;
    if (skipExpensiveCrossings) {
        m.edgeEdgeCrossings    = -1;  // N/A for large graphs
        m.topologicalCrossings = -1;
    }

    // Crossings are counted between hyper-edge pairs (not individual edge pairs), matching Java ELK.
    // For each hyper-edge, its "wire" is the union of all segments across all member edges.
    // Two hyper-edges cross if ANY segment of one crosses ANY segment of the other,
    // but same-group edges (same hyper-edge) are not counted against each other.
    std::map<std::pair<CNode*, std::string>, std::vector<size_t>> hyperEdgeGroups;
    if (!skipExpensiveCrossings) {
        for (size_t i = 0; i < graph.edges.size(); ++i) {
            const auto* edge = graph.edges[i];
            hyperEdgeGroups[{edge->source, edge->sourcePort}].push_back(i);
        }

        // Collect one list of segments per hyper-edge group.
        struct GroupData {
            std::vector<QPointF> segs;
            float minX, maxX, minY, maxY; // bounding box for fast rejection
        };
        std::vector<GroupData> groupSegs;
        groupSegs.reserve(hyperEdgeGroups.size());
        for (auto& kv : hyperEdgeGroups) {
            GroupData gd;
            gd.minX = gd.minY =  1e30f;
            gd.maxX = gd.maxY = -1e30f;
            for (size_t idx : kv.second) {
                const auto* edge = graph.edges[idx];
                if (edge->points.size() < 2) continue;
                for (size_t i = 0; i + 1 < edge->points.size(); ++i) {
                    gd.segs.push_back(edge->points[i]);
                    gd.segs.push_back(edge->points[i + 1]);
                }
                for (const auto& p : edge->points) {
                    float fx = (float)p.x(), fy = (float)p.y();
                    if (fx < gd.minX) gd.minX = fx;
                    if (fx > gd.maxX) gd.maxX = fx;
                    if (fy < gd.minY) gd.minY = fy;
                    if (fy > gd.maxY) gd.maxY = fy;
                }
            }
            groupSegs.push_back(std::move(gd));
        }

        // Parallel O(N^2) crossing check — partition outer loop across threads.
        const int numThreads = std::max(1u, std::thread::hardware_concurrency());
        std::vector<std::future<int>> futures;
        futures.reserve(numThreads);
        const size_t G = groupSegs.size();
        for (int t = 0; t < numThreads; ++t) {
            futures.push_back(std::async(std::launch::async, [&, t]() {
                int localCrossings = 0;
                for (size_t i = (size_t)t; i < G; i += (size_t)numThreads) {
                    const auto& g1 = groupSegs[i];
                    if (g1.segs.size() < 4) continue;
                    for (size_t j = i + 1; j < G; ++j) {
                        const auto& g2 = groupSegs[j];
                        if (g2.segs.size() < 4) continue;
                        if (g1.maxX < g2.minX || g2.maxX < g1.minX ||
                            g1.maxY < g2.minY || g2.maxY < g1.minY) continue;
                        bool crossed = false;
                        const auto& segs1 = g1.segs;
                        const auto& segs2 = g2.segs;
                        for (size_t si = 0; si + 1 < segs1.size() && !crossed; si += 2) {
                            for (size_t sj = 0; sj + 1 < segs2.size() && !crossed; sj += 2) {
                                if (segmentsIntersect(segs1[si].x(),   segs1[si].y(),
                                                      segs1[si+1].x(), segs1[si+1].y(),
                                                      segs2[sj].x(),   segs2[sj].y(),
                                                      segs2[sj+1].x(), segs2[sj+1].y())) {
                                    localCrossings++;
                                    crossed = true;
                                }
                            }
                        }
                    }
                }
                return localCrossings;
            }));
        }
        for (auto& f : futures) m.edgeEdgeCrossings += f.get();
    }
    
    // 4. Compute total edge length, avg waypoints, H/V ratio, detour ratio, congestion
    // Congestion is hyper-edge aware: fan-out edges from the same source port share V segments
    // (they route to the same gX), so their overlapping V segments count as one in the congestion grid.
    int totalWaypoints = 0;
    // For large graphs the layout can be extremely wide (thousands of px per row), so a fixed
    // cellSize=20 can produce tens-of-millions of grid insertions.  Adaptively scale the cell
    // size so that the grid stays manageable (target ≤ 1000 cells along the widest dimension).
    // For small graphs, keep the original 20px resolution.
    const double cellSize = skipQuadraticChecks ? 200.0 : 20.0;
    const int    congThresh  = 4;
    // Use flat int64 key = (cx<<20)|cy for O(1) unordered_map ops vs O(logN) std::map.
    auto cellKey = [](int cx, int cy) -> int64_t { return ((int64_t)(cx + 100000) << 20) | (uint32_t)(cy + 100000); };
    std::unordered_map<int64_t, int> congestionGrid;
    double detourRatioSum  = 0.0;
    int    routedEdgeCount = 0;
    std::vector<double> detourSamples;
    std::vector<double> waypointSamples;

    // Track which (cell, edge-group) pairs have already been counted for V segment congestion.
    // Group = (source, sourcePort) → hyper-edge id.
    struct PairHash {
        size_t operator()(const std::pair<CNode*, std::string>& p) const {
            size_t h1 = std::hash<void*>{}(p.first);
            size_t h2 = std::hash<std::string>{}(p.second);
            return h1 ^ (h2 * 2654435761ULL);
        }
    };
    std::unordered_map<std::pair<CNode*, std::string>, int, PairHash> hyperGroupId;
    int nextGroupId = 0;
    for (const auto* edge : graph.edges) {
        auto key = std::make_pair(edge->source, edge->sourcePort);
        if (!hyperGroupId.count(key)) hyperGroupId[key] = nextGroupId++;
    }
    fprintf(stderr, "[METRICS-PHASE] hyperGroupId built: %d groups\n", nextGroupId);
    auto dedupKey = [](int gid, int cx, int cy) -> int64_t { return ((int64_t)(gid) << 40) ^ ((int64_t)(cx + 100000) << 20) ^ (uint32_t)(cy + 100000); };
    std::unordered_set<int64_t> vCellCounted;
    std::unordered_set<int64_t> hCellCounted;

    {
        // Estimate total points to check for pathological edge routing
        size_t totalPoints = 0;
        for (const auto* e : graph.edges) totalPoints += e->points.size();
        fprintf(stderr, "[METRICS-PHASE] total edge points: %zu (avg %.1f per edge)\n",
                totalPoints, (double)totalPoints / std::max(1u, (unsigned)graph.edges.size()));
    }

    auto t_cong_start = std::chrono::steady_clock::now();
    for (const auto* edge : graph.edges) {
        if (edge->points.size() < 2) continue;
        const QPointF& src = edge->points.front();
        const QPointF& tgt = edge->points.back();
        double manhattan = std::abs(tgt.x() - src.x()) + std::abs(tgt.y() - src.y());
        double eActual   = 0.0;
        int gid = hyperGroupId[{edge->source, edge->sourcePort}];
        for (size_t i = 0; i + 1 < edge->points.size(); ++i) {
            const auto& p1 = edge->points[i];
            const auto& p2 = edge->points[i + 1];
            double dx  = p2.x() - p1.x();
            double dy  = p2.y() - p1.y();
            double len = std::sqrt(dx * dx + dy * dy);
            m.totalEdgeLength += len;
            eActual           += len;
            if (std::abs(dx) < 0.001) {
                // Vertical segment: deduplicate per hyper-edge group (fan-out edges share V seg).
                m.vSegLength += len;
                int cx = (int)std::floor(p1.x() / cellSize);
                int y0 = (int)std::floor(std::min(p1.y(), p2.y()) / cellSize);
                int y1 = (int)std::floor(std::max(p1.y(), p2.y()) / cellSize);
                for (int cy = y0; cy <= y1; ++cy) {
                    int64_t vk = dedupKey(gid, cx, cy);
                    if (vCellCounted.insert(vk).second)
                        congestionGrid[cellKey(cx, cy)]++;
                }
            } else if (std::abs(dy) < 0.001) {
                // Horizontal segment: deduplicate per hyper-edge group (same stem for fan-out).
                m.hSegLength += len;
                int cy = (int)std::floor(p1.y() / cellSize);
                int x0 = (int)std::floor(std::min(p1.x(), p2.x()) / cellSize);
                int x1 = (int)std::floor(std::max(p1.x(), p2.x()) / cellSize);
                for (int cx = x0; cx <= x1; ++cx) {
                    int64_t hk = dedupKey(gid, cx, cy);
                    if (hCellCounted.insert(hk).second)
                        congestionGrid[cellKey(cx, cy)]++;
                }
            }
        }
        totalWaypoints += std::max(0, (int)edge->points.size() - 2);
        waypointSamples.push_back(std::max(0, (int)edge->points.size() - 2));
        if (manhattan > 0.1) {
            double detour = (eActual - manhattan) / manhattan;
            detourRatioSum += detour;
            detourSamples.push_back(detour);
            ++routedEdgeCount;
        }
    }
    {
        auto t_cong_end = std::chrono::steady_clock::now();
        int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(t_cong_end - t_cong_start).count();
        fprintf(stderr, "[METRICS-PHASE] congestion grid loop: %d ms  vCells=%zu hCells=%zu gridSize=%zu\n",
                ms, vCellCounted.size(), hCellCounted.size(), congestionGrid.size());
    }
    if (m.edgeCount > 0) {
        m.avgWaypoints = static_cast<double>(totalWaypoints) / m.edgeCount;
    }
    double hvTotal = m.hSegLength + m.vSegLength;
    if (hvTotal > 0.001) m.hvRatio = m.hSegLength / hvTotal;
    if (routedEdgeCount > 0) m.avgDetourRatio = detourRatioSum / routedEdgeCount;
    if (!detourSamples.empty()) {
        m.detourP50 = percentile(detourSamples, 50.0);
        m.detourP90 = percentile(detourSamples, 90.0);
        m.detourP95 = percentile(detourSamples, 95.0);
        m.detourMax = percentile(detourSamples, 100.0);
    }
    if (!waypointSamples.empty()) {
        m.waypointsP90 = percentile(waypointSamples, 90.0);
        m.waypointsP95 = percentile(waypointSamples, 95.0);
        m.waypointsMax = percentile(waypointSamples, 100.0);
    }
    if (!congestionGrid.empty()) {
        double totalCong = 0;
        for (const auto& kv : congestionGrid) {
            totalCong      += kv.second;
            m.maxCongestion = std::max(m.maxCongestion, kv.second);
            if (kv.second >= congThresh) ++m.congestionHotspots;
        }
        m.avgCongestion = totalCong / congestionGrid.size();
        // Print top hotspot cells to stderr for debugging.
        std::vector<std::pair<int,int64_t>> cells;
        for (const auto& kv : congestionGrid) if (kv.second >= 7) cells.push_back({kv.second, kv.first});
        std::sort(cells.begin(), cells.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
        for (int ci = 0; ci < std::min((int)cells.size(), 8); ++ci) {
            int64_t k = cells[ci].second;
            int cx = (int)((k >> 20) & 0xFFFFF) - 100000;
            int cy = (int)(k & 0xFFFFF) - 100000;
            fprintf(stderr, "[CONG] cell(%d,%d)=cong%d  world(%.0f,%.0f)\n",
                cx, cy, cells[ci].first, cx * cellSize, cy * cellSize);
        }
    }
    
    // 5. Topological crossings (straight-line src→dst per edge, ignores routing bends)
    if (!skipExpensiveCrossings) {
        int n = (int)graph.edges.size();
        for (int i = 0; i < n; ++i) {
            if (graph.edges[i]->points.size() < 2) continue;
            const auto& A0 = graph.edges[i]->points.front();
            const auto& A1 = graph.edges[i]->points.back();
            for (int j = i + 1; j < n; ++j) {
                if (graph.edges[j]->points.size() < 2) continue;
                const auto& B0 = graph.edges[j]->points.front();
                const auto& B1 = graph.edges[j]->points.back();
                if (segmentsIntersect(A0.x(), A0.y(), A1.x(), A1.y(),
                                      B0.x(), B0.y(), B1.x(), B1.y())) {
                    ++m.topologicalCrossings;
                }
            }
        }
    }

    // 6. Derived metrics
    if (m.edgeCount > 0) {
        m.edgeCollisionRate = static_cast<double>(m.edgesWithInstanceCollision) / m.edgeCount;
        m.edgeClarity = 1.0 - m.edgeCollisionRate;
        m.collisionPairsPer1kEdges = static_cast<double>(m.edgeInstanceCollisionPairs) * 1000.0 / m.edgeCount;
        if (m.edgeEdgeCrossings >= 0) {
            m.crossingsPerEdge    = static_cast<double>(m.edgeEdgeCrossings) * 1.0 / m.edgeCount;
            m.crossingsPer1kEdges = static_cast<double>(m.edgeEdgeCrossings) * 1000.0 / m.edgeCount;
        }
        if (m.topologicalCrossings >= 0) {
            m.topoCrossingsPer1kEdges = static_cast<double>(m.topologicalCrossings) * 1000.0 / m.edgeCount;
        }
    }

    return m;
}

void printMetrics(const std::string& filename, const LayoutMetrics& m) {
    std::cout << "\n" << std::string(70, '=') << "\n";
    std::cout << "Layout Metrics for: " << filename << "\n";
    std::cout << std::string(70, '=') << "\n";
    
    std::cout << "Graph Structure:\n";
    std::cout << "  Nodes: " << m.nodeCount << "\n";
    std::cout << "  Edges: " << m.edgeCount << "\n";
    
    std::cout << "\nOverlap Analysis:\n";
    if (m.instanceOverlapCount < 0)
        std::cout << "  Instance Overlaps: N/A (large graph)\n";
    else
        std::cout << "  Instance Overlaps: " << m.instanceOverlapCount << "\n";
    
    std::cout << "\nConnectivity & Routing Quality:\n";
    if (m.edgeInstanceCollisionPairs < 0) {
        std::cout << "  Edge-Instance Collision Pairs: N/A (large graph)\n";
        std::cout << "  Edges With Instance Collision: N/A (large graph)\n";
    } else {
        std::cout << "  Edge-Instance Collision Pairs: " << m.edgeInstanceCollisionPairs << "\n";
        std::cout << "  Edges With Instance Collision: " << m.edgesWithInstanceCollision;
        if (m.edgeCount > 0) {
            std::cout << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * m.edgeCollisionRate) << "%)";
        }
        std::cout << "\n";
    }

    if (m.edgeEdgeCrossings < 0) {
        std::cout << "  Edge-Edge Crossings: N/A (large graph, >5k edges)\n";
    } else {
        std::cout << "  Edge-Edge Crossings: " << m.edgeEdgeCrossings;
        if (m.edgeCount > 0) {
            std::cout << " (" << std::fixed << std::setprecision(2)
                      << m.crossingsPerEdge << " per edge)";
        }
        std::cout << "\n";
    }
    if (m.topologicalCrossings < 0) {
        std::cout << "  Topological Crossings (placement): N/A (large graph)\n";
    } else {
        std::cout << "  Topological Crossings (placement): " << m.topologicalCrossings
                  << " (straight-line src->dst)\n";
    }
    
    std::cout << "\nEdge Geometry:\n";
    std::cout << "  Total Edge Length: " << std::fixed << std::setprecision(1)
              << m.totalEdgeLength << " units\n";
    std::cout << "  Avg Waypoints per Edge: " << std::setprecision(2)
              << m.avgWaypoints << "\n";
    std::cout << "  Waypoints P90/P95/Max: "
              << std::setprecision(2) << m.waypointsP90 << " / "
              << m.waypointsP95 << " / " << m.waypointsMax << "\n";

    std::cout << "\nRouting Analysis:\n";
    std::cout << "  H/V segment ratio: "
              << std::fixed << std::setprecision(1)
              << (m.hvRatio * 100.0) << "% H / "
              << ((1.0 - m.hvRatio) * 100.0) << "% V"
              << "  (H=" << std::setprecision(0) << m.hSegLength
              << "  V=" << m.vSegLength << " units)\n";
    std::cout << "  Avg Detour Ratio: "
              << std::fixed << std::setprecision(3) << m.avgDetourRatio
              << "  (0=straight, >0=overhead)\n";
    std::cout << "  Detour P50/P90/P95/Max: "
              << std::fixed << std::setprecision(3)
              << m.detourP50 << " / " << m.detourP90 << " / "
              << m.detourP95 << " / " << m.detourMax << "\n";
    std::cout << "  Congestion (cell=20u): max=" << m.maxCongestion
              << "  avg=" << std::fixed << std::setprecision(2) << m.avgCongestion
              << "  hotspots(>=4)=" << m.congestionHotspots << "\n";

    std::cout << "\nNormalized KPIs (per 1k edges):\n";
    std::cout << "  Crossings/1kE: " << std::fixed << std::setprecision(2)
              << m.crossingsPer1kEdges << "\n";
    std::cout << "  TopoCrossings/1kE: " << m.topoCrossingsPer1kEdges << "\n";
    std::cout << "  CollisionPairs/1kE: " << m.collisionPairsPer1kEdges << "\n";
    
    std::cout << "\nLayout Clarity Score:\n";
    int barWidth = std::max(0, std::min(30, static_cast<int>(m.edgeClarity * 30)));
    std::string bar(barWidth, '#');
    std::string empty(30 - barWidth, '-');
    std::cout << "  Edge Clarity (0-1): " << std::fixed << std::setprecision(3) 
              << m.edgeClarity << " [" << bar << empty << "]\n";
    
    std::cout << std::string(70, '=') << "\n";
}

int main(int argc, char** argv) {
    std::string filename = "1_2_yosys.v";
    if (argc > 1) filename = argv[1];
    
    CircuitGraph graph;
    std::cout << "Parsing: " << filename << "\n";
    Netlist netlist = NetlistParser::parse(filename);
    
    if (netlist.modules.empty()) {
        std::cerr << "Failed to parse modules\n";
        return 1;
    }
    
    std::string topModule = netlist.topModule;
    if (topModule.empty() && !netlist.modules.empty()) {
        topModule = netlist.modules.begin()->first;
    }
    
    std::cout << "Building graph for: " << topModule << "\n";
    graph.buildHierarchical(netlist, topModule);
    
    std::cout << "Applying layout...\n";
    const auto t0 = std::chrono::steady_clock::now();
    graph.applyLayout();
    const auto t1 = std::chrono::steady_clock::now();
    const auto layoutMs = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Layout Runtime: " << layoutMs << " ms\n";
    
    LayoutMetrics metrics = computeMetrics(graph);
    printMetrics(filename, metrics);
    
    // Return non-zero if problems detected
    if (metrics.instanceOverlapCount > 0 || metrics.edgesWithInstanceCollision > 0) {
        return 1;
    }
    return 0;
}
