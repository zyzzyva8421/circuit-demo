// test_metrics_elkjs.cpp
// Runs elkjs layout on the given Verilog and prints the same KPIs as test_metrics.cpp
// Usage: ./test_metrics_elkjs [filename.v]

#include <iostream>
#include <cmath>
#include <vector>
#include <map>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include "circuitgraph.h"
#include "netlistparser.h"

struct LayoutMetrics {
    int nodeCount = 0;
    int edgeCount = 0;
    int instanceOverlapCount = 0;
    int edgeInstanceCollisionPairs = 0;
    int edgesWithInstanceCollision = 0;
    int edgeEdgeCrossings = 0;
    int topologicalCrossings = 0;
    double totalEdgeLength = 0.0;
    double avgWaypoints = 0.0;
    double edgeCollisionRate = 0.0;
    double crossingsPerEdge = 0.0;
    double edgeClarity = 0.0;

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

bool lineSegmentIntersectsRect(double x1, double y1, double x2, double y2,
                                double rx, double ry, double rw, double rh, double margin = 4.0) {
    rx -= margin; ry -= margin; rw += 2*margin; rh += 2*margin;
    // endpoint inside?
    auto inside = [&](double px, double py) {
        return px >= rx && px <= rx+rw && py >= ry && py <= ry+rh;
    };
    if (inside(x1,y1) || inside(x2,y2)) return true;
    // Cohen-Sutherland clip test
    auto code = [&](double px, double py) {
        int c = 0;
        if (px < rx)      c |= 1;
        if (px > rx+rw)   c |= 2;
        if (py < ry)      c |= 4;
        if (py > ry+rh)   c |= 8;
        return c;
    };
    int c1 = code(x1,y1), c2 = code(x2,y2);
    if (c1 & c2) return false; // trivially outside
    if (!(c1 | c2)) return true; // trivially inside
    // parametric intersection with all 4 edges
    auto seg_intersects_hline = [](double y, double ax, double ay, double bx, double by) {
        if (std::abs(ay-by) < 1e-9) return false;
        double t = (y-ay)/(by-ay);
        if (t < 0 || t > 1) return false;
        double ix = ax + t*(bx-ax);
        (void)ix; return true;
    };
    (void)seg_intersects_hline;
    // simplified: just check if bounding boxes overlap for axis-aligned segments
    double mnx = std::min(x1,x2), mxx = std::max(x1,x2);
    double mny = std::min(y1,y2), mxy = std::max(y1,y2);
    return !(mxx < rx || mnx > rx+rw || mxy < ry || mny > ry+rh);
}

bool segmentsIntersect(double ax, double ay, double bx, double by,
                       double cx, double cy, double dx, double dy) {
    auto cross2d = [](double ux, double uy, double vx, double vy) {
        return ux*vy - uy*vx;
    };
    double d1x=bx-ax, d1y=by-ay;
    double d2x=dx-cx, d2y=dy-cy;
    double cross = cross2d(d1x,d1y,d2x,d2y);
    if (std::abs(cross) < 1e-9) return false;
    double t = cross2d(cx-ax,cy-ay,d2x,d2y)/cross;
    double u = cross2d(cx-ax,cy-ay,d1x,d1y)/cross;
    return t > 0.001 && t < 0.999 && u > 0.001 && u < 0.999;
}

LayoutMetrics computeMetrics(const CircuitGraph& graph) {
    LayoutMetrics m;
    m.nodeCount = (int)graph.nodes.size();
    m.edgeCount = (int)graph.edges.size();

    // Instance overlap
    std::vector<CNode*> instances;
    for (auto* n : graph.nodes) {
        if (n->data.type == CircuitNodeType::ModuleInstance ||
            n->data.type == CircuitNodeType::ExpandedInstance) {
            instances.push_back(n);
        }
    }
    for (size_t i = 0; i < instances.size(); ++i) {
        for (size_t j = i+1; j < instances.size(); ++j) {
            auto* a = instances[i]; auto* b = instances[j];
            bool ox = !(a->x+a->width <= b->x || b->x+b->width <= a->x);
            bool oy = !(a->y+a->height <= b->y || b->y+b->height <= a->y);
            if (ox && oy) ++m.instanceOverlapCount;
        }
    }

    // Edge-instance collisions
    for (auto* e : graph.edges) {
        if (e->points.size() < 2) continue;
        bool edgeHit = false;
        for (auto* n : instances) {
            if (n == e->source || n == e->target) continue;
            bool hit = false;
            for (size_t k = 1; k < e->points.size(); ++k) {
                double x1=e->points[k-1].x(), y1=e->points[k-1].y();
                double x2=e->points[k].x(),   y2=e->points[k].y();
                if (lineSegmentIntersectsRect(x1,y1,x2,y2, n->x,n->y,n->width,n->height)) {
                    hit = true; break;
                }
            }
            if (hit) { ++m.edgeInstanceCollisionPairs; edgeHit = true; }
        }
        if (edgeHit) ++m.edgesWithInstanceCollision;
    }

    // Edge-edge crossings: group by (source, sourcePort) hyper-edge, count crossing pairs.
    {
        auto& edges = graph.edges;
        std::map<std::pair<CNode*, std::string>, std::vector<size_t>> hyperEdgeGroups;
        for (size_t i = 0; i < edges.size(); ++i)
            hyperEdgeGroups[{edges[i]->source, edges[i]->sourcePort}].push_back(i);

        std::vector<std::vector<QPointF>> groupSegs;
        for (auto& kv : hyperEdgeGroups) {
            std::vector<QPointF> segs;
            for (size_t idx : kv.second) {
                auto* e = edges[idx];
                if (e->points.size() < 2) continue;
                for (size_t p = 1; p < e->points.size(); ++p) {
                    segs.push_back(e->points[p-1]);
                    segs.push_back(e->points[p]);
                }
            }
            groupSegs.push_back(std::move(segs));
        }

        for (size_t i = 0; i < groupSegs.size(); ++i) {
            auto& s1 = groupSegs[i];
            if (s1.size() < 4) continue;
            for (size_t j = i+1; j < groupSegs.size(); ++j) {
                auto& s2 = groupSegs[j];
                if (s2.size() < 4) continue;
                bool crossed = false;
                for (size_t p = 0; p+1 < s1.size() && !crossed; p += 2)
                    for (size_t q = 0; q+1 < s2.size() && !crossed; q += 2)
                        if (segmentsIntersect(s1[p].x(),s1[p].y(),s1[p+1].x(),s1[p+1].y(),
                                             s2[q].x(),s2[q].y(),s2[q+1].x(),s2[q+1].y())) {
                            ++m.edgeEdgeCrossings; crossed = true;
                        }
            }
        }
    }

    // Topological crossings (straight-line src->dst)
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        if (graph.edges[i]->points.size() < 2) continue;
        const auto& A0 = graph.edges[i]->points.front();
        const auto& A1 = graph.edges[i]->points.back();
        for (size_t j = i+1; j < graph.edges.size(); ++j) {
            if (graph.edges[j]->points.size() < 2) continue;
            const auto& B0 = graph.edges[j]->points.front();
            const auto& B1 = graph.edges[j]->points.back();
            if (segmentsIntersect(A0.x(),A0.y(),A1.x(),A1.y(),
                                  B0.x(),B0.y(),B1.x(),B1.y()))
                ++m.topologicalCrossings;
        }
    }

    // Edge length, waypoints, H/V ratio, detour ratio, congestion
    // Congestion is hyper-edge aware: fan-out edges from same source port share V segments.
    double totalLen = 0.0, totalWP = 0.0;
    const double cellSize    = 20.0;
    const int    congThresh  = 4;
    std::map<std::pair<int,int>, int> congestionGrid;
    double detourRatioSum  = 0.0;
    int    routedEdgeCount = 0;
    std::vector<double> detourSamples;
    std::vector<double> waypointSamples;

    std::map<std::pair<CNode*, std::string>, int> hyperGroupId2;
    int nextGroupId2 = 0;
    for (auto* e : graph.edges) {
        auto key = std::make_pair(e->source, e->sourcePort);
        if (!hyperGroupId2.count(key)) hyperGroupId2[key] = nextGroupId2++;
    }
    std::set<std::pair<int,std::pair<int,int>>> vCellCounted2;

    for (auto* e : graph.edges) {
        if (e->points.size() < 2) continue;
        const QPointF& src = e->points.front();
        const QPointF& tgt = e->points.back();
        double manhattan = std::abs(tgt.x() - src.x()) + std::abs(tgt.y() - src.y());
        double eActual   = 0.0;
        int gid2 = hyperGroupId2[{e->source, e->sourcePort}];
        for (size_t k = 1; k < e->points.size(); ++k) {
            double dx = e->points[k].x()-e->points[k-1].x();
            double dy = e->points[k].y()-e->points[k-1].y();
            double len = std::sqrt(dx*dx+dy*dy);
            totalLen += len;
            eActual  += len;
            if (std::abs(dx) < 0.001) {
                m.vSegLength += len;
                int cx = (int)std::floor(e->points[k-1].x() / cellSize);
                int y0 = (int)std::floor(std::min(e->points[k-1].y(), e->points[k].y()) / cellSize);
                int y1 = (int)std::floor(std::max(e->points[k-1].y(), e->points[k].y()) / cellSize);
                for (int cy = y0; cy <= y1; ++cy) {
                    auto vkey = std::make_pair(gid2, std::make_pair(cx, cy));
                    if (vCellCounted2.insert(vkey).second)
                        congestionGrid[{cx, cy}]++;
                }
            } else if (std::abs(dy) < 0.001) {
                m.hSegLength += len;
                int cy = (int)std::floor(e->points[k-1].y() / cellSize);
                int x0 = (int)std::floor(std::min(e->points[k-1].x(), e->points[k].x()) / cellSize);
                int x1 = (int)std::floor(std::max(e->points[k-1].x(), e->points[k].x()) / cellSize);
                for (int cx = x0; cx <= x1; ++cx)
                    congestionGrid[{cx, cy}]++;
            }
        }
        totalWP += (double)std::max(0,(int)e->points.size()-2);
        waypointSamples.push_back(std::max(0,(int)e->points.size()-2));
        if (manhattan > 0.1) {
            double detour = (eActual - manhattan) / manhattan;
            detourRatioSum += detour;
            detourSamples.push_back(detour);
            ++routedEdgeCount;
        }
    }
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
        std::vector<std::pair<int,std::pair<int,int>>> cells;
        for (const auto& kv : congestionGrid) if (kv.second >= 4) cells.push_back({kv.second, kv.first});
        std::sort(cells.begin(), cells.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
        for (int ci = 0; ci < std::min((int)cells.size(), 8); ++ci)
            fprintf(stderr, "[CONG_REF] cell(%d,%d)=cong%d  world(%.0f,%.0f)\n",
                cells[ci].second.first, cells[ci].second.second, cells[ci].first,
                cells[ci].second.first * cellSize, cells[ci].second.second * cellSize);
    }
    m.totalEdgeLength = totalLen;
    m.avgWaypoints = m.edgeCount > 0 ? totalWP/m.edgeCount : 0.0;
    double hvTotal = m.hSegLength + m.vSegLength;
    if (hvTotal > 0.001) m.hvRatio = m.hSegLength / hvTotal;
    m.edgeCollisionRate = m.edgeCount > 0 ? (double)m.edgesWithInstanceCollision/m.edgeCount : 0.0;
    m.crossingsPerEdge  = m.edgeCount > 0 ? (double)m.edgeEdgeCrossings/m.edgeCount : 0.0;
    m.edgeClarity = 1.0 - m.edgeCollisionRate;
    if (m.edgeCount > 0) {
        m.crossingsPer1kEdges = (double)m.edgeEdgeCrossings * 1000.0 / m.edgeCount;
        m.collisionPairsPer1kEdges = (double)m.edgeInstanceCollisionPairs * 1000.0 / m.edgeCount;
        m.topoCrossingsPer1kEdges = (double)m.topologicalCrossings * 1000.0 / m.edgeCount;
    }
    return m;
}

void printMetrics(const std::string& file, const LayoutMetrics& m) {
    std::cout << std::string(70,'=') << "\n";
    std::cout << "File: " << file << "  [Layout: elkjs]\n";
    std::cout << std::string(70,'=') << "\n";
    std::cout << "Graph Structure:\n";
    std::cout << "  Nodes: " << m.nodeCount << "\n";
    std::cout << "  Edges: " << m.edgeCount << "\n\n";
    std::cout << "Overlap Analysis:\n";
    std::cout << "  Instance Overlaps: " << m.instanceOverlapCount << "\n\n";
    std::cout << "Connectivity & Routing Quality:\n";
    std::cout << "  Edge-Instance Collision Pairs: " << m.edgeInstanceCollisionPairs << "\n";
    std::cout << "  Edges With Instance Collision: " << m.edgesWithInstanceCollision
              << " (" << std::fixed << std::setprecision(1) << m.edgeCollisionRate*100 << "%)\n";
    std::cout << "  Edge-Edge Crossings: " << m.edgeEdgeCrossings
              << " (" << std::setprecision(2) << m.crossingsPerEdge << " per edge)\n";
    std::cout << "  Topological Crossings (placement): " << m.topologicalCrossings
              << " (straight-line src->dst)\n\n";
    std::cout << "Edge Geometry:\n";
    std::cout << "  Total Edge Length: " << std::setprecision(1) << m.totalEdgeLength << " units\n";
    std::cout << "  Avg Waypoints per Edge: " << std::setprecision(2) << m.avgWaypoints << "\n";
    std::cout << "  Waypoints P90/P95/Max: "
              << std::setprecision(2) << m.waypointsP90 << " / "
              << m.waypointsP95 << " / " << m.waypointsMax << "\n";

    std::cout << "\nRouting Analysis:\n";
    std::cout << "  H/V segment ratio: " << std::fixed << std::setprecision(1)
              << (m.hvRatio * 100.0) << "% H / " << ((1.0 - m.hvRatio) * 100.0) << "% V"
              << "  (H=" << std::setprecision(0) << m.hSegLength
              << "  V=" << m.vSegLength << " units)\n";
    std::cout << "  Avg Detour Ratio: " << std::fixed << std::setprecision(3) << m.avgDetourRatio
              << "  (0=straight, >0=overhead)\n";
    std::cout << "  Detour P50/P90/P95/Max: "
              << std::fixed << std::setprecision(3)
              << m.detourP50 << " / " << m.detourP90 << " / "
              << m.detourP95 << " / " << m.detourMax << "\n";
    std::cout << "  Congestion (cell=20u): max=" << m.maxCongestion
              << "  avg=" << std::fixed << std::setprecision(2) << m.avgCongestion
              << "  hotspots(>=4)=" << m.congestionHotspots << "\n\n";
    std::cout << "Normalized KPIs (per 1k edges):\n";
    std::cout << "  Crossings/1kE: " << std::fixed << std::setprecision(2)
              << m.crossingsPer1kEdges << "\n";
    std::cout << "  TopoCrossings/1kE: " << m.topoCrossingsPer1kEdges << "\n";
    std::cout << "  CollisionPairs/1kE: " << m.collisionPairsPer1kEdges << "\n\n";
    std::cout << "Layout Clarity Score:\n";
    std::cout << "  Edge Clarity (0-1): " << std::setprecision(3) << m.edgeClarity << "\n";
    std::cout << std::string(70,'=') << "\n";
}

int main(int argc, char** argv) {
    std::string filename = "1_2_yosys.v";
    if (argc > 1) filename = argv[1];

    QCoreApplication app(argc, argv);  // needed for QProcess inside elkjs call

    CircuitGraph graph;
    std::cout << "Parsing: " << filename << "\n";
    Netlist netlist = NetlistParser::parse(filename);
    if (netlist.modules.empty()) { std::cerr << "Parse failed\n"; return 1; }

    std::string top = netlist.topModule;
    if (top.empty() && !netlist.modules.empty()) top = netlist.modules.begin()->first;

    std::cout << "Building graph for: " << top << "\n";
    graph.buildHierarchical(netlist, top);

    std::cout << "Applying elkjs layout...\n";
    const auto t0 = std::chrono::steady_clock::now();
    graph.applyLayoutViaElkjs();
    const auto t1 = std::chrono::steady_clock::now();
    const auto layoutMs = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    std::cout << "Layout Runtime: " << layoutMs << " ms\n";

    // Debug: count H vs V segments per edge
    int hSegs=0, vSegs=0; double hLen=0, vLen=0;
    std::map<int,int> edgePtCountHistogram;
    for (auto* e : graph.edges) {
        edgePtCountHistogram[e->points.size()]++;
        for (int i=1; i<(int)e->points.size(); ++i) {
            double dx = std::abs(e->points[i].x()-e->points[i-1].x());
            double dy = std::abs(e->points[i].y()-e->points[i-1].y());
            if (dx > dy) { hSegs++; hLen += dx; }
            else { vSegs++; vLen += dy; }
        }
    }
    std::cout << "Segs: H=" << hSegs << "(len=" << (int)hLen << ") V=" << vSegs << "(len=" << (int)vLen << ")\n";
    std::cout << "Edge point count histogram: ";
    for (auto& p : edgePtCountHistogram) std::cout << p.first << "pts:" << p.second << " ";
    std::cout << "\n";

    LayoutMetrics m = computeMetrics(graph);
    printMetrics(filename, m);

    if (m.instanceOverlapCount > 0 || m.edgesWithInstanceCollision > 0) return 1;
    return 0;
}
