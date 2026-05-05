// test_compare_elk.cpp
// Compares C++ ELK layout output with elkjs layout output on the same netlist.
// Usage: ./test_compare_elk [filename.v] [--verbose]
// Output: Side-by-side metrics comparison with diff percentage.

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <map>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QProcess>
#include <QDir>
#include <QFile>
#include "circuitgraph.h"
#include "netlistparser.h"

struct NodeMetrics {
    double x, y, width, height;
};

struct EdgeMetrics {
    std::vector<QPointF> points;
    double length;
    int waypointCount;
};

struct LayoutResult {
    std::string netlist;
    std::unordered_map<std::string, NodeMetrics> nodes;
    std::vector<EdgeMetrics> edges;
    int nodeCount = 0;
    int edgeCount = 0;
    int crossings = 0;
    double totalEdgeLength = 0.0;
    double avgWaypoints = 0.0;
    double hvRatio = 0.0;
    int maxCongestion = 0;
    double avgCongestion = 0.0;
    double avgDetourRatio = 0.0;
    double layoutTimeMs = 0.0;
};

struct CompareResult {
    double nodePositionDiffAvg = 0.0;   // pixels
    double nodePositionDiffMax = 0.0;   // pixels
    int crossingDiff = 0;              // absolute difference
    double edgeLengthDiffPct = 0.0;     // percentage
    double waypointDiffAvg = 0.0;       // absolute difference
    double hvRatioDiff = 0.0;           // absolute difference
    double layoutTimeRatio = 0.0;       // cpp_time / elkjs_time
    bool cppFailed = false;
    bool elkjsFailed = false;
};

// Helper: compute edge crossings
int countEdgeCrossings(const std::vector<EdgeMetrics>& edges) {
    int crossings = 0;
    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            const auto& e1 = edges[i].points;
            const auto& e2 = edges[j].points;
            for (size_t a = 1; a < e1.size(); ++a) {
                for (size_t b = 1; b < e2.size(); ++b) {
                    double x1 = e1[a-1].x(), y1 = e1[a-1].y();
                    double x2 = e1[a].x(), y2 = e1[a].y();
                    double x3 = e2[b-1].x(), y3 = e2[b-1].y();
                    double x4 = e2[b].x(), y4 = e2[b].y();
                    auto cross2d = [](double ux, double uy, double vx, double vy) {
                        return ux * vy - uy * vx;
                    };
                    double d1x = x2 - x1, d1y = y2 - y1;
                    double d2x = x4 - x3, d2y = y4 - y3;
                    double cross = cross2d(d1x, d1y, d2x, d2y);
                    if (std::abs(cross) < 1e-9) continue;
                    double t = cross2d(x3 - x1, y3 - y1, d2x, d2y) / cross;
                    double u = cross2d(x3 - x1, y3 - y1, d1x, d1y) / cross;
                    if (t > 0.001 && t < 0.999 && u > 0.001 && u < 0.999) {
                        ++crossings;
                    }
                }
            }
        }
    }
    return crossings;
}

double manhattanDistance(const QPointF& a, const QPointF& b) {
    return std::abs(a.x() - b.x()) + std::abs(a.y() - b.y());
}

LayoutResult runCppLayout(const std::string& filename) {
    LayoutResult result;
    result.netlist = filename;

    CircuitGraph graph;
    Netlist netlist = NetlistParser::parse(filename);
    if (netlist.modules.empty()) {
        std::cerr << "Parse failed for " << filename << "\n";
        return result;
    }

    std::string top = netlist.topModule.empty() ? netlist.modules.begin()->first : netlist.topModule;
    graph.buildHierarchical(netlist, top);

    auto t0 = std::chrono::steady_clock::now();
    graph.applyLayout();  // C++ ELK
    auto t1 = std::chrono::steady_clock::now();
    result.layoutTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.nodeCount = (int)graph.nodes.size();
    result.edgeCount = (int)graph.edges.size();

    // Collect node metrics
    for (auto* n : graph.nodes) {
        NodeMetrics nm;
        nm.x = n->x; nm.y = n->y;
        nm.width = n->width; nm.height = n->height;
        result.nodes[n->id] = nm;
    }

    // Collect edge metrics
    result.edges.resize(graph.edges.size());
    double totalLen = 0.0;
    int totalWp = 0;
    double hLen = 0.0, vLen = 0.0;
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        auto* e = graph.edges[i];
        result.edges[i].points = e->points;
        result.edges[i].waypointCount = (int)e->points.size();
        double len = 0.0;
        for (size_t j = 1; j < e->points.size(); ++j) {
            double dx = std::abs(e->points[j].x() - e->points[j-1].x());
            double dy = std::abs(e->points[j].y() - e->points[j-1].y());
            len += std::sqrt(dx*dx + dy*dy);
            if (dx > dy) hLen += dx;
            else vLen += dy;
        }
        result.edges[i].length = len;
        totalLen += len;
        totalWp += (int)e->points.size() - 1;
    }

    result.totalEdgeLength = totalLen;
    result.avgWaypoints = graph.edges.empty() ? 0.0 : (double)totalWp / graph.edges.size();
    double hPlusV = hLen + vLen;
    result.hvRatio = (hPlusV > 0) ? hLen / hPlusV : 0.0;

    result.crossings = countEdgeCrossings(result.edges);

    // Compute average detour ratio
    double totalDetour = 0.0;
    int edgesWithManhattan = 0;
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        const auto& pts = result.edges[i].points;
        if (pts.size() >= 2) {
            double actual = result.edges[i].length;
            double manh = manhattanDistance(pts.front(), pts.back());
            if (manh > 0.001) {
                totalDetour += (actual - manh) / manh;
                ++edgesWithManhattan;
            }
        }
    }
    result.avgDetourRatio = edgesWithManhattan ? totalDetour / edgesWithManhattan : 0.0;

    // Congestion (simple grid-based)
    const double cellSize = 20.0;
    std::map<std::pair<int,int>, int> cellCount;
    for (const auto& e : result.edges) {
        for (size_t i = 1; i < e.points.size(); ++i) {
            int cx = (int)std::floor(e.points[i].x() / cellSize);
            int cy = (int)std::floor(e.points[i].y() / cellSize);
            cellCount[{cx,cy}]++;
        }
    }
    int maxC = 0;
    int sumC = 0;
    for (auto& kv : cellCount) {
        maxC = std::max(maxC, kv.second);
        sumC += kv.second;
    }
    result.maxCongestion = maxC;
    result.avgCongestion = cellCount.empty() ? 0.0 : (double)sumC / cellCount.size();

    return result;
}

LayoutResult runElkjsLayout(const std::string& filename) {
    LayoutResult result;
    result.netlist = filename;

    CircuitGraph graph;
    Netlist netlist = NetlistParser::parse(filename);
    if (netlist.modules.empty()) {
        std::cerr << "Parse failed for " << filename << "\n";
        return result;
    }

    std::string top = netlist.topModule.empty() ? netlist.modules.begin()->first : netlist.topModule;
    graph.buildHierarchical(netlist, top);

    auto t0 = std::chrono::steady_clock::now();
    graph.applyLayoutViaElkjs();
    auto t1 = std::chrono::steady_clock::now();
    result.layoutTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.nodeCount = (int)graph.nodes.size();
    result.edgeCount = (int)graph.edges.size();

    // Collect node metrics
    for (auto* n : graph.nodes) {
        NodeMetrics nm;
        nm.x = n->x; nm.y = n->y;
        nm.width = n->width; nm.height = n->height;
        result.nodes[n->id] = nm;
    }

    // Collect edge metrics
    result.edges.resize(graph.edges.size());
    double totalLen = 0.0;
    int totalWp = 0;
    double hLen = 0.0, vLen = 0.0;
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        auto* e = graph.edges[i];
        result.edges[i].points = e->points;
        result.edges[i].waypointCount = (int)e->points.size();
        double len = 0.0;
        for (size_t j = 1; j < e->points.size(); ++j) {
            double dx = std::abs(e->points[j].x() - e->points[j-1].x());
            double dy = std::abs(e->points[j].y() - e->points[j-1].y());
            len += std::sqrt(dx*dx + dy*dy);
            if (dx > dy) hLen += dx;
            else vLen += dy;
        }
        result.edges[i].length = len;
        totalLen += len;
        totalWp += (int)e->points.size() - 1;
    }

    result.totalEdgeLength = totalLen;
    result.avgWaypoints = graph.edges.empty() ? 0.0 : (double)totalWp / graph.edges.size();
    double hPlusV = hLen + vLen;
    result.hvRatio = (hPlusV > 0) ? hLen / hPlusV : 0.0;

    result.crossings = countEdgeCrossings(result.edges);

    // Compute average detour ratio
    double totalDetour = 0.0;
    int edgesWithManhattan = 0;
    for (size_t i = 0; i < graph.edges.size(); ++i) {
        const auto& pts = result.edges[i].points;
        if (pts.size() >= 2) {
            double actual = result.edges[i].length;
            double manh = manhattanDistance(pts.front(), pts.back());
            if (manh > 0.001) {
                totalDetour += (actual - manh) / manh;
                ++edgesWithManhattan;
            }
        }
    }
    result.avgDetourRatio = edgesWithManhattan ? totalDetour / edgesWithManhattan : 0.0;

    // Congestion (simple grid-based)
    const double cellSize = 20.0;
    std::map<std::pair<int,int>, int> cellCount;
    for (const auto& e : result.edges) {
        for (size_t i = 1; i < e.points.size(); ++i) {
            int cx = (int)std::floor(e.points[i].x() / cellSize);
            int cy = (int)std::floor(e.points[i].y() / cellSize);
            cellCount[{cx,cy}]++;
        }
    }
    int maxC = 0;
    int sumC = 0;
    for (auto& kv : cellCount) {
        maxC = std::max(maxC, kv.second);
        sumC += kv.second;
    }
    result.maxCongestion = maxC;
    result.avgCongestion = cellCount.empty() ? 0.0 : (double)sumC / cellCount.size();

    return result;
}

CompareResult compareResults(const LayoutResult& cpp, const LayoutResult& elkjs) {
    CompareResult cmp;

    if (cpp.nodeCount != elkjs.nodeCount) {
        std::cerr << "Node count mismatch: C++=" << cpp.nodeCount << " elkjs=" << elkjs.nodeCount << "\n";
    }

    // Node position diff
    double sumDiff = 0.0, maxDiff = 0.0;
    int matched = 0;
    for (const auto& kv : cpp.nodes) {
        auto it = elkjs.nodes.find(kv.first);
        if (it != elkjs.nodes.end()) {
            double dx = std::abs(kv.second.x - it->second.x);
            double dy = std::abs(kv.second.y - it->second.y);
            double dist = std::sqrt(dx*dx + dy*dy);
            sumDiff += dist;
            maxDiff = std::max(maxDiff, dist);
            ++matched;
        }
    }
    cmp.nodePositionDiffAvg = matched ? sumDiff / matched : 0.0;
    cmp.nodePositionDiffMax = maxDiff;

    // Crossing diff
    cmp.crossingDiff = std::abs(cpp.crossings - elkjs.crossings);

    // Edge length diff
    if (elkjs.totalEdgeLength > 0.001) {
        cmp.edgeLengthDiffPct = 100.0 * std::abs(cpp.totalEdgeLength - elkjs.totalEdgeLength) / elkjs.totalEdgeLength;
    }

    // Waypoint diff
    cmp.waypointDiffAvg = std::abs(cpp.avgWaypoints - elkjs.avgWaypoints);

    // HV ratio diff
    cmp.hvRatioDiff = std::abs(cpp.hvRatio - elkjs.hvRatio);

    // Layout time ratio
    if (elkjs.layoutTimeMs > 0.001) {
        cmp.layoutTimeRatio = cpp.layoutTimeMs / elkjs.layoutTimeMs;
    }

    return cmp;
}

void printResult(const LayoutResult& r, const std::string& label) {
    std::cout << std::setw(12) << label << ":\n";
    std::cout << "  Nodes: " << r.nodeCount << "  Edges: " << r.edgeCount << "\n";
    std::cout << "  Runtime: " << std::fixed << std::setprecision(1) << r.layoutTimeMs << " ms\n";
    std::cout << "  Edge crossings: " << r.crossings << "\n";
    std::cout << "  Total edge length: " << std::setprecision(1) << r.totalEdgeLength << "\n";
    std::cout << "  Avg waypoints: " << std::setprecision(2) << r.avgWaypoints << "\n";
    std::cout << "  H/V ratio: " << std::setprecision(3) << r.hvRatio << "\n";
    std::cout << "  Max congestion: " << r.maxCongestion << "\n";
    std::cout << "  Avg detour ratio: " << std::setprecision(3) << r.avgDetourRatio << "\n";
}

void printDiff(const CompareResult& cmp) {
    std::cout << "\n" << std::string(50, '-') << "\n";
    std::cout << "COMPARISON SUMMARY:\n";
    std::cout << std::setw(30) << std::left << "  Avg node position diff:" << std::right << std::fixed << std::setprecision(2) << cmp.nodePositionDiffAvg << " px\n";
    std::cout << std::setw(30) << "  Max node position diff:" << std::setprecision(2) << cmp.nodePositionDiffMax << " px\n";
    std::cout << std::setw(30) << "  Crossing diff:" << std::right << " " << cmp.crossingDiff << "\n";
    std::cout << std::setw(30) << "  Edge length diff:" << std::setprecision(1) << (cmp.edgeLengthDiffPct > 0.01 ? std::to_string(cmp.edgeLengthDiffPct)+"%" : "N/A") << "\n";
    std::cout << std::setw(30) << "  Avg waypoint diff:" << std::setprecision(2) << cmp.waypointDiffAvg << "\n";
    std::cout << std::setw(30) << "  HV ratio diff:" << std::setprecision(3) << cmp.hvRatioDiff << "\n";
    if (cmp.layoutTimeRatio > 0.001) {
        std::cout << std::setw(30) << "  C++/elkjs time ratio:" << std::setprecision(2) << cmp.layoutTimeRatio << "x\n";
    }
    std::cout << "\nVERDICT:\n";
    bool pass = true;
    if (cmp.nodePositionDiffAvg > 5.0) {
        std::cout << "  [WARN] Node position diff > 5px (avg " << cmp.nodePositionDiffAvg << ")\n";
    }
    if (cmp.crossingDiff > 5) {
        std::cout << "  [WARN] Crossing diff > 5\n";
    }
    if (cmp.edgeLengthDiffPct > 10.0) {
        std::cout << "  [WARN] Edge length diff > 10%\n";
    }
    if (cmp.nodePositionDiffAvg <= 2.0 && cmp.crossingDiff == 0 && cmp.edgeLengthDiffPct <= 5.0) {
        std::cout << "  [PASS] Layouts are functionally equivalent\n";
    } else {
        std::cout << "  [INFO] Layouts differ - review algorithm differences\n";
        pass = false;
    }
    std::cout << "\n";
}

int main(int argc, char** argv) {
    std::string filename = "test_circuit.v";
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg[0] != '-') {
            filename = arg;
        }
    }

    QCoreApplication app(argc, argv);  // for QProcess (elkjs)

    std::cout << std::string(60, '=') << "\n";
    std::cout << "ELK Layout Comparison: C++ vs elkjs\n";
    std::cout << "Netlist: " << filename << "\n";
    std::cout << std::string(60, '=') << "\n\n";

    std::cout << "Running C++ ELK layout...\n";
    auto cppResult = runCppLayout(filename);
    if (cppResult.nodeCount == 0) {
        std::cerr << "C++ layout failed or produced no nodes\n";
    } else {
        printResult(cppResult, "C++ ELK");
    }

    std::cout << "\nRunning elkjs layout...\n";
    auto elkjsResult = runElkjsLayout(filename);
    if (elkjsResult.nodeCount == 0) {
        std::cerr << "elkjs layout failed or produced no nodes\n";
    } else {
        printResult(elkjsResult, "elkjs");
    }

    if (cppResult.nodeCount > 0 && elkjsResult.nodeCount > 0) {
        CompareResult cmp = compareResults(cppResult, elkjsResult);
        printDiff(cmp);
    }

    return 0;
}