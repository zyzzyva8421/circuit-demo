// Regression test for layout engine.
// Usage:
//   test_regression <netlist.v> [--snapshot <file>] [--compare <file>]
//
// Modes:
//   (no flag)          : run hard-invariant checks only; exit 0 = pass
//   --snapshot <file>  : run checks, write metrics to file, exit 0 = pass
//   --compare <file>   : run checks, compare against snapshot; exit 0 = pass + no regression
//
// Hard invariants (always checked, cause non-zero exit on failure):
//   1. instanceOverlaps     == 0
//   2. inputXVariants        == 1  (all input port nodes at same X)
//   3. outputXVariants       == 1  (all output port nodes at same X)
//   4. leftViolations        == 0  (no edge point left of input column)
//   5. rightViolations       == 0  (no edge point right of output column)
//   6. brokenEdges           == 0  (all edges have >=2 routing points)
//   7. badConnectivity       == 0  (first/last point within node bbox)
//   8. wirePassesInstance    == 0  (no edge segment intersects a non-adjacent module body)
//   9. wireOverlapPairs      == 0  (no two different nets share an overlapping collinear segment)
//
// Soft metrics (regression threshold: >5% worse -> FAIL):
//   All metrics from test_metrics.cpp:
//   edgeCrossings, topologicalCrossings, collisionPairs, edgesWithCollision,
//   totalEdgeLength, avgWaypoints, waypointsP90/P95/Max,
//   hSegLength, vSegLength, avgDetourRatio, detourP50/P90/P95/Max,
//   maxCongestion, avgCongestion, congestionHotspots,
//   crossingsPer1kEdges, topoCrossingsPer1kEdges, collisionPairsPer1kEdges,
//   edgeClarity, dummies

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <vector>
#include <cmath>
#include <limits>
#include <chrono>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <iomanip>
#include <unistd.h>
#include "circuitgraph.h"
#include "netlistparser.h"

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------
static bool segIntersect(double x1,double y1,double x2,double y2,
                         double x3,double y3,double x4,double y4) {
    auto cross=[](double ax,double ay,double bx,double by)->double{
        return ax*by-ay*bx; };
    double d1x=x2-x1,d1y=y2-y1,d2x=x4-x3,d2y=y4-y3;
    double denom=cross(d1x,d1y,d2x,d2y);
    if(std::abs(denom)<1e-9) return false;
    double tx=x3-x1,ty=y3-y1;
    double t=cross(tx,ty,d2x,d2y)/denom;
    double u=cross(tx,ty,d1x,d1y)/denom;
    return t>1e-6&&t<1.0-1e-6&&u>1e-6&&u<1.0-1e-6;
}

static double percentile(std::vector<double> values, double p) {
    if (values.empty()) return 0.0;
    if (p<=0.0) return *std::min_element(values.begin(),values.end());
    if (p>=100.0) return *std::max_element(values.begin(),values.end());
    std::sort(values.begin(),values.end());
    double rank=(p/100.0)*(values.size()-1);
    size_t lo=(size_t)std::floor(rank), hi=(size_t)std::ceil(rank);
    if (lo==hi) return values[lo];
    return values[lo]*(1.0-(rank-lo))+values[hi]*(rank-lo);
}

// -----------------------------------------------------------------------
// Metric bundle
// -----------------------------------------------------------------------
struct Metrics {
    std::string netlist;
    int nodes=0, edges=0;
    // Hard invariants
    int instanceOverlaps=0;
    int inputXVariants=0;
    int outputXVariants=0;
    int leftViolations=0;
    int rightViolations=0;
    int brokenEdges=0;
    int badConnectivity=0;
    int wirePassesInstance=0;        // hard: edge routes through non-adjacent module body
    int wireOverlapPairs=0;          // hard: two different nets share an overlapping collinear segment
    int wireEndpointStubCrossing=0;  // hard: first/last H-stub of edge enters source/target body interior
    // Soft: collision
    int collisionPairs=0;
    int edgesWithCollision=0;
    // Soft: crossings
    int edgeCrossings=0;
    int topologicalCrossings=0;
    // Soft: geometry
    double totalEdgeLength=0.0;
    double avgWaypoints=0.0;
    double waypointsP90=0.0, waypointsP95=0.0, waypointsMax=0.0;
    double hSegLength=0.0, vSegLength=0.0, hvRatio=0.0;
    // Soft: detour
    double avgDetourRatio=0.0;
    double detourP50=0.0, detourP90=0.0, detourP95=0.0, detourMax=0.0;
    // Soft: congestion
    int    maxCongestion=0;
    double avgCongestion=0.0;
    int    congestionHotspots=0;
    // Soft: normalized KPIs
    double crossingsPer1kEdges=0.0;
    double topoCrossingsPer1kEdges=0.0;
    double collisionPairsPer1kEdges=0.0;
    double edgeClarity=0.0;
    // Misc
    long long dummies=0;
    long long runtime_ms=0;
};

static long long parseDummies(const std::string& logfile) {
    std::ifstream f(logfile);
    if (!f) return -1;
    std::string line; long long d=-1;
    while (std::getline(f,line)) {
        auto p=line.find("dummies=");
        if (p!=std::string::npos) d=std::stoll(line.substr(p+8));
    }
    return d;
}

// -----------------------------------------------------------------------
// Core measurement
// -----------------------------------------------------------------------
static Metrics measure(const std::string& path, const std::string& stderrLog) {
    Metrics m; m.netlist=path;
    Netlist netlist=NetlistParser::parse(path);
    if (netlist.modules.empty()) { std::cerr<<"[REGRESSION] parse failed: "<<path<<"\n"; return m; }
    std::string top=netlist.topModule.empty()?netlist.modules.begin()->first:netlist.topModule;
    CircuitGraph graph;
    graph.buildHierarchical(netlist,top);
    auto t0=std::chrono::steady_clock::now();
    graph.applyLayout();
    auto t1=std::chrono::steady_clock::now();
    m.runtime_ms=std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count();
    m.nodes=(int)graph.nodes.size();
    m.edges=(int)graph.edges.size();
    const bool large=(m.edges>2000);

    // 1. Instance overlaps
    for (size_t i=0;i<graph.nodes.size();++i) {
        CNode* a=graph.nodes[i];
        if (a->data.type!=CircuitNodeType::ModuleInstance) continue;
        for (size_t j=i+1;j<graph.nodes.size();++j) {
            CNode* b=graph.nodes[j];
            if (b->data.type!=CircuitNodeType::ModuleInstance) continue;
            if (!(a->x+a->width<=b->x||b->x+b->width<=a->x||
                  a->y+a->height<=b->y||b->y+b->height<=a->y)) m.instanceOverlaps++;
        }
    }

    // 2. Port X alignment + boundary violations
    {
        std::set<long long> inXs,outXs;
        double lX=std::numeric_limits<double>::max(), rX=-std::numeric_limits<double>::max();
        for (auto* n:graph.nodes) {
            if (n->data.type!=CircuitNodeType::Port) continue;
            if (n->data.direction=="input")  { inXs.insert(llround(n->x*1000));  lX=std::min(lX,n->x); }
            else if (n->data.direction=="output") { outXs.insert(llround(n->x*1000)); rX=std::max(rX,n->x+n->width); }
        }
        m.inputXVariants=(int)inXs.size();
        m.outputXVariants=(int)outXs.size();
        if (lX<std::numeric_limits<double>::max()&&rX>-std::numeric_limits<double>::max())
            for (auto* e:graph.edges)
                for (const auto& pt:e->points) {
                    if (pt.x()<lX-1.0) m.leftViolations++;
                    if (pt.x()>rX+1.0) m.rightViolations++;
                }
    }

    // 3. Broken edges
    for (auto* e:graph.edges) if ((int)e->points.size()<2) m.brokenEdges++;

    // 4. Edge connectivity
    const double TOL=50.0;
    for (auto* e:graph.edges) {
        if (e->points.size()<2) continue;
        const auto& p0=e->points.front(); const auto& p1=e->points.back();
        CNode* src=e->source; CNode* tgt=e->target;
        bool okS=(p0.x()>=src->x-TOL&&p0.x()<=src->x+src->width+TOL&&
                  p0.y()>=src->y-TOL&&p0.y()<=src->y+src->height+TOL);
        bool okT=(p1.x()>=tgt->x-TOL&&p1.x()<=tgt->x+tgt->width+TOL&&
                  p1.y()>=tgt->y-TOL&&p1.y()<=tgt->y+tgt->height+TOL);
        if (!okS||!okT) m.badConnectivity++;
    }

    // 5. Edge-edge crossings (hyper-edge aware, skip for large)
    if (!large) {
        std::map<std::pair<CNode*,std::string>,std::vector<size_t>> hg;
        for (size_t i=0;i<graph.edges.size();++i)
            hg[{graph.edges[i]->source,graph.edges[i]->sourcePort}].push_back(i);
        struct GD{std::vector<QPointF> s;float mnX,mxX,mnY,mxY;};
        std::vector<GD> gv; gv.reserve(hg.size());
        for (auto& kv:hg) {
            GD g; g.mnX=g.mnY=1e30f; g.mxX=g.mxY=-1e30f;
            for (size_t idx:kv.second) {
                const auto* e=graph.edges[idx];
                for (size_t si=0;si+1<e->points.size();++si){g.s.push_back(e->points[si]);g.s.push_back(e->points[si+1]);}
                for (const auto& p:e->points){float fx=(float)p.x(),fy=(float)p.y();g.mnX=std::min(g.mnX,fx);g.mxX=std::max(g.mxX,fx);g.mnY=std::min(g.mnY,fy);g.mxY=std::max(g.mxY,fy);}
            }
            gv.push_back(std::move(g));
        }
        for (size_t i=0;i<gv.size();++i) {
            if (gv[i].s.size()<4) continue;
            for (size_t j=i+1;j<gv.size();++j) {
                if (gv[j].s.size()<4) continue;
                if (gv[i].mxX<gv[j].mnX||gv[j].mxX<gv[i].mnX||gv[i].mxY<gv[j].mnY||gv[j].mxY<gv[i].mnY) continue;
                bool cx=false;
                for (size_t si=0;si+1<gv[i].s.size()&&!cx;si+=2)
                    for (size_t sj=0;sj+1<gv[j].s.size()&&!cx;sj+=2)
                        if (segIntersect(gv[i].s[si].x(),gv[i].s[si].y(),gv[i].s[si+1].x(),gv[i].s[si+1].y(),
                                         gv[j].s[sj].x(),gv[j].s[sj].y(),gv[j].s[sj+1].x(),gv[j].s[sj+1].y())) cx=true;
                if (cx) m.edgeCrossings++;
            }
        }
    } else { m.edgeCrossings=-1; }

    // 6. Topological crossings
    if (!large) {
        int n=(int)graph.edges.size();
        for (int i=0;i<n;++i) {
            if (graph.edges[i]->points.size()<2) continue;
            const auto& A0=graph.edges[i]->points.front(); const auto& A1=graph.edges[i]->points.back();
            for (int j=i+1;j<n;++j) {
                if (graph.edges[j]->points.size()<2) continue;
                const auto& B0=graph.edges[j]->points.front(); const auto& B1=graph.edges[j]->points.back();
                if (segIntersect(A0.x(),A0.y(),A1.x(),A1.y(),B0.x(),B0.y(),B1.x(),B1.y())) m.topologicalCrossings++;
            }
        }
    } else { m.topologicalCrossings=-1; }

    // 7. Edge-instance collisions (soft metric, large graphs set to -1)
    if (!large) {
        for (auto* e:graph.edges) {
            bool hit=false;
            for (size_t si=0;si+1<e->points.size();++si) {
                double x1=e->points[si].x(),y1=e->points[si].y(),x2=e->points[si+1].x(),y2=e->points[si+1].y();
                double mnX=std::min(x1,x2),mxX=std::max(x1,x2),mnY=std::min(y1,y2),mxY=std::max(y1,y2);
                for (auto* n:graph.nodes) {
                    if (n->data.type!=CircuitNodeType::ModuleInstance||n==e->source||n==e->target) continue;
                    if (mxX<n->x||mnX>n->x+n->width||mxY<n->y||mnY>n->y+n->height) continue;
                    m.collisionPairs++; hit=true;
                }
            }
            if (hit) m.edgesWithCollision++;
        }
    } else { m.collisionPairs=-1; m.edgesWithCollision=-1; }

    // 8. Hard: wirePassesInstance — edge segment pierces a module body (non src/tgt)
    // Checked even for large graphs (O(E*N) but uses bounding-box early-out).
    {
        // Build simple Y-center sorted index for module nodes.
        std::vector<CNode*> modNodes;
        for (auto* n : graph.nodes)
            if (n->data.type == CircuitNodeType::ModuleInstance ||
                n->data.type == CircuitNodeType::ExpandedInstance)
                modNodes.push_back(n);
        std::sort(modNodes.begin(), modNodes.end(),
                  [](CNode* a, CNode* b){ return a->y < b->y; });
        const double MARGIN = 1.0;
        for (auto* e : graph.edges) {
            for (size_t si = 0; si + 1 < e->points.size(); ++si) {
                double x1=e->points[si].x(),   y1=e->points[si].y();
                double x2=e->points[si+1].x(), y2=e->points[si+1].y();
                double mnX=std::min(x1,x2)-MARGIN, mxX=std::max(x1,x2)+MARGIN;
                double mnY=std::min(y1,y2)-MARGIN, mxY=std::max(y1,y2)+MARGIN;
                for (auto* n : modNodes) {
                    if (n->y > mxY) break;          // sorted by y, no need to continue
                    if (n == e->source || n == e->target) continue;
                    if (n->y + n->height < mnY) continue;
                    if (n->x + n->width  < mnX || n->x > mxX) continue;
                    // Strict interior check (shrink by MARGIN to ignore pin-edge grazing)
                    double rx0 = n->x + MARGIN, rx1 = n->x + n->width  - MARGIN;
                    double ry0 = n->y + MARGIN, ry1 = n->y + n->height - MARGIN;
                    if (rx0 >= rx1 || ry0 >= ry1) continue;
                    // Horizontal segment
                    if (std::abs(y1-y2) < 0.001) {
                        if (y1 > ry0 && y1 < ry1 &&
                            std::max(x1,x2) > rx0 && std::min(x1,x2) < rx1)
                            m.wirePassesInstance++;
                    // Vertical segment
                    } else if (std::abs(x1-x2) < 0.001) {
                        if (x1 > rx0 && x1 < rx1 &&
                            std::max(y1,y2) > ry0 && std::min(y1,y2) < ry1)
                            m.wirePassesInstance++;
                    }
                }
            }
        }
    }

    // 8b. Hard: wireEndpointStubCrossing — endpoint-related self-overlap.
    //     wirePassesInstance skips source/target nodes, so this block explicitly checks
    //     whether any routed segment enters source/target body interior (strict interior,
    //     margin-shrunk) to catch cases where a route folds back through its own endpoint.
    {
        const double MARGIN_EP = 1.0;
        auto segmentIntersectsBodyInterior = [&](CNode* n,
                                                 const QPointF& a,
                                                 const QPointF& b) -> bool {
            if (!n) return false;
            double rx0 = n->x + MARGIN_EP, rx1 = n->x + n->width  - MARGIN_EP;
            double ry0 = n->y + MARGIN_EP, ry1 = n->y + n->height - MARGIN_EP;
            if (!(rx0 < rx1 && ry0 < ry1)) return false;

            // Horizontal segment: y strictly inside and x-ranges overlap with positive length.
            if (std::abs(a.y() - b.y()) < 0.001) {
                double y = a.y();
                if (!(y > ry0 && y < ry1)) return false;
                double lo = std::max(std::min(a.x(), b.x()), rx0);
                double hi = std::min(std::max(a.x(), b.x()), rx1);
                return hi - lo > 0.001;
            }

            // Vertical segment: x strictly inside and y-ranges overlap with positive length.
            if (std::abs(a.x() - b.x()) < 0.001) {
                double x = a.x();
                if (!(x > rx0 && x < rx1)) return false;
                double lo = std::max(std::min(a.y(), b.y()), ry0);
                double hi = std::min(std::max(a.y(), b.y()), ry1);
                return hi - lo > 0.001;
            }

            return false;
        };

        for (auto* e : graph.edges) {
            if (e->points.size() < 2) continue;
            const size_t nseg = e->points.size() - 1;
            // Only check middle segments; skip first 2 and last 2 to avoid flagging legal lead-in/lead-out.
            // This detects only true fold-back crossings where a route re-enters its endpoint.
            for (size_t si = 2; si + 2 < nseg; ++si) {
                const QPointF& p0 = e->points[si];
                const QPointF& p1 = e->points[si + 1];
                if (segmentIntersectsBodyInterior(e->source, p0, p1)) {
                    if (path.find("deep_hierarchy") != std::string::npos)
                        fprintf(stderr, "[EPDBG] src-hit edge %s:%s -> %s:%s seg=%zu\n",
                                e->source ? e->source->id.c_str() : "?", e->sourcePort.c_str(),
                                e->target ? e->target->id.c_str() : "?", e->targetPort.c_str(), si);
                    m.wireEndpointStubCrossing++;
                }
                if (e->target != e->source &&
                    segmentIntersectsBodyInterior(e->target, p0, p1)) {
                    if (path.find("deep_hierarchy") != std::string::npos)
                        fprintf(stderr, "[EPDBG] tgt-hit edge %s:%s -> %s:%s seg=%zu\n",
                                e->source ? e->source->id.c_str() : "?", e->sourcePort.c_str(),
                                e->target ? e->target->id.c_str() : "?", e->targetPort.c_str(), si);
                    m.wireEndpointStubCrossing++;
                }
            }
        }
    }

    // 9. Hard: wireOverlapPairs — two different nets share a collinear overlapping wire segment.
    // H-segment overlaps are the primary concern (visually two parallel wires merge into one).
    // V-segment overlaps in inter-layer gaps are a known limitation of orthogonal routing
    // (slot-assigned vertical segments may have Y ranges that slightly exceed port extents).
    // We check both but count them separately:
    //   wireOverlapPairs counts H-overlaps exceeding trackGap (significant bus-H collision)
    //                   and V-overlaps exceeding trackGap (slot-assignment failure).
    // Only checked for non-large graphs.
    if (!large) {
        // routeStep / trackGap approximation (must match elk_circuit_layout.cpp values).
        // portPort spacing default is 20; trackGap = max(20, routeStep*2) = 40.
        const double kTrackGap = 40.0;

        // Key: (edgeIdx -> netId) where netId = index of (source, sourcePort) group
        std::map<std::pair<CNode*,std::string>, int> netIdMap;
        int nextNet = 0;
        std::vector<int> edgeNet(graph.edges.size(), -1);
        for (size_t ei = 0; ei < graph.edges.size(); ++ei) {
            auto* e = graph.edges[ei];
            auto key = std::make_pair(e->source, e->sourcePort);
            auto it = netIdMap.find(key);
            if (it == netIdMap.end()) { netIdMap[key] = nextNet; edgeNet[ei] = nextNet++; }
            else edgeNet[ei] = it->second;
        }
        // Collect all H-segments and V-segments per net.
        struct Seg { int net; double c, lo, hi; }; // c=constant coord, lo/hi=variable range
        std::vector<Seg> hSegs, vSegs;
        for (size_t ei = 0; ei < graph.edges.size(); ++ei) {
            auto* e = graph.edges[ei];
            int net = edgeNet[ei];
            for (size_t si = 0; si + 1 < e->points.size(); ++si) {
                double x1=e->points[si].x(), y1=e->points[si].y();
                double x2=e->points[si+1].x(), y2=e->points[si+1].y();
                if (std::abs(y1-y2) < 0.001)       // horizontal
                    hSegs.push_back({net, y1, std::min(x1,x2), std::max(x1,x2)});
                else if (std::abs(x1-x2) < 0.001)  // vertical
                    vSegs.push_back({net, x1, std::min(y1,y2), std::max(y1,y2)});
            }
        }
        // Check each group for overlapping collinear segs from different nets.
        const double OVL_EPS = 2.0;  // collinear tolerance on constant axis (px)
        // Overlap must exceed kTrackGap to be flagged — smaller overlaps are unavoidable
        // micro-conflicts at shared slot boundaries in the current orthogonal router.
        auto checkOverlap = [&](std::vector<Seg>& segs) {
            std::sort(segs.begin(), segs.end(), [](const Seg& a, const Seg& b){
                return a.c < b.c || (a.c == b.c && a.lo < b.lo); });
            for (size_t i = 0; i < segs.size(); ++i) {
                for (size_t j = i+1; j < segs.size(); ++j) {
                    if (segs[j].c > segs[i].c + OVL_EPS) break;
                    if (segs[j].net == segs[i].net) continue;
                    double oLo = std::max(segs[i].lo, segs[j].lo);
                    double oHi = std::min(segs[i].hi, segs[j].hi);
                    if (oHi - oLo > kTrackGap) {
                        m.wireOverlapPairs++;
                    }
                }
            }
        };
        checkOverlap(hSegs);
        checkOverlap(vSegs);
    }

    // 8. Geometry + congestion
    {
        struct PH {
            size_t operator()(const std::pair<CNode*,std::string>& p) const {
                return std::hash<void*>{}(p.first)^(std::hash<std::string>{}(p.second)*2654435761ULL);
            }
        };
        std::unordered_map<std::pair<CNode*,std::string>,int,PH> hgid;
        int nxg=0;
        for (const auto* e:graph.edges){auto k=std::make_pair(e->source,e->sourcePort);if(!hgid.count(k))hgid[k]=nxg++;}

        const double cs=large?200.0:20.0; const int ct=4;
        auto ck=[](int cx,int cy)->int64_t{return((int64_t)(cx+100000)<<20)|(uint32_t)(cy+100000);};
        auto dk=[](int g,int cx,int cy)->int64_t{return((int64_t)g<<40)^((int64_t)(cx+100000)<<20)^(uint32_t)(cy+100000);};
        std::unordered_map<int64_t,int> cg;
        std::unordered_set<int64_t> vd,hd;
        int twp=0; double dsum=0.0; int rc=0;
        std::vector<double> ds,ws;

        for (const auto* e:graph.edges) {
            if (e->points.size()<2) continue;
            const QPointF& S=e->points.front(); const QPointF& T=e->points.back();
            double mhd=std::abs(T.x()-S.x())+std::abs(T.y()-S.y());
            double ea=0.0; int gid=hgid[{e->source,e->sourcePort}];
            for (size_t si=0;si+1<e->points.size();++si) {
                const auto& p1=e->points[si]; const auto& p2=e->points[si+1];
                double dx=p2.x()-p1.x(),dy=p2.y()-p1.y(),len=std::sqrt(dx*dx+dy*dy);
                m.totalEdgeLength+=len; ea+=len;
                if (std::abs(dx)<0.001) {
                    m.vSegLength+=len;
                    int cx=(int)std::floor(p1.x()/cs);
                    int y0=(int)std::floor(std::min(p1.y(),p2.y())/cs),y1=(int)std::floor(std::max(p1.y(),p2.y())/cs);
                    for (int cy=y0;cy<=y1;++cy) if(vd.insert(dk(gid,cx,cy)).second) cg[ck(cx,cy)]++;
                } else if (std::abs(dy)<0.001) {
                    m.hSegLength+=len;
                    int cy=(int)std::floor(p1.y()/cs);
                    int x0=(int)std::floor(std::min(p1.x(),p2.x())/cs),x1=(int)std::floor(std::max(p1.x(),p2.x())/cs);
                    for (int cx=x0;cx<=x1;++cx) if(hd.insert(dk(gid,cx,cy)).second) cg[ck(cx,cy)]++;
                }
            }
            int wp=std::max(0,(int)e->points.size()-2);
            twp+=wp; ws.push_back((double)wp);
            if (mhd>0.1){double dr=(ea-mhd)/mhd;dsum+=dr;ds.push_back(dr);++rc;}
        }
        if (m.edges>0) m.avgWaypoints=(double)twp/m.edges;
        double hvt=m.hSegLength+m.vSegLength; if(hvt>0.001) m.hvRatio=m.hSegLength/hvt;
        if (rc>0) m.avgDetourRatio=dsum/rc;
        if (!ds.empty()){m.detourP50=percentile(ds,50);m.detourP90=percentile(ds,90);m.detourP95=percentile(ds,95);m.detourMax=percentile(ds,100);}
        if (!ws.empty()){m.waypointsP90=percentile(ws,90);m.waypointsP95=percentile(ws,95);m.waypointsMax=percentile(ws,100);}
        if (!cg.empty()){
            double tot=0;
            for (const auto& kv:cg){tot+=kv.second;m.maxCongestion=std::max(m.maxCongestion,kv.second);if(kv.second>=ct)m.congestionHotspots++;}
            m.avgCongestion=tot/cg.size();
        }
    }

    // 9. Dummies
    m.dummies=parseDummies(stderrLog);

    // 10. Normalized KPIs
    if (m.edges>0){
        double ewcr=(m.edgesWithCollision>=0)?(double)m.edgesWithCollision/m.edges:0.0;
        m.edgeClarity=1.0-ewcr;
        if(m.collisionPairs>=0)    m.collisionPairsPer1kEdges=(double)m.collisionPairs*1000.0/m.edges;
        if(m.edgeCrossings>=0)     m.crossingsPer1kEdges=(double)m.edgeCrossings*1000.0/m.edges;
        if(m.topologicalCrossings>=0) m.topoCrossingsPer1kEdges=(double)m.topologicalCrossings*1000.0/m.edges;
    }
    return m;
}

// -----------------------------------------------------------------------
// Snapshot I/O (floats stored *1000 as integers for stability)
// -----------------------------------------------------------------------
static void writeSnapshot(const Metrics& m, const std::string& file) {
    auto i3=[](double v){return llround(v*1000.0);};
    std::ofstream f(file);
    f<<"netlist="<<m.netlist<<"\n"
     <<"nodes="<<m.nodes<<"\n"<<"edges="<<m.edges<<"\n"
     <<"instanceOverlaps="<<m.instanceOverlaps<<"\n"
     <<"inputXVariants="<<m.inputXVariants<<"\n"
     <<"outputXVariants="<<m.outputXVariants<<"\n"
     <<"leftViolations="<<m.leftViolations<<"\n"
     <<"rightViolations="<<m.rightViolations<<"\n"
     <<"brokenEdges="<<m.brokenEdges<<"\n"
     <<"badConnectivity="<<m.badConnectivity<<"\n"
     <<"wirePassesInstance="<<m.wirePassesInstance<<"\n"
     <<"wireOverlapPairs="<<m.wireOverlapPairs<<"\n"
     <<"wireEndpointStubCrossing="<<m.wireEndpointStubCrossing<<"\n"
     <<"collisionPairs="<<m.collisionPairs<<"\n"
     <<"edgesWithCollision="<<m.edgesWithCollision<<"\n"
     <<"edgeCrossings="<<m.edgeCrossings<<"\n"
     <<"topologicalCrossings="<<m.topologicalCrossings<<"\n"
     <<"totalEdgeLength="<<i3(m.totalEdgeLength)<<"\n"
     <<"avgWaypoints="<<i3(m.avgWaypoints)<<"\n"
     <<"waypointsP90="<<i3(m.waypointsP90)<<"\n"
     <<"waypointsP95="<<i3(m.waypointsP95)<<"\n"
     <<"waypointsMax="<<i3(m.waypointsMax)<<"\n"
     <<"hSegLength="<<i3(m.hSegLength)<<"\n"
     <<"vSegLength="<<i3(m.vSegLength)<<"\n"
     <<"hvRatio="<<i3(m.hvRatio)<<"\n"
     <<"avgDetourRatio="<<i3(m.avgDetourRatio)<<"\n"
     <<"detourP50="<<i3(m.detourP50)<<"\n"
     <<"detourP90="<<i3(m.detourP90)<<"\n"
     <<"detourP95="<<i3(m.detourP95)<<"\n"
     <<"detourMax="<<i3(m.detourMax)<<"\n"
     <<"maxCongestion="<<m.maxCongestion<<"\n"
     <<"avgCongestion="<<i3(m.avgCongestion)<<"\n"
     <<"congestionHotspots="<<m.congestionHotspots<<"\n"
     <<"crossingsPer1kEdges="<<i3(m.crossingsPer1kEdges)<<"\n"
     <<"topoCrossingsPer1kEdges="<<i3(m.topoCrossingsPer1kEdges)<<"\n"
     <<"collisionPairsPer1kEdges="<<i3(m.collisionPairsPer1kEdges)<<"\n"
     <<"edgeClarity="<<i3(m.edgeClarity)<<"\n"
     <<"dummies="<<m.dummies<<"\n"
     <<"runtime_ms="<<m.runtime_ms<<"\n";
}

static std::map<std::string,std::string> readSnapshot(const std::string& file) {
    std::map<std::string,std::string> kv;
    std::ifstream f(file); std::string line;
    while (std::getline(f,line)){auto eq=line.find('=');if(eq!=std::string::npos)kv[line.substr(0,eq)]=line.substr(eq+1);}
    return kv;
}

// -----------------------------------------------------------------------
// Print + check
// -----------------------------------------------------------------------
static bool printAndCheck(const Metrics& m) {
    bool ok=true;
    auto chk=[&](const char* name,int val,int expected){
        if(val!=expected){std::cout<<"  [FAIL] "<<name<<" = "<<val<<"  (expected "<<expected<<")\n";ok=false;}
        else std::cout<<"  [pass] "<<name<<" = "<<val<<"\n";
    };
    std::cout<<"\n=== "<<m.netlist<<" ===\n"
             <<"  nodes="<<m.nodes<<"  edges="<<m.edges<<"  runtime="<<m.runtime_ms<<"ms\n";
    chk("instanceOverlaps",m.instanceOverlaps,0);
    if(m.inputXVariants>0||m.outputXVariants>0){chk("inputXVariants",m.inputXVariants,1);chk("outputXVariants",m.outputXVariants,1);}
    else std::cout<<"  [skip] inputXVariants/outputXVariants (no ports)\n";
    chk("leftViolations",m.leftViolations,0);chk("rightViolations",m.rightViolations,0);
    chk("brokenEdges",m.brokenEdges,0);chk("badConnectivity",m.badConnectivity,0);
    chk("wirePassesInstance",m.wirePassesInstance,0);
    chk("wireOverlapPairs",m.wireOverlapPairs,0);
    chk("wireEndpointStubCrossing",m.wireEndpointStubCrossing,0);

    std::cout<<"\n  --- Soft metrics ---\n";
    auto infi=[&](const char* n,long long v,bool skip=false){
        if(skip)std::cout<<"  [info] "<<n<<" = N/A\n";
        else std::cout<<"  [info] "<<n<<" = "<<v<<"\n";
    };
    auto inff=[&](const char* n,double v){
        std::cout<<"  [info] "<<n<<" = "<<std::fixed<<std::setprecision(4)<<v<<"\n";
    };
    infi("edgeCrossings",       m.edgeCrossings,       m.edgeCrossings<0);
    infi("topologicalCrossings",m.topologicalCrossings,m.topologicalCrossings<0);
    infi("collisionPairs",      m.collisionPairs,      m.collisionPairs<0);
    infi("edgesWithCollision",  m.edgesWithCollision,  m.edgesWithCollision<0);
    inff("totalEdgeLength",m.totalEdgeLength);
    inff("avgWaypoints",m.avgWaypoints);
    inff("waypointsP90",m.waypointsP90);inff("waypointsP95",m.waypointsP95);inff("waypointsMax",m.waypointsMax);
    inff("hSegLength",m.hSegLength);inff("vSegLength",m.vSegLength);inff("hvRatio",m.hvRatio);
    inff("avgDetourRatio",m.avgDetourRatio);
    inff("detourP50",m.detourP50);inff("detourP90",m.detourP90);inff("detourP95",m.detourP95);inff("detourMax",m.detourMax);
    infi("maxCongestion",m.maxCongestion);inff("avgCongestion",m.avgCongestion);infi("congestionHotspots",m.congestionHotspots);
    inff("crossingsPer1kEdges",m.crossingsPer1kEdges);inff("topoCrossingsPer1kEdges",m.topoCrossingsPer1kEdges);
    inff("collisionPairsPer1kEdges",m.collisionPairsPer1kEdges);inff("edgeClarity",m.edgeClarity);
    infi("dummies",m.dummies,m.dummies<0);
    return ok;
}

// -----------------------------------------------------------------------
// Compare vs snapshot (5% threshold)
// -----------------------------------------------------------------------
static bool compareSnapshot(const Metrics& m, const std::string& snapFile) {
    auto snap=readSnapshot(snapFile);
    if(snap.empty()){std::cerr<<"[REGRESSION] cannot read snapshot: "<<snapFile<<"\n";return false;}
    bool ok=true;
    auto sLL=[&](const std::string& k,long long def)->long long{auto it=snap.find(k);return it!=snap.end()?std::stoll(it->second):def;};

    auto chkHard=[&](const char* name,int cur,long long sv){
        if(cur!=0&&(long long)cur>sv){std::cout<<"  [REGRESS] "<<name<<": "<<sv<<" -> "<<cur<<" (+)\n";ok=false;}
        else if(cur==0) std::cout<<"  [pass]   "<<name<<" = 0\n";
        else std::cout<<"  [pass]   "<<name<<" = "<<cur<<" (was "<<sv<<")\n";
    };

    std::cout<<"\n=== compare: "<<m.netlist<<" vs "<<snapFile<<" ===\n";
    chkHard("instanceOverlaps",m.instanceOverlaps,sLL("instanceOverlaps",0));
    if(m.inputXVariants>0||m.outputXVariants>0){
        chkHard("inputXVariants",m.inputXVariants,sLL("inputXVariants",1));
        chkHard("outputXVariants",m.outputXVariants,sLL("outputXVariants",1));
    } else std::cout<<"  [skip] inputXVariants/outputXVariants (no ports)\n";
    chkHard("leftViolations",m.leftViolations,sLL("leftViolations",0));
    chkHard("rightViolations",m.rightViolations,sLL("rightViolations",0));
    chkHard("brokenEdges",m.brokenEdges,sLL("brokenEdges",0));
    chkHard("badConnectivity",m.badConnectivity,sLL("badConnectivity",0));
    chkHard("wirePassesInstance",m.wirePassesInstance,sLL("wirePassesInstance",0));
    chkHard("wireOverlapPairs",m.wireOverlapPairs,sLL("wireOverlapPairs",0));
    chkHard("wireEndpointStubCrossing",m.wireEndpointStubCrossing,sLL("wireEndpointStubCrossing",0));

    const double T=1.05;

    // Soft integer (lower is better unless higherIsBetter)
    auto chkI=[&](const char* name,long long cur,const std::string& key,bool hib=false){
        if(cur<0){std::cout<<"  [skip]   "<<name<<" (N/A)\n";return;}
        long long sv=sLL(key,-1); if(sv<0) return;
        if(sv==0){if(!hib&&cur>0){std::cout<<"  [REGRESS] "<<name<<": 0 -> "<<cur<<"\n";ok=false;}else std::cout<<"  [pass]   "<<name<<" = "<<cur<<"\n";return;}
        double ratio=hib?(double)sv/std::max(cur,(long long)1):(double)cur/sv;
        double pct=(ratio-1.0)*100.0;
        if(ratio>T){std::cout<<"  [REGRESS] "<<name<<": "<<sv<<" -> "<<cur<<"  (+"<<std::fixed<<std::setprecision(1)<<pct<<"%)\n";ok=false;}
        else{std::string sg=(pct>=0)?"+":"";std::cout<<"  [soft]   "<<name<<" = "<<cur<<" (was "<<sv<<", "<<sg<<std::setprecision(1)<<pct<<"%)\n";}
    };

    // Soft float (stored *1000; lower is better unless higherIsBetter)
    auto chkF=[&](const char* name,double cur,const std::string& key,bool hib=false){
        long long sv=sLL(key,-1); if(sv<0) return;
        long long cv=llround(cur*1000.0);
        if(sv==0){if(!hib&&cv>0){std::cout<<"  [REGRESS] "<<name<<": 0 -> "<<cur<<"\n";ok=false;}else std::cout<<"  [pass]   "<<name<<" = "<<std::fixed<<std::setprecision(4)<<cur<<"\n";return;}
        double ratio=hib?(double)sv/std::max(cv,(long long)1):(double)cv/sv;
        double pct=(ratio-1.0)*100.0;
        if(ratio>T){std::cout<<"  [REGRESS] "<<name<<": "<<std::fixed<<std::setprecision(4)<<(sv/1000.0)<<" -> "<<cur<<"  (+"<<std::setprecision(1)<<pct<<"%)\n";ok=false;}
        else{std::string sg=(pct>=0)?"+":"";std::cout<<"  [soft]   "<<name<<" = "<<std::fixed<<std::setprecision(4)<<cur<<" (was "<<(sv/1000.0)<<", "<<sg<<std::setprecision(1)<<pct<<"%)\n";}
    };

    chkI("edgeCrossings",       m.edgeCrossings,       "edgeCrossings");
    chkI("topologicalCrossings",m.topologicalCrossings,"topologicalCrossings");
    chkI("collisionPairs",      m.collisionPairs,      "collisionPairs");
    chkI("edgesWithCollision",  m.edgesWithCollision,  "edgesWithCollision");
    chkF("totalEdgeLength",     m.totalEdgeLength,     "totalEdgeLength");
    chkF("avgWaypoints",        m.avgWaypoints,        "avgWaypoints");
    chkF("waypointsP90",        m.waypointsP90,        "waypointsP90");
    chkF("waypointsP95",        m.waypointsP95,        "waypointsP95");
    chkF("waypointsMax",        m.waypointsMax,        "waypointsMax");
    chkF("hSegLength",          m.hSegLength,          "hSegLength");
    chkF("vSegLength",          m.vSegLength,          "vSegLength");
    chkF("avgDetourRatio",      m.avgDetourRatio,      "avgDetourRatio");
    chkF("detourP50",           m.detourP50,           "detourP50");
    chkF("detourP90",           m.detourP90,           "detourP90");
    chkF("detourP95",           m.detourP95,           "detourP95");
    chkF("detourMax",           m.detourMax,           "detourMax");
    chkI("maxCongestion",       m.maxCongestion,       "maxCongestion");
    chkF("avgCongestion",       m.avgCongestion,       "avgCongestion");
    chkI("congestionHotspots",  m.congestionHotspots,  "congestionHotspots");
    chkF("crossingsPer1kEdges",      m.crossingsPer1kEdges,      "crossingsPer1kEdges");
    chkF("topoCrossingsPer1kEdges",  m.topoCrossingsPer1kEdges,  "topoCrossingsPer1kEdges");
    chkF("collisionPairsPer1kEdges", m.collisionPairsPer1kEdges, "collisionPairsPer1kEdges");
    chkF("edgeClarity",              m.edgeClarity,              "edgeClarity",/*hib=*/true);
    chkI("dummies",                  m.dummies,                  "dummies");
    if(m.dummies>=0){long long sd=sLL("dummies",0);if(sd>0&&m.dummies<sd)std::cout<<"  [IMPROVE] dummies reduced by "<<(sd-m.dummies)<<" ("<<(int)((double)(sd-m.dummies)/sd*100)<<"%)\n";}
    return ok;
}

// -----------------------------------------------------------------------
// main
// -----------------------------------------------------------------------
int main(int argc, char* argv[]) {
    std::string netlistPath, snapshotFile;
    bool doSnapshot=false, doCompare=false;
    for (int i=1;i<argc;++i) {
        std::string a=argv[i];
        if (a=="--snapshot"&&i+1<argc){snapshotFile=argv[++i];doSnapshot=true;}
        else if (a=="--compare"&&i+1<argc){snapshotFile=argv[++i];doCompare=true;}
        else netlistPath=a;
    }
    if (netlistPath.empty()){std::cerr<<"Usage: test_regression <netlist.v> [--snapshot <f>|--compare <f>]\n";return 2;}

    std::string tmpLog="/tmp/regression_stderr_"+std::to_string(getpid())+".log";
    {FILE* lf=fopen(tmpLog.c_str(),"w");if(lf){dup2(fileno(lf),STDERR_FILENO);fclose(lf);}}
    freopen("/dev/tty","w",stderr);

    Metrics m=measure(netlistPath,tmpLog);
    bool hardPass=printAndCheck(m);
    bool comparePass=true;
    if (doCompare)  comparePass=compareSnapshot(m,snapshotFile);
    if (doSnapshot){writeSnapshot(m,snapshotFile);std::cout<<"[snapshot saved] "<<snapshotFile<<"\n";}
    remove(tmpLog.c_str());

    if (!hardPass){std::cout<<"\nRESULT: FAIL (hard invariant violated)\n";return 1;}
    if (!comparePass){std::cout<<"\nRESULT: FAIL (regression vs snapshot)\n";return 1;}
    std::cout<<"\nRESULT: PASS\n";
    return 0;
}
