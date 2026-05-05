# ELK C++ 完整移植计划

## 目标
将 Java ELK layered layout 算法完整移植到 C++，保持算法和调用逻辑完全一致。

## 当前状态

| 组件 | 状态 | 说明 |
|------|------|------|
| ELK 数据结构 | ✅ | ElkGraph, ElkNode, ElkPort, ElkEdge, LGraph 等 |
| 分层算法 | ⚠️ | Bellman-Ford 迭代 vs Java DFS 递归后序 |
| 交叉最小化 | ✅ | barycenter + transpose + greedy switch (完整) |
| BK Y坐标 | ⚠️ | 4-pass median，但缺少 type-1 conflict marking |
| X坐标放置 | ❌ | 简单 layerX 累加，未用 Brandes-Koepf 水平算法 |
| 正交路由 | ⚠️ | 自研算法，缺少依赖图和关键循环打破 |
| 中间处理器 | ❌ | 14种 processor 未实现 |

---

## Phase 1: 修复入口函数 + 创建回归测试

### 1.1 创建分支
```bash
git checkout -b elk-cpp-complete-port
```

### 1.2 创建对比测试工具
新建 `test_compare_elk.cpp`：
- 运行 C++ `applyElkLayout()` 获取结果
- 通过 `layout_script.js` 运行 elkjs 获取结果
- 对比两者输出的节点坐标、边 bend points
- 输出差异报告（diff 百分比）

**关键指标对比:**
```cpp
struct LayoutDiff {
    double nodePositionDiffAvg;   // 节点位置平均差异
    double nodePositionDiffMax;   // 最大差异
    double edgeBendPointDiff;     // 边拐点差异
    int crossingDiff;             // 交叉数差异
    double edgeLengthDiff;        // 边长度差异
};
```

### 1.3 验证现有实现基准
用 `test_circuit.v` (最小) 验证当前 C++ 输出是否合理。

---

## Phase 2: 实现分层算法 (Layer Assignment)

### 2.1 实现 `elk::layout(ElkGraph*)` 入口
在 `elk_layered.cpp` 中实现，遵循 Java 的 Phase 顺序：
```cpp
void elk::layout(ElkGraph* graph) {
    // P1: Cycle breaking (已有)
    // P2: Layering
    longestPathLayering(lgraph);
    // P3: Node ordering (已有)
    // P4: Node placement
    brandesKoepfPlace(lgraph, options);
    // P5: Edge routing
    orthogonalRoute(graph, options);
}
```

### 2.2 实现 `longestPathLayering(LGraph*)`
遵循 Java `LongestPathLayerer.java`:
- 用 DFS 递归后序遍历替代 Bellman-Ford
- 处理 self-loop (跳过)
- 处理 hierarchical ports
- 处理 Input/Output 特殊层级

**关键差异修复:**
```cpp
// C++ 当前: Bellman-Ford
for (size_t pass = 0; pass < nodes.size(); ++pass) {
    for (auto* edge : edges) {
        layerMap[target] = max(layerMap[target], layerMap[source] + 1);
    }
}

// Java: DFS 后序遍历
int visit(LNode* node) {
    if (visited[node->id] >= 0) return visited[node->id];
    int maxHeight = 1;
    for (auto* port : node.ports) {
        for (auto* edge : port.outgoingEdges) {
            maxHeight = max(maxHeight, visit(edge.target) + 1);
        }
    }
    putNode(node, maxHeight);
    return maxHeight;
}
```

---

## Phase 3: 实现 BK 节点放置

### 3.1 实现 `brandesKoepfPlace(LGraph*, LayoutOptions)`

按 Java `BKNodePlacer.java` 的 5 步算法:

**Step 1: Mark Type 1 Conflicts**
```cpp
// 短边跨过长边的情况需要标记
// 与 markedEdges.add(edge) 对应
```

**Step 2: Vertical Alignment (4 passes)**
- RIGHTDOWN, RIGHTUP, LEFTDOWN, LEFTUP
- 对每层从某个方向遍历，找 median neighbor 对齐

**Step 3: Inside Block Shift**
```cpp
// portPosDiff = srcPort.y + srcAnchor - tgtPort.y - tgtAnchor
innerShift[next] = innerShift[curr] + portPosDiff;
```

**Step 4: Block Compaction**
- 递归放置 block，考虑 innerShift

**Step 5: 选择/合并布局**
- 如果 fixedAlignment != BALANCED: 选择最优
- 否则: 4-pass median

### 3.2 修复 Y 坐标计算
当前 C++ 的 BK 实现缺少 `type 1 conflict marking`，需要加入。

---

## Phase 4: 实现正交路由

### 4.1 实现 `orthogonalRoute(ElkGraph*, LayoutOptions)`

遵循 Java `OrthogonalRoutingGenerator.java`:

**Step 1: Create HyperEdge Segments**
```cpp
// 为每个 port 创建 HyperEdgeSegment
// 收集 outgoingCoords 和 incomingCoords
```

**Step 2: Build Dependency Graph**
```cpp
// 对每对 segment:
// - 检查 critical conflict (阈值: 0.2 * minimumDistance)
// - 检查普通 conflict (阈值: 0.5 * edgeSpacing)
// - 计算 penalty = CONFLICT_PENALTY * conflicts + CROSSING_PENALTY * crossings
// - 创建带权重的依赖边
```

**Step 3: Break Critical Cycles**
```cpp
// Tarjan 或 DFS 找环
// 随机打破循环 (用 random seed)
```

**Step 4: Topological Numbering**
```cpp
// 从 source 到 target 拓扑排序
// 分配 topological index
```

**Step 5: Assign Routing Slots**
```cpp
// 贪心着色: 按 startCoord 排序，维护 active 堆
```

### 4.2 对比验证
用 `test_metrics` vs `test_metrics_elkjs` 对比:
- edgeCrossings
- waypoints count
- totalEdgeLength
- congestion metrics

---

## Phase 5: 实现 Intermediate Processors

### 5.1 优先级
1. `LABEL_DUMMY_INSERTER` - 中心标签处理
2. `SELF_LOOP_PROCESSOR` - 自环边处理
3. `HYPEREDGE_DUMMY_MERGER` - Hyperedge dummy 合并
4. `NORTH_SOUTH_PORT_PREPROCESSOR` - 北/南端口处理

---

## 测试策略

### 单元测试
| 测试 | 文件 | 说明 |
|------|------|------|
| 分层测试 | `test_layering.cpp` | 相同 DAG，C++ vs Java 分层结果一致 |
| BK 测试 | `test_bk_placement.cpp` | Y 坐标误差 < 0.01 |
| 路由测试 | `test_routing.cpp` | Bend points 一致 |

### 回归测试
```bash
# 对每个 netlist 运行对比
for netlist in test_circuit.v complex.v aes_cipher_top.v; do
    ./build/test_compare_elk $netlist
done
```

### 验收标准
- `nodePositionDiffAvg < 1.0` (坐标误差 < 1 pixel)
- `edgeBendPointDiff < 5%`
- `crossingDiff == 0` (crossing 数相同)
- 所有 hard invariants 保持 0

---

## 实现顺序

```
Week 1: Phase 1 - 入口 + 回归测试框架
Week 2: Phase 2 - LongestPathLayering
Week 3: Phase 3 - BK Node Placement (Y坐标)
Week 4: Phase 3 - BK Node Placement (X坐标 + 整合)
Week 5: Phase 4 - Orthogonal Routing
Week 6: Phase 5 - Intermediate Processors
Week 7: 完整回归测试 + bug 修复
Week 8: 最终验证
```

---

## 文件变更清单

### 新建文件
- `src/elk/elk_layered.cpp` - 实现三个 declared 函数
- `test_compare_elk.cpp` - C++ vs Java 对比工具
- `test_layering.cpp` - 分层算法单元测试

### 修改文件
- `src/elk/elk_circuit_adapter.hpp` - 修复调用 `elk::layout()`
- `src/elk/elk_circuit_layout.cpp` - 重构为调用标准 pipeline
- `CMakeLists.txt` - 添加新文件
- `run_regression.sh` - 添加 `--compare-java` 选项

---

## 风险和缓解

| 风险 | 缓解 |
|------|------|
| DFS 递归深度导致 stack overflow | 改用迭代实现或增加 stack size |
| 浮点精度导致 Y 坐标差异 | 容差比较，或对关键计算使用定点数 |
| 测试数据不足 | 用 10+ netlist 覆盖各种场景 |