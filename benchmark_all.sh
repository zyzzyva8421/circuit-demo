#!/bin/bash

# Comprehensive benchmark: C++ ELK, elkjs full, elkjs placement-only
INPUT="${1:-1_2_yosys.v}"

echo "=== Benchmark: $INPUT ==="
echo

# Parse with netlist parser to create layout-ready graph
LAYOUT_JSON=$(mktemp)
node run_repro.js "$INPUT" > "$LAYOUT_JSON" 2>/dev/null

if [ ! -s "$LAYOUT_JSON" ]; then
  echo "Error: Failed to generate layout JSON"
  rm -f "$LAYOUT_JSON"
  exit 1
fi

echo "--- C++ ELK ---"
time ./build/test_metrics "$INPUT" 2>&1 | grep -E "Edge-Edge Crossings|Topological Crossings|Avg Waypoints"
echo

echo "--- elkjs (full layout with routing) ---"
time node layout_script.js < "$LAYOUT_JSON" > /tmp/elkjs_full.json 2>&1
if [ $? -eq 0 ]; then
  # Parse output to get timing and compute metrics
  node run_repro.js "$INPUT" --layout-json=/tmp/elkjs_full.json > /dev/null 2>&1
  ./build/test_metrics_elkjs "$INPUT" 2>&1 | grep -E "Edge-Edge Crossings|Topological Crossings|Avg Waypoints"
fi
echo

echo "--- elkjs (placement only, no routing) ---"
time node test_placement_only.js < "$LAYOUT_JSON" > /tmp/elkjs_placement.json 2>&1
echo

rm -f "$LAYOUT_JSON"
