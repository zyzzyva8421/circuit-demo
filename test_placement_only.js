// test_placement_only.js
// Runs elkjs with ONLY placement (no edge routing)
const ELK = require('elkjs');
const fs = require('fs');

const elk = new ELK();

let inputData = '';
process.stdin.on('data', chunk => {
  inputData += chunk;
});

process.stdin.on('end', () => {
  try {
    const graph = JSON.parse(inputData);
    
    // Disable edge routing completely - only do placement
    // Set edgeRouting to UNDEFINED to skip routing step
    if (graph.layoutOptions) {
      delete graph.layoutOptions['elk.edgeRouting'];
    }
    
    // Clear all edge routing on all edges recursively
    function clearRouting(node) {
      if (node.edges) {
        node.edges.forEach(e => {
          delete e['layoutOptions'];
          delete e['points'];
        });
      }
      if (node.children) {
        node.children.forEach(clearRouting);
      }
    }
    clearRouting(graph);
    
    const start = Date.now();
    elk.layout(graph)
      .then(g => {
        const elapsed = Date.now() - start;
        console.error(`Placement-only time: ${elapsed}ms`);
        console.log(JSON.stringify(g));
      })
      .catch(console.error);
  } catch (e) {
    console.error(e);
    process.exit(1);
  }
});
