const ELK = require('elkjs');

const elk = new ELK();

// Read JSON from stdin
let inputData = '';
process.stdin.on('data', chunk => {
  inputData += chunk;
});

process.stdin.on('end', () => {
  try {
    const t0 = Date.now();
    const graph = JSON.parse(inputData);
    const t1 = Date.now();
    const nodeCount = (graph.children || []).length;
    const edgeCount = (graph.edges || []).length;
    process.stderr.write(`[ELKJS] JSON parsed: ${t1-t0}ms  nodes=${nodeCount} edges=${edgeCount} inputSize=${inputData.length}\n`);

    elk.layout(graph)
      .then(g => {
        const t2 = Date.now();
        process.stderr.write(`[ELKJS] layout done: ${t2-t1}ms\n`);
        console.log(JSON.stringify(g));
      })
      .catch(err => {
        process.stderr.write(`[ELKJS] layout error: ${err}\n`);
        console.error(err);
      });
  } catch (e) {
    console.error(e);
    process.exit(1);
  }

});