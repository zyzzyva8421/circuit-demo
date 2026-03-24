const ELK = require('elkjs');

const elk = new ELK();

// Read JSON from stdin
let inputData = '';
process.stdin.on('data', chunk => {
  inputData += chunk;
});

process.stdin.on('end', () => {
  try {
    const graph = JSON.parse(inputData);
    
    // console.error(JSON.stringify(graph, null, 2)); 
    
    elk.layout(graph)
      .then(g => {
        console.log(JSON.stringify(g));
      })
      .catch(console.error);
  } catch (e) {
    console.error(e);
    process.exit(1);
  }

});