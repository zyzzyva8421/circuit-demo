const ELK = require('elkjs');
const fs = require('fs');

const elk = new ELK();

const input = fs.readFileSync('reproduce_elk.json', 'utf8');
const graph = JSON.parse(input);

elk.layout(graph)
  .then(g => {
    const e = g.edges[0];
    const s = e.sections[0];
    const n1 = g.children.find(n => n.id === 'inst_cpu') || g.children[0];
    const n2 = g.children.find(n => n.id === 'inst_mem') || g.children[1];
    
    console.log('N1:', n1.id, n1.x, n1.y);
    console.log('N2:', n2.id, n2.x, n2.y);
    
    console.log('Edge Start:', s.startPoint);
    console.log('Edge End:', s.endPoint);
    
    if(n1.ports) n1.ports.forEach(p => console.log('N1 Port:', p.id, p.x, p.y));
    if(n2.ports) n2.ports.forEach(p => console.log('N2 Port:', p.id, p.x, p.y));
  })
  .catch(console.error);
