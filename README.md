# Circuit Schematic Demo with Qt + ELK

## Netlist Format (Simple SPICE-like)

```
* Comment line
R1 in  gnd  1k
R2 out gnd  2k
C1 in  out  10n
V1  in  gnd  5
```

Format: `<component_type><index> <node1> <node2> [value]`

Component types:
- R - Resistor
- C - Capacitor
- L - Inductor
- V - Voltage source
- I - Current source
- D - Diode
- Q - Transistor
