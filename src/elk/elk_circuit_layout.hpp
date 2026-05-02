/**
 * ELK C++ Port - CircuitGraph Layout Declaration
 */

#ifndef ELK_CIRCUIT_LAYOUT_HPP
#define ELK_CIRCUIT_LAYOUT_HPP

// Forward declaration to avoid including circuitgraph.h
class CircuitGraph;

// Declare the function - implementation in elk_circuit_layout.cpp
void applyElkLayout(CircuitGraph& cg);

#endif // ELK_CIRCUIT_LAYOUT_HPP