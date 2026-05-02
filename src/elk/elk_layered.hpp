/**
 * ELK C++ Port - Layered Layout Algorithm
 * Migration from org.eclipse.elk.alg.layered
 */

#ifndef ELK_LAYERED_HPP
#define ELK_LAYERED_HPP

#include "elk_lgraph.hpp"

namespace elk {

// Layering
void longestPathLayering(LGraph* lgraph);

// Node placement
void brandesKoepfPlace(LGraph* lgraph, const LayoutOptions& options);

// Edge routing  
void orthogonalRoute(ElkGraph* graph, const LayoutOptions& options);

} // namespace elk

#endif // ELK_LAYERED_HPP
