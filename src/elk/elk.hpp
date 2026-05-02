/**
 * ELK C++ Port - Aggregated Include
 * Migration from org.eclipse.elk.graph (Eclipse Layout Kernel)
 */

#ifndef ELK_HPP
#define ELK_HPP

// Core data structures
#include "elk_types.hpp"
#include "elk_shape.hpp"
#include "elk_port.hpp"
#include "elk_node.hpp"
#include "elk_edge.hpp"
#include "elk_graph.hpp"
#include "elk_lgraph.hpp"
#include "elk_layered.hpp"

namespace elk {

// Version
constexpr const char* VERSION = "0.1.0";
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 1;
constexpr int VERSION_PATCH = 0;

} // namespace elk

#endif // ELK_HPP