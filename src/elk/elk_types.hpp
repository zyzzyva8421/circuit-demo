/**
 * ELK C++ Port - Data Structures
 * Migration from org.eclipse.elk.graph (Eclipse Layout Kernel)
 * EPL-2.0 License
 */

#ifndef ELK_TYPES_HPP
#define ELK_TYPES_HPP

#include <string>
#include <cstddef>

namespace elk {

// Port side enumeration
enum class PortSide {
    UNDEFINED,
    NORTH,
    SOUTH,
    EAST,
    WEST
};

// Direction utility
inline PortSide opposite(PortSide side) {
    switch (side) {
        case PortSide::NORTH: return PortSide::SOUTH;
        case PortSide::SOUTH: return PortSide::NORTH;
        case PortSide::EAST:  return PortSide::WEST;
        case PortSide::WEST:  return PortSide::EAST;
        default: return PortSide::UNDEFINED;
    }
}

inline std::string toString(PortSide side) {
    switch (side) {
        case PortSide::NORTH: return "NORTH";
        case PortSide::SOUTH: return "SOUTH";
        case PortSide::EAST:  return "EAST";
        case PortSide::WEST:  return "WEST";
        default: return "UNDEFINED";
    }
}

// Algorithm identifiers
enum class Algorithm {
    LAYERED,
    BOX,
    FORCE,
    DISCO,
    RECTPACKING,
    RADIAL,
    MRTREE,
    SPORE,
    VERTIFLEX,
    GRAPHVIZ_DOT
};

// Node placement strategy
enum class NodePlacementStrategy {
    SIMPLEX,
    BRANDES_KOEPF,
    LINEAR_SEGMENTS,
    NETWORK_SIMPLEX,
    FIXED_ALIGNMENT,
    undefined
};

// Layering constraint (for port ordering)
enum class LayeringConstraint {
    FIRST,
    LAST,
    ANY
};

// Node size constraints
enum class NodeSizeConstraints {
    FIXED,
    MINIMUM_SIZE,
    MAXIMUM_SIZE,
    NONE
};

// Port constraints
enum class PortConstraints {
    FIXED_POS,
    FREE,
    FIXED_SIDE,
    undefined
};

// Direction
enum class Direction {
    RIGHT,
    LEFT,
    UP,
    DOWN
};

// Fixed alignment
enum class FixedAlignment {
    TERMINAL_ALIGNED,
    LEFT_ALIAS,
    RIGHT_ALIAS,
    undefined
};

} // namespace elk

#endif // ELK_TYPES_HPP