#pragma once
#include <string>
#include <utility>

namespace warehouse {

enum class CellType {
    FREE,
    OBSTACLE,       // General static obstacle
    DYNAMIC,        // Occupied by dynamic obstacle
    SHELF,          // Storage shelf
    CHARGING_STATION,
    LOADING_ZONE,
    PICKUP_STATION
};

struct Cell {
    CellType type{CellType::FREE};
    bool occupied{false};   // true if robot or dynamic obstacle is currently here
    
    // Cost modifier for pathfinding (e.g., aisles might be cheaper)
    float costMultiplier{1.0f};
};

} // namespace warehouse
