// Cell.hpp - Represents a single cell in the warehouse grid.
#pragma once
#include <cstdint>

namespace warehouse {

/// Type of content in a grid cell.
enum class CellType : uint8_t {
    FREE     = 0,  ///< Navigable empty floor
    OBSTACLE = 1,  ///< Static obstacle (shelf, wall)
    DYNAMIC  = 2,  ///< Occupied by dynamic obstacle (transient, not stored)
};

/// A single cell in the 2-D warehouse grid.
struct Cell {
    int     col{0};
    int     row{0};
    CellType type{CellType::FREE};

    /// Cost weight — reserved for terrain costs (always 1.0 in base version).
    float   cost{1.0f};

    Cell() = default;
    Cell(int c, int r, CellType t = CellType::FREE)
        : col(c), row(r), type(t) {}

    [[nodiscard]] bool isObstacle() const noexcept { return type == CellType::OBSTACLE; }
    [[nodiscard]] bool isFree()     const noexcept { return type == CellType::FREE; }
};

}  // namespace warehouse
