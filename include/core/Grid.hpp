// Grid.hpp - 2D warehouse grid: obstacle storage, bounds checking, neighbour lookup.
#pragma once

#include "Cell.hpp"
#include <vector>
#include <utility>
#include <cstddef>
#include <functional>

namespace warehouse {

/// 2-D grid of Cell objects.  col = x-axis (columns), row = y-axis (rows).
class Grid {
public:
    Grid(int cols, int rows);

    // ── Dimensions ────────────────────────────────────────────────────────
    [[nodiscard]] int cols() const noexcept { return m_cols; }
    [[nodiscard]] int rows() const noexcept { return m_rows; }

    // ── Bounds ────────────────────────────────────────────────────────────
    [[nodiscard]] bool inBounds(int col, int row) const noexcept;

    // ── Cell access ───────────────────────────────────────────────────────
    [[nodiscard]] const Cell& at(int col, int row) const;
    [[nodiscard]]       Cell& at(int col, int row);

    // ── Obstacle interface ────────────────────────────────────────────────
    [[nodiscard]] bool isObstacle(int col, int row) const noexcept;
    [[nodiscard]] bool isFree    (int col, int row) const noexcept;

    void setObstacle(int col, int row, bool value = true);
    bool toggleObstacle(int col, int row);   ///< Returns new obstacle state
    void clear();

    // ── Map generation ────────────────────────────────────────────────────

    /// Pre-built demo warehouse with shelf rows.
    void loadDefaultMap(std::pair<int,int> robotPos, std::pair<int,int> goalPos);

    /// Random map; protected = cells that must stay free.
    void generateRandom(float density,
                        const std::vector<std::pair<int,int>>& protected_cells);

    // ── Statistics ────────────────────────────────────────────────────────
    [[nodiscard]] int obstacleCount() const noexcept;
    [[nodiscard]] std::vector<std::pair<int,int>> freeCells() const;

    // ── Neighbours ───────────────────────────────────────────────────────
    /// Returns 4-direction (or 8-direction) walkable neighbours.
    [[nodiscard]] std::vector<std::pair<int,int>>
        neighbours(int col, int row, bool diagonal = false) const;

private:
    int m_cols;
    int m_rows;
    std::vector<Cell> m_cells;  ///< Row-major: index = row*cols + col

    [[nodiscard]] int idx(int col, int row) const noexcept {
        return row * m_cols + col;
    }
};

}  // namespace warehouse
