// Grid.cpp - 2D warehouse grid implementation.
#include "core/Grid.hpp"
#include <stdexcept>
#include <random>
#include <algorithm>

namespace warehouse {

Grid::Grid(int cols, int rows)
    : m_cols(cols), m_rows(rows),
      m_cells(static_cast<std::size_t>(cols * rows))
{
    if (cols <= 0 || rows <= 0)
        throw std::invalid_argument("Grid dimensions must be positive");
    // Initialize all cells
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            m_cells[static_cast<std::size_t>(idx(c, r))] = Cell(c, r);
}

bool Grid::inBounds(int col, int row) const noexcept {
    return col >= 0 && col < m_cols && row >= 0 && row < m_rows;
}

const Cell& Grid::at(int col, int row) const {
    if (!inBounds(col, row))
        throw std::out_of_range("Grid::at out of bounds");
    return m_cells[static_cast<std::size_t>(idx(col, row))];
}

Cell& Grid::at(int col, int row) {
    if (!inBounds(col, row))
        throw std::out_of_range("Grid::at out of bounds");
    return m_cells[static_cast<std::size_t>(idx(col, row))];
}

bool Grid::isObstacle(int col, int row) const noexcept {
    if (!inBounds(col, row)) return true;   // Out-of-bounds = wall
    return m_cells[static_cast<std::size_t>(idx(col, row))].isObstacle();
}

bool Grid::isFree(int col, int row) const noexcept {
    return !isObstacle(col, row);
}

void Grid::setObstacle(int col, int row, bool value) {
    if (!inBounds(col, row)) return;
    m_cells[static_cast<std::size_t>(idx(col, row))].type =
        value ? CellType::OBSTACLE : CellType::FREE;
}

bool Grid::toggleObstacle(int col, int row) {
    if (!inBounds(col, row)) return false;
    auto& cell = m_cells[static_cast<std::size_t>(idx(col, row))];
    cell.type = (cell.type == CellType::OBSTACLE) ? CellType::FREE : CellType::OBSTACLE;
    return cell.isObstacle();
}

void Grid::clear() {
    for (auto& c : m_cells)
        c.type = CellType::FREE;
}

void Grid::loadDefaultMap(std::pair<int,int> robotPos, std::pair<int,int> goalPos) {
    clear();
    // Shelf blocks: (col_start, col_end, row) — horizontal shelf rows
    struct ShelfRow { int c0, c1, row; };
    static const ShelfRow shelves[] = {
        {3,7,3},{3,7,4},{10,14,3},{10,14,4},{17,21,3},{17,21,4},{24,28,3},{24,28,4},
        {3,7,8},{3,7,9},{10,14,8},{10,14,9},{17,21,8},{17,21,9},{24,28,8},{24,28,9},
        {3,7,13},{3,7,14},{10,14,13},{10,14,14},{17,21,13},{17,21,14},{24,28,13},{24,28,14},
        {3,7,18},{3,7,19},{10,14,18},{10,14,19},{17,21,18},{17,21,19},
    };
    for (auto& s : shelves) {
        for (int c = s.c0; c <= s.c1; ++c) {
            if (!inBounds(c, s.row)) continue;
            if (std::make_pair(c, s.row) == robotPos) continue;
            if (std::make_pair(c, s.row) == goalPos)  continue;
            setObstacle(c, s.row);
        }
    }
}

void Grid::generateRandom(float density,
                           const std::vector<std::pair<int,int>>& protected_cells)
{
    clear();
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            auto pos = std::make_pair(c, r);
            bool prot = std::find(protected_cells.begin(),
                                  protected_cells.end(), pos) != protected_cells.end();
            if (!prot && dist(rng) < density)
                setObstacle(c, r);
        }
    }
}

int Grid::obstacleCount() const noexcept {
    int n = 0;
    for (const auto& c : m_cells)
        if (c.isObstacle()) ++n;
    return n;
}

std::vector<std::pair<int,int>> Grid::freeCells() const {
    std::vector<std::pair<int,int>> result;
    result.reserve(m_cells.size());
    for (const auto& c : m_cells)
        if (c.isFree()) result.emplace_back(c.col, c.row);
    return result;
}

std::vector<std::pair<int,int>>
Grid::neighbours(int col, int row, bool diagonal) const {
    static const int dx4[] = {0, 0, -1, 1};
    static const int dy4[] = {-1, 1, 0, 0};
    static const int dx8[] = {0, 0,-1, 1,-1, 1,-1, 1};
    static const int dy8[] = {-1, 1, 0, 0,-1,-1, 1, 1};

    const int* dx = diagonal ? dx8 : dx4;
    const int* dy = diagonal ? dy8 : dy4;
    int n = diagonal ? 8 : 4;

    std::vector<std::pair<int,int>> result;
    result.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        int nc = col + dx[i], nr = row + dy[i];
        if (inBounds(nc, nr) && isFree(nc, nr))
            result.emplace_back(nc, nr);
    }
    return result;
}

}  // namespace warehouse
