#include "core/Grid.hpp"
#include <algorithm>
#include <random>
#include <chrono>

namespace warehouse {

Grid::Grid(int cols, int rows) : m_cols(cols), m_rows(rows) {
    m_cells.resize(cols * rows);
    m_objects.resize(cols * rows, nullptr);
}

bool Grid::inBounds(int c, int r) const {
    return c >= 0 && c < m_cols && r >= 0 && r < m_rows;
}

Cell& Grid::getCell(int c, int r) {
    return m_cells[idx(c, r)];
}

const Cell& Grid::getCell(int c, int r) const {
    return m_cells[idx(c, r)];
}

bool Grid::isFree(int c, int r) const {
    if (!inBounds(c, r)) return false;
    int i = idx(c, r);
    return m_cells[i].type == CellType::FREE || 
           m_cells[i].type == CellType::LOADING_ZONE || 
           m_cells[i].type == CellType::CHARGING_STATION ||
           m_cells[i].type == CellType::PICKUP_STATION;
}

bool Grid::isObstacle(int c, int r) const {
    if (!inBounds(c, r)) return true;
    int i = idx(c, r);
    return m_cells[i].type == CellType::OBSTACLE || 
           m_cells[i].type == CellType::SHELF || 
           m_cells[i].occupied;
}

void Grid::setObstacle(int c, int r, bool isObs) {
    if (!inBounds(c, r)) return;
    int i = idx(c, r);
    if (isObs) {
        m_cells[i].type = CellType::OBSTACLE;
        m_objects[i] = nullptr;
    } else {
        if (m_cells[i].type == CellType::OBSTACLE) {
            m_cells[i].type = CellType::FREE;
        }
    }
}

void Grid::placeObject(std::shared_ptr<WarehouseObject> obj) {
    if (!obj || !inBounds(obj->col, obj->row)) return;
    int i = idx(obj->col, obj->row);
    m_objects[i] = obj;
    m_cells[i].type = obj->getType();
}

std::shared_ptr<WarehouseObject> Grid::getObjectAt(int c, int r) const {
    if (!inBounds(c, r)) return nullptr;
    return m_objects[idx(c, r)];
}

void Grid::clear() {
    for (int i = 0; i < m_cols * m_rows; ++i) {
        m_cells[i] = Cell();
        m_objects[i] = nullptr;
    }
}

void Grid::generateRealisticWarehouse(int layoutType) {
    clear();
    
    // Top and Bottom Aisle margins
    int marginX = 4;
    int marginY = 4;
    
    // Place Shelves in blocks
    int shelfWidth = 6;
    int shelfHeight = 2;
    int aisleWidth = 3;
    int aisleHeight = 3;
    
    for (int r = marginY; r < m_rows - marginY - shelfHeight; r += shelfHeight + aisleHeight) {
        for (int c = marginX; c < m_cols - marginX - shelfWidth; c += shelfWidth + aisleWidth) {
            for (int sr = 0; sr < shelfHeight; ++sr) {
                for (int sc = 0; sc < shelfWidth; ++sc) {
                    placeObject(std::make_shared<Shelf>(c + sc, r + sr, "ShelfBlock"));
                }
            }
        }
    }
    
    // Place Charging Stations on the left wall
    for (int r = 5; r < m_rows - 5; r += 5) {
        placeObject(std::make_shared<ChargingStation>(0, r, "Charger_" + std::to_string(r)));
        placeObject(std::make_shared<ChargingStation>(1, r, "Charger_" + std::to_string(r) + "_B"));
    }
    
    int pickStartC, pickStartR, loadStartC, loadStartR;
    
    if (layoutType == 0) {
        // Layout 0: Pickup Top-Right, Loading Bottom-Right (Original)
        pickStartC = m_cols - 12; pickStartR = 1;
        loadStartC = m_cols - 12; loadStartR = m_rows - 4;
    } else if (layoutType == 1) {
        // Layout 1: Pickup Top-Left, Loading Bottom-Right (Cross Map)
        pickStartC = 2; pickStartR = 1;
        loadStartC = m_cols - 12; loadStartR = m_rows - 4;
    } else {
        // Layout 2: Pickup Bottom-Left, Loading Top-Right (Cross Map 2)
        pickStartC = 2; pickStartR = m_rows - 4;
        loadStartC = m_cols - 12; loadStartR = 1;
    }
    
    // Place Loading Zones
    for (int c = loadStartC; c < loadStartC + 10; ++c) {
        for (int r = loadStartR; r < loadStartR + 3; ++r) {
            // Remove any shelves that might have generated here
            m_cells[idx(c, r)].type = CellType::FREE;
            placeObject(std::make_shared<LoadingZone>(c, r, "LoadingZone"));
        }
    }
    
    // Place Pickup Stations
    for (int c = pickStartC; c < pickStartC + 10; ++c) {
        for (int r = pickStartR; r < pickStartR + 3; ++r) {
            // Remove any shelves that might have generated here
            m_cells[idx(c, r)].type = CellType::FREE;
            placeObject(std::make_shared<PickupStation>(c, r, "PickupZone"));
        }
    }
}

void Grid::generateRandom(float density, const std::vector<std::pair<int,int>>& protectedCells) {
    clear();
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            bool isProtected = false;
            for (const auto& p : protectedCells) {
                if (p.first == c && p.second == r) {
                    isProtected = true;
                    break;
                }
            }
            if (!isProtected && dist(rng) < density) {
                setObstacle(c, r, true);
            }
        }
    }
}

std::vector<std::pair<int,int>> Grid::neighbours(int c, int r) const {
    std::vector<std::pair<int,int>> res;
    res.reserve(4);
    if (c > 0 && !isObstacle(c-1, r)) res.emplace_back(c-1, r);
    if (c < m_cols-1 && !isObstacle(c+1, r)) res.emplace_back(c+1, r);
    if (r > 0 && !isObstacle(c, r-1)) res.emplace_back(c, r-1);
    if (r < m_rows-1 && !isObstacle(c, r+1)) res.emplace_back(c, r+1);
    return res;
}

std::vector<std::pair<int,int>> Grid::freeCells() const {
    std::vector<std::pair<int,int>> res;
    for (int r = 0; r < m_rows; ++r) {
        for (int c = 0; c < m_cols; ++c) {
            if (isFree(c, r)) res.emplace_back(c, r);
        }
    }
    return res;
}

int Grid::obstacleCount() const {
    int count = 0;
    for (const auto& cell : m_cells) {
        if (cell.type == CellType::OBSTACLE || cell.type == CellType::SHELF) count++;
    }
    return count;
}

} // namespace warehouse
