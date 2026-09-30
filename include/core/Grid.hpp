#pragma once
#include <vector>
#include <memory>
#include "Cell.hpp"
#include "WarehouseObject.hpp"

namespace warehouse {

class Grid {
public:
    Grid(int cols, int rows);

    int cols() const { return m_cols; }
    int rows() const { return m_rows; }
    bool inBounds(int c, int r) const;

    Cell& getCell(int c, int r);
    const Cell& getCell(int c, int r) const;

    bool isFree(int c, int r) const;
    bool isObstacle(int c, int r) const;
    
    // Core object management
    void setObstacle(int c, int r, bool isObs = true);
    void placeObject(std::shared_ptr<WarehouseObject> obj);
    std::shared_ptr<WarehouseObject> getObjectAt(int c, int r) const;
    void clear();

    // Map generators
    void generateRealisticWarehouse();
    void generateRandom(float density, const std::vector<std::pair<int,int>>& protectedCells = {});
    
    // A* Helpers
    std::vector<std::pair<int,int>> neighbours(int c, int r) const;
    std::vector<std::pair<int,int>> freeCells() const;
    int obstacleCount() const;

private:
    int m_cols;
    int m_rows;
    std::vector<Cell> m_cells;
    std::vector<std::shared_ptr<WarehouseObject>> m_objects;

    int idx(int c, int r) const { return r * m_cols + c; }
};

} // namespace warehouse
