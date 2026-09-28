// DynamicObstacle.cpp - Bouncing dynamic obstacle implementation.
#include "core/DynamicObstacle.hpp"
#include <cmath>
#include <algorithm>

namespace warehouse {

DynamicObstacle::DynamicObstacle(int id_,
                                  int col, int row,
                                  ObstacleAxis axis,
                                  int minPos, int maxPos,
                                  float speed, int cellSize)
    : id(id_),
      m_col(col), m_row(row),
      m_axis(axis), m_minPos(minPos), m_maxPos(maxPos),
      m_speed(speed), m_cellSize(cellSize),
      m_initCol(col), m_initRow(row)
{
    m_pixelX = centreX(m_col);
    m_pixelY = centreY(m_row);
}

float DynamicObstacle::centreX(int c) const noexcept {
    return static_cast<float>(c * m_cellSize) + m_cellSize * 0.5f;
}

float DynamicObstacle::centreY(int r) const noexcept {
    return static_cast<float>(r * m_cellSize) + m_cellSize * 0.5f;
}

bool DynamicObstacle::update(float dt, float speedMultiplier) {
    float effective = m_speed * speedMultiplier;
    m_progress += effective * dt;
    bool moved = false;

    while (m_progress >= 1.0f) {
        m_progress -= 1.0f;

        int nextCol = m_col, nextRow = m_row;
        int posVal;

        if (m_axis == ObstacleAxis::HORIZONTAL) {
            nextCol = m_col + m_direction;
            posVal  = nextCol;
        } else {
            nextRow = m_row + m_direction;
            posVal  = nextRow;
        }

        // Bounce
        if (posVal < m_minPos || posVal > m_maxPos) {
            m_direction = -m_direction;
            if (m_axis == ObstacleAxis::HORIZONTAL)
                nextCol = m_col + m_direction;
            else
                nextRow = m_row + m_direction;
        }

        // Clamp
        nextCol = std::clamp(nextCol, 0, 9999);
        nextRow = std::clamp(nextRow, 0, 9999);

        m_col = nextCol;
        m_row = nextRow;
        moved = true;
    }

    updatePixels();
    return moved;
}

void DynamicObstacle::updatePixels() {
    // Smooth interpolation toward next cell
    float targetX = centreX(m_col);
    float targetY = centreY(m_row);

    int nextCol = m_col + (m_axis == ObstacleAxis::HORIZONTAL ? m_direction : 0);
    int nextRow = m_row + (m_axis == ObstacleAxis::VERTICAL   ? m_direction : 0);
    float nextX = centreX(nextCol);
    float nextY = centreY(nextRow);

    m_pixelX = targetX + (nextX - targetX) * m_progress;
    m_pixelY = targetY + (nextY - targetY) * m_progress;
}

void DynamicObstacle::reset(int col, int row) {
    m_col      = col;
    m_row      = row;
    m_progress = 0.0f;
    m_direction = 1;
    m_pixelX   = centreX(col);
    m_pixelY   = centreY(row);
}

}  // namespace warehouse
