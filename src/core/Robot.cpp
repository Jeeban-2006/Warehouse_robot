// Robot.cpp - Autonomous robot implementation.
#include "core/Robot.hpp"
#include <algorithm>
#include <cmath>

namespace warehouse {

Robot::Robot(int id_,
             std::pair<int,int> startPos,
             std::pair<int,int> goalPos,
             int cellSize,
             float baseSpeed)
    : id(id_),
      m_col(startPos.first), m_row(startPos.second),
      m_startPos(startPos), m_goalPos(goalPos),
      m_cellSize(cellSize), m_baseSpeed(baseSpeed)
{
    m_pixelX = centreX(m_col);
    m_pixelY = centreY(m_row);
}

float Robot::centreX(int c) const noexcept {
    return static_cast<float>(c * m_cellSize) + m_cellSize * 0.5f;
}

float Robot::centreY(int r) const noexcept {
    return static_cast<float>(r * m_cellSize) + m_cellSize * 0.5f;
}

void Robot::setPath(const std::vector<std::pair<int,int>>& path) {
    m_path      = path;
    m_pathIndex = 1;       // index 0 is the current cell
    m_progress  = 0.0f;

    if (path.size() > 1) {
        m_state = RobotState::MOVING;
    } else if (path.size() == 1 && path[0] == m_goalPos) {
        m_state = RobotState::REACHED_GOAL;
    } else {
        m_state = RobotState::IDLE;
    }
}

void Robot::clearPath() {
    m_path.clear();
    m_pathIndex = 0;
    m_progress  = 0.0f;
}

std::vector<std::pair<int,int>> Robot::remainingPath() const {
    if (m_pathIndex >= static_cast<int>(m_path.size()))
        return {};
    return {m_path.begin() + m_pathIndex, m_path.end()};
}

void Robot::update(float dt, float speedMultiplier) {
    if (m_state != RobotState::MOVING) return;
    if (m_path.empty() || m_pathIndex >= static_cast<int>(m_path.size())) {
        onPathComplete();
        return;
    }

    float effectiveSpeed = m_baseSpeed * speedMultiplier;
    m_progress += effectiveSpeed * dt;

    // Commit to cells
    while (m_progress >= 1.0f &&
           m_pathIndex < static_cast<int>(m_path.size()))
    {
        m_progress -= 1.0f;
        auto [nc, nr] = m_path[static_cast<std::size_t>(m_pathIndex)];
        int dc = nc - m_col, dr = nr - m_row;
        if (dc != 0 || dr != 0)
            m_facing = {dc, dr};
        m_col = nc;
        m_row = nr;
        ++m_pathIndex;

        if (m_pathIndex >= static_cast<int>(m_path.size())) {
            onPathComplete();
            break;
        }
    }

    // Smooth pixel interpolation
    if (m_pathIndex < static_cast<int>(m_path.size())) {
        auto [nc, nr] = m_path[static_cast<std::size_t>(m_pathIndex)];
        float tx = centreX(nc), ty = centreY(nr);
        float cx = centreX(m_col), cy = centreY(m_row);
        float t  = std::min(m_progress, 1.0f);
        m_pixelX = cx + (tx - cx) * t;
        m_pixelY = cy + (ty - cy) * t;
    } else {
        syncPixels();
    }
}

void Robot::onPathComplete() {
    syncPixels();
    m_progress = 0.0f;
    m_state = (pos() == m_goalPos) ? RobotState::REACHED_GOAL : RobotState::IDLE;
}

void Robot::syncPixels() {
    m_pixelX = centreX(m_col);
    m_pixelY = centreY(m_row);
}

void Robot::pause() {
    if (m_state == RobotState::MOVING)
        m_state = RobotState::PAUSED;
}

void Robot::resume() {
    if (m_state == RobotState::PAUSED && hasPath())
        m_state = RobotState::MOVING;
}

void Robot::setBlocked()   { m_state = RobotState::BLOCKED;    }
void Robot::setPlanning()  { m_state = RobotState::PLANNING;   }
void Robot::setNoPath()    { clearPath(); m_state = RobotState::ERROR; }

void Robot::reset() {
    m_col  = m_startPos.first;
    m_row  = m_startPos.second;
    syncPixels();
    clearPath();
    m_state  = RobotState::IDLE;
    m_facing = {1, 0};
}

void Robot::setGoalPos(std::pair<int,int> g) {
    m_goalPos = g;
    clearPath();
    m_state = RobotState::IDLE;
}

void Robot::setStartPos(std::pair<int,int> s) {
    m_startPos = s;
    m_col = s.first;
    m_row = s.second;
    syncPixels();
    clearPath();
    m_state = RobotState::IDLE;
}

}  // namespace warehouse
