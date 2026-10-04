#include "core/Robot.hpp"
#include <cmath>

namespace warehouse {

std::string robotStateStr(RobotState s) {
    switch (s) {
        case RobotState::IDLE: return "IDLE";
        case RobotState::PLANNING: return "PLANNING";
        case RobotState::MOVING: return "MOVING";
        case RobotState::PAUSED: return "PAUSED";
        case RobotState::BLOCKED: return "BLOCKED";
        case RobotState::REPLANNING: return "REPLANNING";
        case RobotState::PICKING: return "PICKING";
        case RobotState::DELIVERING: return "DELIVERING";
        case RobotState::CHARGING: return "CHARGING";
        case RobotState::REACHED_GOAL: return "REACHED_GOAL";
        case RobotState::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

Robot::Robot(int id, std::pair<int,int> start, std::pair<int,int> goal, int cellSize, float speed)
    : m_id(id), m_col(start.first), m_row(start.second),
      m_startPos(start), m_goalPos(goal),
      m_cellSize(cellSize), m_speed(speed),
      m_state(RobotState::IDLE), m_prevState(RobotState::IDLE),
      m_pathIndex(0), m_progress(0.0f), m_battery(100.0f) {}

float Robot::centreX() const {
    if (m_path.empty() || m_pathIndex >= m_path.size() || m_progress == 0.0f) {
        return m_col * m_cellSize + m_cellSize / 2.0f;
    }
    float currX = m_col * m_cellSize + m_cellSize / 2.0f;
    float nextX = m_path[m_pathIndex].first * m_cellSize + m_cellSize / 2.0f;
    return currX + (nextX - currX) * m_progress;
}

float Robot::centreY() const {
    if (m_path.empty() || m_pathIndex >= m_path.size() || m_progress == 0.0f) {
        return m_row * m_cellSize + m_cellSize / 2.0f;
    }
    float currY = m_row * m_cellSize + m_cellSize / 2.0f;
    float nextY = m_path[m_pathIndex].second * m_cellSize + m_cellSize / 2.0f;
    return currY + (nextY - currY) * m_progress;
}

void Robot::setStartPos(std::pair<int,int> p) {
    m_startPos = p;
    m_col = p.first;
    m_row = p.second;
    clearPath();
    m_state = RobotState::IDLE;
}

void Robot::setGoalPos(std::pair<int,int> p) {
    m_goalPos = p;
    clearPath();
    m_state = RobotState::IDLE;
}

void Robot::setPath(const std::vector<std::pair<int,int>>& p) {
    m_path = p;
    m_pathIndex = (m_path.size() > 1) ? 1 : 0;
    m_progress = 0.0f;
    if (m_path.empty()) {
        m_state = RobotState::IDLE;
    } else if (m_path.size() == 1 && m_col == m_path[0].first && m_row == m_path[0].second) {
        m_state = RobotState::REACHED_GOAL;
    } else {
        m_state = RobotState::MOVING;
    }
}

void Robot::clearPath() {
    m_path.clear();
    m_pathIndex = 0;
    m_progress = 0.0f;
}

void Robot::drainBattery(float amt) {
    m_battery -= amt;
    if (m_battery < 0.0f) m_battery = 0.0f;
}

void Robot::chargeBattery(float amt) {
    m_battery += amt;
    if (m_battery > 100.0f) m_battery = 100.0f;
}

void Robot::assignTask(std::shared_ptr<Task> task) {
    m_currentTask = task;
    if (m_currentTask) {
        m_currentTask->status = TaskStatus::ASSIGNED;
    }
}

void Robot::pause() {
    if (m_state == RobotState::MOVING) {
        m_prevState = m_state;
        m_state = RobotState::PAUSED;
    }
}

void Robot::resume() {
    if (m_state == RobotState::PAUSED) {
        m_state = m_prevState;
    }
}

void Robot::reset() {
    m_col = m_startPos.first;
    m_row = m_startPos.second;
    clearPath();
    m_state = RobotState::IDLE;
    m_battery = 100.0f;
    if (m_currentTask) {
        m_currentTask->status = TaskStatus::CREATED;
        m_currentTask = nullptr;
    }
}

void Robot::update(float dt, float speedMultiplier) {
    if (m_state == RobotState::CHARGING) {
        chargeBattery(dt * 10.0f); // 10% per second
        if (m_battery >= 100.0f) {
            m_battery = 100.0f;
            m_state = RobotState::IDLE;
        }
        return;
    }

    if (m_state == RobotState::ERROR) return;
    if (m_state != RobotState::MOVING) return;
    if (m_path.empty() || m_pathIndex >= m_path.size()) return;

    // Movement drains battery (Lowered to allow full warehouse traversal)
    drainBattery(dt * 0.5f);
    if (m_battery <= 0.0f) {
        m_state = RobotState::ERROR;
        return;
    }

    float actualSpeed = m_speed * speedMultiplier;
    m_progress += actualSpeed * dt;

    if (m_progress >= 1.0f) {
        m_progress -= 1.0f;
        m_col = m_path[m_pathIndex].first;
        m_row = m_path[m_pathIndex].second;
        m_pathIndex++;

        if (m_pathIndex >= m_path.size()) {
            m_progress = 0.0f;
            m_state = RobotState::REACHED_GOAL;
        }
    }
}

} // namespace warehouse
