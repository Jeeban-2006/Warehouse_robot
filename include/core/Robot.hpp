#pragma once
#include <vector>
#include <utility>
#include <memory>
#include <string>
#include "Task.hpp"

namespace warehouse {

enum class RobotState {
    IDLE,
    PLANNING,
    MOVING,
    PAUSED,
    BLOCKED,
    REPLANNING,
    PICKING,
    DELIVERING,
    CHARGING,
    REACHED_GOAL,
    ERROR
};

std::string robotStateStr(RobotState s);

class Robot {
public:
    Robot(int id, std::pair<int,int> start, std::pair<int,int> goal, int cellSize, float speed);

    int id() const { return m_id; }
    int col() const { return m_col; }
    int row() const { return m_row; }
    float centreX() const;
    float centreY() const;
    
    RobotState state() const { return m_state; }
    void setState(RobotState s) { m_state = s; }

    std::pair<int,int> pos() const { return {m_col, m_row}; }
    std::pair<int,int> startPos() const { return m_startPos; }
    std::pair<int,int> goalPos() const { return m_goalPos; }

    void setStartPos(std::pair<int,int> p);
    void setGoalPos(std::pair<int,int> p);
    void setPath(const std::vector<std::pair<int,int>>& p);
    void clearPath();

    bool hasPath() const { return !m_path.empty(); }
    const std::vector<std::pair<int,int>>& path() const { return m_path; }
    size_t pathIndex() const { return m_pathIndex; }

    // Task & Battery
    float batteryPercentage() const { return m_battery; }
    void drainBattery(float amt);
    void chargeBattery(float amt);
    bool needsCharging() const { return m_battery < 20.0f; }

    void assignTask(std::shared_ptr<Task> task);
    void clearTask() { m_currentTask = nullptr; }
    std::shared_ptr<Task> currentTask() const { return m_currentTask; }

    void update(float dt, float speedMultiplier = 1.0f);
    void pause();
    void resume();
    void reset();

private:
    int m_id;
    int m_col, m_row;
    std::pair<int,int> m_startPos;
    std::pair<int,int> m_goalPos;
    int m_cellSize;
    float m_speed;

    RobotState m_state;
    RobotState m_prevState;

    std::vector<std::pair<int,int>> m_path;
    size_t m_pathIndex;
    float m_progress;

    // Battery system
    float m_battery{100.0f};
    
    // Mission system
    std::shared_ptr<Task> m_currentTask{nullptr};
};

} // namespace warehouse
