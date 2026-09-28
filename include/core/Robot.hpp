// Robot.hpp - Autonomous warehouse robot model.
#pragma once

#include <utility>
#include <vector>
#include <atomic>
#include <string>

namespace warehouse {

/// Robot state machine states.
enum class RobotState {
    IDLE,
    PLANNING,
    MOVING,
    PAUSED,
    BLOCKED,
    REPLANNING,
    REACHED_GOAL,
    ERROR
};

[[nodiscard]] inline const char* robotStateStr(RobotState s) {
    switch (s) {
        case RobotState::IDLE:         return "IDLE";
        case RobotState::PLANNING:     return "PLANNING";
        case RobotState::MOVING:       return "MOVING";
        case RobotState::PAUSED:       return "PAUSED";
        case RobotState::BLOCKED:      return "BLOCKED";
        case RobotState::REPLANNING:   return "REPLANNING";
        case RobotState::REACHED_GOAL: return "REACHED_GOAL";
        case RobotState::ERROR:        return "ERROR";
        default:                       return "UNKNOWN";
    }
}

/// Autonomous warehouse robot.
class Robot {
public:
    /// Unique robot identifier.
    const int id;

    Robot(int id,
          std::pair<int,int> startPos,
          std::pair<int,int> goalPos,
          int cellSize   = 32,
          float baseSpeed = 4.0f);

    // ── Grid position ─────────────────────────────────────────────────────
    [[nodiscard]] int col() const noexcept { return m_col; }
    [[nodiscard]] int row() const noexcept { return m_row; }
    [[nodiscard]] std::pair<int,int> pos() const noexcept { return {m_col, m_row}; }
    [[nodiscard]] std::pair<int,int> startPos() const noexcept { return m_startPos; }
    [[nodiscard]] std::pair<int,int> goalPos()  const noexcept { return m_goalPos; }

    // ── Pixel position (for smooth rendering) ────────────────────────────
    [[nodiscard]] float pixelX() const noexcept { return m_pixelX; }
    [[nodiscard]] float pixelY() const noexcept { return m_pixelY; }

    // ── Facing direction ─────────────────────────────────────────────────
    [[nodiscard]] std::pair<int,int> facing() const noexcept { return m_facing; }

    // ── State ─────────────────────────────────────────────────────────────
    [[nodiscard]] RobotState state() const noexcept { return m_state; }
    [[nodiscard]] bool atGoal()   const noexcept { return pos() == m_goalPos; }
    [[nodiscard]] bool hasPath()  const noexcept { return !m_path.empty(); }

    // ── Path management ───────────────────────────────────────────────────
    void setPath(const std::vector<std::pair<int,int>>& path);
    void clearPath();
    [[nodiscard]] const std::vector<std::pair<int,int>>& path() const noexcept { return m_path; }
    [[nodiscard]] int pathIndex() const noexcept { return m_pathIndex; }
    [[nodiscard]] std::vector<std::pair<int,int>> remainingPath() const;

    // ── Control ───────────────────────────────────────────────────────────
    void update(float dt, float speedMultiplier = 1.0f);
    void pause();
    void resume();
    void setBlocked();
    void setPlanning();
    void setNoPath();
    void reset();

    // ── Goal/Start repositioning ──────────────────────────────────────────
    void setGoalPos(std::pair<int,int> g);
    void setStartPos(std::pair<int,int> s);

    // ── Speed ─────────────────────────────────────────────────────────────
    void setBaseSpeed(float s) noexcept { m_baseSpeed = s; }
    [[nodiscard]] float baseSpeed() const noexcept { return m_baseSpeed; }

    // ── Cell-size (for pixel conversion) ─────────────────────────────────
    void setCellSize(int cs) noexcept { m_cellSize = cs; }

private:
    int   m_col, m_row;
    float m_pixelX, m_pixelY;
    float m_progress{0.0f};   ///< Interpolation progress to next cell [0,1)
    int   m_pathIndex{0};     ///< Next cell index in path

    std::pair<int,int> m_startPos;
    std::pair<int,int> m_goalPos;
    std::pair<int,int> m_facing{1, 0};

    std::vector<std::pair<int,int>> m_path;
    RobotState m_state{RobotState::IDLE};

    int   m_cellSize;
    float m_baseSpeed;

    // ── Helpers ───────────────────────────────────────────────────────────
    [[nodiscard]] float centreX(int c) const noexcept;
    [[nodiscard]] float centreY(int r) const noexcept;
    void onPathComplete();
    void syncPixels();
};

}  // namespace warehouse
