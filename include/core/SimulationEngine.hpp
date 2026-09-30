// SimulationEngine.hpp - Top-level simulation orchestrator.
#pragma once

#include "Grid.hpp"
#include "Robot.hpp"
#include "DynamicObstacle.hpp"
#include "AStarPlanner.hpp"
#include "Metrics.hpp"

#include <vector>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <functional>

namespace warehouse {

enum class SimState {
    IDLE,
    PLANNING,
    RUNNING,
    PAUSED,
    COMPLETED,
    NO_PATH,
    ERROR
};

[[nodiscard]] inline const char* simStateStr(SimState s) {
    switch(s) {
        case SimState::IDLE:      return "IDLE";
        case SimState::PLANNING:  return "PLANNING";
        case SimState::RUNNING:   return "RUNNING";
        case SimState::PAUSED:    return "PAUSED";
        case SimState::COMPLETED: return "COMPLETED";
        case SimState::NO_PATH:   return "NO_PATH";
        case SimState::ERROR:     return "ERROR";
        default:                  return "UNKNOWN";
    }
}

struct SimConfig {
    int   gridCols{30};
    int   gridRows{22};
    int   cellSize{32};
    float robotBaseSpeed{4.0f};
    float replanInterval{0.25f};
    float dynObsSpeed{2.0f};
    int   numDynObs{3};
    float densityLow{0.10f};
    float densityMedium{0.20f};
    float densityHigh{0.32f};
};

class SimulationEngine {
public:
    explicit SimulationEngine(SimConfig cfg = {});

    // ── Control ───────────────────────────────────────────────────────────
    void start();
    void pause();
    void resume();
    void reset();
    void fullReset();
    void findPathOnly();

    // ── Update (called per frame) ─────────────────────────────────────────
    /// dt = delta time in seconds.
    void update(float dt);

    // ── Map management ────────────────────────────────────────────────────
    void loadDefaultMap();
    void generateRandomMap(float density);
    void clearObstacles();
    void setObstacle(int col, int row, bool value);
    void removeObstacle(int col, int row);

    // ── Robot/Goal repositioning ──────────────────────────────────────────
    void setRobotStart(int col, int row);
    void setRobotGoal (int col, int row);

    // ── Read-only access for renderer ─────────────────────────────────────
    [[nodiscard]] const Grid& grid() const noexcept;
    [[nodiscard]] const Robot& robot() const noexcept;
    [[nodiscard]] const std::vector<std::unique_ptr<DynamicObstacle>>& dynObs() const noexcept;
    [[nodiscard]] const Metrics& metrics() const noexcept;
    [[nodiscard]] SimState state() const noexcept;
    [[nodiscard]] const std::string& statusMessage() const noexcept;
    [[nodiscard]] const PathResult& lastPathResult() const noexcept;

    // ── Settings ──────────────────────────────────────────────────────────
    float speedMultiplier{1.0f};
    bool  dynamicObsEnabled{true};
    bool  showPath{true};
    bool  showExplored{false};
    bool  debugMode{false};
    float density{0.20f};

    // ── Replanning banner ─────────────────────────────────────────────────
    [[nodiscard]] bool replanningBanner() const noexcept { return m_replanBanner; }

    // ── Sensor data (for driver interface) ───────────────────────────────
    /// Returns Manhattan distance to nearest obstacle in each direction.
    /// Returns -1 if direction is free up to the grid edge.
    struct SensorReading {
        int front{-1}, rear{-1}, left{-1}, right{-1};
        bool obstacleDetected{false};
        int  seq{0};
    };
    [[nodiscard]] SensorReading computeSensorReading() const;

    // ── Thread safety ─────────────────────────────────────────────────────
    /// Lock before reading state from a different thread.
    mutable std::mutex stateMutex;

private:
    SimConfig m_cfg;

    // ── Core entities (protected by stateMutex where needed) ──────────────
    std::unique_ptr<Grid>   m_grid;
    std::unique_ptr<Robot>  m_robot;
    std::vector<std::unique_ptr<DynamicObstacle>> m_dynObs;
    std::unique_ptr<AStarPlanner> m_planner;

    SimState    m_state{SimState::IDLE};
    Metrics     m_metrics;
    PathResult  m_lastPathResult;
    std::string m_statusMsg{"Press START to begin."};

    // ── Replanning timer ──────────────────────────────────────────────────
    float m_replanTimer{0.0f};
    float m_replanBannerTimer{0.0f};
    bool  m_replanBanner{false};
    mutable int   m_sensorSeq{0};

    // ── Internal helpers ──────────────────────────────────────────────────
    void doPlanning();
    void updateDynObs(float dt);
    void checkAndReplan(float dt);
    void checkCompletion();
    PosSet dynObsPositions() const;
    bool isCurrentPathValid() const;
    void createDefaultDynObs();
    void createRandomDynObs();
};

}  // namespace warehouse
