#pragma once
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <string>

#include "Grid.hpp"
#include "Robot.hpp"
#include "Task.hpp"
#include "DynamicObstacle.hpp"
#include "AStarPlanner.hpp"
#include "Metrics.hpp"
#include "SensorProcessing.hpp"

namespace warehouse {

enum class SimState {
    IDLE,
    RUNNING,
    PAUSED,
    NO_PATH,
    COMPLETED
};

std::string simStateStr(SimState s);

struct SimConfig {
    int gridCols = 60;
    int gridRows = 45;
    int cellSize = 16;
    float robotSpeed = 4.0f;
    float replanInterval = 0.25f;
};

class SimulationEngine {
public:
    SimulationEngine(const SimConfig& config);
    ~SimulationEngine() = default;

    // Controls
    void start();
    void pause();
    void resume();
    void reset();
    
    // Core Managers
    void update(float dt);
    
    // Commands
    void setRobotStart(int c, int r);
    void setRobotGoal(int c, int r);
    void assignTask(int pickupCol, int pickupRow, int dropCol, int dropRow);
    void setObstacle(int c, int r, bool isObs);
    void removeObstacle(int c, int r);
    void clearObstacles();
    void loadDefaultMap(int layoutType = 0);
    void generateRandomMap(float density);
    void findPathOnly(); // Used for debug/viz

    // Accessors
    const Grid& grid() const { return *m_grid; }
    const Robot& robot() const { return *m_robot; }
    const std::vector<std::unique_ptr<DynamicObstacle>>& dynObs() const { return m_dynObs; }
    const Metrics& metrics() const { return m_metrics; }
    SimState state() const { return m_state; }
    std::string statusMessage() const { return m_statusMessage; }
    const LidarScan& latestLidarScan() const { return m_lastScan; }

    // Display Settings
    bool showPath = true;
    bool showExplored = false;
    bool debugMode = false;
    bool dynamicObsEnabled = true;
    float speedMultiplier = 1.0f;

    // Concurrency
    mutable std::mutex stateMutex;

    // Driver IO
    struct SensorReading {
        int front, rear, left, right;
        bool obstacleDetected;
        int seq;
    };
    SensorReading computeSensorReading();

private:
    void triggerReplan();
    void updateSensorData();
    std::pair<int,int> findNearestCharger() const;

    SimConfig m_config;
    SimState m_state;
    std::string m_statusMessage;

    std::unique_ptr<Grid> m_grid;
    std::unique_ptr<Robot> m_robot;
    std::vector<std::unique_ptr<DynamicObstacle>> m_dynObs;
    std::unique_ptr<AStarPlanner> m_planner;
    SensorProcessing m_sensorProc;
    LidarScan m_lastScan;

    Metrics m_metrics;
    
    int m_taskCounter{0};
    float m_replanTimer{0.0f};
    float m_pauseTimer{0.0f};
    int m_sensorSeq{0};
};

} // namespace warehouse
