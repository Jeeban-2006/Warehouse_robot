// SimulationEngine.cpp - Top-level simulation orchestrator.
#include "core/SimulationEngine.hpp"
#include "system/Logger.hpp"

#include <random>
#include <algorithm>
#include <sstream>
#include <chrono>
#include <cmath>

namespace warehouse {

// ── Default dynamic obstacle positions ───────────────────────────────────────
struct DynObsDef {
    int col, row;
    ObstacleAxis axis;
    int minPos, maxPos;
    float speed;
};
static const DynObsDef DEFAULT_DYN[] = {
    {8,  6,  ObstacleAxis::HORIZONTAL, 8,  15, 2.0f},
    {16, 11, ObstacleAxis::VERTICAL,  10,  17, 1.5f},
    {23, 7,  ObstacleAxis::HORIZONTAL, 22, 27, 2.5f},
};

// ── Constructor ───────────────────────────────────────────────────────────────
SimulationEngine::SimulationEngine(SimConfig cfg)
    : m_cfg(cfg)
{
    m_grid    = std::make_unique<Grid>(cfg.gridCols, cfg.gridRows);
    m_robot   = std::make_unique<Robot>(0,
                                        std::make_pair(1, 1),
                                        std::make_pair(28, 20),
                                        cfg.cellSize, cfg.robotBaseSpeed);
    m_planner = std::make_unique<AStarPlanner>(*m_grid);
    loadDefaultMap();
}

// ── Map management ────────────────────────────────────────────────────────────
void SimulationEngine::loadDefaultMap() {
    m_grid->loadDefaultMap(m_robot->startPos(), m_robot->goalPos());
    m_robot->reset();
    createDefaultDynObs();
    m_lastPathResult = {};
    m_statusMsg = "Press START to begin.";
    m_state = SimState::IDLE;
    LOG_INFO("Warehouse: default map loaded");
}

void SimulationEngine::generateRandomMap(float den) {
    auto rp = m_robot->startPos();
    auto gp = m_robot->goalPos();
    std::vector<std::pair<int,int>> prot;
    prot.push_back(rp); prot.push_back(gp);
    // Protect neighbours for connectivity
    for (auto [dc,dr] : std::vector<std::pair<int,int>>{{0,1},{0,-1},{1,0},{-1,0}}) {
        prot.push_back({rp.first+dc, rp.second+dr});
        prot.push_back({gp.first+dc, gp.second+dr});
    }
    m_grid->generateRandom(den, prot);
    m_robot->reset();
    createRandomDynObs();
    m_lastPathResult = {};
    m_state = SimState::IDLE;
    m_statusMsg = "Random map generated.";
    LOG_INFO("Warehouse: random map generated density=" + std::to_string(den));
}

void SimulationEngine::clearObstacles() {
    m_grid->clear();
    m_robot->reset();
    m_lastPathResult = {};
    m_state = SimState::IDLE;
    m_statusMsg = "Map cleared.";
}

void SimulationEngine::setObstacle(int col, int row, bool value) {
    auto rp = m_robot->pos();
    auto gp = m_robot->goalPos();
    if (std::make_pair(col,row) == rp) return;
    if (std::make_pair(col,row) == gp) return;
    m_grid->setObstacle(col, row, value);
    m_robot->clearPath();
    m_state = SimState::IDLE;
}

void SimulationEngine::removeObstacle(int col, int row) {
    m_grid->setObstacle(col, row, false);
}

void SimulationEngine::setRobotStart(int col, int row) {
    if (m_grid->isObstacle(col, row)) return;
    if (std::make_pair(col,row) == m_robot->goalPos()) return;
    m_robot->setStartPos({col, row});
    m_state = SimState::IDLE;
}

void SimulationEngine::setRobotGoal(int col, int row) {
    if (m_grid->isObstacle(col, row)) return;
    if (std::make_pair(col,row) == m_robot->pos()) return;
    m_robot->setGoalPos({col, row});
    m_state = SimState::IDLE;
}

// ── Control ───────────────────────────────────────────────────────────────────
void SimulationEngine::start() {
    if (m_state == SimState::RUNNING) return;
    m_metrics.reset();
    m_metrics.elapsedSeconds = 0.0;
    m_state = SimState::PLANNING;
    m_statusMsg = "Planning path...";
    LOG_INFO("Simulation started");
    doPlanning();
}

void SimulationEngine::pause() {
    if (m_state != SimState::RUNNING) return;
    m_state = SimState::PAUSED;
    m_robot->pause();
    m_statusMsg = "Simulation paused.";
    LOG_INFO("Simulation paused");
}

void SimulationEngine::resume() {
    if (m_state != SimState::PAUSED) return;
    m_state = SimState::RUNNING;
    m_robot->resume();
    m_statusMsg = "Simulation running.";
    LOG_INFO("Simulation resumed");
}

void SimulationEngine::reset() {
    m_robot->reset();
    for (auto& obs : m_dynObs)
        obs->reset(obs->col(), obs->row());
    m_state = SimState::IDLE;
    m_metrics.reset();
    m_replanTimer = 0.0f;
    m_replanBanner = false;
    m_lastPathResult = {};
    m_statusMsg = "Press START to begin.";
    LOG_INFO("Simulation reset");
}

void SimulationEngine::fullReset() {
    loadDefaultMap();
    reset();
}

void SimulationEngine::findPathOnly() {
    auto extra = dynObsPositions();
    m_lastPathResult = m_planner->findPath(m_robot->pos(), m_robot->goalPos(), extra);
    m_metrics.recordPlanResult(m_lastPathResult.success,
                               m_lastPathResult.pathLength,
                               m_lastPathResult.nodesExplored,
                               m_lastPathResult.planningTimeMs);
    if (m_lastPathResult.success) {
        m_robot->setPath(m_lastPathResult.path);
        // Keep robot in IDLE: path displayed but not moving
        if (m_state != SimState::RUNNING)
            m_robot->pause();
        m_statusMsg = "Path found: " + std::to_string(m_lastPathResult.pathLength) + " cells.";
        LOG_INFO("Path found: " + std::to_string(m_lastPathResult.pathLength) + " cells");
    } else {
        m_robot->clearPath();
        m_state = SimState::NO_PATH;
        m_statusMsg = m_lastPathResult.message;
        LOG_WARN("No path: " + m_lastPathResult.message);
    }
}

// ── Per-frame update ──────────────────────────────────────────────────────────
void SimulationEngine::update(float dt) {
    if (m_state == SimState::RUNNING) {
        m_metrics.elapsedSeconds += dt;
        updateDynObs(dt);
        checkAndReplan(dt);
        m_robot->update(dt, speedMultiplier);
        checkCompletion();
    }

    // Decay replanning banner
    if (m_replanBanner) {
        m_replanBannerTimer -= dt;
        if (m_replanBannerTimer <= 0.0f)
            m_replanBanner = false;
    }
}

// ── Internal helpers ──────────────────────────────────────────────────────────
void SimulationEngine::doPlanning() {
    auto extra = dynObsPositions();
    m_lastPathResult = m_planner->findPath(m_robot->pos(), m_robot->goalPos(), extra);
    m_metrics.recordPlanResult(m_lastPathResult.success,
                               m_lastPathResult.pathLength,
                               m_lastPathResult.nodesExplored,
                               m_lastPathResult.planningTimeMs);
    if (m_lastPathResult.success) {
        m_robot->setPath(m_lastPathResult.path);
        m_state = SimState::RUNNING;
        m_statusMsg = "Moving - path: " + std::to_string(m_lastPathResult.pathLength) + " cells.";
        LOG_INFO("Path found: " + std::to_string(m_lastPathResult.pathLength) + " cells, " +
                 std::to_string(m_lastPathResult.nodesExplored) + " explored, " +
                 std::to_string(m_lastPathResult.planningTimeMs) + " ms");
    } else {
        m_state = SimState::NO_PATH;
        m_statusMsg = m_lastPathResult.message;
        LOG_WARN("Pathfinding failed: " + m_lastPathResult.message);
    }
}

void SimulationEngine::updateDynObs(float dt) {
    if (!dynamicObsEnabled) return;
    for (auto& obs : m_dynObs)
        obs->update(dt, speedMultiplier);
}

void SimulationEngine::checkAndReplan(float dt) {
    m_replanTimer -= dt;
    if (m_replanTimer > 0.0f) return;
    m_replanTimer = m_cfg.replanInterval;

    if (m_robot->state() != RobotState::MOVING) return;
    if (!isCurrentPathValid()) {
        m_metrics.incrementReplans();
        m_replanBanner = true;
        m_replanBannerTimer = 2.0f;
        m_statusMsg = "PATH BLOCKED - REPLANNING...";
        LOG_WARN("Path blocked, replanning. Replan #" +
                 std::to_string(m_metrics.replanCount));
        m_robot->setPlanning();
        doPlanning();
        if (m_lastPathResult.success)
            m_statusMsg = "Replanned: " +
                          std::to_string(m_lastPathResult.pathLength) + " cells.";
    }
}

bool SimulationEngine::isCurrentPathValid() const {
    auto remaining = m_robot->remainingPath();
    if (remaining.empty()) return false;
    auto extra = dynObsPositions();
    return m_planner->isPathValid(remaining, extra);
}

void SimulationEngine::checkCompletion() {
    if (m_robot->state() == RobotState::REACHED_GOAL) {
        m_state = SimState::COMPLETED;
        std::ostringstream oss;
        oss << "Goal reached! Path=" << m_metrics.lastPathLength
            << " cells, Replans=" << m_metrics.replanCount
            << ", Time=" << std::fixed << std::setprecision(1)
            << m_metrics.elapsedSeconds << "s";
        m_statusMsg = oss.str();
        LOG_INFO("Robot reached goal. " + m_metrics.summary());
    } else if (m_robot->state() == RobotState::ERROR) {
        m_state = SimState::NO_PATH;
    }
}

PosSet SimulationEngine::dynObsPositions() const {
    PosSet s;
    for (const auto& obs : m_dynObs)
        s.insert(obs->pos());
    return s;
}

void SimulationEngine::createDefaultDynObs() {
    m_dynObs.clear();
    int id = 0;
    for (const auto& d : DEFAULT_DYN) {
        m_dynObs.push_back(
            std::make_unique<DynamicObstacle>(id++, d.col, d.row,
                                              d.axis, d.minPos, d.maxPos,
                                              d.speed, m_cfg.cellSize));
    }
}

void SimulationEngine::createRandomDynObs() {
    m_dynObs.clear();
    auto free = m_grid->freeCells();
    auto rp   = m_robot->startPos();
    auto gp   = m_robot->goalPos();
    free.erase(std::remove_if(free.begin(), free.end(),
        [&](auto p){ return p == rp || p == gp; }), free.end());

    std::mt19937 rng(std::random_device{}());
    std::shuffle(free.begin(), free.end(), rng);

    int n = std::min(m_cfg.numDynObs, static_cast<int>(free.size()));
    std::uniform_int_distribution<int> spanDist(3, 6);

    for (int i = 0; i < n; ++i) {
        auto [col, row] = free[static_cast<std::size_t>(i)];
        auto axis = (i % 2 == 0) ? ObstacleAxis::HORIZONTAL : ObstacleAxis::VERTICAL;
        int span  = spanDist(rng);
        int minP, maxP;
        if (axis == ObstacleAxis::HORIZONTAL) {
            minP = std::max(0, col - span);
            maxP = std::min(m_cfg.gridCols - 1, col + span);
        } else {
            minP = std::max(0, row - span);
            maxP = std::min(m_cfg.gridRows - 1, row + span);
        }
        m_dynObs.push_back(
            std::make_unique<DynamicObstacle>(i, col, row, axis, minP, maxP,
                                              m_cfg.dynObsSpeed, m_cfg.cellSize));
    }
}

// ── Accessors ─────────────────────────────────────────────────────────────────
const Grid&   SimulationEngine::grid()    const noexcept { return *m_grid;   }
const Robot&  SimulationEngine::robot()   const noexcept { return *m_robot;  }
const std::vector<std::unique_ptr<DynamicObstacle>>&
              SimulationEngine::dynObs()  const noexcept { return m_dynObs;  }
const Metrics& SimulationEngine::metrics()const noexcept { return m_metrics; }
SimState       SimulationEngine::state()  const noexcept { return m_state;   }
const std::string& SimulationEngine::statusMessage() const noexcept { return m_statusMsg; }
const PathResult&  SimulationEngine::lastPathResult()const noexcept { return m_lastPathResult; }

// ── Sensor reading ────────────────────────────────────────────────────────────
SimulationEngine::SensorReading SimulationEngine::computeSensorReading() const {
    SensorReading sr;
    sr.seq       = ++m_sensorSeq;
    auto [col, row] = m_robot->pos();

    // Scan in each direction until obstacle or grid boundary
    auto scan = [&](int dc, int dr) -> int {
        for (int d = 1; d < std::max(m_cfg.gridCols, m_cfg.gridRows); ++d) {
            int nc = col + dc * d, nr = row + dr * d;
            if (!m_grid->inBounds(nc, nr)) return d;
            if (m_grid->isObstacle(nc, nr)) return d;
            // Check dynamic obstacles
            for (const auto& obs : m_dynObs)
                if (obs->pos() == std::make_pair(nc, nr)) return d;
        }
        return -1;
    };

    auto [fc, fr] = m_robot->facing();
    sr.front = scan(fc, fr);
    sr.rear  = scan(-fc, -fr);

    // Left/right perpendicular to facing
    sr.left  = scan(-fr,  fc);
    sr.right = scan( fr, -fc);

    sr.obstacleDetected = (sr.front >= 0 && sr.front <= 3) ||
                          (sr.rear  >= 0 && sr.rear  <= 3) ||
                          (sr.left  >= 0 && sr.left  <= 3) ||
                          (sr.right >= 0 && sr.right <= 3);
    return sr;
}

}  // namespace warehouse
