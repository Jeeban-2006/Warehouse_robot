// AStarPlanner.hpp - A* pathfinding from scratch.
// f(n) = g(n) + h(n), Manhattan heuristic, priority_queue open set.
#pragma once

#include "Grid.hpp"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <chrono>
#include <string>
#include <functional>

namespace warehouse {

// ── Custom hash for std::pair<int,int> ────────────────────────────────────────
struct PairHash {
    std::size_t operator()(const std::pair<int,int>& p) const noexcept {
        // Combine hashes: shift col to upper 16 bits
        return std::hash<long long>()(((long long)p.first << 32) | (unsigned int)p.second);
    }
};

using PosSet = std::unordered_set<std::pair<int,int>, PairHash>;

// ── Path result ───────────────────────────────────────────────────────────────
struct PathResult {
    bool   success{false};
    std::vector<std::pair<int,int>> path;       ///< (col, row) sequence start→goal
    PosSet explored;                             ///< All nodes expanded during search
    int    pathLength{0};
    int    nodesExplored{0};
    double planningTimeMs{0.0};
    std::string message;
};

// ── A* statistics (cumulative) ────────────────────────────────────────────────
struct PlannerStats {
    int    totalPlanCalls{0};
    int    successfulPlans{0};
    int    failedPlans{0};
    double totalPlanTimeMs{0.0};
    double maxPlanTimeMs{0.0};
    int    totalNodesExplored{0};
};

// ── AStarPlanner ─────────────────────────────────────────────────────────────
class AStarPlanner {
public:
    enum class Heuristic { MANHATTAN, EUCLIDEAN, CHEBYSHEV };

    explicit AStarPlanner(const Grid& grid,
                          bool diagonal = false,
                          Heuristic h = Heuristic::MANHATTAN);

    /// Find path from start to goal.
    /// extraBlocked: dynamic obstacle positions treated as impassable.
    [[nodiscard]] PathResult findPath(
        std::pair<int,int> start,
        std::pair<int,int> goal,
        const PosSet& extraBlocked = {},
        Heuristic heuristic = Heuristic::MANHATTAN) const;

    /// Check whether every cell in path is still free.
    [[nodiscard]] bool isPathValid(
        const std::vector<std::pair<int,int>>& path,
        const PosSet& extraBlocked = {}) const;

    // ── Statistics ────────────────────────────────────────────────────────
    [[nodiscard]] const PlannerStats& stats() const noexcept { return m_stats; }
    void resetStats() noexcept;

    // ── Configuration ────────────────────────────────────────────────────
    void setDiagonal(bool d) noexcept { m_diagonal = d; }
    void setGrid(const Grid& g) noexcept { m_grid = &g; }

private:
    const Grid* m_grid;
    bool        m_diagonal;
    Heuristic   m_defaultHeuristic;
    mutable PlannerStats m_stats;

    // ── Heuristic functions ────────────────────────────────────────────────
    static float hManhattan (std::pair<int,int> a, std::pair<int,int> b) noexcept;
    static float hEuclidean (std::pair<int,int> a, std::pair<int,int> b) noexcept;
    static float hChebyshev (std::pair<int,int> a, std::pair<int,int> b) noexcept;

    float heuristic(std::pair<int,int> a, std::pair<int,int> b,
                    Heuristic h) const noexcept;

    // ── Internal node for priority queue ──────────────────────────────────
    struct Node {
        float f, g, h;
        int   col, row;
        int   parentCol, parentRow;   ///< -1,-1 means no parent (start node)
        bool operator>(const Node& o) const noexcept {
            return (f != o.f) ? f > o.f : h > o.h;
        }
    };

    // ── Path reconstruction ───────────────────────────────────────────────
    static std::vector<std::pair<int,int>> reconstruct(
        const std::unordered_map<long long, Node>& cameFrom,
        std::pair<int,int> goal,
        int cols);
};

}  // namespace warehouse
