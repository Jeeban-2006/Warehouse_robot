// AStarPlanner.cpp - Complete A* implementation from scratch.
// f(n) = g(n) + h(n), priority_queue open set, unordered_map for fast lookup.
#include "core/AStarPlanner.hpp"

#include <queue>
#include <cmath>
#include <chrono>
#include <limits>

namespace warehouse {

// ── Key encoding ────────────────────────────────────────────────────────────
static inline long long encodePos(int col, int row, int cols) noexcept {
    return static_cast<long long>(row) * cols + col;
}

// ── Constructor ──────────────────────────────────────────────────────────────
AStarPlanner::AStarPlanner(const Grid& grid, bool diagonal, Heuristic h)
    : m_grid(&grid), m_diagonal(diagonal), m_defaultHeuristic(h)
{}

// ── Heuristics ───────────────────────────────────────────────────────────────
float AStarPlanner::hManhattan(std::pair<int,int> a, std::pair<int,int> b) noexcept {
    return static_cast<float>(std::abs(a.first - b.first) + std::abs(a.second - b.second));
}

float AStarPlanner::hEuclidean(std::pair<int,int> a, std::pair<int,int> b) noexcept {
    float dx = static_cast<float>(a.first  - b.first);
    float dy = static_cast<float>(a.second - b.second);
    return std::sqrt(dx*dx + dy*dy);
}

float AStarPlanner::hChebyshev(std::pair<int,int> a, std::pair<int,int> b) noexcept {
    return static_cast<float>(std::max(std::abs(a.first - b.first),
                                       std::abs(a.second - b.second)));
}

float AStarPlanner::heuristic(std::pair<int,int> a, std::pair<int,int> b,
                               Heuristic h) const noexcept {
    switch (h) {
        case Heuristic::EUCLIDEAN:  return hEuclidean(a, b);
        case Heuristic::CHEBYSHEV:  return hChebyshev(a, b);
        default:                    return hManhattan(a, b);
    }
}

// ── Path reconstruction ───────────────────────────────────────────────────────
std::vector<std::pair<int,int>>
AStarPlanner::reconstruct(const std::unordered_map<long long, Node>& cameFrom,
                           std::pair<int,int> goal, int cols)
{
    std::vector<std::pair<int,int>> path;
    long long key = encodePos(goal.first, goal.second, cols);

    while (true) {
        auto it = cameFrom.find(key);
        if (it == cameFrom.end()) break;
        const Node& n = it->second;
        path.emplace_back(n.col, n.row);
        if (n.parentCol == -1 && n.parentRow == -1) break;
        key = encodePos(n.parentCol, n.parentRow, cols);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ── Main search ───────────────────────────────────────────────────────────────
PathResult AStarPlanner::findPath(std::pair<int,int> start,
                                   std::pair<int,int> goal,
                                   const PosSet& extraBlocked,
                                   Heuristic heuristic_) const
{
    auto tStart = std::chrono::steady_clock::now();

    PathResult result;
    result.success = false;

    // ── Edge cases ────────────────────────────────────────────────────────
    if (!m_grid->inBounds(start.first, start.second)) {
        result.message = "Start position out of bounds.";
        goto done;
    }
    if (!m_grid->inBounds(goal.first, goal.second)) {
        result.message = "Goal position out of bounds.";
        goto done;
    }
    if (m_grid->isObstacle(start.first, start.second)) {
        result.message = "Start is blocked by a static obstacle.";
        goto done;
    }
    if (m_grid->isObstacle(goal.first, goal.second)) {
        result.message = "Goal is blocked by a static obstacle.";
        goto done;
    }
    if (extraBlocked.count(goal)) {
        result.message = "Goal is blocked by a dynamic obstacle.";
        goto done;
    }
    if (start == goal) {
        result.path    = {start};
        result.success = true;
        result.pathLength = 1;
        result.message = "Already at goal.";
        goto done;
    }

    {
        // ── Search ────────────────────────────────────────────────────────
        const int cols = m_grid->cols();

        // Open set: min-heap ordered by f (then h for tie-breaking)
        using MinHeap = std::priority_queue<Node, std::vector<Node>, std::greater<Node>>;
        MinHeap openHeap;

        // Best g-cost seen for each cell
        std::unordered_map<long long, float> gScore;

        // For path reconstruction: stores the node data keyed by position
        std::unordered_map<long long, Node> cameFrom;

        float h0 = heuristic(start, goal, heuristic_);
        Node startNode{h0, 0.0f, h0,
                       start.first, start.second, -1, -1};
        gScore[encodePos(start.first, start.second, cols)] = 0.0f;
        openHeap.push(startNode);

        static const int dx4[] = {0, 0,-1, 1};
        static const int dy4[] = {-1, 1, 0, 0};
        static const int dx8[] = {0, 0,-1, 1,-1, 1,-1, 1};
        static const int dy8[] = {-1, 1, 0, 0,-1,-1, 1, 1};
        const int* dx = m_diagonal ? dx8 : dx4;
        const int* dy = m_diagonal ? dy8 : dy4;
        const int  nd = m_diagonal ? 8 : 4;

        while (!openHeap.empty()) {
            Node current = openHeap.top();
            openHeap.pop();

            long long curKey = encodePos(current.col, current.row, cols);
            result.explored.insert({current.col, current.row});

            // Skip if we've already found a better path to this cell
            auto gIt = gScore.find(curKey);
            if (gIt != gScore.end() && current.g > gIt->second + 1e-6f)
                continue;

            if (current.col == goal.first && current.row == goal.second) {
                // Reconstruct path
                cameFrom[curKey] = current;
                result.path    = reconstruct(cameFrom, goal, cols);
                result.success = true;
                result.pathLength    = static_cast<int>(result.path.size());
                result.nodesExplored = static_cast<int>(result.explored.size());
                result.message = "Path found (" + std::to_string(result.nodesExplored) +
                                 " nodes explored).";
                goto done;
            }

            cameFrom[curKey] = current;

            for (int i = 0; i < nd; ++i) {
                int nc = current.col + dx[i];
                int nr = current.row + dy[i];

                if (!m_grid->inBounds(nc, nr))        continue;
                if (m_grid->isObstacle(nc, nr))        continue;
                if (extraBlocked.count({nc, nr}))      continue;

                // Diagonal movement cost = √2
                float step = (dx[i] != 0 && dy[i] != 0) ? 1.4142f : 1.0f;
                float g_new = current.g + step;

                long long nKey = encodePos(nc, nr, cols);
                auto it = gScore.find(nKey);
                if (it != gScore.end() && g_new >= it->second) continue;

                gScore[nKey] = g_new;
                float h_new  = heuristic({nc, nr}, goal, heuristic_);
                Node neigh{g_new + h_new, g_new, h_new,
                           nc, nr, current.col, current.row};
                openHeap.push(neigh);
            }
        }

        result.message = "No valid path found. Goal is unreachable.";
        result.nodesExplored = static_cast<int>(result.explored.size());
    }

done:
    {
        auto tEnd = std::chrono::steady_clock::now();
        result.planningTimeMs =
            std::chrono::duration<double, std::milli>(tEnd - tStart).count();

        // Update cumulative statistics
        ++m_stats.totalPlanCalls;
        m_stats.totalNodesExplored += result.nodesExplored;
        m_stats.totalPlanTimeMs    += result.planningTimeMs;
        if (result.planningTimeMs > m_stats.maxPlanTimeMs)
            m_stats.maxPlanTimeMs = result.planningTimeMs;
        if (result.success)
            ++m_stats.successfulPlans;
        else
            ++m_stats.failedPlans;
    }
    return result;
}

// ── Path validation ───────────────────────────────────────────────────────────
bool AStarPlanner::isPathValid(const std::vector<std::pair<int,int>>& path,
                                const PosSet& extraBlocked) const
{
    for (const auto& [c, r] : path) {
        if (m_grid->isObstacle(c, r))  return false;
        if (extraBlocked.count({c, r})) return false;
    }
    return true;
}

void AStarPlanner::resetStats() noexcept {
    m_stats = {};
}

}  // namespace warehouse
