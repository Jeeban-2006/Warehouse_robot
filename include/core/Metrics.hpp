// Metrics.hpp - Tracks simulation performance statistics.
#pragma once

#include <cstdint>
#include <chrono>
#include <string>

namespace warehouse {

struct Metrics {
    // ── Path planning ─────────────────────────────────────────────────────
    int    totalPathCalls{0};
    int    successfulPlans{0};
    int    failedPlans{0};
    int    replanCount{0};
    int    lastPathLength{0};
    int    lastNodesExplored{0};
    int    totalNodesExplored{0};
    double lastPlanTimeMs{0.0};
    double totalPlanTimeMs{0.0};
    double maxPlanTimeMs{0.0};

    // ── Movement ──────────────────────────────────────────────────────────
    int    collisionsPrevented{0};

    // ── Timing ────────────────────────────────────────────────────────────
    double elapsedSeconds{0.0};

    // ── Methods ───────────────────────────────────────────────────────────
    void reset() noexcept;

    void recordPlanResult(bool success,
                          int  pathLen,
                          int  explored,
                          double planTimeMs) noexcept;

    void incrementReplans() noexcept { ++replanCount; }
    void incrementCollisionsPrevented() noexcept { ++collisionsPrevented; }

    [[nodiscard]] std::string summary() const;
};

}  // namespace warehouse
