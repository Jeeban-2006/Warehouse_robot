// Metrics.cpp
#include "core/Metrics.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace warehouse {

void Metrics::reset() noexcept {
    *this = Metrics{};
}

void Metrics::recordPlanResult(bool success,
                                int  pathLen,
                                int  explored,
                                double planTimeMs) noexcept
{
    ++totalPathCalls;
    lastPathLength      = pathLen;
    lastNodesExplored   = explored;
    lastPlanTimeMs      = planTimeMs;
    totalNodesExplored += explored;
    totalPlanTimeMs    += planTimeMs;
    maxPlanTimeMs       = std::max(maxPlanTimeMs, planTimeMs);
    if (success) ++successfulPlans;
    else         ++failedPlans;
}

std::string Metrics::summary() const {
    std::ostringstream oss;
    oss << "PathCalls=" << totalPathCalls
        << " OK=" << successfulPlans
        << " Fail=" << failedPlans
        << " Replans=" << replanCount
        << " LastPath=" << lastPathLength
        << " Explored=" << lastNodesExplored
        << std::fixed << std::setprecision(2)
        << " PlanMs=" << lastPlanTimeMs
        << " MaxMs=" << maxPlanTimeMs;
    return oss.str();
}

}  // namespace warehouse
