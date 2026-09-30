#include "core/SensorProcessing.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace warehouse {

LidarScan SensorProcessing::perform360Scan(const Grid& grid, float x, float y, int numRays) {
    LidarScan scan;
    scan.rays.reserve(numRays);
    float angleStep = 360.0f / numRays;
    for (int i = 0; i < numRays; ++i) {
        SensorRay ray;
        raycast(grid, x, y, i * angleStep, ray);
        scan.rays.push_back(ray);
    }
    return scan;
}

void SensorProcessing::raycast(const Grid& grid, float startX, float startY, float angleDeg, SensorRay& outRay) {
    outRay.angleDeg = angleDeg;
    outRay.hitObstacle = false;
    outRay.distance = m_maxRange;
    outRay.hitCol = -1;
    outRay.hitRow = -1;

    float rad = angleDeg * (M_PI / 180.0f);
    float dirX = std::cos(rad);
    float dirY = std::sin(rad);

    float stepSize = 0.5f;
    float currentDist = 0.0f;

    while (currentDist <= m_maxRange) {
        float testX = startX + dirX * currentDist;
        float testY = startY + dirY * currentDist;
        
        int gridCol = static_cast<int>(testX);
        int gridRow = static_cast<int>(testY);

        if (!grid.inBounds(gridCol, gridRow) || grid.isObstacle(gridCol, gridRow)) {
            outRay.hitObstacle = true;
            outRay.distance = currentDist;
            outRay.hitCol = gridCol;
            outRay.hitRow = gridRow;
            return;
        }
        currentDist += stepSize;
    }
}

void SensorProcessing::getDirectionalDistances(const Grid& grid, int c, int r, 
                                               int& front, int& rear, int& left, int& right) {
    auto castAxis = [&](int dc, int dr) -> int {
        int dist = 0;
        int testC = c + dc;
        int testR = r + dr;
        while (grid.inBounds(testC, testR)) {
            dist++;
            if (grid.isObstacle(testC, testR)) return dist;
            testC += dc;
            testR += dr;
        }
        return -1; // -1 means infinite/out of bounds
    };

    // Assume robot facing UP (0,-1)
    front = castAxis(0, -1);
    rear  = castAxis(0, 1);
    left  = castAxis(-1, 0);
    right = castAxis(1, 0);
}

} // namespace warehouse
