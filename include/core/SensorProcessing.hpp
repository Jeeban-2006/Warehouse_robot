#pragma once
#include "Grid.hpp"
#include <vector>
#include <utility>

namespace warehouse {

struct SensorRay {
    float angleDeg;
    float distance;
    int hitCol;
    int hitRow;
    bool hitObstacle;
};

struct LidarScan {
    std::vector<SensorRay> rays;
};

class SensorProcessing {
public:
    SensorProcessing(float maxRangeCells = 15.0f) : m_maxRange(maxRangeCells) {}

    // Simulates a 360-degree LiDAR scan from the given world coordinate
    LidarScan perform360Scan(const Grid& grid, float x, float y, int numRays = 36);

    // Get 4-directional distances (used for Linux driver integration)
    void getDirectionalDistances(const Grid& grid, int c, int r, 
                                 int& front, int& rear, int& left, int& right);

private:
    float m_maxRange;
    void raycast(const Grid& grid, float startX, float startY, float angleDeg, SensorRay& outRay);
};

} // namespace warehouse
