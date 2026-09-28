// SensorTypes.hpp - Sensor-related types shared between driver interface and simulation.
#pragma once

namespace warehouse {

/// Mirror of the kernel struct WarehouseSensorReading (for C++ use).
struct SensorReading {
    int  frontDistance{-1};
    int  rearDistance{-1};
    int  leftDistance{-1};
    int  rightDistance{-1};
    bool obstacleDetected{false};
    int  sequence{0};
    int  robotCol{0};
    int  robotRow{0};
    int  robotState{0};

    [[nodiscard]] bool isValid() const noexcept { return sequence >= 0; }
};

}  // namespace warehouse
