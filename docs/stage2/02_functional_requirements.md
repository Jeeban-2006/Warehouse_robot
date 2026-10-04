# Functional Requirements
FR-01: System renders 30x22 grid. (High, Visual)
FR-02: User places start/end. (High, Test)
FR-03: Calculate A* path. (High, Unit Test)
FR-04: Static obstacles. (High, Test)
FR-05: Sensor data from driver. (High, Integration)
FR-06: Dynamic replanning. (High, Test)
FR-07: Pause/resume. (Medium, UI)
FR-08: Robot state display. (Medium, Visual)
FR-09: Reset grid. (High, UI)
FR-10: Driver `read`. (High, Test)
FR-11: Driver `ioctl`. (Low, Test)
FR-12: Uses `poll`. (High, Code Review)
FR-13: Independent UI thread. (High, Code Review)
FR-14: 1 cell per tick. (High, Test)
FR-15: Bounds checking. (High, Unit Test)
FR-16: Stop at goal. (High, Test)
FR-17: Random obstacles from driver. (Medium, Integration)
FR-18: Graceful exit. (High, Valgrind)
FR-19: SIGINT handling. (Medium, Test)
FR-20: Event logging. (Low, Test)



### V3.0 System Upgrade Notes
The simulator has evolved into a Professional Autonomous Warehouse Robotics System. Key features include:
- **Warehouse Environment:** 60x45 grid with a realistic layout (Shelves, Charging Stations, Loading Zones, Pickup Stations).
- **Robot Intelligence & Tasks:** Task/Mission System with PICKING, DELIVERING, CHARGING, and ERROR states. Multi-step missions (Pickup -> Navigate -> Deliver).
- **Battery System:** Active drain during movement, automatic routing to a Charging Station when battery drops below 20%.
- **Advanced Sensors & LiDAR:** 360-degree LiDAR raycasting (SensorProcessing.cpp) complementing the 4-directional Linux character driver (/dev/warehouse_sensor).
- **System Architecture:** SimulationEngine decoupled, foundations for multi-robot architecture (RobotManager, TaskManager).
- **Advanced UI Dashboard:** Dynamic rendering of Battery %, Mission Status, live Sensor readings, and A* performance metrics.



#### V3.0 Additional Functional Requirements
1. **Battery System:** The robot shall actively drain battery while moving. When battery drops below 20%, it shall automatically pause its mission and route to a Charging Station.
2. **Mission System:** The system shall assign multi-step missions to the robot (e.g., Pickup -> Navigate -> Deliver).
3. **LiDAR:** The system shall use 360-degree LiDAR raycasting for advanced obstacle detection.
4. **Realistic Grid:** The warehouse shall be modeled on a 60x45 grid containing Shelves, Charging Stations, Loading Zones, and Pickup Stations.

### V3.0 System Upgrade Notes (Updated)
The simulator has undergone a MASSIVE architectural upgrade to V3.0. Key features include:
- **Smart Predictive BMS (Battery Management System):** Calculates distance to goal + distance back to the nearest charging station. If battery is insufficient, it aborts the mission and auto-routes to the nearest of 20 charging stations, charges for 3s, and resumes the task.
- **60x45 Grid & Dedicated Zones:** The warehouse map is huge. It has Purple Pickup Stations, Yellow Loading Zones, and Cyan Charging Stations.
- **Continuous Demo Engine:** Automatically creates endless tasks strictly routing the robot from a random Purple Pickup zone to a random Yellow Loading zone.
- **Layout Switching:** The user can cycle between 3 different layouts (Standard, Cross Map 1, Cross Map 2) by clicking "Generate Warehouse" to force complex diagonal routes.
- **Interactive Real-Time Walls:** The user can left-click the map to instantly drop concrete walls in front of the robot, testing its <1ms A* replanning speed.
- **Deadlock Auto-Retry:** If the robot is perfectly trapped by dynamic moving workers and has no path, it intelligently waits and polls every 0.25s until the worker moves, instead of giving up.
- **Complete Dashboard Legend:** The UI renders a complete color-coded legend, dynamic metrics, and battery percentage.
