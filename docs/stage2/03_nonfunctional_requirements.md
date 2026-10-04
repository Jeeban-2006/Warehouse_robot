# Non-Functional Requirements
NFR-01: A* < 10ms (30x22).
NFR-02: UI 60 FPS.
NFR-03: No crashes in 1 hr.
NFR-04: Compiles on WSL2.
NFR-05: Modern C++17.
NFR-06: Modular code.
NFR-07: Safe multithreading.
NFR-08: Memory < 100MB.
NFR-09: Driver loads on Linux.
NFR-10: Doxygen comments.



### V3.0 System Upgrade Notes
The simulator has evolved into a Professional Autonomous Warehouse Robotics System. Key features include:
- **Warehouse Environment:** 60x45 grid with a realistic layout (Shelves, Charging Stations, Loading Zones, Pickup Stations).
- **Robot Intelligence & Tasks:** Task/Mission System with PICKING, DELIVERING, CHARGING, and ERROR states. Multi-step missions (Pickup -> Navigate -> Deliver).
- **Battery System:** Active drain during movement, automatic routing to a Charging Station when battery drops below 20%.
- **Advanced Sensors & LiDAR:** 360-degree LiDAR raycasting (SensorProcessing.cpp) complementing the 4-directional Linux character driver (/dev/warehouse_sensor).
- **System Architecture:** SimulationEngine decoupled, foundations for multi-robot architecture (RobotManager, TaskManager).
- **Advanced UI Dashboard:** Dynamic rendering of Battery %, Mission Status, live Sensor readings, and A* performance metrics.
