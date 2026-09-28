# Sequence Diagram
```mermaid
sequenceDiagram
    Driver->>Sensor: poll
    Sensor->>Robot: new obstacle
    Robot->>Pathfinder: replan
```
