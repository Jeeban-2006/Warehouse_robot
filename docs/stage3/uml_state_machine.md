# State Machine
```mermaid
stateDiagram-v2
    IDLE --> PLANNING
    PLANNING --> MOVING
    MOVING --> BLOCKED
    BLOCKED --> REPLANNING
```
