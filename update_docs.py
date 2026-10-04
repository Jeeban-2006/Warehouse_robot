import os
import glob
import re

docs_dir = r"C:\Users\ASUS\Downloads\Wipro_Project\warehouse_robot\docs"
md_files = glob.glob(os.path.join(docs_dir, "**", "*.md"), recursive=True)

v3_summary = """
### V3.0 System Upgrade Notes
The simulator has evolved into a Professional Autonomous Warehouse Robotics System. Key features include:
- **Warehouse Environment:** 60x45 grid with a realistic layout (Shelves, Charging Stations, Loading Zones, Pickup Stations).
- **Robot Intelligence & Tasks:** Task/Mission System with PICKING, DELIVERING, CHARGING, and ERROR states. Multi-step missions (Pickup -> Navigate -> Deliver).
- **Battery System:** Active drain during movement, automatic routing to a Charging Station when battery drops below 20%.
- **Advanced Sensors & LiDAR:** 360-degree LiDAR raycasting (SensorProcessing.cpp) complementing the 4-directional Linux character driver (/dev/warehouse_sensor).
- **System Architecture:** SimulationEngine decoupled, foundations for multi-robot architecture (RobotManager, TaskManager).
- **Advanced UI Dashboard:** Dynamic rendering of Battery %, Mission Status, live Sensor readings, and A* performance metrics.
"""

for file_path in md_files:
    with open(file_path, "r", encoding="utf-8") as f:
        content = f.read()
    
    if "V3.0 System Upgrade Notes" not in content:
        with open(file_path, "a", encoding="utf-8") as f:
            f.write("\n\n" + v3_summary)

# Update stage2/02_functional_requirements.md
req_file = os.path.join(docs_dir, "stage2", "02_functional_requirements.md")
with open(req_file, "r", encoding="utf-8") as f:
    req_content = f.read()

req_additions = """
#### V3.0 Additional Functional Requirements
1. **Battery System:** The robot shall actively drain battery while moving. When battery drops below 20%, it shall automatically pause its mission and route to a Charging Station.
2. **Mission System:** The system shall assign multi-step missions to the robot (e.g., Pickup -> Navigate -> Deliver).
3. **LiDAR:** The system shall use 360-degree LiDAR raycasting for advanced obstacle detection.
4. **Realistic Grid:** The warehouse shall be modeled on a 60x45 grid containing Shelves, Charging Stations, Loading Zones, and Pickup Stations.
"""
if "V3.0 Additional Functional Requirements" not in req_content:
    with open(req_file, "a", encoding="utf-8") as f:
        f.write("\n\n" + req_additions)

# Update stage3/uml_class_diagram.md
class_diagram_file = os.path.join(docs_dir, "stage3", "uml_class_diagram.md")
with open(class_diagram_file, "r", encoding="utf-8") as f:
    cd_content = f.read()

if "WarehouseObject" not in cd_content:
    with open(class_diagram_file, "a", encoding="utf-8") as f:
        f.write("\n\n```mermaid\nclassDiagram\n    class WarehouseObject\n    class RobotManager\n    class TaskManager\n    class SensorProcessing\n```\n*Updated for V3.0 classes*")

# Update stage3/uml_state_machine.md
state_machine_file = os.path.join(docs_dir, "stage3", "uml_state_machine.md")
with open(state_machine_file, "r", encoding="utf-8") as f:
    sm_content = f.read()
if "PICKING" not in sm_content:
    with open(state_machine_file, "a", encoding="utf-8") as f:
        f.write("\n\n```mermaid\nstateDiagram-v2\n    IDLE --> PICKING : Assign Task\n    PICKING --> DELIVERING : Reached Pickup\n    DELIVERING --> IDLE : Reached Loading Zone\n    DELIVERING --> CHARGING : Battery < 20%\n    PICKING --> CHARGING : Battery < 20%\n    CHARGING --> IDLE : Battery Full\n    [*] --> ERROR\n```\n*Updated for V3.0 states*")

# Update stage3/uml_sequence_diagram.md
seq_diagram_file = os.path.join(docs_dir, "stage3", "uml_sequence_diagram.md")
with open(seq_diagram_file, "a", encoding="utf-8") as f:
    f.write("\n\n*Updated for V3.0: Sequence now includes TaskManager assigning multi-step missions, and SensorProcessing handling 360-degree LiDAR.*")

# Update stage5/01_testing_strategy.md
test_file = os.path.join(docs_dir, "stage5", "01_testing_strategy.md")
with open(test_file, "a", encoding="utf-8") as f:
    f.write("\n\n### V3.0 Testing Matrices\n- **Battery Depletion:** Verify battery drains during movement and robot routes to charging station below 20%.\n- **Task Assignments:** Verify multi-step missions complete correctly.\n- **LiDAR:** Verify obstacle detection using 360-degree rays.\n")

# Update stage6/09_demo_script.md
demo_script_file = os.path.join(docs_dir, "stage6", "09_demo_script.md")
with open(demo_script_file, "a", encoding="utf-8") as f:
    f.write("\n\n### V3.0 Demo Script\n1. **Assign Task:** Assign a multi-step mission to the robot.\n2. **Watch Battery:** Observe the dynamic UI rendering battery drain.\n3. **Charging Route:** Watch the robot automatically reroute to a Charging Station when battery < 20%.\n4. **LiDAR:** Point out the 360-degree LiDAR rays detecting obstacles.\n")

print(f"Updated all required documentation files for V3.0.")
