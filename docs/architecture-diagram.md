# CSTAM-TUNIBOT Software Architecture & Data Flow Specification

## 1. High-Level System Architecture

The CSTAM Autonomous Indoor Delivery Robot system is divided into three primary layers: **Simulation Layer**, **ROS 2 Core Robotics Layer**, and **Web Control & Telemetry Layer**.

```mermaid
graph TD
    subgraph Web Layer
        UI["Web Dashboard UI (HTML5 / CSS / Canvas 2D)"]
        Bridge["cstam_web_bridge (FastAPI REST & WebSockets)"]
        UI <-->|HTTP REST & WSS| Bridge
    end

    subgraph ROS 2 Core Robotics Stack
        TaskManager["delivery_task_manager Node"]
        BatterySim["battery_simulator Node"]
        DockCtrl["docking_controller Node"]
        
        Bridge <-->|ROS Topics & Services| TaskManager
        Bridge <-->|/battery_state| BatterySim
        Bridge <-->|/dock_command| DockCtrl
    end

    subgraph Nav2 Stack & SLAM
        AMCL["amcl Localization"]
        Planner["planner_server (Navfn)"]
        Controller["controller_server (DWB Local Planner)"]
        Behaviors["behavior_server (Clear Costmap, Spin, Backup, Wait)"]
        BTNav["bt_navigator"]

        TaskManager -->|NavigateToPose Action| BTNav
        BTNav --> Planner
        BTNav --> Controller
        BTNav --> Behaviors
    end

    subgraph Gazebo Simulation Environment
        GazeboWorld["Gazebo World (Indoor Floor, Furniture, Doorway)"]
        RobotURDF["cstam_robot (Diff-Drive, LiDAR, IMU)"]
        ActorPlugin["Walking Human Actor (Dynamic Obstacle)"]

        Controller -->|/cmd_vel| RobotURDF
        RobotURDF -->|/scan & /odom| AMCL
        RobotURDF -->|/scan & /odom| Controller
        RobotURDF <--> GazeboWorld
        ActorPlugin <--> GazeboWorld
    end
```

---

## 2. ROS 2 Topic, Service, and Action Matrix

| Component | Interface Name | Interface Type | Message / Action Type | Description |
| :--- | :--- | :--- | :--- | :--- |
| Gazebo / Robot | `/cmd_vel` | Topic (Pub/Sub) | `geometry_msgs/msg/Twist` | Differential wheel velocity commands |
| Gazebo / Robot | `/scan` | Topic (Pub) | `sensor_msgs/msg/LaserScan` | 2D RPLiDAR scan data (360 samples, 12m max range) |
| Gazebo / Robot | `/odom` | Topic (Pub) | `nav_msgs/msg/Odometry` | Wheel odometry state |
| Nav2 Stack | `/navigate_to_pose` | Action | `nav2_msgs/action/NavigateToPose` | Goal pose navigation action interface |
| Battery Sim | `/battery_state` | Topic (Pub) | `sensor_msgs/msg/BatteryState` | Battery percentage, voltage, charging status |
| Task Manager | `/delivery_queue_status`| Topic (Pub) | `std_msgs/msg/String` (JSON) | Live queue state, current task, robot state |
| Task Manager | `/robot_system_state` | Topic (Pub) | `std_msgs/msg/String` | Simple state string (`idle`, `navigating`, `docked`) |
| Docking Ctrl | `/dock_state` | Topic (Pub) | `std_msgs/msg/Bool` | Boolean indicating if robot is physically docked |
| Docking Ctrl | `/dock_command` | Topic (Pub) | `std_msgs/msg/String` | Auto-docking trigger command (`DOCK_NOW`) |

---

## 3. Delivery Execution Sequence Diagram

```mermaid
sequenceDiagram
    autonumber
    actor User as User / Admin
    participant Web as Web Dashboard
    participant Bridge as FastAPI Server
    participant TaskMgr as delivery_task_manager
    participant Nav2 as Nav2 Stack
    participant BatSim as battery_simulator
    participant Dock as docking_controller

    User->>Web: Select "Table 1" & Submit Order
    Web->>Bridge: POST /api/delivery {target: "Table 1", item: "Espresso"}
    Bridge->>TaskMgr: Add to FIFO Delivery Queue
    TaskMgr->>TaskMgr: State transition: queued -> en_route
    TaskMgr->>Nav2: Send Action Goal (NavigateToPose Table 1)
    Nav2->>Nav2: Path Planning & Dynamic Obstacle Avoidance (/cmd_vel)
    
    par Battery Drain & Telemetry
        BatSim->>Bridge: Publish /battery_state (95%)
        Bridge->>Web: Broadcast WebSocket Telemetry
    end

    Nav2->>TaskMgr: Action Goal Succeeded (Arrived at Table 1)
    TaskMgr->>TaskMgr: State transition: arrived -> completed
    TaskMgr->>Bridge: Queue Status Update

    opt Low Battery Interrupt (<20%)
        BatSim->>TaskMgr: Battery level 18% (<20% threshold)
        TaskMgr->>TaskMgr: Preempt current queue -> Reroute to Dock
        TaskMgr->>Nav2: Send Action Goal (NavigateToPose Dock)
        Nav2->>TaskMgr: Arrived at Docking Station
        TaskMgr->>Dock: State transition: docked / charging
        BatSim->>BatSim: Charge battery up to 90%
        TaskMgr->>TaskMgr: Resume pending delivery queue
    end
```
