# CSTAM-TUNIBOT Interface & ROS 2 API Documentation

## Overview
This document specifies the messaging protocols, topic schemas, JSON payloads, and service interfaces implemented across the **CSTAM 3.0 Autonomous Waiter Service Robot (Phase 1 MVP)**.

---

## 1. ROS 2 Topic & Interface Specification

### 1. Delivery Request Interface
- **Topic**: `/delivery_request`
- **Type**: `std_msgs/msg/String`
- **Publisher**: Delivery Simulator Interface (`delivery_simulator_ui.py`)
- **Subscriber**: Task Manager Node (`delivery_task_manager.py`)
- **Payload Schema (JSON)**:
```json
{
  "target": "Table 4",
  "item": "🍔 Bella Burger & Fries"
}
```
- **Validation**: `target` must resolve to one of the 24 dining tables (`Table 0` .. `Table 23`) or `Kitchen/Pickup`.

---

### 2. Task Management Action Interface
- **Topic**: `/delivery_task_action`
- **Type**: `std_msgs/msg/String`
- **Publisher**: Delivery Simulator Interface (`delivery_simulator_ui.py`)
- **Subscriber**: Task Manager Node (`delivery_task_manager.py`)

#### Action A: Modify Queued Task
Updates the target table number or food/beverage item for an order currently in the pending queue:
```json
{
  "action": "modify",
  "task_id": "TASK-0002",
  "target": "Table 7",
  "item": "🍝 Chef's Pasta Carbonara"
}
```

#### Action B: Delete Queued Task
Cancels and removes a pending order from the delivery queue:
```json
{
  "action": "delete",
  "task_id": "TASK-0002"
}
```

#### Action C: Stop / Cancel Docking
Interrupts active auto-docking and returns the robot to `IDLE`:
```json
{
  "action": "stop_dock"
}
```

---

### 3. Auto-Docking Command Interface
- **Topic**: `/dock_command`
- **Type**: `std_msgs/msg/String`
- **Payloads**:
  - `"DOCK_NOW:manual_ui"`: Commands the robot to navigate to the Southeast Charging Station (`7.06, -12.00`).
  - `"DOCK_NOW:dock_idle"`: Dispatched by `docking_controller` upon 15s idle timeout.
  - `"STOP_DOCK"` / `"CANCEL_DOCK"`: Cancels active docking, applies zero velocity on `/cmd_vel`, and resets state to `IDLE`.

---

### 4. Dock State Telemetry
- **Topic**: `/dock_state`
- **Type**: `std_msgs/msg/Bool`
- **Direction**: Task Manager $\rightarrow$ Simulator & Controllers
- **Value**:
  - `true`: Robot is docked at charging station.
  - `false`: Robot is undocked or in transit.

---

### 5. Delivery Queue Status Telemetry
- **Topic**: `/delivery_queue_status`
- **Type**: `std_msgs/msg/String`
- **Payload Schema (JSON)**:
```json
{
  "robot_state": "navigating",
  "current_task": {
    "id": "TASK-0001",
    "target": "Table 1",
    "item": "☕ Espresso & Croissant",
    "status": "en_route",
    "created_at": 1727720000.0,
    "completed_at": null
  },
  "queue_length": 2,
  "pending_tasks": [
    {
      "id": "TASK-0002",
      "target": "Table 4",
      "item": "🍔 Bella Burger & Fries",
      "status": "queued",
      "created_at": 1727720005.0,
      "completed_at": null
    }
  ],
  "completed_count": 5
}
```

---

### 6. Robot System State Telemetry
- **Topic**: `/robot_system_state`
- **Type**: `std_msgs/msg/String`
- **Values**:
  - `IDLE`: Robot parked and ready for orders.
  - `NAVIGATING`: Robot actively driving towards a service table.
  - `AT_TABLE`: Robot arrived at table, handing over delivery (3s dwell).
  - `DOCKING`: Robot autonomously returning to charging dock.
  - `DOCKED`: Robot parked at docking pad.

---

## 2. Nav2 Navigation Stack Interface

### Goal Navigation Action
- **Action Server**: `/navigate_to_pose`
- **Action Type**: `nav2_msgs/action/NavigateToPose`
- **Goal Field**: `geometry_msgs/msg/PoseStamped` in `'map'` frame.

### Velocity Command Interface
- **Topic**: `/cmd_vel`
- **Type**: `geometry_msgs/msg/Twist`
- **Linear**: `linear.x` (forward/backward velocity in m/s).
- **Angular**: `angular.z` (rotational velocity in rad/s).

---

## 3. Predefined Waypoints Schema (`waypoints.yaml`)

```yaml
waypoints:
  Dock:
    name: "Docking & Charging Station"
    x: 7.06
    y: -12.00
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 0.7071
    qw: 0.7071
    type: "dock"

  Kitchen/Pickup:
    name: "Kitchen & Order Pickup Counter"
    x: -9.60
    y: -1.39
    z: 0.0
    qx: 0.0
    qy: 0.0
    qz: 1.0
    qw: 0.0
    type: "pickup"

  Table 0 .. Table 23:
    name: "Dining Table Service Point"
    # Service points positioned in free aisles with >=0.55m clearance facing the table
```
