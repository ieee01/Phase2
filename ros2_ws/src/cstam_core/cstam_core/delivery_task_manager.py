#!/usr/bin/env python3
from __future__ import annotations

import os
import json
import time
import math
import threading
from collections import deque

try:
    import yaml
    HAVE_YAML = True
except ImportError:
    HAVE_YAML = False

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_ros import Buffer, TransformListener
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs):
            pass
    class String: pass  # type: ignore
    class PoseStamped: pass  # type: ignore
    class PoseWithCovarianceStamped: pass  # type: ignore
    class Odometry: pass  # type: ignore
    class Bool: pass  # type: ignore
    class Twist: pass  # type: ignore
    class Buffer: pass  # type: ignore
    class TransformListener: pass  # type: ignore

try:
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose, BackUp  # type: ignore
    from action_msgs.msg import GoalStatus  # type: ignore
    HAVE_NAV2 = True
except ImportError:
    HAVE_NAV2 = False
    class GoalStatus:  # type: ignore
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6


DEFAULT_WAYPOINTS = {
    "Dock": {"x": 7.06, "y": -12.00, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Kitchen/Pickup": {"x": -9.60, "y": -1.39, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 1.0, "qw": 0.0},
    "Table 0": {"x": -8.24, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 1": {"x": -5.64, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 2": {"x": -3.04, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 3": {"x": -0.44, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 4": {"x": 2.16, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 5": {"x": 4.76, "y": 0.50, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 6": {"x": -8.24, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 7": {"x": -5.64, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 8": {"x": -3.04, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 9": {"x": -0.44, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 10": {"x": 2.16, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 11": {"x": 4.76, "y": -3.10, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.7071, "qw": 0.7071},
    "Table 12": {"x": -8.24, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 13": {"x": -5.64, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 14": {"x": -3.04, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 15": {"x": -0.44, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 16": {"x": 2.16, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 17": {"x": 4.76, "y": -5.85, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 18": {"x": -8.24, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 19": {"x": -5.64, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 20": {"x": -3.04, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 21": {"x": -0.44, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 22": {"x": 2.16, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
    "Table 23": {"x": 4.76, "y": -9.45, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": -0.7071, "qw": 0.7071},
}


class TaskQueueManager:
    """
    Core Task Queue and State Controller logic.
    Manages pending orders, dynamic modification, and task lifecycle.
    """
    def __init__(self, waypoints=None):
        self.waypoints = waypoints or DEFAULT_WAYPOINTS
        self.queue = deque()
        self.current_task = None
        self.task_history = []
        self.robot_state = "idle"  # idle, navigating, at_table, docked, docking
        self.task_id_counter = 1
        self.lock = threading.Lock()

    def _resolve_target(self, target_input: str) -> str | None:
        target_clean = target_input.strip()
        if target_clean in self.waypoints:
            return target_clean
        lower = target_clean.lower()
        for wp in self.waypoints:
            if wp.lower() == lower or wp.lower() == f"table {lower}":
                return wp
        return None

    def add_delivery_request(self, target_location: str, item_description: str = "") -> dict:
        with self.lock:
            resolved = self._resolve_target(target_location)
            if not resolved:
                return {"success": False, "error": f"Unknown target location '{target_location}'"}
            
            task = {
                "id": f"TASK-{self.task_id_counter:04d}",
                "target": resolved,
                "item": item_description or "Food & Beverage Order",
                "status": "queued",
                "created_at": time.time(),
                "completed_at": None
            }
            self.task_id_counter += 1
            self.queue.append(task)
            return {"success": True, "task": task}

    def get_next_task(self):
        with self.lock:
            if not self.queue:
                return None
            task = self.queue.popleft()
            task["status"] = "en_route"
            self.current_task = task
            return task

    def complete_current_task(self, success=True):
        with self.lock:
            if self.current_task:
                self.current_task["status"] = "completed" if success else "failed"
                self.current_task["completed_at"] = time.time()
                self.task_history.append(self.current_task)
                finished_task = self.current_task
                self.current_task = None
                return finished_task
            return None

    def delete_task(self, task_id: str) -> bool:
        with self.lock:
            for i, task in enumerate(self.queue):
                if task["id"] == task_id:
                    del self.queue[i]
                    task["status"] = "cancelled"
                    task["completed_at"] = time.time()
                    self.task_history.append(task)
                    return True
            return False

    def modify_task(self, task_id: str, new_target: str = None, new_item: str = None) -> bool:
        with self.lock:
            for task in self.queue:
                if task["id"] == task_id:
                    if new_target:
                        resolved = self._resolve_target(new_target)
                        if not resolved:
                            return False
                        task["target"] = resolved
                    if new_item is not None:
                        item_clean = new_item.strip()
                        if item_clean:
                            task["item"] = item_clean
                    return True
            return False

    def requeue_current_task(self, front: bool = True):
        """Returns the in-flight task to the pending queue (front by default) so it is never lost."""
        with self.lock:
            task = self.current_task
            if not task:
                return None
            task["status"] = "queued"
            if front:
                self.queue.appendleft(task)
            else:
                self.queue.append(task)
            self.current_task = None
            return task

    def cancel_current_task(self, task_id: str):
        with self.lock:
            task = self.current_task
            if not task or task["id"] != task_id:
                return None
            task["status"] = "cancelled"
            task["completed_at"] = time.time()
            self.task_history.append(task)
            self.current_task = None
            return task

    def modify_current_task(self, task_id: str, new_target: str = None, new_item: str = None):
        """Edits the in-flight task. Returns (found, target_changed)."""
        with self.lock:
            task = self.current_task
            if not task or task["id"] != task_id:
                return False, False
            target_changed = False
            if new_target:
                resolved = self._resolve_target(new_target)
                if not resolved:
                    return False, False
                target_changed = resolved != task["target"]
                task["target"] = resolved
            if new_item is not None and new_item.strip():
                task["item"] = new_item.strip()
            return True, target_changed

    def get_status_summary(self):
        with self.lock:
            completed_success = sum(1 for t in self.task_history if t.get("status") == "completed")
            return {
                "robot_state": self.robot_state,
                "current_task": self.current_task,
                "queue_length": len(self.queue),
                "pending_tasks": list(self.queue),
                "completed_count": completed_success
            }


class DeliveryTaskManagerNode(Node):
    def _load_waypoints(self, wp_file: str = '') -> tuple[dict, str]:
        if not HAVE_YAML:
            return DEFAULT_WAYPOINTS, ''

        target_path = wp_file.strip() if wp_file else ''
        if not target_path or not os.path.exists(target_path):
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_nav = get_package_share_directory('cstam_navigation')
                env_world = os.environ.get('CSTAM_WORLD', '').lower()
                if 'resto' in env_world or 'arbi' in env_world:
                    candidate = os.path.join(pkg_nav, 'config', 'resto_arbi_waypoints.yaml')
                else:
                    candidate = os.path.join(pkg_nav, 'config', 'waypoints.yaml')
                if os.path.exists(candidate):
                    target_path = candidate
            except Exception:
                pass

        if target_path and os.path.exists(target_path):
            try:
                with open(target_path, 'r') as f:
                    data = yaml.safe_load(f)
                    wps = data.get('waypoints', {})
                    if wps and isinstance(wps, dict):
                        self.get_logger().info(f"Loaded {len(wps)} waypoints from '{target_path}'.")
                        return wps, target_path
            except Exception as e:
                self.get_logger().warn(f"Failed to load waypoints from '{target_path}': {e}. Using defaults.")

        return DEFAULT_WAYPOINTS, ''

    def __init__(self):
        super().__init__('delivery_task_manager')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('low_battery_threshold', 25.0)
        self.declare_parameter('critical_battery_threshold', 15.0)
        self.declare_parameter('min_recharge_pct', 80.0)
        self.declare_parameter('dwell_time', 2.5)
        self.declare_parameter('arrival_tolerance', 0.35)
        self.declare_parameter('backup_distance', 0.25)
        self.declare_parameter('backup_speed', 0.15)

        wp_param = self.get_parameter('waypoints_file').value
        self.low_battery_threshold = float(self.get_parameter('low_battery_threshold').value)
        self.critical_battery_threshold = float(self.get_parameter('critical_battery_threshold').value)
        self.min_recharge_pct = float(self.get_parameter('min_recharge_pct').value)
        self.dwell_time = float(self.get_parameter('dwell_time').value)
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.backup_distance = float(self.get_parameter('backup_distance').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)

        loaded_wps, wp_path = self._load_waypoints(wp_param)
        self.manager = TaskQueueManager(waypoints=loaded_wps)
        self.world_name = 'resto_arbi' if 'resto_arbi' in os.path.basename(wp_path) else 'restaurant'

        # Position tracking (map frame) and goal management
        dock_wp = self.manager.waypoints.get("Dock", {})
        self.current_x = float(dock_wp.get("x", 7.06))
        self.current_y = float(dock_wp.get("y", -12.00))
        self.target_coords = None
        self.target_name = None
        self.dwell_start_time = None
        self.current_goal_handle = None
        self.nav_goal_succeeded = False
        self.dock_retry_count = 0
        self.delivery_retry_count = 0
        self.active_goal_id = 0
        self.recovery_resume = None  # (state, target_name, waypoint) to resume after a recovery maneuver

        # TF Buffer and Listener for accurate map-frame robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action Clients for Nav2 (navigation + collision-checked BackUp behavior)
        if HAVE_NAV2:
            self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.backup_action_client = ActionClient(self, BackUp, 'backup')
        else:
            self.nav_action_client = None
            self.backup_action_client = None

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dock_pub = self.create_publisher(Bool, '/dock_state', 10)
        self.status_pub = self.create_publisher(String, '/delivery_queue_status', 10)
        self.state_pub = self.create_publisher(String, '/robot_system_state', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.waypoints_pub = self.create_publisher(String, '/delivery_waypoints', latched)

        # Battery state tracking
        self.battery_pct = 100.0
        self.battery_docking_lock = False

        # Dynamic Obstacle & Perception Tracking
        self.obs_dist = 99.0
        self.obs_blocking = False
        self.obs_mode = "unknown"

        # Costmap clearing clients for dynamic recovery
        try:
            from nav2_msgs.srv import ClearEntireCostmap
            self.clear_global_costmap_cli = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            self.clear_local_costmap_cli = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        except Exception:
            self.clear_global_costmap_cli = None
            self.clear_local_costmap_cli = None

        # Subscribers
        self.create_subscription(String, '/delivery_request', self.delivery_request_cb, 10)
        self.create_subscription(String, '/delivery_task_action', self.task_action_cb, 10)
        self.create_subscription(String, '/dock_command', self.dock_command_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(String, '/battery_status', self.battery_status_cb, 10)
        self.create_subscription(String, '/dynamic_obstacle/status', self.obstacle_status_cb, 10)

        wp_msg = String()
        wp_msg.data = json.dumps({"world": self.world_name, "waypoints": list(self.manager.waypoints.keys())})
        self.waypoints_pub.publish(wp_msg)

        # Control loop at 2Hz for responsive execution
        self.timer = self.create_timer(0.5, self.control_loop)
        self.get_logger().info("Delivery Task Manager Node active (Complex Layout & Battery Safety Active).")

    def _now(self) -> float:
        """Node clock in seconds (follows /clock when use_sim_time is set)."""
        return self.get_clock().now().nanoseconds / 1e9

    def obstacle_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.obs_dist = float(data.get("distance_to_robot", 99.0))
            self.obs_blocking = bool(data.get("is_blocking", False))
            self.obs_mode = str(data.get("mode", "unknown"))
        except Exception:
            pass

    def _clear_costmaps(self):
        try:
            from nav2_msgs.srv import ClearEntireCostmap
            req = ClearEntireCostmap.Request()
            if self.clear_global_costmap_cli and self.clear_global_costmap_cli.service_is_ready():
                self.clear_global_costmap_cli.call_async(req)
            if self.clear_local_costmap_cli and self.clear_local_costmap_cli.service_is_ready():
                self.clear_local_costmap_cli.call_async(req)
        except Exception:
            pass

    def battery_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.battery_pct = float(data.get("percentage", self.battery_pct))
        except Exception:
            return
        # A docked robot at low charge must recharge fully before taking new work
        if self.manager.robot_state == "docked" and self.battery_pct <= self.low_battery_threshold and not self.battery_docking_lock:
            self.battery_docking_lock = True
            self.get_logger().warn(f"Docked at low battery ({self.battery_pct:.1f}%). Holding orders until {self.min_recharge_pct:.0f}%.")
        if self.battery_pct >= self.min_recharge_pct and self.battery_docking_lock:
            self.battery_docking_lock = False
            self.get_logger().info(f"Battery sufficiently recharged (>={self.min_recharge_pct:.0f}%). Normal order dispatch unlocked.")

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def task_action_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            action = data.get("action", "")
            if action == "delete":
                task_id = data.get("task_id", "")
                if self.manager.delete_task(task_id):
                    self.get_logger().info(f"Deleted task {task_id} from pending queue.")
                    self.publish_status()
                elif self.manager.cancel_current_task(task_id):
                    self.get_logger().info(f"Cancelled active delivery {task_id}. Robot halting and returning to IDLE.")
                    self._abort_active_delivery()
                else:
                    self.get_logger().warn(f"Could not delete task {task_id}: not found.")
            elif action == "modify":
                task_id = data.get("task_id", "")
                target = data.get("target")
                item = data.get("item")
                if self.manager.modify_task(task_id, target, item):
                    self.get_logger().info(f"Modified task {task_id}: target={target}, item={item}")
                    self.publish_status()
                else:
                    found, target_changed = self.manager.modify_current_task(task_id, target, item)
                    if not found:
                        self.get_logger().warn(f"Could not modify task {task_id}: not found or target invalid.")
                    elif target_changed and self.manager.robot_state in ["navigating", "recovering", "at_table"]:
                        new_target = self.manager.current_task["target"]
                        self.get_logger().info(f"Active delivery {task_id} redirected to {new_target}.")
                        self.delivery_retry_count = 0
                        self.recovery_resume = None
                        self.manager.robot_state = "navigating"
                        self.dispatch_goal(new_target, self.manager.waypoints[new_target])
                        self.publish_status()
                    else:
                        self.get_logger().info(f"Modified active task {task_id}: item={item}")
                        self.publish_status()
            elif action in ["stop_dock", "cancel_dock"]:
                self.request_stop_docking()
        except Exception as e:
            self.get_logger().error(f"Error handling task action: {e}")

    def _abort_active_delivery(self):
        """Stops the robot after its in-flight delivery was cancelled."""
        self.cancel_active_navigation()
        self.dwell_start_time = None
        self.target_coords = None
        self.target_name = None
        self.nav_goal_succeeded = False
        self.recovery_resume = None
        self.delivery_retry_count = 0
        self.manager.robot_state = "idle"
        self.publish_status()

    def delivery_request_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            target = data.get('target', '')
            item = data.get('item', '')
            res = self.manager.add_delivery_request(target, item)
            if res['success']:
                task_id = res['task']['id']
                resolved_target = res['task']['target']
                self.get_logger().info(f"Accepted new delivery: {task_id} -> {resolved_target} ({item})")

                # If currently docking, check if battery lock allows preemption
                if self._is_docking():
                    if self.battery_docking_lock or self.battery_pct <= self.low_battery_threshold:
                        self.get_logger().warn(f"Battery is low ({self.battery_pct:.1f}%). Holding new order in queue until recharge complete.")
                    else:
                        self.get_logger().info("New delivery order received while docking! Preempting docking sequence.")
                        self.stop_docking()

                # Dispatch next task if robot is ready
                self.check_and_dispatch_next_task()
                self.publish_status()
            else:
                self.get_logger().warn(f"Rejected delivery request: {res['error']}")
        except Exception as e:
            self.get_logger().error(f"Error parsing delivery request payload: {e}")

    def dock_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        if "STOP" in cmd or "CANCEL" in cmd:
            self.request_stop_docking()
        elif "DOCK_NOW" in cmd:
            if "BATTERY" in cmd:
                self.battery_docking_lock = True
                self.get_logger().warn(f"Battery-protective docking lock ENGAGED: {cmd}")

            # If robot is dwelling at a table, preempt dwell and return to dock
            if self.manager.robot_state == "at_table":
                self.get_logger().info("Preempting table dwell for return-to-dock!")
                self.dwell_start_time = None
                self.nav_goal_succeeded = False
                if self.manager.current_task:
                    self.manager.complete_current_task(success=True)

            # An order still en route goes back to the head of the queue instead of being lost
            if self.manager.robot_state in ["navigating", "recovering"] and self.manager.current_task:
                task = self.manager.requeue_current_task(front=True)
                self.get_logger().warn(f"Return-to-dock interrupted {task['id']} -> {task['target']}. Order returned to head of queue.")
                self.delivery_retry_count = 0
                self.recovery_resume = None

            if self.manager.robot_state != "docked" and not self._is_docking():
                self.dock_retry_count = 0
                self.start_docking()

    def request_stop_docking(self):
        """Operator/controller request to abort docking; refused while the battery is critical."""
        if self.battery_pct <= self.critical_battery_threshold:
            self.get_logger().warn(f"Emergency override: Cannot cancel docking while battery is critical (<={self.critical_battery_threshold:.0f}%)!")
            return
        self.battery_docking_lock = False
        self.stop_docking()

    def _is_docking(self) -> bool:
        if self.manager.robot_state == "docking":
            return True
        return self.manager.robot_state == "recovering" and bool(self.recovery_resume) and self.recovery_resume[0] == "docking"

    def start_docking(self):
        dock_wp = self.manager.waypoints.get("Dock")
        if not dock_wp:
            self.get_logger().error("Dock waypoint not found in configuration.")
            return

        tx = float(dock_wp.get('x', 7.06))
        ty = float(dock_wp.get('y', -12.00))

        # Reset arrival flags
        self.dwell_start_time = None
        self.nav_goal_succeeded = False
        self.recovery_resume = None

        if self.manager.robot_state == "docked":
            self.get_logger().info("Robot is already in DOCKED state.")
            return

        # Check if robot is at initial Dock pose (and has never left)
        if len(self.manager.task_history) == 0 and self.target_name is None and self.manager.current_task is None:
            dist = math.hypot(self.current_x - tx, self.current_y - ty)
            if dist < 0.60:
                self.get_logger().info(f"Robot is at initial Dock pose (distance: {dist:.2f}m). Setting state to DOCKED.")
                self.manager.robot_state = "docked"
                self.target_coords = None
                self.target_name = None
                dock_msg = Bool()
                dock_msg.data = True
                self.dock_pub.publish(dock_msg)
                self.publish_status()
                return

        # Navigate to resting dock
        self.get_logger().info(f"Routing robot to Resting Dock Station ({tx:.2f}, {ty:.2f}).")
        self.manager.robot_state = "docking"
        self.dispatch_goal("Dock", dock_wp)
        self.publish_status()

    def stop_docking(self):
        if self._is_docking():
            self.get_logger().info("Stopping/Cancelling docking procedure...")
            self.cancel_active_navigation()
            self.recovery_resume = None
            self.manager.robot_state = "idle"
            self.target_coords = None
            self.target_name = None
            dock_msg = Bool()
            dock_msg.data = False
            self.dock_pub.publish(dock_msg)
            self.publish_status()

    def cancel_active_navigation(self):
        self.active_goal_id += 1
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"Exception cancelling goal: {e}")
            self.current_goal_handle = None

        # Stop robot base
        stop_cmd = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(stop_cmd)

    def _goal_response_cb(self, future, goal_id: int, target_name: str):
        if goal_id != self.active_goal_id:
            try:
                gh = future.result()
                if gh and gh.accepted:
                    gh.cancel_goal_async()
            except Exception:
                pass
            return

        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"Nav2 Goal #{goal_id} for '{target_name}' was rejected by server.")
                if self.manager.robot_state == "docking":
                    self._handle_docking_failure()
                else:
                    self._handle_delivery_failure()
                return
            self.current_goal_handle = goal_handle
            self.get_logger().info(f"Nav2 Goal #{goal_id} for '{target_name}' accepted by server.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda fut, gid=goal_id, tgt=target_name: self._goal_result_cb(fut, gid, tgt)
            )
        except Exception as e:
            self.get_logger().warn(f"Goal response exception: {e}")

    def trigger_arrival(self):
        """Called when robot reaches destination (via Nav2 success or arrival tolerance)."""
        if self.manager.robot_state in ["navigating", "en_route", "at_table"]:
            if self.dwell_start_time is None:
                self.delivery_retry_count = 0
                self.dwell_start_time = self._now()
                self.manager.robot_state = "at_table"
                self.get_logger().info(f"Reached destination '{self.target_name}'. Handing over delivery...")
                self.publish_status()
        elif self.manager.robot_state == "docking":
            self.manager.robot_state = "docked"
            self.dock_retry_count = 0
            self.target_coords = None
            self.target_name = None
            self.current_goal_handle = None

            # Brake robot base cleanly at dock
            stop_cmd = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(stop_cmd)

            dock_msg = Bool()
            dock_msg.data = True
            self.dock_pub.publish(dock_msg)
            self.get_logger().info("Robot safely parked at resting dock.")
            self.publish_status()

    def _goal_result_cb(self, future, goal_id: int, target_name: str):
        if goal_id != self.active_goal_id:
            self.get_logger().info(f"Ignoring stale Nav2 result for goal #{goal_id} ('{target_name}'). Active goal is #{self.active_goal_id}.")
            return

        try:
            res = future.result()
            status = res.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Nav2 trajectory for goal #{goal_id} ('{target_name}') reported SUCCESS.")
                self.nav_goal_succeeded = True
                self.trigger_arrival()
            elif status in [GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED]:
                self.get_logger().warn(f"Nav2 goal #{goal_id} for '{target_name}' aborted or cancelled (status {status}).")
                if self.manager.robot_state == "docking":
                    self._handle_docking_failure()
                elif self.manager.robot_state in ["navigating", "en_route"] and self.target_coords is not None:
                    self._handle_delivery_failure()
        except Exception as e:
            self.get_logger().warn(f"Goal result callback exception: {e}")

    def _start_recovery(self, resume_state: str, target_name: str, wp: dict):
        """
        Non-blocking recovery maneuver for tight corridors, narrow doorways, and dynamic obstacles.
        1. Backs up the robot via Nav2's collision-checked BackUp behavior (manual reverse if unavailable).
        2. Clears costmaps and re-dispatches the interrupted goal.
        Any new goal or cancellation bumps active_goal_id, which aborts a pending recovery.
        """
        self.active_goal_id += 1
        token = self.active_goal_id
        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

        self.recovery_resume = (resume_state, target_name, wp)
        self.manager.robot_state = "recovering"
        self.get_logger().warn(f"Executing Corridor & Doorway Recovery: Backing up {self.backup_distance:.2f}m and resetting trajectory...")
        self.publish_status()

        if self.backup_action_client and self.backup_action_client.server_is_ready():
            goal = BackUp.Goal()
            goal.target.x = self.backup_distance
            goal.speed = self.backup_speed
            goal.time_allowance.sec = 10
            future = self.backup_action_client.send_goal_async(goal)
            future.add_done_callback(lambda fut, t=token: self._backup_response_cb(fut, t))
        else:
            self._manual_backup(token)

    def _backup_response_cb(self, future, token: int):
        try:
            goal_handle = future.result()
        except Exception:
            goal_handle = None
        if token != self.active_goal_id:
            if goal_handle and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle or not goal_handle.accepted:
            self._manual_backup(token)
            return
        self.current_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(lambda _fut, t=token: self._finish_recovery(t))

    def _manual_backup(self, token: int):
        """Fallback reverse pulse on /cmd_vel when the Nav2 BackUp behavior is unavailable."""
        duration = self.backup_distance / max(0.01, self.backup_speed)
        end_time = self._now() + duration
        backup_cmd = Twist()
        backup_cmd.linear.x = -abs(self.backup_speed)
        self.cmd_vel_pub.publish(backup_cmd)

        def tick():
            if token != self.active_goal_id:
                self.destroy_timer(timer)
                return
            if self._now() >= end_time:
                self.destroy_timer(timer)
                self.cmd_vel_pub.publish(Twist())
                self._finish_recovery(token)
                return
            self.cmd_vel_pub.publish(backup_cmd)

        timer = self.create_timer(0.1, tick)

    def _finish_recovery(self, token: int):
        if token != self.active_goal_id or self.manager.robot_state != "recovering" or not self.recovery_resume:
            return
        resume_state, target_name, wp = self.recovery_resume
        self.recovery_resume = None
        self.current_goal_handle = None
        self.manager.robot_state = resume_state
        self._clear_costmaps()
        self.dispatch_goal(target_name, wp)
        self.publish_status()

    def _handle_docking_failure(self):
        self.dock_retry_count += 1
        dock_wp = self.manager.waypoints.get("Dock")
        if self.dock_retry_count <= 3 and dock_wp:
            self.get_logger().warn(f"Docking goal aborted by Nav2. Triggering layout recovery attempt {self.dock_retry_count}/3...")
            self._start_recovery("docking", "Dock", dock_wp)
            return

        self.get_logger().warn("Docking procedure failed after retries. Resetting robot state to IDLE.")
        self.dock_retry_count = 0
        self.target_coords = None
        self.target_name = None
        self.current_goal_handle = None
        self.manager.robot_state = "idle"
        dock_msg = Bool()
        dock_msg.data = False
        self.dock_pub.publish(dock_msg)
        self.publish_status()

    def _handle_delivery_failure(self):
        self.delivery_retry_count += 1
        target = self.target_name
        wp = self.manager.waypoints.get(target) if target else None
        if self.delivery_retry_count <= 3 and wp:
            self.get_logger().warn(f"Delivery goal for '{target}' aborted by Nav2. Triggering layout recovery attempt {self.delivery_retry_count}/3 (Obs Dist: {self.obs_dist:.2f}m)...")
            self._start_recovery("navigating", target, wp)
            return

        # If retries exhausted, keep task in queue so it is never dropped or skipped
        self.get_logger().warn(f"Delivery to '{self.target_name}' postponed after retries. Keeping order in queue.")
        self.delivery_retry_count = 0
        self.manager.requeue_current_task(front=False)

        self.dwell_start_time = None
        self.target_coords = None
        self.target_name = None
        self.current_goal_handle = None
        self.manager.robot_state = "idle"
        self.check_and_dispatch_next_task()
        self.publish_status()

    def dispatch_goal(self, target_name: str, wp: dict):
        x = float(wp.get('x', 0.0))
        y = float(wp.get('y', 0.0))
        qz = float(wp.get('qz', 0.0))
        qw = float(wp.get('qw', 1.0))

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.target_coords = (x, y)
        self.target_name = target_name
        self.dwell_start_time = None
        self.nav_goal_succeeded = False

        self.active_goal_id += 1
        goal_id = self.active_goal_id

        if self.current_goal_handle:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

        if self.nav_action_client and (self.nav_action_client.server_is_ready() or self.nav_action_client.wait_for_server(timeout_sec=0.2)):
            action_goal = NavigateToPose.Goal()
            action_goal.pose = goal
            send_goal_future = self.nav_action_client.send_goal_async(action_goal)
            send_goal_future.add_done_callback(
                lambda fut, gid=goal_id, tgt=target_name: self._goal_response_cb(fut, gid, tgt)
            )
        else:
            self.goal_pub.publish(goal)

        self.get_logger().info(f"Dispatched Nav2 Goal #{goal_id} for '{target_name}' at ({x:.2f}, {y:.2f})")

    def _battery_allows_dispatch(self) -> bool:
        if self.battery_docking_lock:
            return False
        if self.battery_pct <= self.critical_battery_threshold:
            return False
        if self.manager.robot_state == "docked" and self.battery_pct <= self.low_battery_threshold:
            return False
        return True

    def check_and_dispatch_next_task(self):
        """Dispatches the next order from the queue if the robot is idle or docked and the battery allows it."""
        if self.manager.robot_state not in ["idle", "docked"] or not self.manager.queue:
            return
        if not self._battery_allows_dispatch():
            return

        if self.manager.robot_state == "docked":
            dock_msg = Bool()
            dock_msg.data = False
            self.dock_pub.publish(dock_msg)

        task = self.manager.get_next_task()
        if not task:
            return
        target = task["target"]
        wp = self.manager.waypoints.get(target)
        if not wp:
            return
        tx = float(wp.get('x', 0.0))
        ty = float(wp.get('y', 0.0))

        # Check if robot is already at this table
        d = math.hypot(self.current_x - tx, self.current_y - ty)
        if d < self.arrival_tolerance and target != "Dock":
            self.get_logger().info(f"Robot is already at {target} (dist: {d:.2f}m). Serving delivery {task['id']} directly.")
            self.target_coords = (tx, ty)
            self.target_name = target
            self.manager.robot_state = "at_table"
            self.dwell_start_time = self._now()
            self.publish_status()
            return

        self.manager.robot_state = "navigating"
        self.delivery_retry_count = 0
        self.dispatch_goal(target, wp)
        self.get_logger().info(f"Popped {task['id']} from queue -> en route to {target}!")
        self.publish_status()

    def publish_status(self):
        summary = self.manager.get_status_summary()
        summary["battery_docking_lock"] = self.battery_docking_lock
        state_msg = String()
        state_msg.data = json.dumps(summary)
        self.status_pub.publish(state_msg)

        sys_state_msg = String()
        sys_state_msg.data = summary['robot_state']
        self.state_pub.publish(sys_state_msg)

    def _update_pose_from_tf(self):
        for frame in ['base_footprint', 'base_link']:
            try:
                t = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                self.current_x = t.transform.translation.x
                self.current_y = t.transform.translation.y
                return
            except Exception:
                pass

    def control_loop(self):
        # 0. Query TF for exact robot pose in map frame
        self._update_pose_from_tf()

        # 1. Monitor active movement and arrival at table or dock
        if self.target_coords is not None:
            tx, ty = self.target_coords
            dist = math.hypot(self.current_x - tx, self.current_y - ty)

            # Trigger arrival if within tight tolerance or Nav2 reported success
            if dist < self.arrival_tolerance or self.nav_goal_succeeded:
                self.trigger_arrival()

        # 2. Check dwell completion at table
        if self.manager.robot_state == "at_table" and self.dwell_start_time is not None:
            if self._now() - self.dwell_start_time >= self.dwell_time:
                finished = self.manager.complete_current_task(success=True)
                task_id = finished['id'] if finished else 'TASK'
                self.get_logger().info(f"Delivery completed for {task_id} at {self.target_name}! Robot now IDLE.")

                # Stop robot cleanly
                stop_cmd = Twist()
                for _ in range(2):
                    self.cmd_vel_pub.publish(stop_cmd)

                self.dwell_start_time = None
                self.target_coords = None
                self.target_name = None
                self.current_goal_handle = None
                self.nav_goal_succeeded = False
                self.manager.robot_state = "idle"
                self.publish_status()

        # 3. Check queue dispatch if robot is ready.
        # Idle return-to-dock is owned by the docking_controller node (DOCK_NOW:dock_idle).
        self.check_and_dispatch_next_task()

        # 4. Publish system status summaries
        self.publish_status()


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)
        node = DeliveryTaskManagerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    else:
        print("Running TaskQueueManager standalone test mode:")
        mgr = TaskQueueManager()
        mgr.add_delivery_request("Table 1", "Burger & Fries")
        mgr.add_delivery_request("Table 2", "Coffee")
        print("Queue length:", mgr.get_status_summary()["queue_length"])
        task = mgr.get_next_task()
        if task:
            print("Dispatched task:", task["id"], "to", task["target"])
        mgr.complete_current_task()
        print("Remaining in queue:", mgr.get_status_summary()["queue_length"])


if __name__ == '__main__':
    main()
