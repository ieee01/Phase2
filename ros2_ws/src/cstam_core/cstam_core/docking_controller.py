#!/usr/bin/env python3
"""
CSTAM Auto-Docking Controller Node
Manages autonomous return-to-dock behaviors:
1. Idle timeout triggers (10s empty queue countdown)
2. Low battery (<25%) auto-dock trigger
3. Critical battery (<15%) emergency dock override
4. Safe CC-CV recharge before releasing to new delivery tasks
5. Preemption by new delivery orders (when battery is safe)
"""

import time
import json

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String, Bool
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs): pass


class AutoDockingController:
    """
    Auto-docking state controller.
    Manages return-to-dock sequence on idle timeout, low battery, and manual triggers.
    """
    def __init__(self, idle_timeout=10.0, low_battery_threshold=25.0, critical_battery_threshold=15.0, min_recharge_pct=80.0):
        self.idle_timeout = float(idle_timeout)
        self.low_battery_threshold = float(low_battery_threshold)
        self.critical_battery_threshold = float(critical_battery_threshold)
        self.min_recharge_pct = float(min_recharge_pct)

        self.idle_since = None
        self.is_docked = False
        self.is_docking_in_progress = False
        self.docking_reason = None
        self.battery_percentage = 100.0

    def evaluate_dock_trigger(self, queue_empty: bool, is_navigating: bool, current_time: float, battery_pct: float | None = None) -> str:
        """
        Determines if docking procedure should be initiated.
        Returns: 'dock_critical_battery', 'dock_low_battery', 'dock_idle', 'remain_docked', 'undock_ready', or 'none'
        """
        if battery_pct is not None:
            self.battery_percentage = float(battery_pct)

        # 1. State while ALREADY DOCKED
        if self.is_docked:
            # If battery is low, MUST remain docked to recharge safely
            if self.battery_percentage < self.min_recharge_pct:
                return 'remain_docked'

            # Once recharged, robot can undock if orders are waiting
            if not queue_empty:
                self.is_docked = False
                self.is_docking_in_progress = False
                self.docking_reason = None
                return 'undock_ready'
            return 'remain_docked'

        # 2. CRITICAL BATTERY EMERGENCY OVERRIDE (< 15%)
        # Overrides any active task to prevent robot shutdown mid-aisle
        if self.battery_percentage <= self.critical_battery_threshold:
            self.docking_reason = 'dock_critical_battery'
            return 'dock_critical_battery'

        # 3. LOW BATTERY RETURN-TO-DOCK (< 25%)
        # Triggered when idle or queue is empty before taking on more work
        if self.battery_percentage <= self.low_battery_threshold:
            if not is_navigating or queue_empty:
                self.docking_reason = 'dock_low_battery'
                return 'dock_low_battery'

        # 4. IDLE TIMEOUT TRIGGER
        if queue_empty and not is_navigating:
            if self.idle_since is None:
                self.idle_since = current_time
            elif current_time - self.idle_since >= self.idle_timeout:
                self.docking_reason = 'dock_idle'
                return 'dock_idle'
        else:
            self.idle_since = None
            if not is_navigating:
                self.is_docking_in_progress = False

        return 'none'


if HAVE_ROS2:
    class DockingControllerNode(Node):
        def __init__(self):
            super().__init__('docking_controller')

            self.declare_parameter('idle_timeout', 10.0)
            self.declare_parameter('low_battery_threshold', 25.0)
            self.declare_parameter('critical_battery_threshold', 15.0)
            self.declare_parameter('min_recharge_pct', 80.0)

            idle_t = self.get_parameter('idle_timeout').value
            low_bat = self.get_parameter('low_battery_threshold').value
            crit_bat = self.get_parameter('critical_battery_threshold').value
            recharge_t = self.get_parameter('min_recharge_pct').value

            self.controller = AutoDockingController(
                idle_timeout=idle_t,
                low_battery_threshold=low_bat,
                critical_battery_threshold=crit_bat,
                min_recharge_pct=recharge_t
            )

            self.command_pub = self.create_publisher(String, '/dock_command', 10)

            # Subscriptions
            self.create_subscription(String, '/delivery_queue_status', self.queue_status_cb, 10)
            self.create_subscription(Bool, '/dock_state', self.dock_state_cb, 10)
            self.create_subscription(BatteryState, '/battery_state', self.battery_state_cb, 10)
            self.create_subscription(String, '/battery_status', self.battery_status_cb, 10)

            self.queue_empty = True
            self.is_navigating = False
            self.battery_pct = 100.0
            self.robot_state = 'idle'
            self.last_dock_cmd_time = 0.0
            self.last_dock_action = None

            self.timer = self.create_timer(1.0, self.timer_callback)
            self.get_logger().info("Docking Controller Node operational (Idle & Battery Monitoring Active).")

        def battery_state_cb(self, msg: BatteryState):
            self.battery_pct = float(msg.percentage * 100.0)
            self.check_dock_trigger()

        def battery_status_cb(self, msg: String):
            try:
                data = json.loads(msg.data)
                self.battery_pct = float(data.get('percentage', self.battery_pct))
                self.check_dock_trigger()
            except Exception:
                pass

        def queue_status_cb(self, msg: String):
            try:
                data = json.loads(msg.data)
                q_len = data.get('queue_length', 0)
                has_curr = data.get('current_task') is not None
                self.queue_empty = (q_len == 0 and not has_curr)
                self.robot_state = data.get('robot_state', 'idle')
                self.is_navigating = self.robot_state in ['navigating', 'en_route', 'at_table', 'delivering', 'docking', 'recovering']
                
                # Preempt docking ONLY if reason was idle, NOT low or critical battery
                if not self.queue_empty and self.robot_state == 'docking':
                    is_battery_dock = self.controller.docking_reason in ['dock_low_battery', 'dock_critical_battery']
                    if not is_battery_dock and self.battery_pct > self.controller.low_battery_threshold:
                        self.get_logger().info("New order detected while docking (safe battery). Preempting docking sequence.")
                        stop_msg = String()
                        stop_msg.data = "STOP_DOCK"
                        self.command_pub.publish(stop_msg)
                        self.controller.is_docking_in_progress = False
                    elif is_battery_dock and self.battery_pct <= self.controller.low_battery_threshold:
                        self.get_logger().warn(f"Battery is low ({self.battery_pct:.1f}%). Preemption refused until recharged.")

                if self.robot_state == 'docked':
                    self.controller.is_docking_in_progress = False
                elif self.robot_state == 'docking':
                    self.controller.is_docking_in_progress = True
                elif not self.queue_empty or self.robot_state == 'idle':
                    if not (self.controller.docking_reason and 'battery' in self.controller.docking_reason and self.battery_pct <= self.controller.low_battery_threshold):
                        self.controller.is_docking_in_progress = False
            except Exception:
                pass

        def dock_state_cb(self, msg: Bool):
            self.controller.is_docked = msg.data
            if msg.data:
                self.controller.is_docking_in_progress = False

        def check_dock_trigger(self):
            # Node clock follows /clock when use_sim_time is set
            now = self.get_clock().now().nanoseconds / 1e9
            action = self.controller.evaluate_dock_trigger(
                self.queue_empty, self.is_navigating, now, battery_pct=self.battery_pct
            )

            if action.startswith('dock_') and not self.controller.is_docked:
                reason_escalated = (action == 'dock_critical_battery' and self.last_dock_action != 'dock_critical_battery')
                if not self.controller.is_docking_in_progress or reason_escalated or (now - self.last_dock_cmd_time > 15.0 and self.robot_state != 'docking'):
                    self.get_logger().warn(f"Triggering auto-docking sequence. Reason: {action} (Battery: {self.battery_pct:.1f}%)")
                    cmd = String()
                    cmd.data = f"DOCK_NOW:{action}"
                    self.command_pub.publish(cmd)
                    self.controller.is_docking_in_progress = True
                    self.last_dock_cmd_time = now
                    self.last_dock_action = action

        def timer_callback(self):
            self.check_dock_trigger()
else:
    class DockingControllerNode:
        def __init__(self, *args, **kwargs):
            pass


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)
        node = DockingControllerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    else:
        print("Running AutoDockingController standalone test mode:")
        ctrl = AutoDockingController(idle_timeout=5.0, low_battery_threshold=25.0)
        res = ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=time.time(), battery_pct=100.0)
        print(f"Trigger evaluation for 100% idle: {res}")
        res_low = ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=False, current_time=time.time(), battery_pct=22.0)
        print(f"Trigger evaluation for 22% low battery: {res_low}")
        res_crit = ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=True, current_time=time.time(), battery_pct=12.0)
        print(f"Trigger evaluation for 12% critical battery: {res_crit}")


if __name__ == '__main__':
    main()
