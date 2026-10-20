"""Regression tests for task-manager, battery and obstacle node behaviour (require rclpy)."""
import json
import math
import time
import unittest

import pytest

rclpy = pytest.importorskip('rclpy')

from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from cstam_core.battery_simulator import BatterySimulatorNode  # noqa: E402
from cstam_core.delivery_task_manager import DeliveryTaskManagerNode  # noqa: E402
from cstam_core.dynamic_obstacle_controller import DynamicObstacleControllerNode  # noqa: E402


def setUpModule():
    if not rclpy.ok():
        rclpy.init()


def tearDownModule():
    if rclpy.ok():
        rclpy.shutdown()


class TaskManagerRegressionTests(unittest.TestCase):

    def setUp(self):
        self.node = DeliveryTaskManagerNode()
        # No Nav2 in unit tests: goals fall back to /goal_pose, nothing moves
        self.node.nav_action_client = None
        self.node.backup_action_client = None
        self.mgr = self.node.manager
        # Start in the dining area, away from the dock and every table
        self.node.current_x, self.node.current_y = 0.0, -1.5

    def tearDown(self):
        self.node.destroy_node()

    def _battery(self, pct):
        self.node.battery_status_cb(String(data=json.dumps({"percentage": pct})))

    def _dock(self):
        self.mgr.robot_state = "docking"
        self.node.trigger_arrival()
        self.assertEqual(self.mgr.robot_state, "docked")

    def test_battery_lock_holds_robot_at_dock_until_recharged(self):
        self._battery(20.0)
        self.node.dock_command_cb(String(data="DOCK_NOW:dock_low_battery"))
        self._dock()
        self.mgr.add_delivery_request("Table 3", "x")

        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "docked", "Robot must not leave the dock before recharge")
        self._battery(79.0)
        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "docked")

        self._battery(80.0)
        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "navigating")
        self.assertEqual(self.node.target_name, "Table 3")

    def test_docked_at_low_battery_without_lock_still_recharges(self):
        self._dock()
        self._battery(22.0)
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "docked")
        self.assertTrue(self.node.battery_docking_lock)

    def test_idle_docked_robot_with_good_battery_serves_orders(self):
        self._battery(70.0)
        self._dock()
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "navigating")

    def test_critical_dock_requeues_in_flight_order(self):
        self.mgr.add_delivery_request("Table 3", "first")
        self.mgr.add_delivery_request("Table 5", "second")
        self.node.check_and_dispatch_next_task()
        in_flight = self.mgr.current_task["id"]

        self._battery(12.0)
        self.node.dock_command_cb(String(data="DOCK_NOW:dock_critical_battery"))
        self.assertEqual(self.mgr.robot_state, "docking")
        self.assertIsNone(self.mgr.current_task)
        self.assertEqual(self.mgr.queue[0]["id"], in_flight, "Interrupted order must be first in the queue")
        self.assertEqual(self.mgr.queue[0]["status"], "queued")

        self._dock()
        self.node.control_loop()
        self.assertEqual(self.mgr.robot_state, "docked", "Critical battery must hold the robot at the dock")
        self.assertEqual(len(self.mgr.queue), 2)

    def test_stop_dock_task_action_refused_at_critical_battery(self):
        self._battery(12.0)
        self.node.dock_command_cb(String(data="DOCK_NOW:dock_critical_battery"))
        self.assertEqual(self.mgr.robot_state, "docking")
        self.node.task_action_cb(String(data=json.dumps({"action": "stop_dock"})))
        self.assertEqual(self.mgr.robot_state, "docking")
        self.node.dock_command_cb(String(data="STOP_DOCK"))
        self.assertEqual(self.mgr.robot_state, "docking")

    def test_stop_dock_allowed_with_safe_battery(self):
        self.node.dock_command_cb(String(data="DOCK_NOW:manual"))
        self.assertEqual(self.mgr.robot_state, "docking")
        self.node.task_action_cb(String(data=json.dumps({"action": "stop_dock"})))
        self.assertEqual(self.mgr.robot_state, "idle")

    def test_cancel_active_delivery(self):
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.check_and_dispatch_next_task()
        task_id = self.mgr.current_task["id"]
        self.node.task_action_cb(String(data=json.dumps({"action": "delete", "task_id": task_id})))
        self.assertIsNone(self.mgr.current_task)
        self.assertEqual(self.mgr.robot_state, "idle")
        self.assertEqual(self.mgr.task_history[-1]["status"], "cancelled")
        self.assertIsNone(self.node.target_name)

    def test_redirect_active_delivery(self):
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.check_and_dispatch_next_task()
        task_id = self.mgr.current_task["id"]
        self.node.task_action_cb(String(data=json.dumps(
            {"action": "modify", "task_id": task_id, "target": "Table 7", "item": "y"})))
        self.assertEqual(self.mgr.current_task["target"], "Table 7")
        self.assertEqual(self.node.target_name, "Table 7")
        self.assertEqual(self.mgr.robot_state, "navigating")

    def test_recovery_resumes_goal_and_can_be_preempted(self):
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.check_and_dispatch_next_task()
        self.node.backup_distance = 0.03  # 0.2 s manual reverse
        self.node._handle_delivery_failure()
        self.assertEqual(self.mgr.robot_state, "recovering")

        deadline = time.time() + 3.0
        while time.time() < deadline and self.mgr.robot_state == "recovering":
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(self.mgr.robot_state, "navigating")
        self.assertEqual(self.node.target_name, "Table 3")

        # A dock command during a recovery cancels it and keeps the order
        self.node._handle_delivery_failure()
        self.node.dock_command_cb(String(data="DOCK_NOW:manual"))
        self.assertEqual(self.mgr.robot_state, "docking")
        self.assertEqual(self.mgr.queue[0]["target"], "Table 3")
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(self.mgr.robot_state, "docking")

    def test_retries_exhausted_keeps_order(self):
        self.mgr.add_delivery_request("Table 3", "x")
        self.node.check_and_dispatch_next_task()
        self.node.delivery_retry_count = 3
        self.node._handle_delivery_failure()
        # Only task -> requeued and immediately re-dispatched, never dropped
        self.assertEqual(self.mgr.current_task["target"], "Table 3")
        self.assertEqual(self.mgr.task_history, [])


class BatteryNodeRegressionTests(unittest.TestCase):

    def setUp(self):
        self.node = BatterySimulatorNode()
        self.msgs = []
        from sensor_msgs.msg import BatteryState
        self.node.battery_state_pub.publish = self.msgs.append  # capture without spinning
        self.BatteryState = BatteryState

    def tearDown(self):
        self.node.destroy_node()

    def test_current_sign_follows_ros_convention(self):
        self.node.sim.is_docked = False
        self.node.timer_callback()
        self.assertLess(self.msgs[-1].current, 0.0, "Discharging current must be negative")
        self.node.sim.is_docked = True
        self.node.timer_callback()
        self.assertGreater(self.msgs[-1].current, 0.0, "Charging current must be positive")

    def test_override_is_clamped(self):
        self.node.set_percentage_cb(String(data="150"))
        self.assertAlmostEqual(self.msgs[-1].percentage, 1.0)

    def test_acceleration_from_odom(self):
        def odom(v):
            m = Odometry()
            m.twist.twist.linear.x = v
            return m
        self.node.odom_cb(odom(0.0))
        time.sleep(0.1)
        self.node.odom_cb(odom(0.4))
        self.assertTrue(self.node.sim.is_accelerating)
        time.sleep(0.1)
        self.node.odom_cb(odom(0.4))
        self.assertFalse(self.node.sim.is_accelerating)


class ObstacleNodeRegressionTests(unittest.TestCase):

    def setUp(self):
        self.node = DynamicObstacleControllerNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_odom_is_converted_to_map_frame(self):
        # Spawned at (-1.5, -1.3) facing +Y; 1 m forward in odom -> (-1.5, -0.3) in the map
        m = Odometry()
        m.pose.pose.position.x = 1.0
        m.pose.pose.orientation.w = 1.0
        self.node.odom_cb(m)
        self.assertAlmostEqual(self.node.agent.x, -1.5, places=2)
        self.assertAlmostEqual(self.node.agent.y, -0.3, places=2)
        self.assertAlmostEqual(self.node.current_yaw, math.pi / 2, places=2)

    def test_unknown_robot_pose_is_not_blocking(self):
        published = []
        self.node.status_pub.publish = published.append
        self.node.cmd_vel_pub.publish = lambda _m: None
        self.node.loop()
        status = json.loads(published[-1].data)
        self.assertFalse(status["is_blocking"])

    def test_block_mode_targets_robot_when_pose_known(self):
        self.node.amcl_cb(type('M', (), {'pose': type('P', (), {'pose': type('Q', (), {
            'position': type('R', (), {'x': -1.5, 'y': 1.0})()})()})()})())
        self.node.control_cb(String(data="block"))
        self.assertAlmostEqual(self.node.agent.target_x, -1.5)
        self.assertAlmostEqual(self.node.agent.target_y, 0.0)


if __name__ == '__main__':
    unittest.main()
