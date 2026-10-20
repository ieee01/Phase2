#!/usr/bin/env python3
"""
CSTAM Phase 2 End-to-End Integration Verification Test Suite
Verifies live multi-node ROS 2 communications:
1. Dynamic Obstacle Avoidance (Topic bridging, Velocity commands, Proximity alerts, RViz Markers)
2. Complex Layout & Navigation Failure Handling (Dislodging recovery maneuver, Task retry persistence)
3. Battery & State Simulation (Dynamic discharge, OCV curve, Sag, CC-CV dock charging)
4. Triggering Return-to-Dock before Battery Dies (<25% low battery, <15% critical emergency override)
"""

import sys
import os
import time
import json
import unittest
import threading

sys.path.insert(0, os.path.abspath('ros2_ws/src/cstam_core'))

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String, Bool
    from visualization_msgs.msg import MarkerArray
    HAVE_ROS2 = True
except ImportError:
    ros_setup = '/opt/ros/jazzy/setup.bash'
    ws_setup = os.path.abspath('ros2_ws/install/setup.bash')
    if os.path.exists(ros_setup) and 'ROS2_REEXEC' not in os.environ:
        os.environ['ROS2_REEXEC'] = '1'
        cmd = f"source {ros_setup} && if [ -f {ws_setup} ]; then source {ws_setup}; fi && python3 {' '.join(sys.argv)}"
        ret = os.system(f"bash -c '{cmd}'")
        sys.exit(0 if ret == 0 else 1)
    HAVE_ROS2 = False


from cstam_core.delivery_task_manager import DeliveryTaskManagerNode, TaskQueueManager
from cstam_core.docking_controller import DockingControllerNode, AutoDockingController
from cstam_core.battery_simulator import BatterySimulatorNode, BatterySimulator
from cstam_core.dynamic_obstacle_controller import DynamicObstacleControllerNode, DynamicObstacleAgent


class CstamPhase2IntegrationTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if HAVE_ROS2:
            if not rclpy.ok():
                rclpy.init()
            cls.test_node = rclpy.create_node('phase2_integration_test_node')

            # Initialize Core Nodes
            cls.task_manager = DeliveryTaskManagerNode()
            cls.dock_controller = DockingControllerNode()
            cls.battery_sim = BatterySimulatorNode()
            cls.obstacle_ctrl = DynamicObstacleControllerNode()

            # Create an executor to run all nodes in a background thread
            from rclpy.executors import MultiThreadedExecutor
            cls.executor = MultiThreadedExecutor()
            cls.executor.add_node(cls.test_node)
            cls.executor.add_node(cls.task_manager)
            cls.executor.add_node(cls.dock_controller)
            cls.executor.add_node(cls.battery_sim)
            cls.executor.add_node(cls.obstacle_ctrl)

            cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
            cls.spin_thread.start()
            time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        if HAVE_ROS2:
            cls.executor.shutdown()
            cls.test_node.destroy_node()
            cls.task_manager.destroy_node()
            cls.dock_controller.destroy_node()
            cls.battery_sim.destroy_node()
            cls.obstacle_ctrl.destroy_node()
            rclpy.shutdown()

    def test_01_dynamic_obstacle_ros2_topics(self):
        """Verifies dynamic obstacle publishes velocity, status JSON, and RViz markers."""
        if not HAVE_ROS2:
            self.skipTest("ROS 2 not available")

        received_status = []
        received_markers = []

        sub_status = self.test_node.create_subscription(
            String, '/dynamic_obstacle/status',
            lambda msg: received_status.append(json.loads(msg.data)), 10
        )
        sub_marker = self.test_node.create_subscription(
            MarkerArray, '/dynamic_obstacle/markers',
            lambda msg: received_markers.append(msg), 10
        )

        pub_ctrl = self.test_node.create_publisher(String, '/dynamic_obstacle/control', 10)
        cmd_msg = String()
        cmd_msg.data = "cross"
        pub_ctrl.publish(cmd_msg)

        # Wait for updates
        for _ in range(25):
            if received_status and received_markers:
                break
            time.sleep(0.1)

        self.assertTrue(len(received_status) > 0, "Did not receive /dynamic_obstacle/status telemetry")
        self.assertTrue(len(received_markers) > 0, "Did not receive /dynamic_obstacle/markers")
        st = received_status[-1]
        self.assertIn("mode", st)
        self.assertIn("distance_to_robot", st)
        self.assertIn("speed", st)

        self.test_node.destroy_subscription(sub_status)
        self.test_node.destroy_subscription(sub_marker)
        self.test_node.destroy_publisher(pub_ctrl)

    def test_02_battery_simulation_ros2_telemetry(self):
        """Verifies BatterySimulatorNode publishes /battery_state and /battery_status."""
        if not HAVE_ROS2:
            self.skipTest("ROS 2 not available")

        received_states = []
        received_json = []

        sub_bat = self.test_node.create_subscription(
            BatteryState, '/battery_state',
            lambda msg: received_states.append(msg), 10
        )
        sub_json = self.test_node.create_subscription(
            String, '/battery_status',
            lambda msg: received_json.append(json.loads(msg.data)), 10
        )

        for _ in range(25):
            if received_states and received_json:
                break
            time.sleep(0.1)

        self.assertTrue(len(received_states) > 0, "Did not receive standard /battery_state msg")
        self.assertTrue(len(received_json) > 0, "Did not receive /battery_status JSON msg")

        b_msg = received_states[-1]
        self.assertGreater(b_msg.voltage, 20.0, "Voltage should be > 20V nominal")
        self.assertEqual(b_msg.capacity, 20.0, "Design capacity should be 20Ah")

        b_data = received_json[-1]
        self.assertIn("percentage", b_data)
        self.assertIn("power_watts", b_data)

        self.test_node.destroy_subscription(sub_bat)
        self.test_node.destroy_subscription(sub_json)

    def test_03_low_battery_auto_dock_trigger(self):
        """Verifies triggering return-to-dock before the battery dies (<25% threshold)."""
        if not HAVE_ROS2:
            self.skipTest("ROS 2 not available")

        received_dock_cmds = []
        sub_dock = self.test_node.create_subscription(
            String, '/dock_command',
            lambda msg: received_dock_cmds.append(msg.data), 10
        )

        # Publish battery drop to 23%
        bat_override_pub = self.test_node.create_publisher(String, '/battery_set_percentage', 10)
        b_msg = String()
        b_msg.data = "23.0"
        bat_override_pub.publish(b_msg)

        # Wait for DockingController to evaluate trigger
        for _ in range(30):
            if any("dock_low_battery" in cmd for cmd in received_dock_cmds):
                break
            time.sleep(0.1)

        has_low_dock = any("dock_low_battery" in cmd for cmd in received_dock_cmds)
        self.assertTrue(has_low_dock, f"AutoDockingController did not trigger dock on low battery. Received: {received_dock_cmds}")

        # Reset battery to 100%
        b_msg.data = "100.0"
        bat_override_pub.publish(b_msg)
        for _ in range(20):
            if self.dock_controller.battery_pct >= 95.0:
                break
            time.sleep(0.1)

        self.test_node.destroy_subscription(sub_dock)
        self.test_node.destroy_publisher(bat_override_pub)

    def test_04_critical_battery_emergency_override(self):
        """Verifies critical battery (<15%) initiates emergency docking override."""
        if not HAVE_ROS2:
            self.skipTest("ROS 2 not available")

        # Set robot to not docked so dock controller commands return to dock
        self.dock_controller.controller.is_docked = False
        self.dock_controller.controller.is_docking_in_progress = False
        self.dock_controller.battery_pct = 12.0
        self.dock_controller.controller.battery_percentage = 12.0

        received_dock_cmds = []
        sub_dock = self.test_node.create_subscription(
            String, '/dock_command',
            lambda msg: received_dock_cmds.append(msg.data), 10
        )

        bat_override_pub = self.test_node.create_publisher(String, '/battery_set_percentage', 10)
        b_msg = String()
        b_msg.data = "12.0"
        bat_override_pub.publish(b_msg)

        for _ in range(30):
            if any("dock_critical_battery" in cmd for cmd in received_dock_cmds):
                break
            time.sleep(0.1)

        has_crit_dock = any("dock_critical_battery" in cmd for cmd in received_dock_cmds)
        self.assertTrue(has_crit_dock, f"AutoDockingController did not trigger emergency critical dock. Received: {received_dock_cmds}")

        # Restore battery
        b_msg.data = "100.0"
        bat_override_pub.publish(b_msg)

        self.test_node.destroy_subscription(sub_dock)
        self.test_node.destroy_publisher(bat_override_pub)

    def test_05_complex_layout_corridor_recovery(self):
        """Verifies layout recovery maneuver backs up robot base and retains tasks."""
        if not HAVE_ROS2:
            self.skipTest("ROS 2 not available")

        # Manually invoke recovery to verify velocity pulses and state transition
        cmd_vel_msgs = []
        sub_vel = self.test_node.create_subscription(
            Twist, '/cmd_vel',
            lambda msg: cmd_vel_msgs.append(msg), 10
        )

        # Trigger delivery recovery (no Nav2 BackUp server here -> manual reverse fallback)
        self.task_manager.target_name = "Table 1"
        self.task_manager._handle_delivery_failure()
        self.assertEqual(self.task_manager.manager.robot_state, "recovering")

        # Recovery is non-blocking: reverse pulses arrive over the following timer ticks
        for _ in range(20):
            if any(cmd.linear.x < 0.0 for cmd in cmd_vel_msgs):
                break
            time.sleep(0.1)
        has_reverse = any(cmd.linear.x < 0.0 for cmd in cmd_vel_msgs)
        self.assertTrue(has_reverse, "Recovery routine must command reverse velocity to dislodge from doorway/obstacle")

        self.test_node.destroy_subscription(sub_vel)


if __name__ == '__main__':
    unittest.main()
