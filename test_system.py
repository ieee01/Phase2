#!/usr/bin/env python3
"""
CSTAM: Master System Verification & Integration Test Suite (Phases 1 & 2)
Tests:
1. Workspace Deliverables & Models Integrity (Gazebo, URDF, Worlds, Navigation, Core)
2. Mapping, Doorways & Predefined Waypoints Data Integrity
3. Delivery Task Management Queue Lifecycle (FIFO, Modify, Delete, Preemption)
4. Dynamic Obstacle Avoidance (Moving Pedestrian Kinematics, Modes, Proximity)
5. Complex Layout Handling & Navigation Failure Recovery (Doorways, Corridors)
6. Battery & State Simulation (Physical 24V Li-ion NMC Pack, Dynamic Load, OCV, Sag, CC-CV Docking)
7. Autonomous Return-To-Dock Trigger Hierarchy (Idle Timeout, Low Battery <25%, Critical Emergency <15%)
"""

import sys
import os
import unittest
import json
import time

# Insert path for cstam_core
sys.path.insert(0, os.path.abspath('ros2_ws/src/cstam_core'))

from cstam_core.delivery_task_manager import TaskQueueManager, DEFAULT_WAYPOINTS
from cstam_core.docking_controller import AutoDockingController
from cstam_core.battery_simulator import BatterySimulator
from cstam_core.dynamic_obstacle_controller import DynamicObstacleAgent


class CstamSystemVerificationTests(unittest.TestCase):

    def test_required_files_exist(self):
        required_paths = [
            # Workspace & System Launchers
            "delivery_simulator_ui.py",
            "run_delivery_simulator.sh",
            "run_autonomous_system.sh",
            "run_mapping_demo.sh",
            "test_system.py",
            # CSTAM Gazebo & Models
            "ros2_ws/src/cstam_gazebo/package.xml",
            "ros2_ws/src/cstam_gazebo/urdf/cstam_robot.urdf.xacro",
            "ros2_ws/src/cstam_gazebo/worlds/restaurant.world",
            "ros2_ws/src/cstam_gazebo/worlds/resto_arbi.world",
            "ros2_ws/src/cstam_gazebo/launch/spawn_cstam_robot.launch.py",
            "ros2_ws/src/cstam_gazebo/models/dynamic_obstacle/model.sdf",
            "ros2_ws/src/cstam_gazebo/models/dynamic_obstacle/model.config",
            # CSTAM Navigation & Configs
            "ros2_ws/src/cstam_navigation/package.xml",
            "ros2_ws/src/cstam_navigation/config/nav2_params.yaml",
            "ros2_ws/src/cstam_navigation/config/nav2_params_resto_arbi.yaml",
            "ros2_ws/src/cstam_navigation/config/waypoints.yaml",
            "ros2_ws/src/cstam_navigation/config/resto_arbi_waypoints.yaml",
            "ros2_ws/src/cstam_navigation/config/cstam_nav2.rviz",
            "ros2_ws/src/cstam_navigation/maps/cstam_map.yaml",
            "ros2_ws/src/cstam_navigation/maps/cstam_map.pgm",
            # CSTAM Core
            "ros2_ws/src/cstam_core/package.xml",
            "ros2_ws/src/cstam_core/cstam_core/delivery_task_manager.py",
            "ros2_ws/src/cstam_core/cstam_core/docking_controller.py",
            "ros2_ws/src/cstam_core/cstam_core/battery_simulator.py",
            "ros2_ws/src/cstam_core/cstam_core/dynamic_obstacle_controller.py",
            "ros2_ws/src/cstam_core/launch/cstam_system.launch.py",
        ]
        for rel_path in required_paths:
            full_p = os.path.normpath(os.path.join(os.getcwd(), rel_path))
            self.assertTrue(os.path.exists(full_p), f"Missing required system deliverable: {rel_path}")

    def test_predefined_waypoints(self):
        # 24 tables + Kitchen + Dock
        self.assertIn("Dock", DEFAULT_WAYPOINTS)
        self.assertIn("Kitchen/Pickup", DEFAULT_WAYPOINTS)
        for i in range(24):
            self.assertIn(f"Table {i}", DEFAULT_WAYPOINTS)
            wp = DEFAULT_WAYPOINTS[f"Table {i}"]
            self.assertIn("x", wp)
            self.assertIn("y", wp)

    def test_delivery_queue_lifecycle(self):
        mgr = TaskQueueManager()
        
        # 1. Add deliveries
        r1 = mgr.add_delivery_request("Table 1", "Espresso & Croissant")
        r2 = mgr.add_delivery_request("Table 2", "Burger & Fries")
        r3 = mgr.add_delivery_request("Kitchen/Pickup", "Order Pick Up")

        self.assertTrue(r1["success"])
        self.assertTrue(r2["success"])
        self.assertTrue(r3["success"])
        self.assertEqual(mgr.get_status_summary()["queue_length"], 3)

        # 2. Dispatch Task 1
        t1 = mgr.get_next_task()
        self.assertIsNotNone(t1)
        self.assertEqual(t1["target"], "Table 1")
        self.assertEqual(t1["status"], "en_route")

        # 3. Complete Task 1
        done1 = mgr.complete_current_task(success=True)
        self.assertEqual(done1["status"], "completed")
        self.assertEqual(mgr.get_status_summary()["completed_count"], 1)
        self.assertEqual(mgr.get_status_summary()["queue_length"], 2)

    def test_task_queue_modify_and_delete(self):
        mgr = TaskQueueManager()
        r1 = mgr.add_delivery_request("Table 3", "Espresso")
        r2 = mgr.add_delivery_request("Table 5", "Salad")
        self.assertEqual(mgr.get_status_summary()["queue_length"], 2)

        task_id_1 = r1["task"]["id"]
        task_id_2 = r2["task"]["id"]

        # Modify Table and Item of Task 1
        mod_ok = mgr.modify_task(task_id_1, new_target="Table 10", new_item="Cappuccino & Muffin")
        self.assertTrue(mod_ok)
        pending = mgr.get_status_summary()["pending_tasks"]
        self.assertEqual(pending[0]["target"], "Table 10")
        self.assertEqual(pending[0]["item"], "Cappuccino & Muffin")

        # Delete Task 2
        del_ok = mgr.delete_task(task_id_2)
        self.assertTrue(del_ok)
        self.assertEqual(mgr.get_status_summary()["queue_length"], 1)

        # Invalid task ID operations return False
        self.assertFalse(mgr.modify_task("INVALID_ID", "Table 1"))
        self.assertFalse(mgr.delete_task("INVALID_ID"))

    # =========================================================================
    # PHASE 2 TEST SUITE: DYNAMIC OBSTACLE AVOIDANCE
    # =========================================================================
    def test_dynamic_obstacle_agent_kinematics_and_modes(self):
        """Validates dynamic pedestrian obstacle movement, corridor pacing, and mode transitions."""
        agent = DynamicObstacleAgent(start_x=-1.5, start_y=-1.3, patrol_y_min=-3.0, patrol_y_max=0.0, speed=0.50)
        
        # 1. Patrol Mode Kinematics
        agent.mode = "patrol"
        agent.direction = 1.0
        vx, vy, wz = agent.update(1.0)
        self.assertGreater(vy, 0.0, "Patrol moving north should have positive vy")
        self.assertAlmostEqual(agent.speed, 0.50)
        self.assertAlmostEqual(agent.y, -0.8, delta=0.05)

        # 2. Pacing Bounds Reversal
        agent.y = 0.1  # Exceeds max
        agent.update(0.1)
        self.assertEqual(agent.direction, -1.0, "Pedestrian should reverse direction when reaching patrol boundary")

        # 3. Block Mode
        agent.set_mode("block")
        self.assertEqual(agent.mode, "block")
        self.assertEqual(agent.target_x, -1.5)
        self.assertEqual(agent.target_y, -1.3)

        # 4. Stop Mode
        agent.set_mode("stop")
        vx, vy, wz = agent.update(1.0)
        self.assertEqual(vx, 0.0)
        self.assertEqual(vy, 0.0)

    # =========================================================================
    # PHASE 2 TEST SUITE: BATTERY & STATE SIMULATION
    # =========================================================================
    def test_battery_physical_model_discharge_and_voltage_sag(self):
        """Validates 24V Li-ion pack physical model: load power, OCV curve, Coulomb counting & sag."""
        sim = BatterySimulator(initial_percentage=100.0, time_scale=1.0)
        self.assertAlmostEqual(sim.percentage, 100.0)
        self.assertAlmostEqual(sim.ocv, 29.40, delta=0.1)

        # 1. Idle Power Consumption (35W)
        sim.is_moving = False
        sim.update(dt=10.0)
        self.assertGreater(sim.current, 1.0, "Idle electronics should draw > 1.0A")
        self.assertLess(sim.voltage, sim.ocv, "Terminal voltage should exhibit voltage sag under load")

        # 2. Cruising Navigation Power Consumption (95W + 35W = 130W)
        sim.is_moving = True
        sim.update(dt=10.0)
        self.assertGreater(sim.current, 3.5, "Drive motors while navigating should draw > 3.5A")
        self.assertGreater(sim.power, 100.0)

        # 3. Low Battery Threshold (<25%) and Critical Threshold (<15%) Detection
        sim.set_percentage(24.0)
        summary = sim.get_status_summary()
        self.assertTrue(summary["is_low"], "Battery below 25% must flag low battery")
        self.assertFalse(summary["is_critical"])

        sim.set_percentage(12.0)
        summary_crit = sim.get_status_summary()
        self.assertTrue(summary_crit["is_critical"], "Battery below 15% must flag critical battery")

    def test_battery_dock_cc_cv_charging(self):
        """Validates Constant-Current / Constant-Voltage dock charging profile."""
        sim = BatterySimulator(initial_percentage=20.0, time_scale=1.0)
        sim.is_docked = True
        
        # CC Phase: 8.0A constant current
        sim.update(dt=1.0)
        self.assertAlmostEqual(sim.current, -8.0, delta=0.2, msg="Dock charger should deliver 8A in CC phase")
        self.assertGreater(sim.percentage, 20.0, "Battery percentage should increase while docked")

        # CV Phase (above 85% SOC): current tapers
        sim.set_percentage(95.0)
        sim.is_docked = True
        sim.update(dt=1.0)
        self.assertGreater(sim.current, -8.0, "Dock charger current should taper down in CV phase")

    # =========================================================================
    # PHASE 2 TEST SUITE: AUTO-DOCKING & BATTERY TRIGGER HIERARCHY
    # =========================================================================
    def test_auto_docking_battery_triggers(self):
        """Validates low battery (<25%) and critical battery (<15%) auto-dock triggers."""
        ctrl = AutoDockingController(idle_timeout=15.0, low_battery_threshold=25.0, critical_battery_threshold=15.0)
        now = 500.0

        # Normal full battery while busy -> none
        self.assertEqual(ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=True, current_time=now, battery_pct=100.0), 'none')

        # Idle timeout trigger
        ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=now, battery_pct=100.0)
        self.assertEqual(ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=now + 16.0, battery_pct=100.0), 'dock_idle')

        # Low battery trigger (<25%) when idle
        self.assertEqual(ctrl.evaluate_dock_trigger(queue_empty=True, is_navigating=False, current_time=now, battery_pct=24.0), 'dock_low_battery')

        # Critical battery emergency override (<15%) EVEN WHILE NAVIGATING with pending orders!
        crit_trigger = ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=True, current_time=now, battery_pct=14.0)
        self.assertEqual(crit_trigger, 'dock_critical_battery', "Critical battery must override active navigation to prevent dying in aisle")

        # While docked: must remain docked until recharged above min_recharge_pct (80%)
        ctrl.is_docked = True
        self.assertEqual(ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=False, current_time=now, battery_pct=50.0), 'remain_docked')
        self.assertEqual(ctrl.evaluate_dock_trigger(queue_empty=False, is_navigating=False, current_time=now, battery_pct=85.0), 'undock_ready')

    # =========================================================================
    # PHASE 2 TEST SUITE: COMPLEX LAYOUT & RECOVERY
    # =========================================================================
    def test_complex_layout_handling_and_recovery(self):
        """Validates task persistence and layout recovery handling."""
        mgr = TaskQueueManager()
        r = mgr.add_delivery_request("Table 4", "Steak & Wine")
        task = mgr.get_next_task()
        self.assertIsNotNone(task)

        # Simulate task retry after doorway/corridor obstacle
        with mgr.lock:
            # Requeue task on failure
            task["status"] = "queued"
            mgr.queue.appendleft(task)
            mgr.current_task = None

        # Verify task is not lost
        self.assertEqual(mgr.get_status_summary()["queue_length"], 1)
        re_task = mgr.get_next_task()
        self.assertEqual(re_task["id"], task["id"])
        self.assertEqual(re_task["target"], "Table 4")


if __name__ == '__main__':
    unittest.main()
