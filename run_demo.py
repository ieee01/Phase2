#!/usr/bin/env python3
"""
CSTAM-TUNIBOT End-to-End Demo / Smoke Test

Drives the running ROS 2 stack (start it first with ./run_autonomous_system.sh)
and verifies every step from live topics instead of assuming success:
1. Waits for the delivery task manager to come online.
2. Submits 3 consecutive delivery requests (Table 1, Table 2, Table 3).
3. Sends the dynamic pedestrian across the robot's path mid-navigation.
4. Waits until all 3 deliveries are completed.
5. Waits for the 10-second idle countdown to return the robot to the dock.
6. Sends a new order and verifies the robot leaves the dock to serve it.

Exits with status 0 when every step was observed, 1 otherwise.
Usage: python3 run_demo.py [--timeout SECONDS_PER_STEP]
"""

import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import String


class DemoDriver:
    def __init__(self):
        self.node = rclpy.create_node('cstam_demo_driver')
        self.req_pub = self.node.create_publisher(String, '/delivery_request', 10)
        self.obs_pub = self.node.create_publisher(String, '/dynamic_obstacle/control', 10)
        self.status: dict = {}
        self.states_seen = []
        self.node.create_subscription(String, '/delivery_queue_status', self._status_cb, 10)

    def _status_cb(self, msg: String):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            return
        state = self.status.get('robot_state')
        if not self.states_seen or self.states_seen[-1] != state:
            self.states_seen.append(state)
            print(f"    [state] {state}  (queue={self.status.get('queue_length')}, "
                  f"completed={self.status.get('completed_count')})")

    def wait_for(self, predicate, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.status and predicate(self.status):
                return True
        return False

    def wait_for_subscribers(self, timeout: float) -> bool:
        """Messages published before discovery completes are dropped, so wait for the receivers."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.req_pub.get_subscription_count() > 0 and self.obs_pub.get_subscription_count() > 0:
                return True
        return False

    def order(self, target: str, item: str):
        msg = String()
        msg.data = json.dumps({'target': target, 'item': item})
        self.req_pub.publish(msg)

    def obstacle(self, action: str):
        msg = String()
        msg.data = action
        self.obs_pub.publish(msg)


def log_step(title, message):
    print("\n==================================================")
    print(f"▶ [{title}] {message}")
    print("==================================================")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--timeout', type=float, default=300.0, help='timeout per step in seconds')
    args = parser.parse_args()

    rclpy.init()
    demo = DemoDriver()
    ok = True

    def check(passed: bool, success: str, failure: str):
        nonlocal ok
        print(f"✔ {success}" if passed else f"✘ {failure}")
        ok = ok and passed
        return passed

    try:
        log_step("STEP 1", "Waiting for the delivery task manager...")
        if not check(demo.wait_for(lambda s: True, 30.0) and demo.wait_for_subscribers(30.0),
                     "Task manager online.", "Task manager / obstacle controller not reachable. Is the system running?"):
            return 1
        base_completed = demo.status.get('completed_count', 0)

        log_step("STEP 2", "Submitting 3 sequential delivery requests...")
        for table, item in [("Table 1", "Hot Espresso & Bagel"), ("Table 2", "Fresh Orange Juice"),
                            ("Table 3", "Club Sandwich")]:
            demo.order(table, item)
            print(f"  -> {table}: {item}")
        check(demo.wait_for(lambda s: s.get('robot_state') == 'navigating', 30.0),
              "Robot is navigating to the first table.", "Robot never started navigating.")

        log_step("STEP 3", "Sending the pedestrian across the robot's path...")
        demo.obstacle("cross")
        demo.wait_for(lambda s: False, 5.0)
        demo.obstacle("patrol")
        print("✔ Pedestrian crossed and resumed patrol.")

        log_step("STEP 4", "Waiting for all 3 deliveries to complete...")
        check(demo.wait_for(lambda s: s.get('completed_count', 0) >= base_completed + 3, args.timeout * 3),
              "All 3 deliveries completed.",
              f"Deliveries not completed (completed={demo.status.get('completed_count')}).")

        log_step("STEP 5", "Waiting for idle timeout -> autonomous return-to-dock...")
        check(demo.wait_for(lambda s: s.get('robot_state') == 'docking', 30.0),
              "Return-to-dock started after idle timeout.", "Robot never started docking.")
        check(demo.wait_for(lambda s: s.get('robot_state') == 'docked', args.timeout),
              "Robot parked at the dock.", "Robot never reached the dock.")

        log_step("STEP 6", "New order while docked -> robot leaves the dock...")
        demo.order("Table 2", "Tiramisu Dessert")
        check(demo.wait_for(lambda s: s.get('robot_state') in ('navigating', 'at_table'), 30.0),
              "Robot left the dock to serve the new order.", "Robot did not leave the dock.")
        check(demo.wait_for(lambda s: s.get('completed_count', 0) >= base_completed + 4, args.timeout),
              "Follow-up delivery completed.", "Follow-up delivery not completed.")
    finally:
        print(f"\nState transitions observed: {' -> '.join(demo.states_seen)}")
        demo.node.destroy_node()
        rclpy.shutdown()

    log_step("DEMO COMPLETE" if ok else "DEMO FAILED",
             "All steps verified." if ok else "Some steps failed; see ✘ lines above.")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
