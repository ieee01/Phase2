#!/usr/bin/env python3
"""
CSTAM Dynamic Obstacle Controller Node
Simulates dynamic obstacles (pedestrians / human waiters walking in the restaurant).
Features:
- Autonomous corridor & aisle patrolling using stable ground DiffDrive kinematics
- Reactive crossing in front of the autonomous delivery robot
- Path blocking & clearing edge-case test modes
- 3D RViz visualization markers and telemetry publishing
"""

import math
import json

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs): pass


class DynamicObstacleAgent:
    """
    Kinematic model and behavior state machine for dynamic obstacles.
    """
    def __init__(self, start_x=-1.5, start_y=-1.3, patrol_y_min=-2.8, patrol_y_max=0.2, speed=0.40):
        self.start_x = float(start_x)
        self.start_y = float(start_y)
        self.x = float(start_x)
        self.y = float(start_y)
        self.speed = float(speed)
        
        self.patrol_y_min = float(patrol_y_min)
        self.patrol_y_max = float(patrol_y_max)
        self.direction = 1.0  # 1.0 = North, -1.0 = South
        
        # Modes: 'idle', 'patrol', 'cross', 'block', 'clear', 'stop'
        self.mode = "patrol"
        self.target_x = self.x
        self.target_y = self.y
        self.is_active = True
        self.vx = 0.0
        self.vy = 0.0

    def set_mode(self, mode: str, target: tuple | None = None, robot_pos: tuple | None = None):
        self.mode = mode.lower()
        if target:
            self.target_x, self.target_y = target
        elif self.mode == "cross":
            # Cross perpendicularly across the service aisle
            if self.x < self.start_x:
                self.target_x = self.start_x + 1.6
            else:
                self.target_x = self.start_x - 1.6
            self.target_y = self.y
        elif self.mode == "block":
            # Step into the center of the corridor to block path
            if robot_pos:
                rx, ry = robot_pos
                if math.hypot(rx - self.x, ry - self.y) < 4.0:
                    self.target_x = rx
                    self.target_y = ry + 1.0 if ry < self.y else ry - 1.0
                else:
                    self.target_x = self.start_x
                    self.target_y = self.start_y
            else:
                self.target_x = self.start_x
                self.target_y = self.start_y
        elif self.mode == "clear":
            # Step aside toward the wall to yield path
            self.target_x = self.start_x - 1.2
            self.target_y = self.y
        elif self.mode == "patrol":
            self.target_x = self.start_x
        elif self.mode in ["idle", "stop"]:
            self.target_x = self.x
            self.target_y = self.y

    def update(self, dt: float) -> tuple[float, float, float]:
        """
        Updates kinematic state based on active behavior mode.
        Returns: (linear_vx_world, linear_vy_world, angular_wz)
        """
        if not self.is_active or self.mode in ["idle", "stop"]:
            self.vx, self.vy = 0.0, 0.0
            return 0.0, 0.0, 0.0

        if self.mode == "patrol":
            # Return smoothly to the patrol centerline X if previously displaced
            dx = self.start_x - self.x
            if abs(dx) > 0.12:
                self.vx = math.copysign(min(self.speed, abs(dx) / dt), dx)
            else:
                self.vx = 0.0

            if self.direction > 0 and self.y >= self.patrol_y_max:
                self.direction = -1.0
            elif self.direction < 0 and self.y <= self.patrol_y_min:
                self.direction = 1.0

            self.vy = self.direction * self.speed
            self.x += self.vx * dt
            self.y += self.vy * dt
            return self.vx, self.vy, 0.0

        elif self.mode in ["cross", "block", "clear"]:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.12:
                self.vx, self.vy = 0.0, 0.0
                return 0.0, 0.0, 0.0

            angle = math.atan2(dy, dx)
            self.vx = math.cos(angle) * self.speed
            self.vy = math.sin(angle) * self.speed
            self.x += self.vx * dt
            self.y += self.vy * dt
            return self.vx, self.vy, 0.0

        return 0.0, 0.0, 0.0


if HAVE_ROS2:
    class DynamicObstacleControllerNode(Node):
        def __init__(self):
            super().__init__('dynamic_obstacle_controller')

            self.declare_parameter('start_x', -1.5)
            self.declare_parameter('start_y', -1.3)
            # Spawn yaw of the obstacle model in the world (must match the <include> pose in the .world file)
            self.declare_parameter('start_yaw', 1.5708)
            self.declare_parameter('patrol_min', -2.8)
            self.declare_parameter('patrol_max', 0.2)
            self.declare_parameter('speed', 0.40)
            self.declare_parameter('auto_patrol', True)

            sx = self.get_parameter('start_x').value
            sy = self.get_parameter('start_y').value
            pmin = self.get_parameter('patrol_min').value
            pmax = self.get_parameter('patrol_max').value
            spd = self.get_parameter('speed').value
            auto = self.get_parameter('auto_patrol').value
            self.spawn_yaw = float(self.get_parameter('start_yaw').value)

            self.agent = DynamicObstacleAgent(
                start_x=sx, start_y=sy, patrol_y_min=pmin, patrol_y_max=pmax, speed=spd
            )
            self.agent.mode = "patrol" if auto else "idle"
            self.current_yaw = self.spawn_yaw

            # Publishers
            self.cmd_vel_pub = self.create_publisher(Twist, '/dynamic_obstacle/cmd_vel', 10)
            self.status_pub = self.create_publisher(String, '/dynamic_obstacle/status', 10)
            self.marker_pub = self.create_publisher(MarkerArray, '/dynamic_obstacle/markers', 10)

            # Subscribers
            self.create_subscription(Odometry, '/dynamic_obstacle/odom', self.odom_cb, 10)
            self.create_subscription(String, '/dynamic_obstacle/control', self.control_cb, 10)

            # Robot map-frame pose (TF, with /amcl_pose as fallback) to calculate dynamic proximity
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.have_robot_pose = False
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)

            # Control loop at 10Hz for smooth kinematic updates
            self.timer = self.create_timer(0.1, self.loop)
            self.get_logger().info("Dynamic Obstacle Controller Node operational (DiffDrive Ground Kinematics).")

        def amcl_cb(self, msg):
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.have_robot_pose = True

        def _update_robot_pose_from_tf(self):
            for frame in ['base_footprint', 'base_link']:
                try:
                    t = self.tf_buffer.lookup_transform('map', frame, rclpy.time.Time())
                    self.robot_x = t.transform.translation.x
                    self.robot_y = t.transform.translation.y
                    self.have_robot_pose = True
                    return
                except Exception:
                    pass

        def odom_cb(self, msg: Odometry):
            # Gazebo DiffDrive odometry starts at (0, 0, 0) at the model's spawn pose.
            # Compose it with the spawn pose to get world (= map) coordinates.
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            oyaw = math.atan2(siny_cosp, cosy_cosp)

            c, s = math.cos(self.spawn_yaw), math.sin(self.spawn_yaw)
            self.agent.x = self.agent.start_x + c * ox - s * oy
            self.agent.y = self.agent.start_y + s * ox + c * oy
            yaw = self.spawn_yaw + oyaw
            self.current_yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        def _robot_pos(self):
            self._update_robot_pose_from_tf()
            return (self.robot_x, self.robot_y) if self.have_robot_pose else None

        def control_cb(self, msg: String):
            try:
                cmd = msg.data.strip()
                if cmd.startswith("{"):
                    data = json.loads(cmd)
                    action = data.get("action", "").lower()
                    target = data.get("target")
                    self.agent.set_mode(action, tuple(target) if target else None, self._robot_pos())
                else:
                    action = cmd.lower()
                    self.agent.set_mode(action, robot_pos=self._robot_pos())
                self.get_logger().info(f"Dynamic Obstacle switched mode to '{self.agent.mode}'.")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse obstacle control command: {e}")

        def loop(self):
            self._update_robot_pose_from_tf()

            # Determine target waypoint based on mode
            if self.agent.mode == "patrol":
                target_x = self.agent.start_x
                if self.agent.direction > 0:
                    target_y = self.agent.patrol_y_max
                else:
                    target_y = self.agent.patrol_y_min

                dist_to_patrol_tgt = math.hypot(target_x - self.agent.x, target_y - self.agent.y)
                if dist_to_patrol_tgt < 0.20:
                    self.agent.direction *= -1.0
                    target_y = self.agent.patrol_y_max if self.agent.direction > 0 else self.agent.patrol_y_min
            else:
                target_x = self.agent.target_x
                target_y = self.agent.target_y

            # Compute navigation command to target
            dx = target_x - self.agent.x
            dy = target_y - self.agent.y
            dist = math.hypot(dx, dy)

            cmd = Twist()
            current_speed = 0.0

            if not self.agent.is_active or self.agent.mode in ["idle", "stop"] or dist < 0.15:
                # Halt model cleanly
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            else:
                desired_yaw = math.atan2(dy, dx)
                yaw_err = math.atan2(math.sin(desired_yaw - self.current_yaw), math.cos(desired_yaw - self.current_yaw))

                # Smooth steering towards target
                cmd.angular.z = max(-1.2, min(1.2, 2.2 * yaw_err))

                # Drive forward if aligned
                if abs(yaw_err) < 1.0:
                    fwd_speed = min(self.agent.speed, max(0.12, dist * 0.8)) * max(0.0, math.cos(yaw_err))
                    cmd.linear.x = fwd_speed
                    current_speed = fwd_speed
                else:
                    # Turn in place first
                    cmd.linear.x = 0.0
                    current_speed = 0.0

            self.cmd_vel_pub.publish(cmd)

            # Proximity calculation (unknown until the robot pose is available)
            if self.have_robot_pose:
                dist_to_robot = math.hypot(self.agent.x - self.robot_x, self.agent.y - self.robot_y)
            else:
                dist_to_robot = 99.0
            is_blocking = dist_to_robot < 1.2

            # Publish Status Telemetry JSON
            status = {
                "mode": self.agent.mode,
                "x": round(self.agent.x, 2),
                "y": round(self.agent.y, 2),
                "speed": round(current_speed, 2),
                "distance_to_robot": round(dist_to_robot, 2),
                "is_blocking": is_blocking
            }
            s_msg = String()
            s_msg.data = json.dumps(status)
            self.status_pub.publish(s_msg)

            # Publish RViz 3D Markers
            self.publish_markers(is_blocking)

        def publish_markers(self, is_blocking: bool):
            markers = MarkerArray()

            # Marker 1: Pedestrian Body Cylinder
            m_body = Marker()
            m_body.header.frame_id = "map"
            m_body.header.stamp = self.get_clock().now().to_msg()
            m_body.ns = "dynamic_pedestrian"
            m_body.id = 1
            m_body.type = Marker.CYLINDER
            m_body.action = Marker.ADD
            m_body.pose.position.x = self.agent.x
            m_body.pose.position.y = self.agent.y
            m_body.pose.position.z = 0.85
            m_body.scale.x = 0.50
            m_body.scale.y = 0.50
            m_body.scale.z = 1.70
            if is_blocking:
                m_body.color.r = 1.0
                m_body.color.g = 0.2
                m_body.color.b = 0.1
            else:
                m_body.color.r = 0.9
                m_body.color.g = 0.45
                m_body.color.b = 0.1
            m_body.color.a = 0.95
            markers.markers.append(m_body)

            # Marker 2: Floating Status Text
            m_text = Marker()
            m_text.header.frame_id = "map"
            m_text.header.stamp = self.get_clock().now().to_msg()
            m_text.ns = "dynamic_pedestrian"
            m_text.id = 2
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = self.agent.x
            m_text.pose.position.y = self.agent.y
            m_text.pose.position.z = 1.95
            m_text.scale.z = 0.25
            label = f"🚶 PEDESTRIAN ({self.agent.mode.upper()})"
            if is_blocking:
                label += " [BLOCKING!]"
            m_text.text = label
            m_text.color.r = 1.0
            m_text.color.g = 1.0
            m_text.color.b = 1.0
            m_text.color.a = 1.0
            markers.markers.append(m_text)

            self.marker_pub.publish(markers)
else:
    class DynamicObstacleControllerNode:
        def __init__(self, *args, **kwargs):
            pass


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)
        node = DynamicObstacleControllerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    else:
        print("Running DynamicObstacleAgent standalone test:")
        agent = DynamicObstacleAgent()
        for i in range(5):
            vx, vy, wz = agent.update(0.5)
            print(f"Step {i+1}: pos=({agent.x:.2f}, {agent.y:.2f}), vel=({vx:.2f}, {vy:.2f})")


if __name__ == '__main__':
    main()
