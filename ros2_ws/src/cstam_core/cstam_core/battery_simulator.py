#!/usr/bin/env python3
"""
CSTAM Physically-Accurate Lithium-Ion Battery & State Simulator Node
Models a 24V 20Ah (480Wh, 7S NMC) mobile robot battery pack:
- Non-linear Open-Circuit Voltage (OCV) discharge curve
- Dynamic load current based on subsystem power (idle electronics vs. drive motors)
- Internal cell resistance (R_int) causing dynamic voltage sag under acceleration
- Multi-stage Constant-Current / Constant-Voltage (CC-CV) dock charging profile
- Thermal dissipation and Joule heating (I^2 * R)
- Low battery (<25%) and Critical battery (<15%) auto-docking triggers
"""

import json

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String, Bool
    from nav_msgs.msg import Odometry
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False
    class Node:  # type: ignore
        def __init__(self, *args, **kwargs): pass


class BatterySimulator:
    """
    Physical Li-ion Battery Simulation Model.
    7S (24V nominal, 29.4V peak, 21.0V cutoff), 20.0 Ah (480 Wh).
    """

    def __init__(
        self,
        initial_percentage=100.0,
        drain_rate_idle=0.02,
        drain_rate_nav=0.15,
        charge_rate=2.0,
        time_scale=15.0,  # Accelerated time scale for interactive simulation
        mode="auto"
    ):
        self.capacity = 20.0  # Nominal capacity in Ah (Amp-hours)
        self.percentage = float(max(0.0, min(100.0, initial_percentage)))
        self.soc = self.percentage / 100.0  # State of Charge [0.0 - 1.0]

        # Operation mode: physical (default) or linear (unit-test/legacy compatibility)
        if mode == "linear" or (mode == "auto" and (drain_rate_idle != 0.02 or drain_rate_nav != 0.15 or charge_rate != 2.0)):
            self.mode = "linear"
        else:
            self.mode = "physical"

        # Internal cell and pack resistance in Ohms (45 mOhm)
        self.r_internal = 0.045

        # Subsystem Power Consumption (Watts)
        self.p_base_electronics = 35.0   # LiDAR, Jetson/NUC, cameras, displays
        self.p_drive_cruise = 95.0       # Motors cruising at nominal speed
        self.p_drive_accel = 160.0       # Motors accelerating or maneuvering

        # Dock Charger Parameters (24V / 8A dock charger)
        self.charger_max_current = 8.0   # Amps in CC phase
        self.charger_max_voltage = 29.4  # Volts peak (4.2V/cell * 7 cells)

        # Operational States
        self.is_docked = False
        self.is_moving = False
        self.is_accelerating = False

        # Live Electrical Measurements
        self.current = 0.0      # Amperes (+ discharging, - charging)
        self.voltage = 29.4     # Terminal voltage under load (V)
        self.ocv = 29.4         # Open-circuit voltage (V)
        self.power = 0.0        # Power in Watts (P = V * I)
        self.temperature = 24.5 # Pack temperature in Celsius

        # Thresholds
        self.low_battery_threshold = 25.0
        self.critical_battery_threshold = 15.0

        # Time Acceleration Factor (1.0 = real-time, 15.0 = demo mode)
        self.time_scale = float(time_scale)

        # Legacy parameters compatibility
        self.drain_rate_idle = float(drain_rate_idle)
        self.drain_rate_nav = float(drain_rate_nav)
        self.charge_rate = float(charge_rate)

        # Initialize voltage based on starting SOC
        self._calculate_ocv()
        self.voltage = self.ocv

    def set_percentage(self, pct: float):
        self.percentage = float(max(0.0, min(100.0, pct)))
        self.soc = self.percentage / 100.0
        self._calculate_ocv()
        self.voltage = self.ocv

    def _calculate_ocv(self):
        """
        Calculates Open-Circuit Voltage (OCV) using an empirical
        7S Li-ion (NMC) non-linear discharge curve.
        """
        s = max(0.001, min(0.999, self.soc))

        if s >= 0.85:
            # Upper curve: 28.0V -> 29.4V
            v_oc = 28.0 + (s - 0.85) / 0.15 * 1.4
        elif s >= 0.20:
            # Nominal plateau: 24.8V -> 28.0V
            v_oc = 24.8 + (s - 0.20) / 0.65 * 3.2
        elif s >= 0.08:
            # Knee region (BMS low battery trigger zone): 23.0V -> 24.8V
            v_oc = 23.0 + (s - 0.08) / 0.12 * 1.8
        else:
            # Cut-off cliff: 21.0V -> 23.0V
            v_oc = 21.0 + (s / 0.08) * 2.0

        self.ocv = round(v_oc, 3)
        return self.ocv

    def update(self, dt=1.0):
        """
        Updates battery state for a time step dt (seconds).
        Integrates current using Coulomb counting and applies internal resistance.
        """
        if self.mode == "linear":
            if self.is_docked:
                self.percentage = min(100.0, round(self.percentage + self.charge_rate * dt, 2))
                self.soc = self.percentage / 100.0
                self.current = -self.charge_rate
                self.power = round(abs(self.current) * 24.0, 1)
                self.voltage = 29.4
            else:
                rate = self.drain_rate_nav if self.is_moving else self.drain_rate_idle
                self.percentage = max(0.0, round(self.percentage - rate * dt, 2))
                self.soc = self.percentage / 100.0
                self.current = round(rate, 2)
                self.power = round(rate * 24.0, 1)
                self._calculate_ocv()
                self.voltage = self.ocv
            return self.percentage

        effective_dt = dt * self.time_scale

        if self.is_docked:
            # --- CC-CV DOCK CHARGING MODEL ---
            if self.soc < 0.85:
                charge_current = self.charger_max_current
            else:
                taper_ratio = max(0.08, (1.0 - self.soc) / 0.15)
                charge_current = self.charger_max_current * taper_ratio

            self.current = -charge_current
            self.power = abs(self.current) * self.voltage

            # Coulomb integration (charging adds Ah)
            delta_ah = (charge_current * effective_dt) / 3600.0
            new_soc = min(1.0, self.soc + delta_ah / self.capacity)
            self.soc = new_soc
            self.percentage = round(self.soc * 100.0, 2)

            self._calculate_ocv()
            self.voltage = round(min(self.charger_max_voltage, self.ocv + charge_current * self.r_internal), 2)

        else:
            # --- DYNAMIC DISCHARGE & LOAD MODEL ---
            if self.is_moving:
                p_load = self.p_base_electronics + (self.p_drive_accel if self.is_accelerating else self.p_drive_cruise)
            else:
                p_load = self.p_base_electronics

            est_v = max(20.0, self.ocv)
            discharge_current = p_load / est_v
            self.current = round(discharge_current, 2)
            self.power = round(p_load, 1)

            # Coulomb integration (discharging removes Ah)
            delta_ah = (discharge_current * effective_dt) / 3600.0
            new_soc = max(0.0, self.soc - delta_ah / self.capacity)
            self.soc = new_soc
            self.percentage = round(self.soc * 100.0, 2)

            self._calculate_ocv()
            voltage_sag = discharge_current * self.r_internal
            self.voltage = round(max(20.0, self.ocv - voltage_sag), 2)

        # Thermal model
        joule_heat = (self.current ** 2) * self.r_internal
        cooling = 0.05 * (self.temperature - 24.0)
        self.temperature = round(self.temperature + (joule_heat * 0.01 - cooling) * dt, 1)

        return self.percentage

    def get_status_summary(self) -> dict:
        is_low = self.percentage <= self.low_battery_threshold
        is_critical = self.percentage <= self.critical_battery_threshold
        
        # Estimate runtime in minutes
        if self.current > 0.01:
            remaining_ah = (self.percentage / 100.0) * self.capacity
            est_minutes = round((remaining_ah / self.current) * 60.0, 1)
        else:
            est_minutes = 999.0

        return {
            "percentage": round(self.percentage, 1),
            "voltage": round(self.voltage, 2),
            "ocv": round(self.ocv, 2),
            "current_amps": round(abs(self.current), 2),
            "power_watts": round(self.power, 1),
            "temperature_c": round(self.temperature, 1),
            "is_docked": self.is_docked,
            "is_moving": self.is_moving,
            "is_low": is_low,
            "is_critical": is_critical,
            "status": "charging" if self.is_docked else ("critical" if is_critical else ("low" if is_low else ("discharging_nav" if self.is_moving else "idle_standby"))),
            "estimated_runtime_min": est_minutes
        }


if HAVE_ROS2:
    class BatterySimulatorNode(Node):
        def __init__(self):
            super().__init__('battery_simulator')

            self.declare_parameter('initial_battery', 100.0)
            self.declare_parameter('time_scale', 15.0)

            init_bat = self.get_parameter('initial_battery').value
            t_scale = self.get_parameter('time_scale').value

            self.sim = BatterySimulator(initial_percentage=init_bat, time_scale=t_scale)

            # Publishers
            self.battery_state_pub = self.create_publisher(BatteryState, '/battery_state', 10)
            self.status_pub = self.create_publisher(String, '/battery_status', 10)
            self.telemetry_pub = self.create_publisher(String, '/battery_telemetry_detailed', 10)

            # Subscriptions
            self.create_subscription(String, '/robot_system_state', self.system_state_cb, 10)
            self.create_subscription(Bool, '/dock_state', self.dock_state_cb, 10)
            self.create_subscription(String, '/battery_set_percentage', self.set_percentage_cb, 10)
            self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

            self.prev_odom_v = None
            self.prev_odom_t = None

            self.timer = self.create_timer(1.0, self.timer_callback)
            self.get_logger().info("Physical 24V Li-ion Battery Simulator Node initialized.")

        def publish_status_messages(self, current_pct=None):
            if current_pct is None:
                current_pct = self.sim.percentage
            summary = self.sim.get_status_summary()

            # Standard ROS 2 BatteryState message
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.percentage = current_pct / 100.0  # Normalized [0.0 - 1.0]
            msg.voltage = float(self.sim.voltage)
            # sensor_msgs/BatteryState convention: negative while discharging, positive while charging
            msg.current = float(-self.sim.current)
            msg.charge = float((current_pct / 100.0) * self.sim.capacity)
            msg.capacity = float(self.sim.capacity)
            msg.design_capacity = 20.0
            msg.temperature = float(self.sim.temperature)
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.sim.is_docked else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
            msg.present = True
            self.battery_state_pub.publish(msg)

            # JSON Status Strings
            str_msg = String()
            str_msg.data = json.dumps(summary)
            self.status_pub.publish(str_msg)
            self.telemetry_pub.publish(str_msg)

        def set_percentage_cb(self, msg: String):
            try:
                val = float(msg.data.strip())
                self.sim.set_percentage(val)
                self.get_logger().info(f"Battery state of charge overridden to {self.sim.percentage:.1f}%.")
                self.publish_status_messages()
            except ValueError:
                pass

        def system_state_cb(self, msg: String):
            state = msg.data.lower()
            self.sim.is_moving = any(k in state for k in ['navigating', 'en_route', 'docking', 'avoiding', 'recovering'])
            if any(k in state for k in ['docked', 'charging']):
                self.sim.is_docked = True
            elif self.sim.is_moving:
                self.sim.is_docked = False

        def odom_cb(self, msg: Odometry):
            # Accelerating / turning draws the higher motor load in the physical model
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            t = self.get_clock().now().nanoseconds / 1e9
            accel = 0.0
            if self.prev_odom_t is not None and t > self.prev_odom_t:
                accel = abs(v - self.prev_odom_v) / (t - self.prev_odom_t)
            self.prev_odom_v = v
            self.prev_odom_t = t
            self.sim.is_accelerating = accel > 0.15 or abs(w) > 0.4

        def dock_state_cb(self, msg: Bool):
            self.sim.is_docked = msg.data

        def timer_callback(self):
            current_pct = self.sim.update(dt=1.0)
            self.publish_status_messages(current_pct)


def main(args=None):
    if HAVE_ROS2:
        rclpy.init(args=args)
        node = BatterySimulatorNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
    else:
        print("Running Physical BatterySimulator standalone test mode:")
        sim = BatterySimulator(initial_percentage=100.0, time_scale=60.0)
        sim.is_moving = True
        print(f"Initial: {sim.percentage:.1f}%, OCV: {sim.ocv}V, Terminal: {sim.voltage}V")
        for i in range(5):
            sim.update(1.0)
            print(f"Step {i+1} (Cruising): SOC={sim.percentage:.1f}%, V={sim.voltage:.2f}V, I={sim.current:.2f}A, P={sim.power:.1f}W")
        sim.is_docked = True
        for i in range(5):
            sim.update(1.0)
            print(f"Step {i+6} (CC Charging): SOC={sim.percentage:.1f}%, V={sim.voltage:.2f}V, I={sim.current:.2f}A")


if __name__ == '__main__':
    main()
