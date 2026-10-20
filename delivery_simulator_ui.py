#!/usr/bin/env python3
"""
CSTAM: Delivery Process Simulator Interface & Mission Control Dashboard
Native graphical and interactive interface to simulate placing food/beverage orders,
monitoring the delivery queue, testing autonomous docking behavior, and evaluating:
- Dynamic Obstacle Avoidance (real-time moving pedestrians)
- Complex Layout Handling (tight corridors, doorways, navigation recovery)
- Battery & State Simulation (accurate Li-ion discharge & auto-dock triggers)
"""
from __future__ import annotations

import sys
import os
import json
import threading

# ROS 2 imports with graceful fallback
try:
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String, Bool
    HAVE_ROS2 = True
except ImportError:
    HAVE_ROS2 = False

# Tkinter imports with CLI fallback
try:
    import tkinter as tk
    from tkinter import ttk
    HAVE_TKINTER = True
except ImportError:
    HAVE_TKINTER = False


WORLDS_CONFIG = {
    "Dar Tunibot (resto_arbi.world)": {
        "short_name": "resto_arbi",
        "title": "Dar Tunibot · Mediterranean Heritage Restaurant",
        "tables": [f"Table {i}" for i in range(1, 11)] + ["Kitchen/Pickup", "Dock"],
        "presets": [
            "☕ Café Direct & Bambalouni",
            "🍲 Traditional Brik à l'Oeuf",
            "🥗 Salade Tunisienne",
            "🥘 Couscous Royal & Lamb",
            "🥤 Citronnade aux Amandes",
            "🍰 Makroudh au Miel"
        ],
        "default_table": "Table 1",
        "default_item": "☕ Café Direct & Bambalouni"
    },
    "Restaurant Lounge (restaurant.world)": {
        "short_name": "restaurant",
        "title": "Modern Restaurant Lounge & Grand Promenade",
        "tables": [f"Table {i}" for i in range(24)] + ["Kitchen/Pickup", "Dock"],
        "presets": [
            "☕ Espresso & Croissant",
            "🍔 Bella Burger & Fries",
            "🍝 Chef's Pasta Carbonara",
            "🥗 Mediterranean Salad",
            "🥤 Sparkling Lemonade",
            "🍰 Tiramisu Dessert"
        ],
        "default_table": "Table 1",
        "default_item": "☕ Espresso & Croissant"
    }
}


def detect_initial_world() -> str:
    for idx, arg in enumerate(sys.argv):
        if arg in ["--world", "-w"] and idx + 1 < len(sys.argv):
            val = sys.argv[idx + 1].lower()
            if "resto" in val or "arbi" in val or "dar" in val:
                return "Dar Tunibot (resto_arbi.world)"
            return "Restaurant Lounge (restaurant.world)"
        if "world:=resto" in arg.lower() or "world:=arbi" in arg.lower():
            return "Dar Tunibot (resto_arbi.world)"
        if "world:=rest" in arg.lower():
            return "Restaurant Lounge (restaurant.world)"

    env_world = os.environ.get("CSTAM_WORLD", "").lower()
    if "resto" in env_world or "arbi" in env_world:
        return "Dar Tunibot (resto_arbi.world)"

    # Same default as cstam_system.launch.py; switched automatically once the
    # running task manager announces its world on /delivery_waypoints.
    return "Restaurant Lounge (restaurant.world)"


def resolve_world_key(world_name: str) -> str:
    wn = str(world_name).lower()
    if "resto" in wn or "arbi" in wn or "dar" in wn:
        return "Dar Tunibot (resto_arbi.world)"
    return "Restaurant Lounge (restaurant.world)"


class Ros2DeliveryBridge:
    def __init__(self):
        self.node = None
        self.req_pub = None
        self.dock_pub = None
        self.action_pub = None
        self.obs_pub = None
        self.bat_override_pub = None

        self.latest_status = {
            "robot_state": "idle",
            "current_task": None,
            "queue_length": 0,
            "pending_tasks": [],
            "completed_count": 0
        }
        self.is_docked = False
        self._mock_task_id = 1
        self.detected_world = None  # world key announced by the running task manager

        self.latest_battery = {
            "percentage": 100.0,
            "voltage": 29.4,
            "current_amps": 0.0,
            "power_watts": 0.0,
            "temperature_c": 24.5,
            "status": "idle_standby",
            "estimated_runtime_min": 450.0,
            "is_low": False,
            "is_critical": False
        }

        self.latest_obstacle = {
            "mode": "patrol",
            "x": -1.5,
            "y": -1.3,
            "speed": 0.40,
            "distance_to_robot": 3.5,
            "is_blocking": False
        }

        if HAVE_ROS2:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.node = rclpy.create_node('cstam_delivery_simulator_ui')
                
                # Publishers
                self.req_pub = self.node.create_publisher(String, '/delivery_request', 10)
                self.dock_pub = self.node.create_publisher(String, '/dock_command', 10)
                self.action_pub = self.node.create_publisher(String, '/delivery_task_action', 10)
                self.obs_pub = self.node.create_publisher(String, '/dynamic_obstacle/control', 10)
                self.bat_override_pub = self.node.create_publisher(String, '/battery_set_percentage', 10)

                # Subscribers
                self.node.create_subscription(String, '/delivery_queue_status', self._status_cb, 10)
                self.node.create_subscription(String, '/robot_system_state', self._state_cb, 10)
                self.node.create_subscription(Bool, '/dock_state', self._dock_cb, 10)
                self.node.create_subscription(String, '/battery_status', self._battery_cb, 10)
                self.node.create_subscription(String, '/dynamic_obstacle/status', self._obs_cb, 10)
                latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.node.create_subscription(String, '/delivery_waypoints', self._waypoints_cb, latched)

                threading.Thread(target=self._spin_worker, daemon=True).start()
            except Exception as e:
                print(f"[Simulator] Warning: Could not initialize ROS 2 node ({e}). Using mock mode.")

    def _spin_worker(self):
        try:
            rclpy.spin(self.node)
        except Exception:
            pass

    def _status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_status.update(data)
        except Exception:
            pass

    def _state_cb(self, msg: String):
        self.latest_status["robot_state"] = msg.data

    def _dock_cb(self, msg: Bool):
        self.is_docked = msg.data
        if msg.data:
            self.latest_status["robot_state"] = "docked"

    def _battery_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_battery.update(data)
        except Exception:
            pass

    def _waypoints_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.detected_world = resolve_world_key(data.get("world", ""))
        except Exception:
            pass

    def _obs_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_obstacle.update(data)
        except Exception:
            pass

    def dispatch_order(self, target: str, item: str):
        payload = json.dumps({"target": target, "item": item})
        if self.req_pub:
            msg = String()
            msg.data = payload
            self.req_pub.publish(msg)
            return True
        else:
            tid = f"TASK-{self._mock_task_id:04d}"
            self._mock_task_id += 1
            self.latest_status["queue_length"] += 1
            self.latest_status["pending_tasks"].append({
                "id": tid,
                "target": target,
                "item": item
            })
            return True

    def delete_task(self, task_id: str):
        payload = json.dumps({"action": "delete", "task_id": task_id})
        if self.action_pub:
            msg = String()
            msg.data = payload
            self.action_pub.publish(msg)
        self.latest_status["pending_tasks"] = [
            t for t in self.latest_status["pending_tasks"] if t.get("id") != task_id
        ]
        self.latest_status["queue_length"] = len(self.latest_status["pending_tasks"])
        return True

    def modify_task(self, task_id: str, new_target: str, new_item: str):
        payload = json.dumps({"action": "modify", "task_id": task_id, "target": new_target, "item": new_item})
        if self.action_pub:
            msg = String()
            msg.data = payload
            self.action_pub.publish(msg)
        for t in self.latest_status["pending_tasks"]:
            if t.get("id") == task_id:
                if new_target: t["target"] = new_target
                if new_item: t["item"] = new_item
        return True

    def cancel_active_task(self):
        cur = self.latest_status.get("current_task")
        if not cur:
            return False
        if self.action_pub:
            msg = String()
            msg.data = json.dumps({"action": "delete", "task_id": cur.get("id")})
            self.action_pub.publish(msg)
        else:
            self.latest_status["current_task"] = None
            self.latest_status["robot_state"] = "idle"
        return True

    def trigger_dock(self, reason="manual"):
        if self.dock_pub:
            msg = String()
            msg.data = f"DOCK_NOW:{reason}"
            self.dock_pub.publish(msg)
        else:
            self.latest_status["robot_state"] = "docking"

    def stop_docking(self):
        if self.dock_pub:
            msg = String()
            msg.data = "STOP_DOCK"
            self.dock_pub.publish(msg)
        if self.action_pub:
            msg2 = String()
            msg2.data = json.dumps({"action": "stop_dock"})
            self.action_pub.publish(msg2)
        if self.latest_status["robot_state"] == "docking":
            self.latest_status["robot_state"] = "idle"

    def set_battery(self, percentage: float):
        self.latest_battery["percentage"] = float(percentage)
        if self.bat_override_pub:
            msg = String()
            msg.data = str(percentage)
            self.bat_override_pub.publish(msg)

    def command_obstacle(self, action: str):
        self.latest_obstacle["mode"] = action
        if self.obs_pub:
            msg = String()
            msg.data = action
            self.obs_pub.publish(msg)
            print(f"[Simulator] Obstacle command published: {action}")


class DeliverySimulatorGUI:
    def __init__(self, root, bridge: Ros2DeliveryBridge, initial_world: str):
        self.root = root
        self.bridge = bridge
        self.current_world = resolve_world_key(initial_world)
        self.root.title("CSTAM - Autonomous Waiter Robot Mission Control (Phase 2)")
        self.root.geometry("1020x840")
        self.root.configure(bg="#1e1e24")

        self.selected_task_id = None
        self._applied_detected_world = None
        self._cached_queue_sig = None
        self.preset_buttons = []

        self._setup_styles()
        self._build_ui()
        self._update_loop()

    def _setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure(".", background="#1e1e24", foreground="#ffffff", font=("DejaVu Sans", 10))
        style.configure("TLabel", background="#1e1e24", foreground="#e0e0e0")
        style.configure("TFrame", background="#1e1e24")
        style.configure("TLabelframe", background="#1e1e24", foreground="#00adb5", bordercolor="#393e46")
        style.configure("TLabelframe.Label", background="#1e1e24", foreground="#00adb5", font=("DejaVu Sans", 11, "bold"))
        style.configure("TButton", background="#00adb5", foreground="#ffffff", font=("DejaVu Sans", 10, "bold"), borderwidth=0)
        style.map("TButton", background=[("active", "#00838f"), ("pressed", "#006064")])
        style.configure("Danger.TButton", background="#e63946", foreground="#ffffff")
        style.map("Danger.TButton", background=[("active", "#d62828"), ("pressed", "#ba181b")])
        style.configure("Success.TButton", background="#2a9d8f", foreground="#ffffff")
        style.map("Success.TButton", background=[("active", "#21867a")])
        style.configure("TCombobox", fieldbackground="#2b2d42", background="#393e46", foreground="#ffffff")

    def _build_ui(self):
        # 1. Header Frame with Status & Battery Gauges
        header_frame = tk.Frame(self.root, bg="#2b2d42", padx=16, pady=10)
        header_frame.pack(fill="x", padx=12, pady=(10, 6))

        top_row = tk.Frame(header_frame, bg="#2b2d42")
        top_row.pack(fill="x")

        self.title_lbl = tk.Label(
            top_row,
            text=f"🤖 CSTAM Autonomous Waiter Control: {WORLDS_CONFIG[self.current_world]['title']}",
            font=("DejaVu Sans", 12, "bold"), fg="#ffffff", bg="#2b2d42"
        )
        self.title_lbl.pack(side="left")

        # Telemetry row
        status_bar = tk.Frame(header_frame, bg="#2b2d42", pady=4)
        status_bar.pack(fill="x")

        tk.Label(status_bar, text="Robot State: ", font=("DejaVu Sans", 10, "bold"), fg="#94a3b8", bg="#2b2d42").pack(side="left")
        self.state_badge = tk.Label(status_bar, text="IDLE", font=("DejaVu Sans", 10, "bold"),
                                    fg="#ffffff", bg="#457b9d", padx=10, pady=2)
        self.state_badge.pack(side="left", padx=6)

        self.dock_badge = tk.Label(status_bar, text="UNDOCKED", font=("DejaVu Sans", 10, "bold"),
                                   fg="#ffffff", bg="#6c757d", padx=8, pady=2)
        self.dock_badge.pack(side="left", padx=4)

        # Battery Badge
        self.battery_badge = tk.Label(status_bar, text="⚡ Battery: 100.0% (29.4V)", font=("DejaVu Sans", 10, "bold"),
                                      fg="#ffffff", bg="#2a9d8f", padx=10, pady=2)
        self.battery_badge.pack(side="right")

        # 2. Main Content Layout
        content_frame = tk.Frame(self.root, bg="#1e1e24")
        content_frame.pack(fill="both", expand=True, padx=12, pady=4)

        # Left Column: Order Dispatcher, Battery Simulation & Dynamic Obstacle Avoidance
        left_col = tk.Frame(content_frame, bg="#1e1e24")
        left_col.pack(side="left", fill="both", expand=True, padx=(0, 6))

        # Order Dispatcher Box
        order_box = ttk.LabelFrame(left_col, text=" 🍽️ Order Dispatcher ", padding=10)
        order_box.pack(fill="x", pady=(0, 8))

        ttk.Label(order_box, text="Floor Plan / World:", font=("DejaVu Sans", 9, "bold")).pack(anchor="w")
        self.world_combo = ttk.Combobox(order_box, values=list(WORLDS_CONFIG.keys()), state="readonly", font=("DejaVu Sans", 9))
        self.world_combo.set(self.current_world)
        self.world_combo.pack(fill="x", pady=(0, 6))
        self.world_combo.bind("<<ComboboxSelected>>", self._on_world_change)

        ttk.Label(order_box, text="Destination Table:", font=("DejaVu Sans", 9, "bold")).pack(anchor="w")
        cfg = WORLDS_CONFIG[self.current_world]
        self.table_combo = ttk.Combobox(order_box, values=cfg["tables"], state="readonly", font=("DejaVu Sans", 10))
        self.table_combo.set(cfg["default_table"])
        self.table_combo.pack(fill="x", pady=(0, 6))

        ttk.Label(order_box, text="Order Item / Specialty:", font=("DejaVu Sans", 9, "bold")).pack(anchor="w")
        self.item_entry = tk.Entry(order_box, font=("DejaVu Sans", 10), bg="#2b2d42", fg="#ffffff",
                                   insertbackground="#00adb5", relief="flat", highlightthickness=1,
                                   highlightbackground="#4b5563", highlightcolor="#00adb5")
        self.item_entry.insert(0, cfg["default_item"])
        self.item_entry.pack(fill="x", ipady=3, pady=(0, 6))

        # Presets
        self.preset_frame = tk.Frame(order_box, bg="#1e1e24")
        self.preset_frame.pack(fill="x", pady=2)
        self.preset_frame.columnconfigure(0, weight=1)
        self.preset_frame.columnconfigure(1, weight=1)
        self._build_preset_buttons()

        dispatch_btn = ttk.Button(order_box, text="🚀 Dispatch Order to Robot", command=self._on_dispatch)
        dispatch_btn.pack(fill="x", pady=(8, 2))

        # Battery & State Simulation Box (Phase 2 Feature)
        bat_box = ttk.LabelFrame(left_col, text=" 🔋 Battery & State Simulation (Physical 24V Pack) ", padding=10)
        bat_box.pack(fill="x", pady=(0, 8))

        self.bat_info_lbl = tk.Label(
            bat_box, text="SOC: 100.0% · 29.40V · 0.00A · Standby\nEstimated Runtime: 450 min",
            font=("DejaVu Sans", 9), fg="#e0e0e0", bg="#2b2d42", padx=8, pady=6, justify="left", anchor="w"
        )
        self.bat_info_lbl.pack(fill="x", pady=(0, 6))

        bat_ctrl_frame = tk.Frame(bat_box, bg="#1e1e24")
        bat_ctrl_frame.pack(fill="x")

        drain_low_btn = tk.Button(
            bat_ctrl_frame, text="⚡ Set Low (24%)", bg="#f4a261", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=4, pady=5,
            command=lambda: self.bridge.set_battery(24.0)
        )
        drain_low_btn.pack(side="left", fill="x", expand=True, padx=(0, 2))

        drain_crit_btn = tk.Button(
            bat_ctrl_frame, text="🚨 Set Crit (12%)", bg="#e63946", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=4, pady=5,
            command=lambda: self.bridge.set_battery(12.0)
        )
        drain_crit_btn.pack(side="left", fill="x", expand=True, padx=2)

        restore_bat_btn = tk.Button(
            bat_ctrl_frame, text="⚡ 100% Full", bg="#2a9d8f", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=4, pady=5,
            command=lambda: self.bridge.set_battery(100.0)
        )
        restore_bat_btn.pack(side="right", fill="x", expand=True, padx=(2, 0))

        # Dynamic Obstacle Control Box (Phase 2 Feature)
        obs_box = ttk.LabelFrame(left_col, text=" 🚶 Dynamic Obstacle Avoidance (Moving Human) ", padding=10)
        obs_box.pack(fill="x", pady=(0, 8))

        self.obs_info_lbl = tk.Label(
            obs_box, text="Pedestrian Mode: PATROL · Speed: 0.40m/s · Distance: 3.5m",
            font=("DejaVu Sans", 9), fg="#e0e0e0", bg="#2b2d42", padx=8, pady=6, justify="left", anchor="w"
        )
        self.obs_info_lbl.pack(fill="x", pady=(0, 6))

        obs_btn_frame = tk.Frame(obs_box, bg="#1e1e24")
        obs_btn_frame.pack(fill="x")

        patrol_btn = tk.Button(
            obs_btn_frame, text="🔄 Patrol", bg="#8338ec", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=3, pady=5,
            command=lambda: self._cmd_obs_and_feedback("patrol")
        )
        patrol_btn.pack(side="left", fill="x", expand=True, padx=(0, 2))

        cross_btn = tk.Button(
            obs_btn_frame, text="🚶 Cross", bg="#3a86ff", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=3, pady=5,
            command=lambda: self._cmd_obs_and_feedback("cross")
        )
        cross_btn.pack(side="left", fill="x", expand=True, padx=2)

        block_btn = tk.Button(
            obs_btn_frame, text="🛑 Block", bg="#e63946", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=3, pady=5,
            command=lambda: self._cmd_obs_and_feedback("block")
        )
        block_btn.pack(side="left", fill="x", expand=True, padx=2)

        clear_btn = tk.Button(
            obs_btn_frame, text="🟩 Clear", bg="#2a9d8f", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", padx=3, pady=5,
            command=lambda: self._cmd_obs_and_feedback("clear")
        )
        clear_btn.pack(side="right", fill="x", expand=True, padx=(2, 0))

        # Docking Controls
        dock_box = ttk.LabelFrame(left_col, text=" ⚓ Resting Dock Station ", padding=8)
        dock_box.pack(fill="x")

        dock_btn_frame = tk.Frame(dock_box, bg="#1e1e24")
        dock_btn_frame.pack(fill="x")

        manual_dock_btn = tk.Button(
            dock_btn_frame, text="⚓ Return-To-Dock", bg="#e76f51", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", pady=6,
            command=lambda: self.bridge.trigger_dock("manual_ui")
        )
        manual_dock_btn.pack(side="left", fill="x", expand=True, padx=(0, 2))

        stop_dock_btn = tk.Button(
            dock_btn_frame, text="🛑 Stop Docking", bg="#e63946", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", pady=6,
            command=self._on_stop_dock
        )
        stop_dock_btn.pack(side="right", fill="x", expand=True, padx=(2, 0))

        # Right Column: Live Task Queue & Monitoring
        right_col = ttk.LabelFrame(content_frame, text=" 📋 Live Delivery Queue & Monitoring ", padding=12)
        right_col.pack(side="right", fill="both", expand=True, padx=(6, 0))

        tk.Label(right_col, text="Current Active Delivery:", font=("DejaVu Sans", 10, "bold"), fg="#00adb5", bg="#1e1e24").pack(anchor="w")
        self.active_task_card = tk.Label(
            right_col, text="No active task (Robot Idle)", font=("DejaVu Sans", 10),
            fg="#f1faee", bg="#2b2d42", relief="groove", padx=10, pady=10, justify="left", anchor="w"
        )
        self.active_task_card.pack(fill="x", pady=(4, 4))
        self.cancel_active_btn = tk.Button(
            right_col, text="✖ Cancel Active Delivery", font=("DejaVu Sans", 9, "bold"),
            bg="#4b5563", fg="white", relief="flat", state="disabled", command=self._on_cancel_active
        )
        self.cancel_active_btn.pack(fill="x", pady=(0, 10))

        tk.Label(right_col, text="Pending Tasks (FIFO Queue):", font=("DejaVu Sans", 10, "bold"),
                 fg="#e0e0e0", bg="#1e1e24").pack(anchor="w", pady=(0, 2))

        list_frame = tk.Frame(right_col, bg="#1e1e24")
        list_frame.pack(fill="both", expand=True, pady=(0, 8))

        self.queue_listbox = tk.Listbox(
            list_frame, font=("DejaVu Sans Mono", 10), bg="#2b2d42", fg="#ffffff",
            selectbackground="#00adb5", selectforeground="#ffffff",
            relief="flat", highlightthickness=1, highlightbackground="#393e46",
            highlightcolor="#00adb5", activestyle="none"
        )
        self.queue_listbox.pack(side="left", fill="both", expand=True)
        self.queue_listbox.bind('<<ListboxSelect>>', self._on_queue_select)

        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.queue_listbox.yview)
        scrollbar.pack(side="right", fill="y")
        self.queue_listbox.config(yscrollcommand=scrollbar.set)

        # Selected Task Operations Box
        edit_box = ttk.LabelFrame(right_col, text=" ✏️ Modify / Delete Selected Task ", padding=8)
        edit_box.pack(fill="x", pady=(0, 8))

        self.selected_task_lbl = tk.Label(
            edit_box, text="Click a queued task above to Edit or Delete",
            font=("DejaVu Sans", 9, "italic"), fg="#94a3b8", bg="#1e1e24"
        )
        self.selected_task_lbl.pack(anchor="w", pady=(0, 4))

        action_btn_frame = tk.Frame(edit_box, bg="#1e1e24")
        action_btn_frame.pack(fill="x")

        self.modify_btn = tk.Button(
            action_btn_frame, text="💾 Save Edit", bg="#4b5563", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", state="disabled",
            padx=10, pady=6, command=self._on_modify_selected
        )
        self.modify_btn.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self.delete_btn = tk.Button(
            action_btn_frame, text="🗑️ Delete Task", bg="#4b5563", fg="#ffffff",
            font=("DejaVu Sans", 9, "bold"), relief="flat", state="disabled",
            padx=10, pady=6, command=self._on_delete_selected
        )
        self.delete_btn.pack(side="left", fill="x", expand=True, padx=4)

        self.clear_sel_btn = tk.Button(
            action_btn_frame, text="✖ Cancel", bg="#374151", fg="#ffffff",
            font=("DejaVu Sans", 9), relief="flat", state="disabled",
            padx=8, pady=6, command=self._on_clear_selection
        )
        self.clear_sel_btn.pack(side="right", fill="x", expand=True, padx=(4, 0))

        # Bottom Statistics Bar
        stats_frame = tk.Frame(right_col, bg="#1e1e24")
        stats_frame.pack(fill="x")
        self.completed_lbl = tk.Label(stats_frame, text="Completed: 0", font=("DejaVu Sans", 10, "bold"),
                                      fg="#4ade80", bg="#1e1e24")
        self.completed_lbl.pack(side="left")

        self.queue_len_lbl = tk.Label(stats_frame, text="Queued: 0", font=("DejaVu Sans", 10, "bold"),
                                      fg="#f4a261", bg="#1e1e24")
        self.queue_len_lbl.pack(side="right")

    def _build_preset_buttons(self):
        for btn in self.preset_buttons:
            btn.destroy()
        self.preset_buttons.clear()

        presets = WORLDS_CONFIG[self.current_world]["presets"]
        for i, item in enumerate(presets):
            r, c = divmod(i, 2)
            btn = tk.Button(
                self.preset_frame, text=item, bg="#2b2d42", fg="#e0e0e0",
                activebackground="#3d405b", activeforeground="#ffffff",
                font=("DejaVu Sans", 8), relief="flat", padx=4, pady=3,
                command=lambda it=item: self._select_preset(it)
            )
            btn.grid(row=r, column=c, sticky="ew", padx=2, pady=2)
            self.preset_buttons.append(btn)

    def _select_preset(self, item_name: str):
        self.item_entry.delete(0, tk.END)
        self.item_entry.insert(0, item_name)

    def _on_world_change(self, event=None):
        self.current_world = self.world_combo.get()
        cfg = WORLDS_CONFIG[self.current_world]
        self.title_lbl.config(text=f"🤖 CSTAM Control Dashboard: {cfg['title']}")
        self.table_combo.config(values=cfg["tables"])
        self.table_combo.set(cfg["default_table"])
        self.item_entry.delete(0, tk.END)
        self.item_entry.insert(0, cfg["default_item"])
        self._build_preset_buttons()

    def _on_dispatch(self):
        table = self.table_combo.get().strip()
        item = self.item_entry.get().strip()
        if not table:
            return
        self.bridge.dispatch_order(table, item)

    def _cmd_obs_and_feedback(self, action: str):
        self.bridge.command_obstacle(action)
        self.obs_info_lbl.config(
            text=f"Pedestrian Mode: {action.upper()} (Command Sent) · Communicating with robot..."
        )

    def _on_cancel_active(self):
        self.bridge.cancel_active_task()

    def _on_stop_dock(self):
        self.bridge.stop_docking()

    def _on_queue_select(self, event=None):
        sel = self.queue_listbox.curselection()
        if not sel:
            return
        idx = sel[0]
        queue = self.bridge.latest_status.get("pending_tasks", [])
        if idx < len(queue):
            task = queue[idx]
            self.selected_task_id = task.get("id")
            self._update_selection_ui(task)

    def _update_selection_ui(self, task=None):
        if task:
            self.selected_task_lbl.config(
                text=f"Selected: {task.get('id')} ({task.get('target')}) - Ready to edit or delete",
                fg="#38bdf8"
            )
            self.modify_btn.config(state="normal", bg="#0284c7")
            self.delete_btn.config(state="normal", bg="#e63946")
            self.clear_sel_btn.config(state="normal", bg="#4b5563")
            self.table_combo.set(task.get("target", ""))
            self.item_entry.delete(0, tk.END)
            self.item_entry.insert(0, task.get("item", ""))
        else:
            self.selected_task_lbl.config(
                text="Click a queued task above to Edit or Delete",
                fg="#94a3b8"
            )
            self.modify_btn.config(state="disabled", bg="#4b5563")
            self.delete_btn.config(state="disabled", bg="#4b5563")
            self.clear_sel_btn.config(state="disabled", bg="#374151")

    def _on_modify_selected(self):
        if not self.selected_task_id:
            return
        new_target = self.table_combo.get().strip()
        new_item = self.item_entry.get().strip()
        self.bridge.modify_task(self.selected_task_id, new_target, new_item)
        self._on_clear_selection()

    def _on_delete_selected(self):
        if not self.selected_task_id:
            return
        self.bridge.delete_task(self.selected_task_id)
        self._on_clear_selection()

    def _on_clear_selection(self):
        self.selected_task_id = None
        self.queue_listbox.selection_clear(0, tk.END)
        self._update_selection_ui(None)

    def _update_loop(self):
        # Follow the world announced by the task manager (once per announcement, so manual picks stick)
        detected = self.bridge.detected_world
        if detected and detected != self._applied_detected_world:
            self._applied_detected_world = detected
            if detected != self.current_world:
                self.world_combo.set(detected)
                self._on_world_change()

        status = self.bridge.latest_status
        state = status.get("robot_state", "idle").upper()

        # Update Robot State Badge
        state_colors = {
            "IDLE": ("#457b9d", "IDLE"),
            "NAVIGATING": ("#2a9d8f", "NAVIGATING 🚀"),
            "EN_ROUTE": ("#2a9d8f", "NAVIGATING 🚀"),
            "AT_TABLE": ("#e76f51", "AT TABLE 🍽️"),
            "DELIVERING": ("#e76f51", "HANDOVER 🍽️"),
            "RECOVERING": ("#e63946", "⚠️ RECOVERING (Door/Corridor)"),
            "DOCKING": ("#9c27b0", "DOCKING ⚓"),
            "DOCKED": ("#10b981", "DOCKED ⚡")
        }
        bg_col, text_lbl = state_colors.get(state, ("#457b9d", state))
        self.state_badge.config(text=text_lbl, bg=bg_col)

        # Update Dock Badge
        if self.bridge.is_docked or state == "DOCKED":
            self.dock_badge.config(text="DOCKED ⚡", bg="#10b981")
        elif state == "DOCKING":
            self.dock_badge.config(text="EN ROUTE TO DOCK", bg="#9c27b0")
        else:
            self.dock_badge.config(text="UNDOCKED", bg="#6c757d")

        # Update Battery Telemetry
        bat = self.bridge.latest_battery
        bpct = bat.get("percentage", 100.0)
        bvolt = bat.get("voltage", 29.4)
        bcurr = bat.get("current_amps", 0.0)
        bpow = bat.get("power_watts", 0.0)
        bstat = bat.get("status", "idle")
        brunt = bat.get("estimated_runtime_min", 999.0)

        if bpct > 50.0:
            bat_col = "#2a9d8f"
        elif bpct > 25.0:
            bat_col = "#f4a261"
        else:
            bat_col = "#e63946"

        self.battery_badge.config(text=f"⚡ {bpct:.1f}% ({bvolt:.1f}V)", bg=bat_col)
        self.bat_info_lbl.config(
            text=f"SOC: {bpct:.1f}% · {bvolt:.2f}V · {bcurr:.2f}A · {bpow:.1f}W · {bstat.upper()}\n"
                 f"Estimated Runtime: {brunt} min · Pack Temp: {bat.get('temperature_c', 24.5):.1f}°C"
        )

        # Update Dynamic Obstacle Telemetry
        obs = self.bridge.latest_obstacle
        obs_mode = obs.get("mode", "idle").upper()
        obs_dist = obs.get("distance_to_robot", 0.0)
        obs_spd = obs.get("speed", 0.0)
        is_block = obs.get("is_blocking", False)
        block_str = "⚠️ BLOCKING PATH!" if is_block else "CLEAR"
        self.obs_info_lbl.config(
            text=f"Pedestrian Mode: {obs_mode} · Speed: {obs_spd:.2f}m/s · Dist to Robot: {obs_dist:.2f}m\n"
                 f"Proximity Status: {block_str}"
        )

        # Active Delivery Display
        cur = status.get("current_task")
        self.cancel_active_btn.config(
            state="normal" if cur else "disabled", bg="#e63946" if cur else "#4b5563"
        )
        if cur:
            tid = cur.get("id", "TASK")
            target = cur.get("target", "Table")
            item = cur.get("item", "Meal")
            st_text = cur.get("status", "in_progress")
            self.active_task_card.config(
                text=f"[{tid}] En Route to {target}\n"
                     f"Item: {item}\n"
                     f"Phase: {st_text.upper()}",
                fg="#38bdf8", bg="#1f2937"
            )
        else:
            if state in ["DOCKED", "RESTING"]:
                self.active_task_card.config(
                    text="Robot is resting & charging at Dock Station.\nAwaiting new delivery orders.",
                    fg="#4ade80", bg="#1f2937"
                )
            elif state == "DOCKING":
                self.active_task_card.config(
                    text="Autonomous Return-To-Dock active.\nRouting through aisle to docking station.",
                    fg="#f472b6", bg="#1f2937"
                )
            elif state == "RECOVERING":
                self.active_task_card.config(
                    text="Corridor & Doorway Recovery in progress.\nSafely backing up and resetting path...",
                    fg="#f87171", bg="#1f2937"
                )
            else:
                self.active_task_card.config(
                    text="Robot is IDLE and awaiting orders.\nSelect a table on the left to dispatch.",
                    fg="#f1faee", bg="#2b2d42"
                )

        # Queue Listbox Update
        queue = status.get("pending_tasks", [])
        completed = status.get("completed_count", 0)

        queue_sig = tuple((t.get("id"), t.get("target"), t.get("item")) for t in queue)
        if queue_sig != self._cached_queue_sig:
            self._cached_queue_sig = queue_sig
            self.queue_listbox.delete(0, tk.END)
            for idx, t in enumerate(queue):
                tid = t.get("id", f"T-{idx}")
                tgt = t.get("target", "Table")
                itm = t.get("item", "Meal")
                self.queue_listbox.insert(tk.END, f"{idx+1:02d}. [{tid}] {tgt:<14} | {itm}")

            if self.selected_task_id:
                reselected = False
                for idx, t in enumerate(queue):
                    if t.get("id") == self.selected_task_id:
                        self.queue_listbox.selection_set(idx)
                        reselected = True
                        break
                if not reselected:
                    self.selected_task_id = None
                    self._update_selection_ui()

        self.completed_lbl.config(text=f"Completed Deliveries: {completed}")
        self.queue_len_lbl.config(text=f"Queued: {len(queue)}")

        self.root.after(300, self._update_loop)


def run_cli_mode(bridge: Ros2DeliveryBridge, initial_world: str):
    current_world = initial_world
    print("=" * 68)
    print("  CSTAM: Autonomous Waiter Robot Mission Control CLI (Phase 2)")
    print(f"  Active World: {current_world}")
    print("=" * 68)
    while True:
        try:
            cfg = WORLDS_CONFIG[current_world]
            bat = bridge.latest_battery
            obs = bridge.latest_obstacle
            print(f"\n[Environment: {cfg['title']}] | Battery: {bat.get('percentage', 100):.1f}% | Pedestrian: {obs.get('mode', 'patrol')}")
            print("Options:")
            print("  1. Dispatch Delivery Order")
            print("  2. View Delivery Queue & Live Status")
            print("  3. Manual Return-to-Dock Command")
            print("  4. Stop / Cancel Docking")
            print("  5. Modify Queued Task (Table or Item)")
            print("  6. Delete Queued Task")
            print("  7. Dynamic Obstacle: Cross Robot Path")
            print("  8. Dynamic Obstacle: Block Corridor / Doorway")
            print("  9. Dynamic Obstacle: Step Aside / Clear")
            print("  10. Simulate Low Battery (24%) -> Triggers Auto-Dock")
            print("  11. Simulate Critical Battery (12%) -> Emergency Override")
            print("  12. Restore Full Battery (100%)")
            print(f"  13. Switch Environment (Currently: {cfg['short_name']})")
            print("  14. Exit")
            choice = input("Select option (1-14): ").strip()

            if choice == "1":
                tables_str = ", ".join(cfg["tables"][:12])
                print(f"\nAvailable Tables: {tables_str}...")
                table = input(f"Enter target table [default: {cfg['default_table']}]: ").strip() or cfg["default_table"]
                item = input(f"Enter meal item [default: {cfg['default_item']}]: ").strip() or cfg["default_item"]
                bridge.dispatch_order(table, item)
                print(f"✔ Order dispatched for {table}: {item}")

            elif choice == "2":
                st = bridge.latest_status
                b = bridge.latest_battery
                o = bridge.latest_obstacle
                print("\n--- Live Robot Status ---")
                print(f"  State: {st.get('robot_state')}")
                print(f"  Current Task: {st.get('current_task')}")
                print(f"  Queue Length: {st.get('queue_length')}")
                print(f"  Completed: {st.get('completed_count')}")
                print(f"  Battery: {b.get('percentage', 100.0):.1f}% ({b.get('voltage', 29.4):.2f}V, {b.get('status')})")
                print(f"  Dynamic Obstacle: Mode={o.get('mode')}, Dist={o.get('distance_to_robot')}m, Blocking={o.get('is_blocking')}")
                tasks = st.get('pending_tasks', [])
                if tasks:
                    print("  Pending Queue:")
                    for idx, t in enumerate(tasks):
                        print(f"    {idx+1}. [{t.get('id')}] {t.get('target')} - {t.get('item')}")
                else:
                    print("  Queue is empty.")

            elif choice == "3":
                bridge.trigger_dock("cli_manual")
                print("✔ Return to dock command broadcasted.")

            elif choice == "4":
                bridge.stop_docking()
                print("✔ Docking stopped / cancelled.")

            elif choice == "5":
                tasks = bridge.latest_status.get('pending_tasks', [])
                if not tasks:
                    print("Pending queue is empty.")
                    continue
                for idx, t in enumerate(tasks):
                    print(f"  [{t.get('id')}] {t.get('target')} - {t.get('item')}")
                tid = input("Enter task ID to modify: ").strip()
                new_t = input("Enter new target table: ").strip()
                new_i = input("Enter new item description: ").strip()
                bridge.modify_task(tid, new_t, new_i)
                print(f"✔ Modification sent for {tid}.")

            elif choice == "6":
                tasks = bridge.latest_status.get('pending_tasks', [])
                if not tasks:
                    print("Pending queue is empty.")
                    continue
                for idx, t in enumerate(tasks):
                    print(f"  [{t.get('id')}] {t.get('target')} - {t.get('item')}")
                tid = input("Enter task ID to delete: ").strip()
                bridge.delete_task(tid)
                print(f"✔ Deletion sent for {tid}.")

            elif choice == "7":
                bridge.command_obstacle("cross")
                print("✔ Dynamic obstacle commanded to cross robot path.")

            elif choice == "8":
                bridge.command_obstacle("block")
                print("✔ Dynamic obstacle commanded to block corridor / doorway.")

            elif choice == "9":
                bridge.command_obstacle("clear")
                print("✔ Dynamic obstacle commanded to step aside / clear path.")

            elif choice == "10":
                bridge.set_battery(24.0)
                print("✔ Battery set to 24.0% (Low Battery Trigger Active).")

            elif choice == "11":
                bridge.set_battery(12.0)
                print("✔ Battery set to 12.0% (Critical Emergency Override Active).")

            elif choice == "12":
                bridge.set_battery(100.0)
                print("✔ Battery restored to 100.0% (Fully Charged).")

            elif choice == "13":
                world_keys = list(WORLDS_CONFIG.keys())
                for i, k in enumerate(world_keys):
                    print(f"  {i+1}. {k}")
                w_choice = input(f"Select world (1-{len(world_keys)}): ").strip()
                if w_choice.isdigit() and 1 <= int(w_choice) <= len(world_keys):
                    current_world = world_keys[int(w_choice) - 1]
                    print(f"✔ Switched environment to {current_world}")

            elif choice == "14":
                print("Exiting.")
                break
        except (KeyboardInterrupt, EOFError):
            break


def main():
    initial_world = detect_initial_world()
    bridge = Ros2DeliveryBridge()

    cli_forced = any(a in sys.argv for a in ["--cli", "-c", "--headless"])
    has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))

    if cli_forced or not has_display or not HAVE_TKINTER:
        run_cli_mode(bridge, initial_world)
    else:
        root = tk.Tk()
        DeliverySimulatorGUI(root, bridge, initial_world)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
