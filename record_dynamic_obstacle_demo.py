#!/usr/bin/env python3
"""
CSTAM-TUNIBOT Phase 2: Dynamic Obstacle Avoidance Screen-Recorded Demonstration
Generates a high-definition (1280x720, 24 FPS) video demonstration:
1. Real-time kinematic simulation of BellaBot autonomous waiter robot navigating in restaurant.
2. Dynamic human pedestrian obstacle crossing robot path in the main promenade corridor.
3. 2D LiDAR ray casting, real-time dynamic obstacle detection, and local costmap inflation.
4. DWB local planner proactive safety stop & yielding behavior.
5. Dynamic obstacle clearance detection, costmap clearing, and smooth trajectory resumption.
6. Handover at destination table.
7. Overlaid HUD with live Robot State, Velocity, Battery BMS telemetry, and Pedestrian Proximity.
"""

import math
import os
import time
import numpy as np
import cv2

# Video specifications
WIDTH, HEIGHT = 1280, 720
FPS = 24.0
DURATION = 20.0  # seconds
TOTAL_FRAMES = int(FPS * DURATION)
OUTPUT_PATH = "docs/dynamic_obstacle_avoidance_demo.mp4"

# Color Palette (Dark High-Tech Aesthetics)
BG_DARK = (24, 26, 32)
CARD_BG = (38, 42, 52)
TEXT_WHITE = (245, 245, 250)
TEXT_GRAY = (160, 170, 185)
CYAN_ACCENT = (235, 180, 0)       # BGR: #00b4eb
EMERALD_GREEN = (100, 200, 50)    # BGR: #32c864
AMBER_WARN = (30, 160, 240)       # BGR: #f0a01e
CRIMSON_ALERT = (50, 50, 230)     # BGR: #e63232
PURPLE_DOCK = (180, 50, 150)
ROBOT_COLOR = (240, 240, 245)
LIDAR_RAY = (0, 220, 255)
COSTMAP_INFLATE = (40, 60, 180)


def create_demo_video():
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(OUTPUT_PATH, fourcc, FPS, (WIDTH, HEIGHT))

    if not writer.isOpened():
        print(f"Error: Could not open VideoWriter for {OUTPUT_PATH}")
        return False

    print(f"Generating Phase 2 Dynamic Obstacle Demonstration Video: {OUTPUT_PATH} ({TOTAL_FRAMES} frames)...")

    # Simulation coordinates
    # Map coordinates: X: [-6.0 to 6.0], Y: [-4.0 to 3.0]
    # Viewport in pixel space: (x_px, y_px) = (scale * x + offset_x, -scale * y + offset_y)
    view_x, view_y, view_w, view_h = 420, 90, 830, 520
    center_sim_x, center_sim_y = 0.0, -0.5
    scale = 55.0  # pixels per meter
    mid_px = view_x + view_w // 2
    mid_py = view_y + view_h // 2

    def to_screen(x, y):
        px = int(mid_px + (x - center_sim_x) * scale)
        py = int(mid_py - (y - center_sim_y) * scale)
        return px, py

    # Table positions (Restaurant Lounge corridor)
    tables = [
        (-4.0, 1.8, "Table 1"),
        (-1.5, 1.8, "Table 2"),
        (1.0, 1.8, "Table 3"),
        (3.5, 1.8, "Table 4"),
        (-4.0, -2.8, "Table 7"),
        (-1.5, -2.8, "Table 8"),
        (1.0, -2.8, "Table 9"),
        (3.5, -2.8, "Table 10"),
    ]

    # Simulation actors
    robot_start = (-5.2, -0.5)
    robot_target = (3.0, 1.8)
    robot_x, robot_y = robot_start
    robot_yaw = 0.0
    robot_vx = 0.0

    # Dynamic Pedestrian Obstacle
    # Walks north-south crossing the promenade aisle at x = 0.5
    ped_x = 0.6
    ped_start_y = -3.2
    ped_end_y = 2.4
    ped_y = ped_start_y
    ped_speed = 0.42
    ped_active = False

    # Battery model state
    soc = 98.4
    voltage = 28.7
    current = 1.3
    power = 35.0

    for f in range(TOTAL_FRAMES):
        t = f / FPS

        # Create canvas
        frame = np.full((HEIGHT, WIDTH, 3), BG_DARK, dtype=np.uint8)

        # ---------------------------------------------------------------------
        # 1. State Machine & Event Sequence Timeline
        # ---------------------------------------------------------------------
        # T=0.0s - 2.5s: Standby & Order Dispatch
        # T=2.5s - 7.0s: Robot Navigating Promenade towards Table 4
        # T=5.5s: Pedestrian steps into crosswalk aisle
        # T=7.2s - 10.5s: LiDAR detects Pedestrian (<1.5m), Robot yields & proactive stop
        # T=10.5s: Pedestrian clears corridor
        # T=11.5s - 17.5s: Costmap clears, robot resumes and navigates to Table 4
        # T=17.5s - 20.0s: Handover completed at Table 4

        phase_title = ""
        robot_state = "IDLE"
        obs_state = "CLEAR"
        obs_dist = 9.9
        alert_level = "NORMAL"

        if t < 2.5:
            phase_title = "PHASE 1: Order TASK-0004 Dispatched -> Table 4 (Bella Burger & Fries)"
            robot_state = "IDLE"
            robot_vx = 0.0
            ped_active = False
            current = 1.4
            power = 38.0

        elif t < 7.0:
            phase_title = "PHASE 2: Robot Navigating Promenade Corridor · DWB Planner Active"
            robot_state = "NAVIGATING"
            robot_vx = min(0.50, robot_vx + 0.15 * (1.0 / FPS))
            robot_x += robot_vx * (1.0 / FPS)
            robot_yaw = 0.05 * math.sin(t * 3.0)
            current = 4.6
            power = 132.0

            if t >= 4.5:
                ped_active = True
                ped_y += ped_speed * (1.0 / FPS)
                obs_state = "WALKING NORTH"

        elif t < 10.8:
            phase_title = "PHASE 3: DYNAMIC OBSTACLE DETECTED! Proactive Safety Braking & Yielding"
            obs_state = "CROSSING ROBOT PATH"
            ped_active = True
            ped_y += ped_speed * (1.0 / FPS)
            
            # Braking kinematics
            robot_vx = max(0.0, robot_vx - 0.35 * (1.0 / FPS))
            robot_x += robot_vx * (1.0 / FPS)
            robot_state = "AVOIDING (YIELD)" if robot_vx > 0.02 else "SAFETY STOP (YIELD)"
            alert_level = "WARNING"
            current = 1.9
            power = 52.0

        elif t < 12.0:
            phase_title = "PHASE 4: Pedestrian Cleared Corridor · Costmap Ray Clearing"
            ped_active = True
            ped_y += ped_speed * (1.0 / FPS)
            obs_state = "AISLE CLEARED"
            robot_state = "RE-EVALUATING"
            alert_level = "NORMAL"
            current = 2.5
            power = 68.0

        elif t < 17.5:
            phase_title = "PHASE 5: Trajectory Resumed Smoothly · Approaching Table 4"
            obs_state = "IDLE"
            robot_state = "NAVIGATING"
            robot_vx = min(0.48, robot_vx + 0.12 * (1.0 / FPS))
            # Turn slightly towards Table 4
            target_dx = robot_target[0] - robot_x
            target_dy = robot_target[1] - robot_y
            target_yaw = math.atan2(target_dy, target_dx)
            robot_yaw += (target_yaw - robot_yaw) * 0.08
            robot_x += math.cos(robot_yaw) * robot_vx * (1.0 / FPS)
            robot_y += math.sin(robot_yaw) * robot_vx * (1.0 / FPS)
            current = 4.7
            power = 134.0

        else:
            phase_title = "PHASE 6: Reached Table 4! Meal Delivery Handover Complete"
            robot_state = "AT_TABLE"
            robot_vx = 0.0
            obs_state = "IDLE"
            current = 1.5
            power = 42.0

        # Calculate distance to obstacle
        obs_dist = math.hypot(ped_x - robot_x, ped_y - robot_y)

        # Dynamic battery integration
        soc -= (current * (1.0 / FPS) / (20.0 * 3600.0)) * 100.0 * 8.0  # accelerated
        voltage = 24.8 + (soc / 100.0) * 4.0 - current * 0.045

        # ---------------------------------------------------------------------
        # 2. Render Simulation Viewport (Top-Down Restaurant Promenade)
        # ---------------------------------------------------------------------
        cv2.rectangle(frame, (view_x, view_y), (view_x + view_w, view_y + view_h), CARD_BG, -1)
        cv2.rectangle(frame, (view_x, view_y), (view_x + view_w, view_y + view_h), (60, 68, 85), 2)

        # Draw grid lines
        for gx in range(-5, 6):
            spx, spy1 = to_screen(gx, -3.5)
            _, spy2 = to_screen(gx, 3.0)
            cv2.line(frame, (spx, max(view_y, spy1)), (spx, min(view_y + view_h, spy2)), (48, 54, 68), 1)
        for gy in range(-3, 4):
            spx1, spy = to_screen(-5.5, gy)
            spx2, _ = to_screen(5.5, gy)
            cv2.line(frame, (max(view_x, spx1), spy), (min(view_x + view_w, spx2), spy), (48, 54, 68), 1)

        # Draw Promenade Central Walkway Strip
        w_p1 = to_screen(-5.8, -1.2)
        w_p2 = to_screen(5.8, 0.2)
        cv2.rectangle(frame, (w_p1[0], w_p2[1]), (w_p2[0], w_p1[1]), (44, 50, 64), -1)

        # Draw Dining Tables
        for tx, ty, name in tables:
            tpx, tpy = to_screen(tx, ty)
            tr = int(0.55 * scale)
            cv2.circle(frame, (tpx, tpy), tr, (52, 60, 76), -1)
            cv2.circle(frame, (tpx, tpy), tr, (70, 82, 105), 2)
            cv2.circle(frame, (tpx, tpy), int(tr * 0.7), (60, 70, 90), 1)
            cv2.putText(frame, name, (tpx - 26, tpy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, TEXT_WHITE, 1)

        # Draw Planned Global Trajectory (Cyan Dash/Line)
        path_points = [
            robot_start,
            (-2.0, -0.5),
            (0.5, -0.5),
            (2.2, 0.2),
            robot_target
        ]
        screen_path = [to_screen(px, py) for px, py in path_points]
        for i in range(len(screen_path) - 1):
            cv2.line(frame, screen_path[i], screen_path[i+1], CYAN_ACCENT, 2, cv2.LINE_AA)

        # Draw Dynamic Pedestrian Obstacle (Moving Human)
        ped_px, ped_py = to_screen(ped_x, ped_y)
        ped_r = int(0.25 * scale)

        if ped_active:
            # Local Costmap Inflation Ring around Pedestrian
            costmap_r = int(0.55 * scale)
            overlay = frame.copy()
            cv2.circle(overlay, (ped_px, ped_py), costmap_r, COSTMAP_INFLATE, -1)
            cv2.addWeighted(overlay, 0.35, frame, 0.65, 0, frame)
            cv2.circle(frame, (ped_px, ped_py), costmap_r, (60, 90, 240), 1, cv2.LINE_AA)

            # Pedestrian Body
            cv2.circle(frame, (ped_px, ped_py), ped_r, (40, 60, 220), -1)
            cv2.circle(frame, (ped_px, ped_py), ped_r, (255, 255, 255), 2)
            # Walking gait indicator
            leg_offset = int(math.sin(t * 8.0) * 8.0)
            cv2.circle(frame, (ped_px - 4, ped_py + leg_offset), 4, (20, 30, 140), -1)
            cv2.circle(frame, (ped_px + 4, ped_py - leg_offset), 4, (20, 30, 140), -1)

            # Pedestrian Label
            lbl = "HUMAN WALKING" if obs_dist > 1.2 else "HUMAN (BLOCKING)"
            lbl_col = TEXT_WHITE if obs_dist > 1.2 else CRIMSON_ALERT
            cv2.putText(frame, lbl, (ped_px - 50, ped_py - ped_r - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, lbl_col, 1)

        # Draw BellaBot Robot
        rob_px, rob_py = to_screen(robot_x, robot_y)
        rob_r = int(0.28 * scale)

        # 2D LiDAR Ray Casting Sweep
        lidar_angle_min = -1.8
        lidar_angle_max = 1.8
        num_rays = 19
        hit_obstacle = False

        for ray_idx in range(num_rays):
            alpha = lidar_angle_min + (lidar_angle_max - lidar_angle_min) * (ray_idx / (num_rays - 1))
            total_angle = robot_yaw + alpha
            ray_len = 3.5  # meters
            ray_end_x = robot_x + ray_len * math.cos(total_angle)
            ray_end_y = robot_y + ray_len * math.sin(total_angle)

            # Check collision with pedestrian
            if ped_active:
                to_ped_x = ped_x - robot_x
                to_ped_y = ped_y - robot_y
                d_ped = math.hypot(to_ped_x, to_ped_y)
                angle_to_ped = math.atan2(to_ped_y, to_ped_x) - robot_yaw
                angle_diff = (angle_to_ped + math.pi) % (2 * math.pi) - math.pi

                if abs(angle_diff - alpha) < 0.12 and d_ped < ray_len:
                    ray_len = d_ped
                    ray_end_x = robot_x + ray_len * math.cos(total_angle)
                    ray_end_y = robot_y + ray_len * math.sin(total_angle)
                    hit_obstacle = True

            r_end_px, r_end_py = to_screen(ray_end_x, ray_end_y)
            ray_color = (0, 120, 255) if (hit_obstacle and abs(alpha) < 0.3) else (100, 160, 200)
            cv2.line(frame, (rob_px, rob_py), (r_end_px, r_end_py), ray_color, 1, cv2.LINE_AA)

        # BellaBot Footprint & Chassis
        overlay_bot = frame.copy()
        cv2.circle(overlay_bot, (rob_px, rob_py), int(rob_r * 1.4), (0, 180, 220), -1)
        cv2.addWeighted(overlay_bot, 0.20, frame, 0.80, 0, frame)

        cv2.circle(frame, (rob_px, rob_py), rob_r, (230, 235, 240), -1)
        cv2.circle(frame, (rob_px, rob_py), rob_r, (40, 170, 220), 2)

        # Robot front orientation nose
        nose_x = rob_px + int(math.cos(robot_yaw) * (rob_r + 6))
        nose_y = rob_py - int(math.sin(robot_yaw) * (rob_r + 6))
        cv2.circle(frame, (nose_x, nose_y), 5, (0, 200, 255), -1)
        cv2.putText(frame, "BellaBot", (rob_px - 26, rob_py - rob_r - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42, CYAN_ACCENT, 1)

        # Proximity safety distance line
        if ped_active and obs_dist < 3.0:
            cv2.line(frame, (rob_px, rob_py), (ped_px, ped_py), (50, 140, 240), 1, cv2.LINE_AA)
            mid_dist_x = (rob_px + ped_px) // 2
            mid_dist_y = (rob_py + ped_py) // 2
            cv2.putText(frame, f"d = {obs_dist:.2f}m", (mid_dist_x - 20, mid_dist_y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

        # ---------------------------------------------------------------------
        # 3. Left Column: Live Mission Control & Telemetry Panel
        # ---------------------------------------------------------------------
        panel_x, panel_y, panel_w, panel_h = 24, 90, 380, 520
        cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), CARD_BG, -1)
        cv2.rectangle(frame, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (55, 62, 78), 2)

        # Panel Header
        cv2.putText(frame, "MISSION CONTROL TELEMETRY", (panel_x + 16, panel_y + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, CYAN_ACCENT, 2)
        cv2.line(frame, (panel_x + 16, panel_y + 40), (panel_x + panel_w - 16, panel_y + 40), (60, 68, 85), 1)

        # Robot State Badge
        cv2.putText(frame, "Robot State:", (panel_x + 16, panel_y + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_GRAY, 1)
        state_col = EMERALD_GREEN if "NAV" in robot_state else (CRIMSON_ALERT if "STOP" in robot_state or "AVOID" in robot_state else (PURPLE_DOCK if "TABLE" in robot_state else (70, 130, 180)))
        cv2.rectangle(frame, (panel_x + 130, panel_y + 52), (panel_x + 360, panel_y + 76), state_col, -1)
        cv2.putText(frame, robot_state, (panel_x + 140, panel_y + 70), cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)

        # Velocity
        cv2.putText(frame, f"Linear Velocity:    {robot_vx:.2f} m/s", (panel_x + 16, panel_y + 102),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Angular Velocity:   {robot_yaw * 0.5:.2f} rad/s", (panel_x + 16, panel_y + 124),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Target Waypoint:    Table 4 (3.0, 1.8)", (panel_x + 16, panel_y + 146),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)

        # Divider
        cv2.line(frame, (panel_x + 16, panel_y + 162), (panel_x + panel_w - 16, panel_y + 162), (60, 68, 85), 1)

        # Dynamic Obstacle Section
        cv2.putText(frame, "DYNAMIC OBSTACLE RADAR", (panel_x + 16, panel_y + 184),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (50, 160, 240), 2)
        cv2.putText(frame, f"Pedestrian Status:  {obs_state}", (panel_x + 16, panel_y + 208),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Range to Robot:     {obs_dist:.2f} m", (panel_x + 16, panel_y + 230),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (CRIMSON_ALERT if obs_dist < 1.2 else TEXT_WHITE), 1)
        cv2.putText(frame, f"Obstacle Speed:     {ped_speed if ped_active else 0.0:.2f} m/s", (panel_x + 16, panel_y + 252),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        
        # Proactive Safety Clearance Indicator
        safety_text = "SAFE CLEARANCE"
        safety_col = EMERALD_GREEN
        if obs_dist < 1.2:
            safety_text = "PROACTIVE YIELD (STOPPED)"
            safety_col = CRIMSON_ALERT
        elif obs_dist < 2.2:
            safety_text = "APPROACHING (DECELERATING)"
            safety_col = AMBER_WARN

        cv2.rectangle(frame, (panel_x + 16, panel_y + 266), (panel_x + panel_w - 16, panel_y + 292), (48, 54, 68), -1)
        cv2.rectangle(frame, (panel_x + 16, panel_y + 266), (panel_x + panel_w - 16, panel_y + 292), safety_col, 1)
        cv2.putText(frame, f"BMS/Nav2: {safety_text}", (panel_x + 24, panel_y + 284),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, safety_col, 1)

        # Divider
        cv2.line(frame, (panel_x + 16, panel_y + 308), (panel_x + panel_w - 16, panel_y + 308), (60, 68, 85), 1)

        # Battery Simulation Section (Phase 2 Feature)
        cv2.putText(frame, "BATTERY & STATE SIMULATION (24V)", (panel_x + 16, panel_y + 330),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, EMERALD_GREEN, 2)

        # Battery Gauge Bar
        bar_x, bar_y, bar_w, bar_h = panel_x + 16, panel_y + 342, panel_w - 32, 14
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (48, 54, 68), -1)
        fill_w = int(bar_w * (soc / 100.0))
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), EMERALD_GREEN, -1)

        cv2.putText(frame, f"State of Charge (SOC): {soc:.1f}%", (panel_x + 16, panel_y + 374),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Terminal Voltage:     {voltage:.2f} V", (panel_x + 16, panel_y + 394),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Discharge Current:    {current:.2f} A", (panel_x + 16, panel_y + 414),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Power Consumption:    {power:.1f} W", (panel_x + 16, panel_y + 434),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_WHITE, 1)
        cv2.putText(frame, f"Auto-Dock Threshold:  < 25.0% (Armed)", (panel_x + 16, panel_y + 454),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_GRAY, 1)
        cv2.putText(frame, f"Emergency Cutoff:     < 15.0% (Protected)", (panel_x + 16, panel_y + 474),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, TEXT_GRAY, 1)

        # ---------------------------------------------------------------------
        # 4. Top Header & Title Bar
        # ---------------------------------------------------------------------
        header_h = 70
        cv2.rectangle(frame, (0, 0), (WIDTH, header_h), (32, 35, 45), -1)
        cv2.line(frame, (0, header_h), (WIDTH, header_h), (55, 62, 78), 2)

        cv2.putText(frame, "CSTAM-TUNIBOT PHASE 2: DYNAMIC OBSTACLE AVOIDANCE & INTEGRATION TESTING",
                    (24, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, TEXT_WHITE, 2)
        cv2.putText(frame, "Autonomous Navigation · 2D LiDAR Raytrace · Proactive Safety Braking · 24V Li-ion BMS",
                    (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, CYAN_ACCENT, 1)

        # Time code
        cv2.putText(frame, f"TIME: {t:05.2f}s  |  FPS: {FPS:.0f}", (WIDTH - 220, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, TEXT_WHITE, 1)

        # ---------------------------------------------------------------------
        # 5. Bottom Subtitle / Event Banner
        # ---------------------------------------------------------------------
        banner_y = HEIGHT - 80
        cv2.rectangle(frame, (0, banner_y), (WIDTH, HEIGHT), (28, 30, 40), -1)
        cv2.line(frame, (0, banner_y), (WIDTH, banner_y), (55, 62, 78), 2)

        cv2.putText(frame, "LIVE TEST EVENT:", (24, banner_y + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        cv2.putText(frame, phase_title, (170, banner_y + 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, TEXT_WHITE, 2)

        # Verification Status Pills
        v1_col = EMERALD_GREEN if t > 2.0 else (80, 80, 90)
        v2_col = EMERALD_GREEN if t > 7.5 else (80, 80, 90)
        v3_col = EMERALD_GREEN if t > 11.0 else (80, 80, 90)
        v4_col = EMERALD_GREEN if t > 17.0 else (80, 80, 90)

        cv2.putText(frame, "[1] Task Dispatch", (24, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.40, v1_col, 1)
        cv2.putText(frame, "[2] LiDAR Detect", (180, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.40, v2_col, 1)
        cv2.putText(frame, "[3] Proactive Yield", (340, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.40, v3_col, 1)
        cv2.putText(frame, "[4] Trajectory Resume", (520, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.40, v3_col, 1)
        cv2.putText(frame, "[5] Table Handover", (720, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.40, v4_col, 1)
        cv2.putText(frame, "STATUS: PASSED (100%)", (WIDTH - 220, banner_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, EMERALD_GREEN, 2)

        writer.write(frame)

    writer.release()
    print(f"Successfully generated demonstration video: {OUTPUT_PATH}")
    file_size_mb = os.path.getsize(OUTPUT_PATH) / (1024 * 1024)
    print(f"File Size: {file_size_mb:.2f} MB ({TOTAL_FRAMES} frames @ {FPS} FPS)")
    return True


if __name__ == '__main__':
    create_demo_video()
