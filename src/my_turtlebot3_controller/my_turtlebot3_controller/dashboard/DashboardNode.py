#!/usr/bin/env python3
"""
Nutrient Nexus Digital Twin Dashboard (digital entity).

This Tkinter GUI is the DIGITAL entity of the twin. It:
  - mirrors physical-robot state published by the Gazebo robot
    (resources, zone, navigation, system health, obstacles);
  - controls the physical robot by publishing weather and fault
    injection events that change the robot's behaviour.

Cross-entity topics:
  - subscribes to robot telemetry; publishes /weather_forecast and
    /twin_fault_state back to the robot;
  - mirrors /system_health and /robot_resources, and injects /twin_fault_state;
  - /obstacle_status reflects what the robot senses in its environment.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import os
import threading
import tkinter as tk
from tkinter import ttk
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from my_turtlebot3_controller.qos import STATE_QOS


class DashboardNode(Node):
    def __init__(self) -> None:
        """Create the publishers (digital -> physical: weather, faults, report),
        the subscribers that mirror physical state, the field-zone caches, then
        build the Tk window and start the refresh loop."""
        super().__init__('nexus_dashboard_node')

        # ── Publishers (Digital → Physical) ──────────────────────────
        self.weather_pub = self.create_publisher(String, '/weather_forecast', STATE_QOS)
        self.report_pub = self.create_publisher(String, '/generate_report', 10)
        self.fault_pub = self.create_publisher(String, '/twin_fault_state', STATE_QOS)

        # ── Subscribers (Physical → Digital) ─────────────────────────
        self.create_subscription(String, '/robot_resources', self.resource_cb, STATE_QOS)
        self.create_subscription(String, '/current_zone', self.zone_cb, STATE_QOS)
        self.create_subscription(String, '/navigation_executor_status', self.nav_cb, STATE_QOS)
        self.create_subscription(String, '/system_health', self.health_cb, STATE_QOS)
        self.create_subscription(String, '/obstacle_status', self.obstacle_cb, STATE_QOS)

        # ── Field zone telemetry (same data that colours the Gazebo tiles) ──
        self.create_subscription(Float32MultiArray, '/field_moisture', self._moisture_cb, STATE_QOS)
        self.create_subscription(Float32MultiArray, '/field_nutrients', self._nutrients_cb, STATE_QOS)
        self.create_subscription(Float32MultiArray, '/field_vulnerability', self._vulnerability_cb, STATE_QOS)

        # ── Mirrored physical state (guarded by lock) ────────────────
        self.state_lock = threading.Lock()
        self.state = {
            'battery': 100.0, 'fertilizer': 100.0,
            'zone': 'Unknown', 'nav': 'IDLE',
            'twin_mode': 'NORMAL',
            'battery_status': 'NO_DATA', 'lidar_status': 'NO_DATA',
            'imu_status': 'NO_DATA',
            'obstacle': 'No obstacle',
        }

        # ── Field zone state (one entry per zone, indexed like the arrays) ──
        self.zone_ids = self._load_zone_ids()
        self.field = {
            'moisture': {z: None for z in self.zone_ids},
            'nutrients': {z: None for z in self.zone_ids},
            'vulnerability': {z: None for z in self.zone_ids},
        }
        # Thresholds match CropDecisionNode / ZoneVisualizerNode.
        self.moisture_threshold = 40.0
        self.nutrient_threshold = 50.0
        self.vulnerability_halt = 70.0

        # Injected fault state — the dashboard's override of the robot.
        self.faults = {'lidar': 'ok', 'motor': 'ok', 'battery': 'normal'}

        self._build_gui()
        self.update_gui_loop()

    # ── GUI construction ──────────────────────────────────────────────

    def _build_gui(self) -> None:
        """Build the dark-themed window and lay out every panel (resources,
        telemetry, zone status, health, obstacle, weather, faults, audit)."""
        self.root = tk.Tk()
        self.root.title('Nutrient Nexus - Digital Twin Dashboard')
        self.root.geometry('980x880')
        self.root.configure(bg='#1E1E1E')

        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TLabel', background='#1E1E1E', foreground='#FFFFFF', font=('Helvetica', 12))
        self.style.configure('Header.TLabel', font=('Helvetica', 16, 'bold'), foreground='#4A90E2')
        self.style.configure('red.Horizontal.TProgressbar', background='#E74C3C')
        self.style.configure('green.Horizontal.TProgressbar', background='#2ECC71')

        main = tk.Frame(self.root, bg='#1E1E1E', padx=20, pady=15)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, text='Nutrient Nexus Command Center', style='Header.TLabel').pack(pady=(0, 5))

        self.mode_var = tk.StringVar(value='TWIN MODE: NORMAL')
        self.mode_lbl = tk.Label(main, textvariable=self.mode_var, bg='#2ECC71', fg='#000',
                                 font=('Helvetica', 13, 'bold'), pady=6)
        self.mode_lbl.pack(fill=tk.X, pady=(0, 10))

        self._build_resources(main)
        self._build_telemetry(main)
        self._build_zone_status(main)
        self._build_health(main)
        self._build_obstacle(main)
        self._build_weather(main)
        self._build_faults(main)
        self._build_audit(main)

    def _build_resources(self, parent: tk.Widget) -> None:
        """Battery and fertilizer progress bars (mirror of /robot_resources)."""
        frame = tk.LabelFrame(parent, text='Physical Robot Resources (Gazebo)', bg='#1E1E1E',
                              fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)

        tk.Label(frame, text='Battery:', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10)).grid(row=0, column=0, sticky='w', pady=2)
        self.bat_bar = ttk.Progressbar(frame, length=240, mode='determinate', style='green.Horizontal.TProgressbar')
        self.bat_bar.grid(row=0, column=1, padx=10)
        self.bat_lbl = tk.Label(frame, text='100%', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10))
        self.bat_lbl.grid(row=0, column=2)

        tk.Label(frame, text='Fertilizer:', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10)).grid(row=1, column=0, sticky='w', pady=2)
        self.fert_bar = ttk.Progressbar(frame, length=240, mode='determinate', style='green.Horizontal.TProgressbar')
        self.fert_bar.grid(row=1, column=1, padx=10)
        self.fert_lbl = tk.Label(frame, text='100%', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10))
        self.fert_lbl.grid(row=1, column=2)

    def _build_telemetry(self, parent: tk.Widget) -> None:
        """Current-zone and navigation-status labels for the robot."""
        frame = tk.LabelFrame(parent, text='Telemetry', bg='#1E1E1E', fg='#FFFFFF',
                              font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        self.zone_var = tk.StringVar(value='Zone: Unknown')
        tk.Label(frame, textvariable=self.zone_var, bg='#1E1E1E', fg='#4A90E2', font=('Helvetica', 11, 'bold')).grid(row=0, column=0, sticky='w', padx=10, pady=2)
        self.nav_var = tk.StringVar(value='Nav: IDLE')
        tk.Label(frame, textvariable=self.nav_var, bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10)).grid(row=1, column=0, sticky='w', padx=10, pady=2)

    def _build_zone_status(self, parent: tk.Widget) -> None:
        """One live row per field zone (moisture / nutrients / colour-coded
        status), kept in self.zone_widgets for update_gui_loop to refresh."""
        frame = tk.LabelFrame(parent, text='Field Zone Status (live moisture / nutrients)',
                              bg='#1E1E1E', fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)

        for col, h in enumerate(('Zone', 'Moisture', 'Nutrients', 'Status')):
            tk.Label(frame, text=h, bg='#1E1E1E', fg='#4A90E2',
                     font=('Helvetica', 10, 'bold')).grid(row=0, column=col, sticky='w', padx=18, pady=(2, 4))

        self.zone_widgets = {}
        for r, z in enumerate(self.zone_ids, start=1):
            tk.Label(frame, text=self._zone_label(z), bg='#1E1E1E', fg='#FFF',
                     font=('Helvetica', 10, 'bold')).grid(row=r, column=0, sticky='w', padx=18, pady=2)
            moist = tk.Label(frame, text='--', bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10))
            moist.grid(row=r, column=1, sticky='w', padx=18, pady=2)
            nutri = tk.Label(frame, text='--', bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10))
            nutri.grid(row=r, column=2, sticky='w', padx=18, pady=2)
            status = tk.Label(frame, text='NO DATA', bg='#1E1E1E', fg='#888',
                              font=('Helvetica', 10, 'bold'))
            status.grid(row=r, column=3, sticky='w', padx=18, pady=2)
            self.zone_widgets[z] = (moist, nutri, status)

    def _build_health(self, parent: tk.Widget) -> None:
        """Battery / LiDAR / IMU status labels (mirror of /system_health)."""
        frame = tk.LabelFrame(parent, text='System Health (/system_health)', bg='#1E1E1E',
                              fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        self.health_bat_var = tk.StringVar(value='Battery: NO_DATA')
        self.health_lidar_var = tk.StringVar(value='LiDAR: NO_DATA')
        self.health_imu_var = tk.StringVar(value='IMU: NO_DATA')
        self.health_bat_lbl = tk.Label(frame, textvariable=self.health_bat_var, bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10))
        self.health_bat_lbl.grid(row=0, column=0, sticky='w', padx=10, pady=2)
        self.health_lidar_lbl = tk.Label(frame, textvariable=self.health_lidar_var, bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10))
        self.health_lidar_lbl.grid(row=0, column=1, sticky='w', padx=10, pady=2)
        self.health_imu_lbl = tk.Label(frame, textvariable=self.health_imu_var, bg='#1E1E1E', fg='#CCC', font=('Helvetica', 10))
        self.health_imu_lbl.grid(row=0, column=2, sticky='w', padx=10, pady=2)

    def _build_obstacle(self, parent: tk.Widget) -> None:
        """Live obstacle readout (mirror of /obstacle_status from SafetyStop)."""
        frame = tk.LabelFrame(parent, text='Environment / Obstacle (/obstacle_status)', bg='#1E1E1E',
                              fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        self.obstacle_var = tk.StringVar(value='No obstacle')
        self.obstacle_lbl = tk.Label(frame, textvariable=self.obstacle_var, bg='#1E1E1E', fg='#2ECC71', font=('Helvetica', 12, 'bold'))
        self.obstacle_lbl.pack(anchor='w', padx=10, pady=2)

    def _build_weather(self, parent: tk.Widget) -> None:
        """Buttons that inject a weather state down to the physical robot."""
        frame = tk.LabelFrame(parent, text='Weather Injection (SDG-14)', bg='#1E1E1E',
                              fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        bar = tk.Frame(frame, bg='#1E1E1E')
        bar.pack()
        tk.Button(bar, text='Sunny', bg='#F1C40F', fg='#000', font=('Helvetica', 10, 'bold'), command=lambda: self.set_weather('sunny')).grid(row=0, column=0, padx=8)
        tk.Button(bar, text='Rainy', bg='#3498DB', fg='#FFF', font=('Helvetica', 10, 'bold'), command=lambda: self.set_weather('rainy')).grid(row=0, column=1, padx=8)
        tk.Button(bar, text='Overcast', bg='#95A5A6', fg='#FFF', font=('Helvetica', 10, 'bold'), command=lambda: self.set_weather('overcast')).grid(row=0, column=2, padx=8)
        tk.Button(bar, text='Storm', bg='#8E44AD', fg='#FFF', font=('Helvetica', 10, 'bold'), command=lambda: self.set_weather('storm')).grid(row=0, column=3, padx=8)

    def _build_faults(self, parent: tk.Widget) -> None:
        """Buttons that inject LiDAR / motor faults onto the physical robot."""
        frame = tk.LabelFrame(parent, text='Fault Injection (Digital → Physical)', bg='#1E1E1E',
                              fg='#FFFFFF', font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        bar = tk.Frame(frame, bg='#1E1E1E')
        bar.pack()

        tk.Label(bar, text='LiDAR:', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10)).grid(row=0, column=0, padx=4)
        self.lidar_btn = tk.Button(bar, text='OK', bg='#2ECC71', fg='#000', width=10,
                                   font=('Helvetica', 10, 'bold'), command=self.cycle_lidar)
        self.lidar_btn.grid(row=0, column=1, padx=4)

        tk.Label(bar, text='Motor:', bg='#1E1E1E', fg='#FFF', font=('Helvetica', 10)).grid(row=0, column=2, padx=4)
        self.motor_btn = tk.Button(bar, text='OK', bg='#2ECC71', fg='#000', width=10,
                                   font=('Helvetica', 10, 'bold'), command=self.toggle_motor)
        self.motor_btn.grid(row=0, column=3, padx=4)

        # tk.Label(bar, text="Battery:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=0, column=4, padx=4)
        # self.battery_btn = tk.Button(bar, text="NORMAL", bg="#2ECC71", fg="#000", width=10,
        #                              font=("Helvetica", 10, "bold"), command=self.toggle_battery)
        # self.battery_btn.grid(row=0, column=5, padx=4)

        tk.Button(bar, text='Clear All Faults', bg='#E67E22', fg='#FFF',
                  font=('Helvetica', 10, 'bold'), command=self.clear_faults).grid(row=0, column=6, padx=12)

    def _build_audit(self, parent: tk.Widget) -> None:
        """Button that asks SustainabilityAuditNode to write its report."""
        frame = tk.LabelFrame(parent, text='Sustainability Audit', bg='#1E1E1E', fg='#FFFFFF',
                              font=('Helvetica', 12, 'bold'))
        frame.pack(fill=tk.X, pady=6, ipadx=10, ipady=8)
        tk.Button(frame, text='Generate Sustainability Report', bg='#2ECC71', fg='#FFF',
                  font=('Helvetica', 11, 'bold'), command=self.generate_report).pack(pady=4)

    # ── Publishers (Digital → Physical actions) ───────────────────────

    def set_weather(self, weather: str) -> None:
        """
        Publish a weather state down to the physical robot.

        @param weather: one of 'sunny' | 'rainy' | 'overcast' | 'storm'.
        @pre  self.weather_pub is created (true after __init__).
        @post A std_msgs/String is published on /weather_forecast.
        @return None.
        """
        self.weather_pub.publish(String(data=weather))
        self.get_logger().info(f'Injected weather: {weather}')

    def generate_report(self) -> None:
        """Trigger the sustainability report via /generate_report."""
        self.report_pub.publish(String(data='generate'))

    def _publish_faults(self) -> None:
        """
        Publish the current injected fault set to the physical side.

        @pre  self.faults holds the lidar/motor/battery modes.
        @post A JSON std_msgs/String (the fault set plus a boolean 'active'
              flag) is published on /twin_fault_state for SystemMonitor,
              SafetyStop and RobotResource to enact.
        @return None.
        """
        payload = dict(self.faults)
        payload['active'] = any(v not in ('ok', 'normal') for v in self.faults.values())
        self.fault_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'Injected fault state: {payload}')

    def cycle_lidar(self) -> None:
        """Cycle the LiDAR fault ok -> degraded -> failed and republish faults."""
        order = ['ok', 'degraded', 'failed']
        self.faults['lidar'] = order[(order.index(self.faults['lidar']) + 1) % len(order)]
        colors = {'ok': '#2ECC71', 'degraded': '#F39C12', 'failed': '#E74C3C'}
        self.lidar_btn.config(text=self.faults['lidar'].upper(), bg=colors[self.faults['lidar']])
        self._publish_faults()

    def toggle_motor(self) -> None:
        """Toggle the motor fault ok <-> stalled and republish faults."""
        self.faults['motor'] = 'stalled' if self.faults['motor'] == 'ok' else 'ok'
        ok = self.faults['motor'] == 'ok'
        self.motor_btn.config(text=self.faults['motor'].upper(), bg='#2ECC71' if ok else '#E74C3C')
        self._publish_faults()

    def toggle_battery(self) -> None:
        """Toggle the battery fault normal <-> clamped and republish faults
        (button currently hidden; kept for the battery-fault cascade demo)."""
        self.faults['battery'] = 'clamped' if self.faults['battery'] == 'normal' else 'normal'
        ok = self.faults['battery'] == 'normal'
        self.battery_btn.config(text=self.faults['battery'].upper(), bg='#2ECC71' if ok else '#E74C3C')
        self._publish_faults()

    def clear_faults(self) -> None:
        """Reset every injected fault to healthy and republish."""
        self.faults = {'lidar': 'ok', 'motor': 'ok', 'battery': 'normal'}
        self.lidar_btn.config(text='OK', bg='#2ECC71')
        self.motor_btn.config(text='OK', bg='#2ECC71')
        self.battery_btn.config(text='NORMAL', bg='#2ECC71')
        self._publish_faults()

    # ── Subscribers (Physical → Digital state mirroring) ──────────────

    def resource_cb(self, msg: String) -> None:
        """
        Mirror battery and fertilizer levels from the physical robot.

        @param msg: std_msgs/String holding JSON from /robot_resources.
        @pre  (none).
        @post Under self.state_lock, self.state['battery'/'fertilizer'] are
              updated for the GUI loop to read.
        @return None.
        @throws (none) malformed JSON is caught and the message is dropped.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.state_lock:
            self.state['battery'] = data.get('battery', 100.0)
            self.state['fertilizer'] = data.get('fertilizer', 100.0)

    def zone_cb(self, msg: String) -> None:
        """Mirror the robot's current zone from /current_zone."""
        with self.state_lock:
            self.state['zone'] = msg.data

    def nav_cb(self, msg: String) -> None:
        """Mirror navigation status from /navigation_executor_status."""
        with self.state_lock:
            self.state['nav'] = msg.data

    def health_cb(self, msg: String) -> None:
        """
        Mirror the fused system health from the physical robot.

        @param msg: std_msgs/String holding JSON from /system_health.
        @pre  (none).
        @post Under self.state_lock, self.state['twin_mode'] and the per-sensor
              status fields are updated for the GUI loop to read.
        @return None.
        @throws (none) malformed JSON is caught and the message is dropped.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.state_lock:
            self.state['twin_mode'] = data.get('twin_mode', 'NORMAL')
            self.state['battery_status'] = data.get('battery', {}).get('status', 'NO_DATA')
            self.state['lidar_status'] = data.get('lidar', {}).get('status', 'NO_DATA')
            self.state['imu_status'] = data.get('imu', {}).get('status', 'NO_DATA')

    def obstacle_cb(self, msg: String) -> None:
        """
        Mirror what the robot senses in its environment.

        @param msg: std_msgs/String holding JSON from /obstacle_status.
        @pre  (none).
        @post Under self.state_lock, self.state['obstacle'] becomes a readable
              line such as 'OBSTACLE: FRONT @ 0.28m' or 'No obstacle'.
        @return None.
        @throws (none) malformed JSON is caught and the message is dropped.
        """
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('blocked'):
            dist = data.get('distance', -1.0)
            sector = data.get('sector', 'UNKNOWN')
            text = f'OBSTACLE: {sector}' + (f' @ {dist:.2f}m' if dist >= 0 else '')
        else:
            text = 'No obstacle'
        with self.state_lock:
            self.state['obstacle'] = text

    # ── Field zone telemetry (arrays indexed by self.zone_ids order) ──
    def _store_field(self, key: str, msg: Float32MultiArray) -> None:
        """Cache a field-telemetry array (moisture/nutrients/vulnerability) into
        self.field[key], indexed by zone in self.zone_ids order, under the lock."""
        with self.state_lock:
            for i, z in enumerate(self.zone_ids):
                if i < len(msg.data):
                    self.field[key][z] = float(msg.data[i])

    def _moisture_cb(self, msg): self._store_field('moisture', msg)
    def _nutrients_cb(self, msg): self._store_field('nutrients', msg)
    def _vulnerability_cb(self, msg): self._store_field('vulnerability', msg)

    def _load_zone_ids(self) -> list:
        """Return the sorted zone ids from zones.yaml (so the table matches the
        array ordering), falling back to zone_0..zone_3 if it can't be read."""
        try:
            pkg = get_package_share_directory('my_turtlebot3_controller')
            with open(os.path.join(pkg, 'config', 'zones.yaml'), encoding='utf-8') as f:
                raw = yaml.safe_load(f) or {}
            ids = sorted([z for z in raw.keys() if z != 'base_station'])
            return ids or ['zone_0', 'zone_1', 'zone_2', 'zone_3']
        except (OSError, yaml.YAMLError, PackageNotFoundError):
            return ['zone_0', 'zone_1', 'zone_2', 'zone_3']

    @staticmethod
    def _zone_label(zid: str) -> str:
        """Pretty-print a zone id, e.g. 'zone_0' -> 'Zone 0'."""
        return zid.replace('_', ' ').title()

    def _zone_category(self, m, n, v):
        """Same classification the Gazebo tiles use -> (label, colour)."""
        if m is None or n is None or v is None:
            return ('NO DATA', '#888888')
        if v > self.vulnerability_halt:
            return ('AT RISK', '#E74C3C')
        low_w = m < self.moisture_threshold
        low_n = n < self.nutrient_threshold
        if low_w and low_n:
            return ('NEEDS WATER + FERT', '#E67E22')
        if low_n:
            return ('NEEDS FERTILISER', '#F1C40F')
        if low_w:
            return ('NEEDS WATER', '#3498DB')
        return ('HEALTHY', '#2ECC71')

    # ── GUI refresh loop (main thread) ────────────────────────────────

    def update_gui_loop(self) -> None:
        """Main-thread refresh (every 100 ms): snapshot the mirrored state under
        the lock and repaint every widget, including the live zone table. Tk is
        only ever touched here, never from the ROS spin thread."""
        with self.state_lock:
            s = dict(self.state)
            moisture = dict(self.field['moisture'])
            nutrients = dict(self.field['nutrients'])
            vulnerability = dict(self.field['vulnerability'])

        self.bat_bar['value'] = s['battery']
        self.bat_lbl.config(text=f"{s['battery']:.1f}%")
        self.bat_bar.configure(style='red.Horizontal.TProgressbar' if s['battery'] < 30 else 'green.Horizontal.TProgressbar')
        self.fert_bar['value'] = s['fertilizer']
        self.fert_lbl.config(text=f"{s['fertilizer']:.1f}%")
        self.fert_bar.configure(style='red.Horizontal.TProgressbar' if s['fertilizer'] < 20 else 'green.Horizontal.TProgressbar')
        self.zone_var.set(f"Zone: {s['zone']}")
        self.nav_var.set(f"Nav: {s['nav']}")

        self.health_bat_var.set(f"Battery: {s['battery_status']}")
        self.health_lidar_var.set(f"LiDAR: {s['lidar_status']}")
        self.health_imu_var.set(f"IMU: {s['imu_status']}")
        self.health_lidar_lbl.config(fg='#E74C3C' if s['lidar_status'] in ('FAILED', 'STALE', 'ALL_INVALID') else
                                     '#F39C12' if s['lidar_status'] in ('DEGRADED', 'HIGH_DROPOUT') else '#CCC')

        self.obstacle_var.set(s['obstacle'])
        self.obstacle_lbl.config(fg='#2ECC71' if s['obstacle'] == 'No obstacle' else '#E74C3C')

        mode = s['twin_mode']
        self.mode_var.set(f'TWIN MODE: {mode}')
        self.mode_lbl.config(bg='#2ECC71' if mode == 'NORMAL' else '#F39C12' if mode == 'DEGRADED' else '#E74C3C')

        # ── Field zone status table ──
        for z in self.zone_ids:
            m, n, v = moisture.get(z), nutrients.get(z), vulnerability.get(z)
            moist_lbl, nutri_lbl, status_lbl = self.zone_widgets[z]
            moist_lbl.config(
                text=f'{m:.0f}%' if m is not None else '--',
                fg='#F39C12' if (m is not None and m < self.moisture_threshold) else '#FFF')
            nutri_lbl.config(
                text=f'{n:.0f}%' if n is not None else '--',
                fg='#F39C12' if (n is not None and n < self.nutrient_threshold) else '#FFF')
            text, color = self._zone_category(m, n, v)
            status_lbl.config(text=text, fg=color)

        self.root.after(100, self.update_gui_loop)

    def run_gui(self) -> None:
        """Hand control to the Tk event loop (blocks on the main thread)."""
        self.root.mainloop()


def ros_spin_thread(node: DashboardNode) -> None:
    rclpy.spin(node)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
