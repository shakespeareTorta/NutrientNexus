#!/usr/bin/env python3
"""
Nutrient Nexus Real-Time Dashboard
Provides a Tkinter GUI for monitoring robot resources, physical zone,
and injecting real-time weather events to demonstrate digital twin synchronization.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import tkinter as tk
from tkinter import ttk
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ── QoS profiles (Lecture Week 6) ────────────────────────────────────
STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__('nexus_dashboard_node')

        # Publishers
        self.weather_pub = self.create_publisher(String, '/weather_forecast', STATE_QOS)
        self.report_pub = self.create_publisher(String, '/generate_report', 10)

        # Subscribers (Robot A - default namespace)
        self.create_subscription(String, '/robot_resources', lambda m: self.resource_cb(m, 'A'), STATE_QOS)
        self.create_subscription(String, '/current_zone', lambda m: self.zone_cb(m, 'A'), STATE_QOS)
        self.create_subscription(String, '/navigation_executor_status', lambda m: self.nav_cb(m, 'A'), STATE_QOS)
        self.create_subscription(String, '/operator_override_request', lambda m: self.override_req_cb(m, 'A'), STATE_QOS)
        
        # Subscribers (Robot B - /tb2 namespace)
        self.create_subscription(String, '/tb2/robot_resources', lambda m: self.resource_cb(m, 'B'), STATE_QOS)
        self.create_subscription(String, '/tb2/current_zone', lambda m: self.zone_cb(m, 'B'), STATE_QOS)
        self.create_subscription(String, '/tb2/navigation_executor_status', lambda m: self.nav_cb(m, 'B'), STATE_QOS)
        self.create_subscription(String, '/tb2/operator_override_request', lambda m: self.override_req_cb(m, 'B'), STATE_QOS)

        # We need publishers to both namespaces for override responses
        self.override_pub_a = self.create_publisher(String, '/operator_override_response', STATE_QOS)
        self.override_pub_b = self.create_publisher(String, '/tb2/operator_override_response', STATE_QOS)

        # State Variables & Thread Safety Lock (Lecture Week 6 fix)
        self.state_lock = threading.Lock()
        self.robots = {
            'A': {'battery': 100.0, 'fertilizer': 100.0, 'zone': "Unknown", 'nav': "IDLE"},
            'B': {'battery': 100.0, 'fertilizer': 100.0, 'zone': "Unknown", 'nav': "IDLE"}
        }

        # Setup GUI
        self.root = tk.Tk()
        self.root.title("Nutrient Nexus - Digital Twin Dashboard")
        self.root.geometry("900x500")
        self.root.configure(bg="#1E1E1E")
        
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure("TLabel", background="#1E1E1E", foreground="#FFFFFF", font=("Helvetica", 12))
        self.style.configure("Header.TLabel", font=("Helvetica", 16, "bold"), foreground="#4A90E2")
        self.style.configure("TButton", font=("Helvetica", 11, "bold"))
        self.style.configure("red.Horizontal.TProgressbar", background="#E74C3C")
        self.style.configure("green.Horizontal.TProgressbar", background="#2ECC71")
        self.style.configure("blue.Horizontal.TProgressbar", background="#3498DB")

        self.setup_ui()
        self.update_gui_loop()

    def setup_ui(self):
        main_frame = tk.Frame(self.root, bg="#1E1E1E", padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Title
        ttk.Label(main_frame, text="Nutrient Nexus Command Center", style="Header.TLabel").pack(pady=(0, 20))

        # --- Resources Frame (Side-by-side) ---
        res_frame = tk.LabelFrame(main_frame, text="Fleet Resources", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        res_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)

        # Robot A Resources
        tk.Label(res_frame, text="Robot A (Twin) - Zones 0,1", bg="#1E1E1E", fg="#4A90E2", font=("Helvetica", 11, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=5)
        tk.Label(res_frame, text="Battery:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=1, column=0, sticky="w", pady=2)
        self.bat_bar_a = ttk.Progressbar(res_frame, length=200, mode="determinate", style="green.Horizontal.TProgressbar")
        self.bat_bar_a.grid(row=1, column=1, padx=10)
        self.bat_lbl_a = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.bat_lbl_a.grid(row=1, column=2)

        tk.Label(res_frame, text="Fertilizer:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=2, column=0, sticky="w", pady=2)
        self.fert_bar_a = ttk.Progressbar(res_frame, length=200, mode="determinate", style="green.Horizontal.TProgressbar")
        self.fert_bar_a.grid(row=2, column=1, padx=10)
        self.fert_lbl_a = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.fert_lbl_a.grid(row=2, column=2)

        # Separator column
        tk.Label(res_frame, text="   |   ", bg="#1E1E1E", fg="#444", font=("Helvetica", 14)).grid(row=0, column=3, rowspan=3)

        # Robot B Resources
        tk.Label(res_frame, text="Robot B (Sim) - Zones 2,3", bg="#1E1E1E", fg="#E67E22", font=("Helvetica", 11, "bold")).grid(row=0, column=4, columnspan=3, sticky="w", pady=5)
        tk.Label(res_frame, text="Battery:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=1, column=4, sticky="w", pady=2)
        self.bat_bar_b = ttk.Progressbar(res_frame, length=200, mode="determinate", style="green.Horizontal.TProgressbar")
        self.bat_bar_b.grid(row=1, column=5, padx=10)
        self.bat_lbl_b = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.bat_lbl_b.grid(row=1, column=6)

        tk.Label(res_frame, text="Fertilizer:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=2, column=4, sticky="w", pady=2)
        self.fert_bar_b = ttk.Progressbar(res_frame, length=200, mode="determinate", style="green.Horizontal.TProgressbar")
        self.fert_bar_b.grid(row=2, column=5, padx=10)
        self.fert_lbl_b = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.fert_lbl_b.grid(row=2, column=6)

        # --- Telemetry Frame ---
        tele_frame = tk.LabelFrame(main_frame, text="Fleet Telemetry", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        tele_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)

        # Robot A Telemetry
        self.zone_var_a = tk.StringVar(value="Robot A Zone: Unknown")
        tk.Label(tele_frame, textvariable=self.zone_var_a, bg="#1E1E1E", fg="#4A90E2", font=("Helvetica", 11, "bold")).grid(row=0, column=0, sticky="w", padx=10, pady=2)
        self.nav_var_a = tk.StringVar(value="Robot A Nav: IDLE")
        tk.Label(tele_frame, textvariable=self.nav_var_a, bg="#1E1E1E", fg="#CCC", font=("Helvetica", 10)).grid(row=1, column=0, sticky="w", padx=10, pady=2)

        # Separator
        tk.Label(tele_frame, text="      |      ", bg="#1E1E1E", fg="#444", font=("Helvetica", 14)).grid(row=0, column=1, rowspan=2)

        # Robot B Telemetry
        self.zone_var_b = tk.StringVar(value="Robot B Zone: Unknown")
        tk.Label(tele_frame, textvariable=self.zone_var_b, bg="#1E1E1E", fg="#E67E22", font=("Helvetica", 11, "bold")).grid(row=0, column=2, sticky="w", padx=10, pady=2)
        self.nav_var_b = tk.StringVar(value="Robot B Nav: IDLE")
        tk.Label(tele_frame, textvariable=self.nav_var_b, bg="#1E1E1E", fg="#CCC", font=("Helvetica", 10)).grid(row=1, column=2, sticky="w", padx=10, pady=2)

        # --- Weather Injection Frame ---
        weather_frame = tk.LabelFrame(main_frame, text="Real-Time Weather Injection (SDG-14)", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        weather_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)
        
        btn_frame = tk.Frame(weather_frame, bg="#1E1E1E")
        btn_frame.pack()

        tk.Button(btn_frame, text="☀️ Sunny", bg="#F1C40F", fg="#000", font=("Helvetica", 10, "bold"), command=lambda: self.set_weather("sunny")).grid(row=0, column=0, padx=10)
        tk.Button(btn_frame, text="🌧️ Rainy", bg="#3498DB", fg="#FFF", font=("Helvetica", 10, "bold"), command=lambda: self.set_weather("rainy")).grid(row=0, column=1, padx=10)
        tk.Button(btn_frame, text="☁️ Overcast", bg="#95A5A6", fg="#FFF", font=("Helvetica", 10, "bold"), command=lambda: self.set_weather("overcast")).grid(row=0, column=2, padx=10)

        # --- Audit Frame ---
        audit_frame = tk.LabelFrame(main_frame, text="Sustainability Audit", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        audit_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)
        
        tk.Button(audit_frame, text="📄 Generate Sustainability Report", bg="#2ECC71", fg="#FFF", font=("Helvetica", 11, "bold"), command=self.generate_report).pack(pady=5)

    def set_weather(self, weather: str):
        msg = String()
        msg.data = weather
        self.weather_pub.publish(msg)
        self.get_logger().info(f"Injected weather: {weather}")

    def generate_report(self):
        msg = String()
        msg.data = "generate"
        self.report_pub.publish(msg)
        self.get_logger().info("Requested Sustainability Audit Report")

    def override_req_cb(self, msg: String, robot: str):
        # We need to run UI updates in the main thread.
        self.root.after(0, self.show_override_popup, msg.data, robot)

    def show_override_popup(self, data_str: str, robot: str):
        try:
            data = json.loads(data_str)
            zone = data.get("zone", "Unknown")
            reason = data.get("reason", "Unknown Risk")

            popup = tk.Toplevel(self.root)
            popup.title(f"⚠️ Robot {robot}: SDG-14 Safety Override Required")
            popup.geometry("500x300")
            popup.configure(bg="#E74C3C")

            tk.Label(popup, text=f"ROBOT {robot}: CRITICAL RUNOFF RISK", bg="#E74C3C", fg="#FFF", font=("Helvetica", 16, "bold")).pack(pady=10)
            tk.Label(popup, text=f"Zone: {zone}", bg="#E74C3C", fg="#FFF", font=("Helvetica", 14)).pack(pady=5)
            
            msg_text = tk.Text(popup, height=4, width=50, bg="#E74C3C", fg="#FFF", font=("Helvetica", 11), wrap=tk.WORD, bd=0)
            msg_text.insert(tk.END, reason)
            msg_text.config(state=tk.DISABLED)
            msg_text.pack(pady=10)

            btn_frame = tk.Frame(popup, bg="#E74C3C")
            btn_frame.pack(pady=20)

            def comply():
                pub = self.override_pub_a if robot == 'A' else self.override_pub_b
                pub.publish(String(data="comply"))
                popup.destroy()

            def override():
                pub = self.override_pub_a if robot == 'A' else self.override_pub_b
                pub.publish(String(data="override"))
                popup.destroy()

            tk.Button(btn_frame, text="🛡️ COMPLY (Abort Spray)", bg="#2ECC71", fg="#FFF", font=("Helvetica", 12, "bold"), command=comply).pack(side=tk.LEFT, padx=10)
            tk.Button(btn_frame, text="⚠️ OVERRIDE (Force Spray)", bg="#1E1E1E", fg="#E74C3C", font=("Helvetica", 12, "bold"), command=override).pack(side=tk.RIGHT, padx=10)

            popup.grab_set()
        except Exception as e:
            self.get_logger().error(f"Error showing popup: {e}")

    def resource_cb(self, msg: String, robot: str):
        try:
            data = json.loads(msg.data)
            with self.state_lock:
                self.robots[robot]['battery'] = data.get("battery", 100.0)
                self.robots[robot]['fertilizer'] = data.get("fertilizer", 100.0)
        except Exception:
            pass

    def zone_cb(self, msg: String, robot: str):
        with self.state_lock:
            self.robots[robot]['zone'] = msg.data

    def nav_cb(self, msg: String, robot: str):
        with self.state_lock:
            self.robots[robot]['nav'] = msg.data

    def update_gui_loop(self):
        with self.state_lock:
            bat_a = self.robots['A']['battery']
            fert_a = self.robots['A']['fertilizer']
            zone_a = self.robots['A']['zone']
            nav_a = self.robots['A']['nav']
            
            bat_b = self.robots['B']['battery']
            fert_b = self.robots['B']['fertilizer']
            zone_b = self.robots['B']['zone']
            nav_b = self.robots['B']['nav']

        # Update Robot A
        self.bat_bar_a['value'] = bat_a
        self.bat_lbl_a.config(text=f"{bat_a:.1f}%")
        self.bat_bar_a.configure(style="red.Horizontal.TProgressbar" if bat_a < 30 else "green.Horizontal.TProgressbar")
        self.fert_bar_a['value'] = fert_a
        self.fert_lbl_a.config(text=f"{fert_a:.1f}%")
        self.fert_bar_a.configure(style="red.Horizontal.TProgressbar" if fert_a < 20 else "green.Horizontal.TProgressbar")
        self.zone_var_a.set(f"Robot A Zone: {zone_a}")
        self.nav_var_a.set(f"Robot A Nav: {nav_a}")

        # Update Robot B
        self.bat_bar_b['value'] = bat_b
        self.bat_lbl_b.config(text=f"{bat_b:.1f}%")
        self.bat_bar_b.configure(style="red.Horizontal.TProgressbar" if bat_b < 30 else "green.Horizontal.TProgressbar")
        self.fert_bar_b['value'] = fert_b
        self.fert_lbl_b.config(text=f"{fert_b:.1f}%")
        self.fert_bar_b.configure(style="red.Horizontal.TProgressbar" if fert_b < 20 else "green.Horizontal.TProgressbar")
        self.zone_var_b.set(f"Robot B Zone: {zone_b}")
        self.nav_var_b.set(f"Robot B Nav: {nav_b}")

        # Schedule next update
        self.root.after(100, self.update_gui_loop)

    def run_gui(self):
        self.root.mainloop()

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    
    # Run ROS 2 spin in background thread
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()
    
    # Run Tkinter main loop in main thread
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
