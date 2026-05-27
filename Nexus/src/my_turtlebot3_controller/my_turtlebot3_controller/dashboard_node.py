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

class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__('nexus_dashboard_node')

        # Publishers
        self.weather_pub = self.create_publisher(String, '/weather_forecast', 10)
        self.report_pub = self.create_publisher(String, '/generate_report', 10)

        # Subscribers
        self.create_subscription(String, '/robot_resources', self.resource_cb, 10)
        self.create_subscription(String, '/current_zone', self.zone_cb, 10)
        self.create_subscription(String, '/navigation_executor_status', self.nav_cb, 10)
        self.create_subscription(String, '/operator_override_request', self.override_req_cb, 10)
        
        self.override_pub = self.create_publisher(String, '/operator_override_response', 10)

        # State Variables
        self.battery = 100.0
        self.fertilizer = 100.0
        self.current_zone = "Unknown"
        self.nav_status = "IDLE"

        # Setup GUI
        self.root = tk.Tk()
        self.root.title("Nutrient Nexus - Digital Twin Dashboard")
        self.root.geometry("600x450")
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

        # --- Resources Frame ---
        res_frame = tk.LabelFrame(main_frame, text="Real-Time Resources", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        res_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)

        # Battery
        tk.Label(res_frame, text="Battery:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=0, column=0, sticky="w", pady=5)
        self.bat_bar = ttk.Progressbar(res_frame, length=300, mode="determinate", style="green.Horizontal.TProgressbar")
        self.bat_bar.grid(row=0, column=1, padx=10)
        self.bat_lbl = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.bat_lbl.grid(row=0, column=2)

        # Fertilizer
        tk.Label(res_frame, text="Fertilizer:", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10)).grid(row=1, column=0, sticky="w", pady=5)
        self.fert_bar = ttk.Progressbar(res_frame, length=300, mode="determinate", style="green.Horizontal.TProgressbar")
        self.fert_bar.grid(row=1, column=1, padx=10)
        self.fert_lbl = tk.Label(res_frame, text="100%", bg="#1E1E1E", fg="#FFF", font=("Helvetica", 10))
        self.fert_lbl.grid(row=1, column=2)

        # --- Telemetry Frame ---
        tele_frame = tk.LabelFrame(main_frame, text="Digital Twin Telemetry", bg="#1E1E1E", fg="#FFFFFF", font=("Helvetica", 12, "bold"))
        tele_frame.pack(fill=tk.X, pady=10, ipadx=10, ipady=10)

        self.zone_var = tk.StringVar(value="Physical Zone: Unknown")
        tk.Label(tele_frame, textvariable=self.zone_var, bg="#1E1E1E", fg="#F1C40F", font=("Helvetica", 11, "bold")).pack(anchor="w", pady=2)
        
        self.nav_var = tk.StringVar(value="Nav Status: IDLE")
        tk.Label(tele_frame, textvariable=self.nav_var, bg="#1E1E1E", fg="#E67E22", font=("Helvetica", 11, "bold")).pack(anchor="w", pady=2)

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

    def override_req_cb(self, msg: String):
        # We need to run UI updates in the main thread.
        self.root.after(0, self.show_override_popup, msg.data)

    def show_override_popup(self, data_str: str):
        try:
            data = json.loads(data_str)
            zone = data.get("zone", "Unknown")
            reason = data.get("reason", "Unknown Risk")

            popup = tk.Toplevel(self.root)
            popup.title("⚠️ SDG-14 Safety Override Required")
            popup.geometry("500x300")
            popup.configure(bg="#E74C3C")

            tk.Label(popup, text="CRITICAL SDG-14 RUNOFF RISK", bg="#E74C3C", fg="#FFF", font=("Helvetica", 16, "bold")).pack(pady=10)
            tk.Label(popup, text=f"Zone: {zone}", bg="#E74C3C", fg="#FFF", font=("Helvetica", 14)).pack(pady=5)
            
            msg_text = tk.Text(popup, height=4, width=50, bg="#E74C3C", fg="#FFF", font=("Helvetica", 11), wrap=tk.WORD, bd=0)
            msg_text.insert(tk.END, reason)
            msg_text.config(state=tk.DISABLED)
            msg_text.pack(pady=10)

            btn_frame = tk.Frame(popup, bg="#E74C3C")
            btn_frame.pack(pady=20)

            def comply():
                self.override_pub.publish(String(data="comply"))
                popup.destroy()

            def override():
                self.override_pub.publish(String(data="override"))
                popup.destroy()

            tk.Button(btn_frame, text="🛡️ COMPLY (Abort Spray)", bg="#2ECC71", fg="#FFF", font=("Helvetica", 12, "bold"), command=comply).pack(side=tk.LEFT, padx=10)
            tk.Button(btn_frame, text="⚠️ OVERRIDE (Force Spray)", bg="#1E1E1E", fg="#E74C3C", font=("Helvetica", 12, "bold"), command=override).pack(side=tk.RIGHT, padx=10)

            popup.grab_set()
        except Exception as e:
            self.get_logger().error(f"Error showing popup: {e}")

    def resource_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.battery = data.get("battery", 100.0)
            self.fertilizer = data.get("fertilizer", 100.0)
        except Exception:
            pass

    def zone_cb(self, msg: String):
        self.current_zone = msg.data

    def nav_cb(self, msg: String):
        self.nav_status = msg.data

    def update_gui_loop(self):
        # Update progress bars
        self.bat_bar['value'] = self.battery
        self.bat_lbl.config(text=f"{self.battery:.1f}%")
        self.bat_bar.configure(style="red.Horizontal.TProgressbar" if self.battery < 30 else "green.Horizontal.TProgressbar")

        self.fert_bar['value'] = self.fertilizer
        self.fert_lbl.config(text=f"{self.fertilizer:.1f}%")
        self.fert_bar.configure(style="red.Horizontal.TProgressbar" if self.fertilizer < 20 else "green.Horizontal.TProgressbar")

        # Update text
        self.zone_var.set(f"Physical Zone: {self.current_zone}")
        self.nav_var.set(f"Nav Status: {self.nav_status}")

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
