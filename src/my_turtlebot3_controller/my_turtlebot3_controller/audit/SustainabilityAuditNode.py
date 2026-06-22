#!/usr/bin/env python3
"""
Sustainability Audit Node

Acts as an independent ledger, monitoring farm operations and SDG-14 interventions.
Generates a comprehensive Markdown report when requested.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import json
import os
from datetime import datetime

class SustainabilityAuditNode(Node):
    """Independent ledger of farm operations. Logs every fertilise/irrigate/
    SDG-14 intervention and, on request, writes a Markdown sustainability
    report estimating prevented runoff."""

    def __init__(self) -> None:
        """Initialise the ledgers and subscribe to the operation and report
        topics."""
        super().__init__('sustainability_audit_node')

        # Ledgers
        self.interventions = []
        self.fertilizations = []
        self.irrigations = []

        # Current State
        self.current_weather = 'sunny'
        self.zone_states = {} # Track latest moisture/nutrients per zone

        # Subscribers
        self.create_subscription(String, '/weather_forecast', self.weather_cb, 10)
        self.create_subscription(String, '/fertilise_zone', self.fert_cb, 10)
        self.create_subscription(String, '/irrigate_zone', self.irrig_cb, 10)
        self.create_subscription(String, '/sdg14_intervention', self.intervention_cb, 10)
        self.create_subscription(String, '/generate_report', self.generate_report_cb, 10)

        self.get_logger().info('Sustainability Audit Node started. Ledger initialized.')

    def weather_cb(self, msg: String):
        """Track current weather so each logged action records its conditions."""
        self.current_weather = msg.data

    def fert_cb(self, msg: String):
        """Log a fertilisation event (time, zone, robot, weather) to the ledger."""
        try:
            data = json.loads(msg.data)
            zone_id = data.get('zone', msg.data)
            robot_id = data.get('robot', 'Unknown')
        except (json.JSONDecodeError, AttributeError):
            zone_id = msg.data
            robot_id = 'Unknown'

        self.fertilizations.append({
            'time': datetime.now().strftime('%H:%M:%S'),
            'zone': zone_id,
            'robot': robot_id,
            'weather': self.current_weather
        })
        self.get_logger().info(f'[AUDIT] Logged fertilisation by Robot {robot_id} in {zone_id}.')

    def irrig_cb(self, msg: String):
        """Log an irrigation event (time, zone, robot, weather) to the ledger."""
        try:
            data = json.loads(msg.data)
            zone_id = data.get('zone', msg.data)
            robot_id = data.get('robot', 'Unknown')
        except (json.JSONDecodeError, AttributeError):
            zone_id = msg.data
            robot_id = 'Unknown'

        self.irrigations.append({
            'time': datetime.now().strftime('%H:%M:%S'),
            'zone': zone_id,
            'robot': robot_id,
            'weather': self.current_weather
        })
        self.get_logger().info(f'[AUDIT] Logged irrigation by Robot {robot_id} in {zone_id}.')

    def intervention_cb(self, msg: String):
        """Receives a JSON string detailing an aborted action due to SDG-14 rules."""
        try:
            data = json.loads(msg.data)
            data['time'] = datetime.now().strftime('%H:%M:%S')
            self.interventions.append(data)
            self.get_logger().info(f"[AUDIT] Logged SDG-14 Intervention in {data.get('zone', 'Unknown')}.")
        except (json.JSONDecodeError, TypeError, AttributeError) as e:
            self.get_logger().error(f'Failed to parse intervention: {e}')

    def generate_report_cb(self, msg: String):
        """Compile the ledgers into a Markdown report (SDG-14 impact, operations,
        recommendations) and write it to nexus_farm_report.md."""
        self.get_logger().info('Compiling Sustainability Report...')
        
        # Estimate savings: Assume each aborted fertilizer spray saves 1.5kg of nitrogen
        saved_nitrogen = len(self.interventions) * 1.5
        
        report_path = os.path.join(os.getcwd(), 'nexus_farm_report.md')
        
        with open(report_path, 'w') as f:
            f.write('# 🌍 Nutrient Nexus Sustainability & Audit Report\n\n')
            f.write(f"**Generated:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write('## 🏆 SDG-14 Environmental Impact\n')
            f.write(f'**Total Prevented Runoff Events:** {len(self.interventions)}\n')
            f.write(f'**Estimated Nitrogen Saved from Waterways:** {saved_nitrogen:.1f} kg\n\n')
            
            f.write('### Intervention Ledger\n')
            if not self.interventions:
                f.write('*No unsafe conditions encountered yet.*\n')
            else:
                for idx, item in enumerate(self.interventions, 1):
                    f.write(f"{idx}. **{item['time']} | {item['zone']}**\n")
                    f.write(f"   - **Reason:** {item['reason']}\n")
                    f.write(f"   - **Vulnerability Score:** {item.get('vulnerability_score', 'N/A')}/100\n\n")

            f.write('## 🚜 Agricultural Operations\n')
            f.write(f'**Total Fertilizations Applied:** {len(self.fertilizations)}\n')
            f.write(f'**Total Irrigations Applied:** {len(self.irrigations)}\n\n')
            
            f.write('## 💡 AI Farm Recommendations\n')
            # Generate smart recommendations based on interventions
            high_risk_zones = {}
            for item in self.interventions:
                z = item['zone']
                high_risk_zones[z] = high_risk_zones.get(z, 0) + 1
                
            if high_risk_zones:
                f.write('Based on recent intervention data, the system recommends the following physical farm improvements:\n\n')
                for z, count in high_risk_zones.items():
                    if count >= 2:
                        f.write(f'- ⚠️ **{z}** has triggered {count} runoff safety aborts. **Recommendation:** Investigate soil drainage capabilities or consider relocating crops further from the water table to reduce continuous risk.\n')
                    else:
                        f.write(f'- ℹ️ **{z}** triggered a weather-based runoff abort. Monitor weather closely before scheduling manual operations here.\n')
            else:
                f.write('*All zones are currently operating within safe parameters. No structural changes recommended at this time.*\n')
                
        self.get_logger().info(f'Report successfully saved to {report_path}')

def main(args=None):
    rclpy.init(args=args)
    node = SustainabilityAuditNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
