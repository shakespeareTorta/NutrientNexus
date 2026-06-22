#!/usr/bin/env python3
"""
Weather Adapter Node — Integrates external weather APIs into the digital twin.

Fetches real-world weather data from Open-Meteo API for a specific coordinate
(e.g., local farm or testing facility) and categorizes it into system weather
states ('sunny', 'overcast', 'rainy', 'storm') published to /weather_forecast.
"""

import json
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_turtlebot3_controller.qos import STATE_QOS


class WeatherAdapterNode(Node):
    """Bridges a real weather API into the twin: periodically fetches live
    conditions and republishes them as a system weather state."""

    def __init__(self):
        """Read location/interval params, create the /weather_forecast publisher,
        and schedule (and immediately run) the first fetch."""
        super().__init__('weather_adapter_node')

        # Parameters for location and update frequency
        self.declare_parameter('latitude', 52.0)
        self.declare_parameter('longitude', 5.0)
        self.declare_parameter('update_interval_sec', 300.0) # Default 5 minutes

        self.lat = self.get_parameter('latitude').get_parameter_value().double_value
        self.lon = self.get_parameter('longitude').get_parameter_value().double_value
        self.interval = self.get_parameter('update_interval_sec').get_parameter_value().double_value

        self.weather_pub = self.create_publisher(String, '/weather_forecast', STATE_QOS)

        # Timer to fetch weather periodically
        self.create_timer(self.interval, self.fetch_weather)
        
        # Initial fetch
        self.fetch_weather()
        
        self.get_logger().info(
            f'WeatherAdapterNode initialized. Tracking location ({self.lat}, {self.lon}) '
            f'updating every {self.interval}s.'
        )

    def fetch_weather(self):
        """Query Open-Meteo, map the WMO code to sunny/overcast/rainy/storm, and
        publish it on /weather_forecast (errors are logged, not raised)."""
        url = f"https://api.open-meteo.com/v1/forecast?latitude={self.lat}&longitude={self.lon}&current_weather=true"
        
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'NutrientNexus-DigitalTwin/1.0'})
            with urllib.request.urlopen(req, timeout=10.0) as response:
                data = json.loads(response.read().decode())
                
                if 'current_weather' in data and 'weathercode' in data['current_weather']:
                    code = data['current_weather']['weathercode']
                    
                    # WMO Weather interpretation codes
                    # https://open-meteo.com/en/docs
                    if code >= 95:
                        weather = 'storm'
                    elif code >= 51 and code <= 82: # drizzle, rain, showers
                        weather = 'rainy'
                    elif code >= 1 and code <= 3: # mainly clear, partly cloudy, overcast
                        weather = 'overcast' if code == 3 else 'sunny'
                    elif code >= 45 and code <= 48: # fog
                        weather = 'overcast'
                    else:
                        weather = 'sunny' # default fallback
                        
                    msg = String()
                    msg.data = weather
                    self.weather_pub.publish(msg)
                    self.get_logger().info(f"Published live weather update: {weather} (WMO code: {code})")
                else:
                    self.get_logger().warning("Unexpected JSON format from weather API.")

        except urllib.error.URLError as e:
            self.get_logger().error(f"Failed to fetch weather data: {e.reason}")
        except Exception as e:
            self.get_logger().error(f"Error processing weather update: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WeatherAdapterNode()
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
