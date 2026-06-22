#!/usr/bin/env python3
"""
System Monitor Node — Hardware health watchdog for Nutrient Nexus.

Monitors TurtleBot3 sensor health and publishes diagnostics on
/system_health as JSON. Works in both real-robot and Gazebo-sim
contexts (gracefully handles missing topics in simulation).

Monitored topics:
    /battery_state  sensor_msgs/BatteryState  (real robot only)
        WARNING   < warn_voltage (11.5 V) or < warn_percent (35%)
        CRITICAL  < crit_voltage (10.8 V) or < crit_percent (20%)

    /scan           sensor_msgs/LaserScan
        STALE     no message for > scan_stale_sec (2.0 s)
        DROPOUT   > dropout_pct (40%) of rays are inf/out-of-range
        ALL_INF   every ray is invalid

    /imu            sensor_msgs/Imu  (real robot only)
        STALE     no message for > imu_stale_sec (2.0 s)
        HIGH_ACCEL  |acceleration| > accel_warn_g (2.5 g)
        HIGH_GYRO   |angular_vel| > gyro_warn_rps (5.0 rad/s)

All thresholds are configurable via nexus_params.yaml.
"""

import json
import math

from my_turtlebot3_controller.qos import STATE_QOS
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String


class SystemMonitorNode(Node):
    """
    Sole publisher of /system_health.

    Subscribes to the raw sensor topics (battery/scan/imu), fuses them with any
    dashboard-injected faults, and publishes one authoritative health view plus
    a derived twin_mode.
    """

    def __init__(self):
        """
        Declare parameters, subscribe to sensors and start the check timer.

        Subscribes to the sensors and the injected /twin_fault_state, and starts
        the periodic health check.
        """
        super().__init__('system_monitor_node')

        # ── Parameters ────────────────────────────────────────────────
        # Topics
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu')

        # Check rate
        self.declare_parameter('check_hz', 2.0)

        # Battery thresholds
        self.declare_parameter('warn_voltage', 11.5)
        self.declare_parameter('crit_voltage', 10.8)
        self.declare_parameter('warn_percent', 35.0)
        self.declare_parameter('crit_percent', 20.0)

        # LiDAR thresholds
        self.declare_parameter('scan_stale_sec', 2.0)
        self.declare_parameter('dropout_pct', 0.40)

        # IMU thresholds
        self.declare_parameter('imu_stale_sec', 2.0)
        self.declare_parameter('accel_warn_g', 2.5)
        self.declare_parameter('gyro_warn_rps', 5.0)

        # Read parameters
        battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        check_hz = self.get_parameter('check_hz').get_parameter_value().double_value

        self.warn_voltage = self.get_parameter('warn_voltage').get_parameter_value().double_value
        self.crit_voltage = self.get_parameter('crit_voltage').get_parameter_value().double_value
        self.warn_pct = self.get_parameter('warn_percent').get_parameter_value().double_value
        self.crit_pct = self.get_parameter('crit_percent').get_parameter_value().double_value
        self.scan_stale = self.get_parameter('scan_stale_sec').get_parameter_value().double_value
        self.dropout_pct = self.get_parameter('dropout_pct').get_parameter_value().double_value
        self.imu_stale = self.get_parameter('imu_stale_sec').get_parameter_value().double_value
        self.accel_limit = (
            self.get_parameter('accel_warn_g').get_parameter_value().double_value * 9.81)
        self.gyro_limit = self.get_parameter('gyro_warn_rps').get_parameter_value().double_value

        # ── State: last received messages and timestamps ──────────────
        self._battery_msg = None
        self._scan_msg = None
        self._imu_msg = None

        self._battery_stamp = None
        self._scan_stamp = None
        self._imu_stamp = None

        self._last_battery_level: str = ''

        # Injected digital-twin faults. SystemMonitor stays the SOLE publisher
        # of /system_health: it fuses these injected overrides with real
        # telemetry into one authoritative health view.
        self._faults = {'lidar': 'ok', 'motor': 'ok', 'battery': 'normal'}

        # ── QoS — BEST_EFFORT for sensor topics ──────────────────────
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            BatteryState, battery_topic,
            self._battery_callback, sensor_qos
        )
        self.create_subscription(
            LaserScan, scan_topic,
            self._scan_callback, sensor_qos
        )
        self.create_subscription(
            Imu, imu_topic,
            self._imu_callback, sensor_qos
        )

        # ── Publisher for system health status ────────────────────────
        self.health_pub = self.create_publisher(String, '/system_health', STATE_QOS)

        # ── Injected fault subscription (digital entity → physical) ────
        self.create_subscription(
            String, '/twin_fault_state', self._fault_callback, STATE_QOS)

        # ── Check timer ───────────────────────────────────────────────
        self.create_timer(1.0 / check_hz, self._check)

        self.get_logger().info(
            f'System Monitor Node started\n'
            f'  battery : {battery_topic}  '
            f'warn={self.warn_voltage}V/{self.warn_pct:.0f}%  '
            f'crit={self.crit_voltage}V/{self.crit_pct:.0f}%\n'
            f'  scan    : {scan_topic}  '
            f'stale>{self.scan_stale}s  dropout>{self.dropout_pct*100:.0f}%\n'
            f'  imu     : {imu_topic}  '
            f'stale>{self.imu_stale}s  '
            f'accel>{self.accel_limit/9.81:.1f}g  '
            f'gyro>{self.gyro_limit:.1f}rad/s\n'
            f'  check rate: {check_hz} Hz')

    # ── Callbacks: store message and wall-clock timestamp ─────────────

    def _battery_callback(self, msg: BatteryState) -> None:
        """Store the latest battery message and its arrival time."""
        self._battery_msg = msg
        self._battery_stamp = self.get_clock().now()

    def _scan_callback(self, msg: LaserScan) -> None:
        """Store the latest LiDAR scan and its arrival time."""
        self._scan_msg = msg
        self._scan_stamp = self.get_clock().now()

    def _imu_callback(self, msg: Imu) -> None:
        """Store the latest IMU message and its arrival time."""
        self._imu_msg = msg
        self._imu_stamp = self.get_clock().now()

    def _fault_callback(self, msg: String) -> None:
        """Record the dashboard-injected fault overrides (lidar/motor/battery)."""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._faults['lidar'] = data.get('lidar', 'ok')
        self._faults['motor'] = data.get('motor', 'ok')
        self._faults['battery'] = data.get('battery', 'normal')

    # ── Master check (called by timer) ────────────────────────────────

    def _check(self) -> None:
        """
        Assemble and publish the fused system-health view.

        Runs the per-sensor checks, overlays any dashboard-injected faults so an
        operator override wins over the raw verdict, derives the single
        twin_mode, and publishes the whole thing as JSON.

        @pre  Called by the check timer; latest sensor messages may be absent
              (handled as NO_DATA in simulation).
        @post One std_msgs/String is published on /system_health.
        @return None.
        """
        health = {}
        health['battery'] = self._check_battery()
        health['lidar'] = self._check_scan()
        health['imu'] = self._check_imu()

        # Fuse injected faults into the authoritative health view. An
        # injected fault overrides the raw sensor verdict so the digital
        # twin presents the operator-declared state.
        health['faults'] = dict(self._faults)
        if self._faults['lidar'] == 'failed':
            health['lidar']['status'] = 'FAILED'
        elif self._faults['lidar'] == 'degraded' and health['lidar'].get('status') == 'OK':
            health['lidar']['status'] = 'DEGRADED'
        if self._faults['motor'] == 'stalled':
            health['motor'] = {'status': 'STALLED'}
        else:
            health['motor'] = {'status': 'OK'}

        health['twin_mode'] = self._derive_twin_mode(health)

        # Publish combined health status
        msg = String()
        msg.data = json.dumps(health)
        self.health_pub.publish(msg)

    def _derive_twin_mode(self, health: dict) -> str:
        """
        Reduce the fused health to one of NORMAL / DEGRADED / FAULTED.

        This is the single mode the dashboard banner reflects.
        """
        if (self._faults['motor'] == 'stalled'
                or self._faults['lidar'] == 'failed'
                or self._faults['battery'] == 'clamped'
                or health['battery'].get('status') == 'CRITICAL'):
            return 'FAULTED'
        if (self._faults['lidar'] == 'degraded'
                or health['lidar'].get('status') in ('STALE', 'HIGH_DROPOUT', 'ALL_INVALID')
                or health['battery'].get('status') == 'WARNING'):
            return 'DEGRADED'
        return 'NORMAL'

    # ── Battery check ─────────────────────────────────────────────────

    def _check_battery(self) -> dict:
        """
        Classify the battery as OK/WARNING/CRITICAL (or NO_DATA in sim).

        Classifies by voltage/percent, logging only on level changes. Returns a
        status dict.
        """
        if self._battery_msg is None:
            if self._battery_stamp is None:
                self.get_logger().info(
                    '[BATTERY] No /battery_state received — '
                    'normal in Gazebo simulation',
                    throttle_duration_sec=30.0)
            return {'status': 'NO_DATA', 'source': 'simulation'}

        msg = self._battery_msg
        voltage = msg.voltage
        pct = msg.percentage * 100.0  # ROS2 spec: 0.0–1.0 → 0–100%

        if voltage <= self.crit_voltage or pct <= self.crit_pct:
            level = 'CRITICAL'
        elif voltage <= self.warn_voltage or pct <= self.warn_pct:
            level = 'WARNING'
        else:
            level = 'OK'

        # Only log when the level changes to avoid flooding
        if level != self._last_battery_level:
            self._last_battery_level = level
            if level == 'CRITICAL':
                self.get_logger().error(
                    f'[BATTERY] CRITICAL  {voltage:.2f}V  {pct:.1f}%  '
                    f'— stop robot immediately')
            elif level == 'WARNING':
                self.get_logger().warn(
                    f'[BATTERY] LOW  {voltage:.2f}V  {pct:.1f}%  '
                    f'— return to charge soon')
            else:
                self.get_logger().info(
                    f'[BATTERY] OK  {voltage:.2f}V  {pct:.1f}%')

        # Health anomalies
        if not msg.present:
            self.get_logger().error(
                '[BATTERY] Battery NOT PRESENT — check connection')

        return {
            'status': level,
            'voltage': round(voltage, 2),
            'percent': round(pct, 1),
            'present': msg.present,
        }

    # ── LiDAR check ───────────────────────────────────────────────────

    def _check_scan(self) -> dict:
        """
        Classify LiDAR health from scan age and ray quality.

        @pre  self._scan_msg / self._scan_stamp hold the latest scan, if any.
        @post No state is mutated (read-only verdict).
        @return A status dict whose 'status' is one of STALE, EMPTY,
                ALL_INVALID, HIGH_DROPOUT or OK, with supporting ray counts.
        """
        now = self.get_clock().now()

        # Staleness
        if self._scan_msg is None or self._scan_stamp is None:
            self.get_logger().error(
                f'[LIDAR] STALE — no scan received yet  '
                f'(threshold {self.scan_stale}s)  '
                f'Check USB cable and LiDAR power',
                throttle_duration_sec=5.0)
            return {'status': 'STALE', 'age_sec': float('inf')}

        age = (now - self._scan_stamp).nanoseconds / 1e9
        if age > self.scan_stale:
            self.get_logger().error(
                f'[LIDAR] STALE — no scan for {age:.1f}s  '
                f'(threshold {self.scan_stale}s)  '
                f'Check USB cable and LiDAR power',
                throttle_duration_sec=5.0)
            return {'status': 'STALE', 'age_sec': round(age, 1)}

        msg = self._scan_msg
        total = len(msg.ranges)

        if total == 0:
            self.get_logger().error('[LIDAR] Empty scan — 0 rays')
            return {'status': 'EMPTY', 'total_rays': 0}

        # Ray quality
        inf_count = sum(1 for r in msg.ranges if not math.isfinite(r))
        oob_count = sum(
            1 for r in msg.ranges
            if math.isfinite(r) and not (msg.range_min <= r <= msg.range_max)
        )
        bad_count = inf_count + oob_count
        bad_ratio = bad_count / total

        if bad_ratio >= 1.0:
            self.get_logger().error(
                f'[LIDAR] ALL RAYS INVALID ({total}/{total})  '
                f'Sensor may be disconnected or obstructed',
                throttle_duration_sec=5.0)
            status = 'ALL_INVALID'
        elif bad_ratio >= self.dropout_pct:
            self.get_logger().warn(
                f'[LIDAR] HIGH DROPOUT  {bad_count}/{total} rays invalid '
                f'({bad_ratio*100:.1f}%)',
                throttle_duration_sec=5.0)
            status = 'HIGH_DROPOUT'
        else:
            status = 'OK'

        # Below-range rays (sensor noise or hardware fault)
        below_min = sum(
            1 for r in msg.ranges
            if math.isfinite(r) and r < msg.range_min
        )
        if below_min > 0:
            self.get_logger().warn(
                f'[LIDAR] {below_min} rays below range_min ({msg.range_min}m)',
                throttle_duration_sec=10.0)

        return {
            'status': status,
            'total_rays': total,
            'valid_rays': total - bad_count,
            'dropout_pct': round(bad_ratio * 100, 1),
        }

    # ── IMU check ─────────────────────────────────────────────────────

    def _check_imu(self) -> dict:
        """
        Classify the IMU health (or NO_DATA in sim).

        Warns on excessive linear acceleration or angular velocity. Returns a
        status dict.
        """
        now = self.get_clock().now()

        # No data received yet
        if self._imu_msg is None or self._imu_stamp is None:
            self.get_logger().info(
                '[IMU] No /imu received — '
                'normal in Gazebo simulation',
                throttle_duration_sec=30.0)
            return {'status': 'NO_DATA', 'source': 'simulation'}

        # Staleness check
        age = (now - self._imu_stamp).nanoseconds / 1e9
        if age > self.imu_stale:
            self.get_logger().error(
                f'[IMU] STALE — no message for {age:.1f}s  '
                f'Check OpenCR board connection',
                throttle_duration_sec=5.0)
            return {'status': 'NO_DATA', 'source': 'simulation'}

        msg = self._imu_msg
        a = msg.linear_acceleration
        g = msg.angular_velocity

        # Linear acceleration magnitude
        accel_mag = math.sqrt(a.x**2 + a.y**2 + a.z**2)
        if accel_mag > self.accel_limit:
            self.get_logger().warn(
                f'[IMU] HIGH ACCELERATION  '
                f'{accel_mag:.2f} m/s² ({accel_mag/9.81:.2f}g)  '
                f'— possible collision or fall')

        # Angular velocity magnitude
        gyro_mag = math.sqrt(g.x**2 + g.y**2 + g.z**2)
        if gyro_mag > self.gyro_limit:
            self.get_logger().warn(
                f'[IMU] HIGH ROTATION  '
                f'{gyro_mag:.2f} rad/s  '
                f'— robot spinning very fast')

        return {
            'status': 'OK',
            'accel_ms2': round(accel_mag, 2),
            'gyro_rps': round(gyro_mag, 2),
        }


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('System Monitor shutting down.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
