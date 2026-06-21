#!/usr/bin/env python3
"""
Zone Visualizer Node — paints the field zones into Gazebo, colour-coded by state.

Each zone (from zones.yaml) is drawn as a flat, semi-transparent coloured tile on
the floor, ANCHORED at the zone's fixed field coordinates (its navigation
target). The colour reflects the live field state (moisture / nutrients /
vulnerability) using the same thresholds CropDecisionNode acts on, so you can
watch zones change colour as the robot treats them.

This relies on accurate localization: ground_truth_localization makes the `map`
frame coincide with the Gazebo world frame, so a tile placed at a zone's target
coordinates is exactly where the robot drives to treat that zone.

Gazebo (gz-sim / Harmonic) has no Python binding here and no reliable way to
recolour an existing visual, so we drive it through the server-side gz services:
  * /world/<world>/create  (gz.msgs.EntityFactory)  — spawn a tile (new colour)
  * /world/<world>/remove  (gz.msgs.Entity)         — delete a tile
A zone is only re-spawned when its colour actually changes, so there is no churn.

Legend:
  GREEN  = healthy            BLUE   = needs water (low moisture)
  YELLOW = needs fertiliser   ORANGE = needs water + fertiliser
  RED    = at risk (high runoff vulnerability — fertilising halted, SDG-14)
  GREY   = no telemetry yet
"""
import os
import subprocess

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

from my_turtlebot3_controller.qos import STATE_QOS

# Colour table: category -> (r, g, b)
COLOURS = {
    'healthy':   (0.10, 0.80, 0.15),
    'water':     (0.10, 0.35, 0.95),
    'nutrient':  (0.95, 0.85, 0.05),
    'both':      (0.95, 0.45, 0.05),
    'risk':      (0.90, 0.06, 0.06),
    'unknown':   (0.55, 0.55, 0.55),
}


class ZoneVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('zone_visualizer_node')

        self.declare_parameter('world_name', 'default')
        self.declare_parameter('moisture_threshold', 40.0)
        self.declare_parameter('nutrient_threshold', 50.0)
        self.declare_parameter('vulnerability_halt_threshold', 70.0)
        self.declare_parameter('update_period_sec', 1.0)
        self.declare_parameter('tile_alpha', 0.70)
        self.declare_parameter('tile_height', 0.02)

        self.world = self.get_parameter('world_name').get_parameter_value().string_value
        self.moisture_threshold = self.get_parameter('moisture_threshold').value
        self.nutrient_threshold = self.get_parameter('nutrient_threshold').value
        self.vulnerability_halt = self.get_parameter('vulnerability_halt_threshold').value
        self.update_period = self.get_parameter('update_period_sec').value
        self.tile_alpha = self.get_parameter('tile_alpha').value
        self.tile_height = self.get_parameter('tile_height').value

        # ── Load zone geometry from zones.yaml ──
        pkg_dir = get_package_share_directory('my_turtlebot3_controller')
        with open(os.path.join(pkg_dir, 'config', 'zones.yaml'), 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}

        # Same ordering CropDecisionNode / FieldSensorMockNode use for the arrays.
        self.zone_ids = sorted([z for z in raw.keys() if z != 'base_station'])
        self.geometry = {}
        for zid in self.zone_ids:
            z = raw[zid]
            # Tile is anchored at the navigation TARGET (== where the robot drives
            # to treat the zone), sized from the zone's bounding box.
            cx = float(z['target_x'])
            cy = float(z['target_y'])
            sx = abs(float(z['max_x']) - float(z['min_x']))
            sy = abs(float(z['max_y']) - float(z['min_y']))
            self.geometry[zid] = (cx, cy, sx, sy)

        # Latest telemetry + currently drawn colour per zone.
        self.moisture = {z: None for z in self.zone_ids}
        self.nutrients = {z: None for z in self.zone_ids}
        self.vulnerability = {z: None for z in self.zone_ids}
        self.drawn_category = {z: None for z in self.zone_ids}

        self.sdf_dir = '/tmp/nexus_zone_viz'
        os.makedirs(self.sdf_dir, exist_ok=True)
        self._gz_ok = True

        self.create_subscription(Float32MultiArray, '/field_moisture', self._moisture_cb, STATE_QOS)
        self.create_subscription(Float32MultiArray, '/field_nutrients', self._nutrients_cb, STATE_QOS)
        self.create_subscription(Float32MultiArray, '/field_vulnerability', self._vulnerability_cb, STATE_QOS)

        self.create_timer(self.update_period, self._update_tick)

        self.get_logger().info(
            f"Zone Visualizer started for world '{self.world}'. "
            f"Drawing {len(self.zone_ids)} anchored zones. "
            "Legend: GREEN=healthy, BLUE=needs water, YELLOW=needs fertiliser, "
            "ORANGE=needs both, RED=at risk.")

    # ── Telemetry callbacks (arrays indexed by self.zone_ids order) ──
    def _store(self, target: dict, msg: Float32MultiArray) -> None:
        for i, zid in enumerate(self.zone_ids):
            if i < len(msg.data):
                target[zid] = float(msg.data[i])

    def _moisture_cb(self, msg): self._store(self.moisture, msg)
    def _nutrients_cb(self, msg): self._store(self.nutrients, msg)
    def _vulnerability_cb(self, msg): self._store(self.vulnerability, msg)

    # ── Map a zone's state to a colour category ──
    def _category(self, zid: str) -> str:
        m, n, v = self.moisture[zid], self.nutrients[zid], self.vulnerability[zid]
        if m is None or n is None or v is None:
            return 'unknown'
        if v > self.vulnerability_halt:
            return 'risk'
        low_water = m < self.moisture_threshold
        low_nutri = n < self.nutrient_threshold
        if low_water and low_nutri:
            return 'both'
        if low_nutri:
            return 'nutrient'
        if low_water:
            return 'water'
        return 'healthy'

    def _update_tick(self) -> None:
        if not self._gz_ok:
            return
        for zid in self.zone_ids:
            category = self._category(zid)
            if category != self.drawn_category[zid]:
                if self._redraw(zid, category):
                    self.drawn_category[zid] = category

    # ── Gazebo entity management ──
    def _redraw(self, zid: str, category: str) -> bool:
        name = f"{zid}_viz"
        cx, cy, sx, sy = self.geometry[zid]
        r, g, b = COLOURS[category]
        self._write_sdf(name, sx, sy, r, g, b)
        # Remove any existing tile first (ignore failure if absent), then
        # (re)create it with the new colour at its anchored position.
        self._gz_remove(name)
        ok = self._gz_create(name, cx, cy)
        if ok:
            self.get_logger().info(f"{zid}: {category.upper()}")
        return ok

    def _write_sdf(self, name, sx, sy, r, g, b) -> None:
        a = self.tile_alpha
        er, eg, eb = r * 0.4, g * 0.4, b * 0.4
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <transparency>{1.0 - a:.2f}</transparency>
        <geometry><box><size>{sx:.3f} {sy:.3f} {self.tile_height:.3f}</size></box></geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.2f}</diffuse>
          <emissive>{er:.3f} {eg:.3f} {eb:.3f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""
        with open(os.path.join(self.sdf_dir, f"{name}.sdf"), 'w', encoding='utf-8') as f:
            f.write(sdf)

    def _gz_create(self, name, cx, cy) -> bool:
        path = os.path.join(self.sdf_dir, f"{name}.sdf")
        req = (f'sdf_filename: "{path}", name: "{name}", '
               f'pose: {{position: {{x: {cx:.3f}, y: {cy:.3f}, z: 0.01}}}}')
        return self._gz_call(f'/world/{self.world}/create', 'gz.msgs.EntityFactory', req)

    def _gz_remove(self, name) -> bool:
        req = f'name: "{name}", type: MODEL'
        return self._gz_call(f'/world/{self.world}/remove', 'gz.msgs.Entity', req)

    def _gz_call(self, service, reqtype, req) -> bool:
        try:
            result = subprocess.run(
                ['gz', 'service', '-s', service,
                 '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '4000', '--req', req],
                capture_output=True, text=True, timeout=6)
            return 'data: true' in result.stdout
        except FileNotFoundError:
            self.get_logger().error(
                "'gz' CLI not found — cannot draw zones. Is Gazebo sourced in this env?")
            self._gz_ok = False
            return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"gz service call timed out: {service}")
            return False

    def remove_all(self) -> None:
        for zid in self.zone_ids:
            self._gz_remove(f"{zid}_viz")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZoneVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.remove_all()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
