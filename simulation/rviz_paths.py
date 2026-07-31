#!/usr/bin/env python3
"""Live 3-D drone trajectories + target/drone labels for RViz 2.

Reads the same mission config as the mission node ($MISSION_CONFIG_FILE, default
/tmp/mission_config.txt), whose lines are:
    ns sysid wait target_x target_y cruise_alt [spawn_x spawn_y]
Subscribes to each drone's /<ns>/fmu/out/vehicle_local_position and republishes,
in a common 'map' frame, a single MarkerArray on /viz/markers containing:
  - a coloured LINE_STRIP trajectory per drone,
  - a moving sphere + "Drone i" label following each drone,
  - a sphere + "Target i (x,y)" label at each target.

Run:  python3 rviz_paths.py     (after sourcing ROS + the workspace)
Then open RViz with Fixed Frame = map and a MarkerArray display on /viz/markers
(or use:  rviz2 -d mission.rviz).
"""
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from px4_msgs.msg import VehicleLocalPosition

PALETTE = [(1.0, 0.25, 0.25), (0.25, 0.8, 0.35), (0.3, 0.5, 1.0),
           (1.0, 0.7, 0.1), (0.8, 0.3, 0.9)]


def rgba(i, a=1.0):
    r, g, b = PALETTE[i % len(PALETTE)]
    return ColorRGBA(r=r, g=g, b=b, a=a)


class DroneState:
    def __init__(self, ns, sx, sy, tx, ty, alt):
        self.ns, self.sx, self.sy = ns, sx, sy
        self.tx, self.ty, self.alt = tx, ty, alt
        self.pts = []      # trajectory points
        self.cur = None    # current (x, y, z) in map frame


class Viz(Node):
    FRAME = 'map'

    def __init__(self):
        super().__init__('rviz_paths')
        path = os.environ.get('MISSION_CONFIG_FILE', '/tmp/mission_config.txt')
        self.drones = []
        with open(path) as f:
            for raw in f:
                tok = raw.split('#')[0].split()
                if len(tok) < 6:
                    continue
                ns = tok[0]
                tx, ty, alt = float(tok[3]), float(tok[4]), float(tok[5])
                sx = float(tok[6]) if len(tok) >= 8 else 0.0
                sy = float(tok[7]) if len(tok) >= 8 else 0.0
                self.drones.append(DroneState(ns, sx, sy, tx, ty, alt))
        self.get_logger().info(f'rviz_paths: {len(self.drones)} drone(s) from {path}')

        self.pub = self.create_publisher(MarkerArray, '/viz/markers', 10)
        for i, d in enumerate(self.drones):
            self.create_subscription(
                VehicleLocalPosition, d.ns + '/fmu/out/vehicle_local_position',
                self._cb(i), qos_profile_sensor_data)
        self.create_timer(0.1, self.publish)   # 10 Hz

    def _cb(self, i):
        d = self.drones[i]

        def cb(msg):
            if not (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
                return
            x, y, z = d.sx + msg.x, d.sy + msg.y, -msg.z   # NED -> map (z up), + spawn offset
            d.cur = (x, y, z)
            if not d.pts or (abs(d.pts[-1].x - x) + abs(d.pts[-1].y - y) + abs(d.pts[-1].z - z)) > 0.15:
                d.pts.append(Point(x=x, y=y, z=z))
        return cb

    def _m(self, ns, i, mtype, now):
        m = Marker()
        m.header.frame_id = self.FRAME
        m.header.stamp = now
        m.ns, m.id, m.type, m.action = ns, i, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, d in enumerate(self.drones):
            c = rgba(i)
            # trajectory
            if len(d.pts) >= 2:
                m = self._m('path', i, Marker.LINE_STRIP, now)
                m.scale.x = 0.25
                m.color = c
                m.points = list(d.pts)
                arr.markers.append(m)
            # current position + moving label
            if d.cur:
                s = self._m('drone', i, Marker.SPHERE, now)
                s.pose.position.x, s.pose.position.y, s.pose.position.z = d.cur
                s.scale.x = s.scale.y = s.scale.z = 0.8
                s.color = c
                arr.markers.append(s)
                t = self._m('drone_label', i, Marker.TEXT_VIEW_FACING, now)
                t.pose.position.x, t.pose.position.y = d.cur[0], d.cur[1]
                t.pose.position.z = d.cur[2] + 1.2
                t.scale.z = 1.2
                t.color = c
                t.text = f'Drone {i + 1}'
                arr.markers.append(t)
            # target marker + label
            wx, wy = d.sx + d.tx, d.sy + d.ty
            ts = self._m('target', i, Marker.SPHERE, now)
            ts.pose.position.x, ts.pose.position.y, ts.pose.position.z = wx, wy, 0.5
            ts.scale.x = ts.scale.y = ts.scale.z = 1.2
            ts.color = rgba(i, 0.6)
            arr.markers.append(ts)
            tl = self._m('target_label', i, Marker.TEXT_VIEW_FACING, now)
            tl.pose.position.x, tl.pose.position.y, tl.pose.position.z = wx, wy, 2.0
            tl.scale.z = 1.2
            tl.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            tl.text = f'Target {i + 1}\n({wx:.0f}, {wy:.0f})'
            arr.markers.append(tl)
        self.pub.publish(arr)


def main():
    rclpy.init()
    node = Viz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
