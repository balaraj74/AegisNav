"""Diagnostic (inside the Jazzy container): where are the scan returns and lethal local-costmap cells?

Prints, once, the closest scan returns (range, bearing) and the lethal cells of
/local_costmap/costmap in the odom frame. Used to debug controller "collision ahead".
"""
import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros


class Probe(Node):
    def __init__(self):
        super().__init__("aegisnav_probe")
        self.scan = None
        self.costmap = None
        self.odom = None
        self.path = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 5)
        self.create_subscription(Path, "/aegisnav/planned_path", lambda m: setattr(self, "path", m), 5)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.on_costmap,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))

    def on_scan(self, msg):
        self.scan = msg

    def on_costmap(self, msg):
        self.costmap = msg


def main():
    rclpy.init()
    node = Probe()
    for _ in range(150):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan and node.costmap and node.odom and node.path:
            break
    if node.scan:
        s = node.scan
        pts = sorted((r, math.degrees(s.angle_min + i * s.angle_increment)) for i, r in enumerate(s.ranges) if math.isfinite(r) and s.range_min <= r <= s.range_max)
        print(f"scan frame={s.header.frame_id} n_valid={len(pts)} closest:", [(round(r, 2), round(a)) for r, a in pts[:8]])
        try:
            t = node.tf_buffer.lookup_transform("map", s.header.frame_id, Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ox, oy = t.transform.translation.x, t.transform.translation.y
            print(f"lidar in map: ({ox:.2f},{oy:.2f}) yaw {math.degrees(yaw):.0f} deg")
            for r, a in pts[:3] + pts[len(pts) // 2:len(pts) // 2 + 1]:
                th = yaw + math.radians(a)
                print(f"  return r={r:.2f} bearing={a:.0f} -> map ({ox + r * math.cos(th):.2f},{oy + r * math.sin(th):.2f})")
        except tf2_ros.TransformException as exc:
            print("no tf map->lidar:", exc)
    if node.costmap:
        c = node.costmap
        lethal = [(round(c.info.origin.position.x + (i % c.info.width + 0.5) * c.info.resolution, 2),
                   round(c.info.origin.position.y + (i // c.info.width + 0.5) * c.info.resolution, 2))
                  for i, v in enumerate(c.data) if v >= 99]
        print(f"costmap frame={c.header.frame_id} origin=({c.info.origin.position.x:.2f},{c.info.origin.position.y:.2f}) lethal_cells={len(lethal)} sample={lethal[:12]}")
    if node.odom and node.costmap:
        c, o = node.costmap, node.odom.pose.pose.position
        near = []
        for i, v in enumerate(c.data):
            if v >= 99:
                x = c.info.origin.position.x + (i % c.info.width + 0.5) * c.info.resolution
                y = c.info.origin.position.y + (i // c.info.width + 0.5) * c.info.resolution
                d = math.hypot(x - o.x, y - o.y)
                if d < 0.6:
                    near.append((round(d, 2), round(x - o.x, 2), round(y - o.y, 2), v))
        near.sort()
        print(f"robot odom ({o.x:.2f},{o.y:.2f}); lethal/inscribed cells within 0.6 m (dist, dx, dy, value): {near[:10]} total {len(near)}")
    if node.path:
        pts = [(round(p.pose.position.x, 2), round(p.pose.position.y, 2)) for p in node.path.poses]
        print(f"planned path ({len(pts)} poses): first {pts[:6]} ... last {pts[-2:]}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
