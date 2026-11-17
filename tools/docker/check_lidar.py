"""Compare one /scan (robot static at spawn, yaw 0) with ranges ray-cast through the core grid.

Run inside the Jazzy container after starting Gazebo + the scan bridge. A large mismatch means the
simulated lidar output is wrong (e.g. software-rendering depth problems), not the map.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from aegisnav_core.models import Cell
from aegisnav_core.scenarios import corridor_scenario

LIDAR_X, LIDAR_Y = -3.8 + 0.07, -2.2  # spawn pose + mount offset, map frame, yaw 0


def expected_range(grid, x, y, theta, max_r=8.0, step=0.005):
    r = 0.0
    while r < max_r:
        px, py = x + r * math.cos(theta), y + r * math.sin(theta)
        cx = math.floor((px - grid.origin_x_m) / grid.resolution_m)
        cy = math.floor((py - grid.origin_y_m) / grid.resolution_m)
        if not grid.contains(Cell(cx, cy)) or grid.is_blocked(Cell(cx, cy)):
            return r
        r += step
    return math.inf


def main():
    rclpy.init()
    node = Node("aegisnav_lidar_check")
    got = []
    node.create_subscription(LaserScan, "/scan", got.append, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.1)
        if got:
            break
    if not got:
        print("no scan received")
        return
    scan, grid = got[-1], corridor_scenario().grid
    errors = []
    for deg in range(-180, 180, 30):
        i = round((math.radians(deg) - scan.angle_min) / scan.angle_increment) % len(scan.ranges)
        measured, exp = scan.ranges[i], expected_range(grid, LIDAR_X, LIDAR_Y, math.radians(deg))
        errors.append(abs(measured - exp))
        print(f"bearing {deg:5d}  measured {measured:5.2f}  expected {exp:5.2f}")
    print(f"max abs error {max(errors):.2f} m")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
