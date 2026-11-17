"""Gazebo check: a hazard-flagged visual in front of the lidar must be invisible; an unflagged one visible.

Run inside the Jazzy container with Gazebo + the scan bridge running and the robot at spawn.
"""
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

WORLD = "aegisnav_rescue_world"
BOX_X, BOX_Y = -3.8 + 0.6, -2.2  # 0.53 m ahead of the lidar in the corridor world, open floor


def box_sdf(name, flags):
    vis = f"<visibility_flags>{flags}</visibility_flags>" if flags is not None else ""
    return (f'<sdf version="1.9"><model name="{name}"><static>true</static><pose>{BOX_X} {BOX_Y} 0.5 0 0 0</pose>'
            f'<link name="l"><visual name="v">{vis}<geometry><box><size>0.2 0.2 1.0</size></box></geometry></visual></link></model></sdf>')


def spawn(name, flags):
    subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/create", "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'sdf: \'{box_sdf(name, flags)}\''],
                   check=True, capture_output=True)


def remove(name):
    subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{name}" type: MODEL'],
                   check=True, capture_output=True)


def forward_range(node, got):
    got.clear()
    t0 = time.time()
    while time.time() - t0 < 5 and len(got) < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    s = got[-1]
    i = round((0.0 - s.angle_min) / s.angle_increment) % len(s.ranges)
    return min(s.ranges[(i + k) % len(s.ranges)] for k in range(-3, 4))


def main():
    rclpy.init()
    node = Node("aegisnav_visibility_check")
    got = []
    node.create_subscription(LaserScan, "/scan", got.append, QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    base = forward_range(node, got)
    spawn("mask_test_flagged", 2); time.sleep(1.5)
    flagged = forward_range(node, got)
    remove("mask_test_flagged"); spawn("mask_test_plain", None); time.sleep(1.5)
    plain = forward_range(node, got)
    remove("mask_test_plain")
    from aegisnav_ros.hazard_tiles import effect_model_sdf
    fx = effect_model_sdf("mask_test_fire", "fire").replace("<static>true</static>",
                                                            f"<static>true</static><pose>{BOX_X} {BOX_Y} 0 0 0 0</pose>")
    subprocess.run(["gz", "service", "-s", f"/world/{WORLD}/create", "--reqtype", "gz.msgs.EntityFactory",
                    "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f"sdf: '{fx}'"], check=True, capture_output=True)
    time.sleep(3.0)
    fire = forward_range(node, got)
    remove("mask_test_fire")
    print(f"forward range: empty {base:.2f} m | hazard-flagged box {flagged:.2f} m | unflagged box {plain:.2f} m | fire marker {fire:.2f} m")
    ok = abs(flagged - base) < 0.05 and plain < base - 0.5 and abs(fire - base) < 0.05
    print("VISIBILITY MASK", "PASS" if ok else "FAIL")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
