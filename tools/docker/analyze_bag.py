"""Offline analysis of a stage-3 rosbag (inside the Jazzy container).

For each "collision ahead" warning time in sim.log (wall clock), report the active AegisNav path,
the robot pose (odom), the robot heading, and the lethal local-costmap cells nearest the footprint.
Usage: python3 analyze_bag.py <bag_dir> <sim.log>
"""
import math
import re
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield topic, deserialize_message(data, get_message(types[topic])), t_ns * 1e-9


def main(bag, log):
    events = [float(m) for m in re.findall(r"\[(\d+\.\d+)\] \[controller_server\]: RegulatedPurePursuitController detected collision ahead", open(log).read())]
    if not events:
        print("no collision events")
        return
    first = events[0]
    last = {}
    for topic, msg, t in read(bag):
        if t > first:
            break
        last[topic] = (msg, t)
    print(f"first collision warning at wall {first:.2f}; {len(events)} warnings total")
    if "/odom" in last:
        o = last["/odom"][0].pose.pose
        yaw = 2 * math.atan2(o.orientation.z, o.orientation.w)
        print(f"odom pose ({o.position.x:.2f},{o.position.y:.2f}) yaw {math.degrees(yaw):.0f} deg")
    if "/cmd_vel" in last:
        c = last["/cmd_vel"][0]
        print(f"last cmd_vel v={c.linear.x:.2f} w={c.angular.z:.2f}")
    if "/aegisnav/planned_path" in last:
        p, t = last["/aegisnav/planned_path"]
        pts = [(round(q.pose.position.x, 2), round(q.pose.position.y, 2)) for q in p.poses]
        print(f"active path (received {first - t:.1f}s before), {len(pts)} poses: {pts[:4]} ... {pts[-3:]}")
    if "/local_costmap/costmap" in last and "/odom" in last:
        c = last["/local_costmap/costmap"][0]
        o = last["/odom"][0].pose.pose.position
        lethal = []
        for i, v in enumerate(c.data):
            if v == 100:
                x = c.info.origin.position.x + (i % c.info.width + 0.5) * c.info.resolution
                y = c.info.origin.position.y + (i // c.info.width + 0.5) * c.info.resolution
                lethal.append((round(math.hypot(x - o.x, y - o.y), 2), round(x - o.x, 2), round(y - o.y, 2)))
        lethal.sort()
        print(f"nearest LETHAL cells (dist, dx, dy) in odom: {lethal[:8]}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
