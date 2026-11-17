"""Save frames from Gazebo cameras as PNG (inside the Jazzy container; no extra Python packages).

Usage: python3 capture_frames.py <out_dir> <topic>:<name> [...] --at 8,25,45
Saves <name>_t<sec>.png when sim time since the first frame passes each requested second.
These are genuine renders from the running simulation (headless ogre2, software rendering).
"""
import struct
import sys
import zlib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def write_png(path, width, height, rgb: bytes, step: int):
    rows = b"".join(b"\x00" + rgb[y * step:y * step + width * 3] for y in range(height))
    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(rows, 6)) + chunk(b"IEND", b"")
    open(path, "wb").write(png)


def main():
    out_dir = sys.argv[1]
    at = [float(v) for v in sys.argv[sys.argv.index("--at") + 1].split(",")]
    streams = [a.split(":") for a in sys.argv[2:sys.argv.index("--at")]]
    rclpy.init()
    node = Node("aegisnav_frame_capture")
    state = {name: {"t0": None, "next": 0} for _, name in streams}

    def make_cb(name):
        def cb(msg: Image):
            st = state[name]
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if st["t0"] is None:
                st["t0"] = t
            if st["next"] < len(at) and t - st["t0"] >= at[st["next"]]:
                if msg.encoding not in ("rgb8", "bgr8"):
                    node.get_logger().error(f"unsupported encoding {msg.encoding}")
                    return
                data = bytes(msg.data)
                if msg.encoding == "bgr8":
                    data = bytes(b for i in range(0, len(data), 3) for b in (data[i + 2], data[i + 1], data[i]))
                path = f"{out_dir}/{name}_t{int(at[st['next']]):03d}.png"
                write_png(path, msg.width, msg.height, data, msg.step)
                node.get_logger().info(f"saved {path}")
                st["next"] += 1
        return cb

    for topic, name in streams:
        node.create_subscription(Image, topic, make_cb(name), 2)
    while rclpy.ok() and any(st["next"] < len(at) for st in state.values()):
        rclpy.spin_once(node, timeout_sec=0.5)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
