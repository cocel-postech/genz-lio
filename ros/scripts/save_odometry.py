#!/usr/bin/env python3
"""Subscribe to ROS 1/2 Odometry and stream a TUM or KITTI trajectory to disk."""

import argparse
import math
import os
from pathlib import Path
import sys
import threading


def format_pose(message, file_format):
    """Keep the message timestamp and child-to-header-frame pose (no TF lookup)."""
    stamp = message.header.stamp
    if hasattr(stamp, "secs"):
        seconds, nanos = stamp.secs, stamp.nsecs
    else:
        seconds, nanos = stamp.sec, stamp.nanosec
    if seconds < 0 or not 0 <= nanos < 1000000000:
        raise ValueError("invalid odometry timestamp")
    pose = message.pose.pose
    p, q = pose.position, pose.orientation
    tx, ty, tz = p.x, p.y, p.z
    x, y, z, w = q.x, q.y, q.z, q.w
    if not all(math.isfinite(v) for v in (tx, ty, tz, x, y, z, w)):
        raise ValueError("non-finite odometry pose")
    norm = math.hypot(x, y, z, w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid odometry quaternion norm")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    if file_format == "tum":
        values = (tx, ty, tz, x, y, z, w)
        return f"{seconds}.{nanos:09d} " + " ".join(f"{v:.12f}" for v in values) + "\n"
    if file_format != "kitti":
        raise ValueError("format must be tum or kitti")
    values = (
        1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w), tx,
        2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w), ty,
        2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y), tz,
    )
    return " ".join(f"{v:.12f}" for v in values) + "\n"


class TrajectoryWriter:
    def __init__(self, path, file_format, overwrite=False):
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.file = self.path.open("w" if overwrite else "x", buffering=1, encoding="utf-8")
        self.file_format = file_format
        self.count = 0
        self.skipped = 0
        self.error = None
        self.frames = None
        self.lock = threading.Lock()

    def receive(self, message):
        with self.lock:
            if self.file.closed or self.error:
                return
            try:
                frames = (message.header.frame_id, message.child_frame_id)
                if self.frames is not None and frames != self.frames:
                    raise ValueError("odometry frames changed; use a separate output for each run")
                line = format_pose(message, self.file_format)
            except ValueError as exc:
                self.skipped += 1
                if self.skipped == 1:
                    print(f"Skipping invalid odometry: {exc}", file=sys.stderr, flush=True)
                return
            try:
                self.file.write(line)
            except OSError as exc:
                self.error = exc
                return
            self.frames = frames
            self.count += 1

    def close(self):
        with self.lock:
            self.file.close()


def record_ros1(args, writer):
    import rospy
    from nav_msgs.msg import Odometry

    rospy.init_node("genz_lio_save_odometry", anonymous=True, argv=[sys.argv[0]])

    def callback(message):
        writer.receive(message)
        if writer.error:
            rospy.signal_shutdown(str(writer.error))

    subscription = rospy.Subscriber(args.topic, Odometry, callback, queue_size=args.queue_size)
    print(f"Listening on {args.topic}; writing {args.format.upper()} to {writer.path}. Ctrl+C to stop.", flush=True)
    try:
        rospy.spin()
    finally:
        subscription.unregister()


def record_ros2(args, writer):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    rclpy.init(args=[])
    node = rclpy.create_node(f"genz_lio_save_odometry_{os.getpid()}")
    # GenZ-LIO publishes Odometry reliably. Keep a bounded backlog for disk writes.
    qos = QoSProfile(depth=args.queue_size, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Odometry, args.topic, writer.receive, qos)
    print(f"Listening on {args.topic}; writing {args.format.upper()} to {writer.path}. Ctrl+C to stop.", flush=True)
    try:
        while rclpy.ok() and writer.error is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--format", type=str.lower, choices=("tum", "kitti"), default="tum")
    parser.add_argument("--output", type=Path, required=True, help="Output file path, including filename")
    parser.add_argument("--topic", default="/Odometry", help="nav_msgs/Odometry topic (default: /Odometry)")
    parser.add_argument("--ros-version", choices=("1", "2"), default=os.environ.get("ROS_VERSION"),
                        help="Defaults to ROS_VERSION from the sourced workspace")
    parser.add_argument("--queue-size", type=int, default=10000, help="Subscriber backlog (default: 10000)")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing output file")
    args = parser.parse_args(argv)
    if args.ros_version not in ("1", "2"):
        parser.error("source your ROS workspace or specify --ros-version 1 or 2")
    if args.queue_size < 1:
        parser.error("--queue-size must be positive")
    return args


def main(argv=None):
    args = parse_args(argv)
    writer = None
    try:
        writer = TrajectoryWriter(args.output, args.format, args.overwrite)
        try:
            (record_ros1 if args.ros_version == "1" else record_ros2)(args, writer)
        except KeyboardInterrupt:
            pass
        finally:
            writer.close()
        print(f"Saved {writer.count} poses to {writer.path} ({writer.skipped} skipped).", flush=True)
        if writer.error:
            raise writer.error
        if not writer.count:
            print("No odometry received. Check the topic and start recording before playback.", file=sys.stderr)
            return 1
        return 0
    except ImportError as exc:
        print(f"ROS Python modules unavailable: {exc}. Source the matching workspace and use its system Python.", file=sys.stderr)
    except OSError as exc:
        print(f"Cannot save odometry: {exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
