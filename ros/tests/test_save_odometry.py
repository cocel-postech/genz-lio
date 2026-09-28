"""Trajectory files must preserve sensor timestamps, poses, and existing results."""
import importlib.util
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

script = Path(__file__).resolve().parents[1] / "scripts/save_odometry.py"
spec = importlib.util.spec_from_file_location("save_odometry", script)
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


def message(ros_version=1):
    stamp = NS(secs=1700000000, nsecs=123456789) if ros_version == 1 else NS(sec=1700000000, nanosec=123456789)
    return NS(header=NS(stamp=stamp, frame_id="camera_init"), child_frame_id="body",
              pose=NS(pose=NS(position=NS(x=1., y=-2., z=3.),
                              orientation=NS(x=0., y=0., z=0., w=1.))))


class SaveOdometryTests(unittest.TestCase):
    def test_tum_exact_timestamp_and_xyzw_order_for_both_ros_versions(self):
        for version in [1, 2]:
            msg = message(version)
            msg.pose.pose.orientation = NS(x=2., y=0., z=0., w=0.)
            row = recorder.format_pose(msg, "tum").split()
            self.assertEqual(row[0], "1700000000.123456789")
            self.assertEqual(list(map(float, row[1:])), [1., -2., 3., 1., 0., 0., 0.])

    def test_kitti_rotated_pose_is_row_major_and_not_recentered(self):
        msg = message(2)
        msg.pose.pose.orientation = NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5))
        row = list(map(float, recorder.format_pose(msg, "kitti").split()))
        expected = [0., -1., 0., 1., 1., 0., 0., -2., 0., 0., 1., 3.]
        self.assertEqual(len(row), 12)
        for value, wanted in zip(row, expected):
            self.assertAlmostEqual(value, wanted, places=10)

    def test_streamed_output_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "path with spaces/custom.txt"
            writer = recorder.TrajectoryWriter(path, "tum")
            try:
                writer.receive(message())
                self.assertEqual(len(path.read_text().splitlines()), 1)
                before = path.read_bytes()
                with self.assertRaises(FileExistsError):
                    recorder.TrajectoryWriter(path, "tum")
                self.assertEqual(path.read_bytes(), before)
            finally:
                writer.close()
            writer = recorder.TrajectoryWriter(path, "kitti", overwrite=True)
            writer.receive(message())
            writer.close()
            self.assertEqual(len(path.read_text().split()), 12)

    def test_invalid_poses_and_changed_frames_are_not_written(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.txt"
            writer = recorder.TrajectoryWriter(path, "tum")
            writer.receive(message())
            invalid = message()
            invalid.pose.pose.position.x = float("nan")
            writer.receive(invalid)
            invalid = message()
            invalid.pose.pose.orientation.w = 0.
            writer.receive(invalid)
            invalid = message()
            invalid.header.frame_id = "another_run"
            writer.receive(invalid)
            invalid = message()
            invalid.header.stamp.nsecs = 1000000000
            writer.receive(invalid)
            writer.close()
            self.assertEqual(writer.count, 1)
            self.assertEqual(writer.skipped, 4)
            self.assertEqual(len(path.read_text().splitlines()), 1)

    def test_cli_format_and_custom_path(self):
        args = recorder.parse_args(["--ros-version", "2", "--format", "KITTI", "--output", "my path/poses.txt"])
        self.assertEqual(args.format, "kitti")
        self.assertEqual(args.output, Path("my path/poses.txt"))


if __name__ == "__main__":
    unittest.main()
