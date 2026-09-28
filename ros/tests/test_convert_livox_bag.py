"""Verify schema mapping, timestamps, payloads and failure handling of bag conversion."""
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

script = Path(__file__).resolve().parents[1] / "scripts/convert_livox_bag.py"
spec = importlib.util.spec_from_file_location("convert_livox_bag", script)
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)


def write_sample(path, altered=False, corrupt=False):
    store = get_typestore(Stores.ROS1_NOETIC)
    defs = converter.livox_types("livox_ros_driver", get_types_from_msg)
    if altered:
        defs.update(get_types_from_msg(converter.POINT_FIELDS.replace("uint32", "uint64"),
                                      "livox_ros_driver/msg/CustomPoint"))
    store.register(defs)
    t = store.types
    header = t["std_msgs/msg/Header"](7, t["builtin_interfaces/msg/Time"](10, 123456789), "lidar")
    point = t["livox_ros_driver/msg/CustomPoint"](98765, 1., -2., 3., 40, 16, 5)
    cloud = t["livox_ros_driver/msg/CustomMsg"](
        header, 10123456789, 1, 2, np.array([1, 2, 3], dtype=np.uint8), [point])
    imu = t["sensor_msgs/msg/Imu"](
        header, t["geometry_msgs/msg/Quaternion"](0., 0., 0., 1.), np.zeros(9),
        t["geometry_msgs/msg/Vector3"](1., 2., 3.), np.arange(9, dtype=float),
        t["geometry_msgs/msg/Vector3"](4., 5., 6.), np.zeros(9))
    with Writer(path) as writer:
        c = writer.add_connection("/livox/lidar", cloud.__msgtype__, typestore=store)
        i = writer.add_connection("/imu/data", imu.__msgtype__, typestore=store)
        metadata = t["std_msgs/msg/String"]("unchanged metadata")
        m = writer.add_connection("/metadata", metadata.__msgtype__, typestore=store, latching=1)
        writer.write(c, 20000000001, b"broken" if corrupt else store.serialize_ros1(cloud, cloud.__msgtype__))
        writer.write(i, 20000000002, store.serialize_ros1(imu, imu.__msgtype__))
        writer.write(m, 20000000003, store.serialize_ros1(metadata, metadata.__msgtype__))


class ConvertLivoxTests(unittest.TestCase):
    def test_livox_and_other_messages_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            source, dest = Path(directory)/"source.bag", Path(directory)/"output with spaces"
            write_sample(source)
            before = hashlib.sha256(source.read_bytes()).digest()
            self.assertEqual(converter.convert(source, dest), (3, 1))
            self.assertEqual(hashlib.sha256(source.read_bytes()).digest(), before)
            with AnyReader([dest]) as reader:
                messages = [(c, stamp, reader.deserialize(data, c.msgtype)) for c, stamp, data in reader.messages()]
            self.assertEqual([v[1] for v in messages], [20000000001, 20000000002, 20000000003])
            c, _, cloud = messages[0]
            self.assertEqual(c.topic, "/livox/lidar")
            self.assertEqual(c.msgtype, "livox_ros_driver2/msg/CustomMsg")
            self.assertEqual(cloud.header.stamp.sec, 10)
            self.assertEqual(cloud.header.stamp.nanosec, 123456789)
            self.assertEqual(cloud.header.frame_id, "lidar")
            self.assertEqual((cloud.timebase, cloud.point_num, cloud.lidar_id), (10123456789, 1, 2))
            np.testing.assert_array_equal(cloud.rsvd, [1, 2, 3])
            point = cloud.points[0]
            self.assertEqual(point.__msgtype__, "livox_ros_driver2/msg/CustomPoint")
            self.assertEqual((point.offset_time, point.x, point.y, point.z, point.reflectivity, point.tag, point.line),
                             (98765, 1., -2., 3., 40, 16, 5))
            self.assertEqual(messages[1][0].topic, "/imu/data")
            self.assertEqual(messages[1][2].linear_acceleration.z, 6.)
            np.testing.assert_array_equal(messages[1][2].angular_velocity_covariance, np.arange(9))
            self.assertEqual(messages[2][2].data, "unchanged metadata")
            self.assertIn("durability: 1", messages[2][0].ext.offered_qos_profiles)

    def test_existing_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            source, dest = Path(directory)/"source.bag", Path(directory)/"output"
            write_sample(source)
            dest.mkdir(); (dest/"keep.txt").write_text("keep")
            with self.assertRaises(FileExistsError):
                converter.convert(source, dest)
            self.assertEqual((dest/"keep.txt").read_text(), "keep")

    def test_incompatible_livox_layout_rejected_before_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source, dest = Path(directory)/"source.bag", Path(directory)/"output"
            write_sample(source, altered=True)
            with self.assertRaisesRegex(ValueError, "unsupported Livox schema"):
                converter.convert(source, dest)
            self.assertFalse(dest.exists())

    def test_corrupt_message_does_not_leave_partial_output(self):
        with tempfile.TemporaryDirectory() as directory:
            source, dest = Path(directory)/"source.bag", Path(directory)/"output"
            write_sample(source, corrupt=True)
            with self.assertRaises(Exception):
                converter.convert(source, dest)
            self.assertFalse(dest.exists())


if __name__ == "__main__":
    unittest.main()
