"""Offline tool tests. Run with ROS Noetic's system Python after building."""
import hashlib
import os
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

import rosbag
import rospy
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header, String
from velodyne_msgs.msg import VelodynePacket, VelodyneScan

BINARY = Path(os.environ.get('GENZ_BAG_TOOL', Path(__file__).resolve().parents[2] / 'build/bag_tools/prepare_rosbag'))


def records(path):
    with rosbag.Bag(str(path)) as bag:
        return [(topic, stamp.to_nsec(), raw[0], raw[1], raw[2])
                for topic, raw, stamp in bag.read_messages(raw=True)]


class PreparationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def call(self, output, *inputs, decode=False, success=True):
        args = [str(BINARY), '--output', str(output)]
        if decode:
            args.append('--decode-vlp16')
        result = subprocess.run(args + [str(p) for p in inputs], capture_output=True, text=True)
        self.assertEqual(result.returncode == 0, success, result.stdout + result.stderr)
        return result

    def make_bag(self, name, stamps):
        path = self.root / name
        with rosbag.Bag(str(path), 'w') as bag:
            for t in stamps:
                imu = Imu(header=Header(seq=t, stamp=rospy.Time(t, 123), frame_id='imu'))
                image = CompressedImage(header=Header(stamp=rospy.Time(t, 321), frame_id='camera'),
                                        format='jpeg', data=bytes([t, 2, 3, 4]))
                bag.write('/imu/data', imu, rospy.Time(t, 1000))
                bag.write('/camera/compressed', image, rospy.Time(t, 2000))
                bag.write('/metadata', String(data=str(t)), rospy.Time(t, 3000))
        return path

    def test_merge_interleaved_parts_preserves_all_serialized_messages(self):
        a, b = self.make_bag('a.bag', [1, 3]), self.make_bag('b.bag', [2, 4])
        hashes = [hashlib.sha256(p.read_bytes()).digest() for p in (a, b)]
        output = self.root / 'merged.bag'
        self.call(output, b, a)
        self.assertEqual(records(output), sorted(records(a) + records(b), key=lambda r: r[1]))
        self.assertEqual(hashes, [hashlib.sha256(p.read_bytes()).digest() for p in (a, b)])

    def test_refuses_overwrite_and_duplicate_input(self):
        a = self.make_bag('a.bag', [1])
        before = a.read_bytes()
        self.call(a, a, success=False)
        self.assertEqual(a.read_bytes(), before)
        output = self.root / 'out.bag'
        self.call(output, a, a, success=False)
        self.assertFalse(output.exists())

    def test_decode_requires_raw_packets(self):
        a = self.make_bag('a.bag', [1])
        output = self.root / 'out.bag'
        self.call(output, a, decode=True, success=False)
        self.assertFalse(output.exists())

    def test_decode_preserves_headers_passthrough_and_point_timing(self):
        path = self.make_bag('raw.bag', [1])
        packet = bytearray(1206)
        for block in range(12):
            struct.pack_into('<HH', packet, block * 100, 0xeeff, block * 100)
            for laser in range(32):
                struct.pack_into('<HB', packet, block * 100 + 4 + laser * 3, 2500, laser)
        packet[1204], packet[1205] = 0x37, 0x22
        scan = VelodyneScan(header=Header(seq=8, stamp=rospy.Time(2, 50), frame_id='velodyne'),
                            packets=[VelodynePacket(stamp=rospy.Time(2, 50), data=bytes(packet))])
        with rosbag.Bag(str(path), 'a') as bag:
            bag.write('/velodyne_packets', scan, rospy.Time(2, 500))
        output = self.root / 'points.bag'
        self.call(output, path, decode=True)
        self.assertEqual([r for r in records(path) if r[0] != '/velodyne_packets'],
                         [r for r in records(output) if r[0] != '/velodyne_points'])
        with rosbag.Bag(str(output)) as bag:
            entries = list(bag.read_messages(topics=['/velodyne_points']))
        self.assertEqual(len(entries), 1)
        _, cloud, timestamp = entries[0]
        self.assertEqual(timestamp, rospy.Time(2, 500))
        self.assertEqual((cloud.header.seq, cloud.header.stamp, cloud.header.frame_id),
                         (scan.header.seq, scan.header.stamp, scan.header.frame_id))
        self.assertEqual(cloud._type, 'sensor_msgs/PointCloud2')
        points = list(point_cloud2.read_points(cloud, field_names=['x', 'y', 'z', 'ring', 'time']))
        self.assertEqual(len(points), 384)
        self.assertEqual({p[3] for p in points}, set(range(16)))
        self.assertGreater(max(p[4] for p in points), min(p[4] for p in points))
        self.assertTrue(all(0 <= p[4] < 0.002 for p in points))

    def test_failed_conversion_leaves_no_output(self):
        path = self.root / 'empty_scan.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            bag.write('/velodyne_packets', VelodyneScan(), rospy.Time(1))
        output = self.root / 'out.bag'
        self.call(output, path, decode=True, success=False)
        self.assertFalse(output.exists())
        self.assertFalse(list(self.root.glob('*.partial.*')))

    def test_decode_rejects_existing_pointcloud_topic(self):
        path = self.root / 'already_decoded.bag'
        with rosbag.Bag(str(path), 'w') as bag:
            bag.write('/velodyne_points', PointCloud2(), rospy.Time(1))
        self.call(self.root / 'out.bag', path, decode=True, success=False)


if __name__ == '__main__':
    unittest.main()
