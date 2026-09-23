"""Decode an SDK-generated PCAP, including IMU units and scan coverage."""
from pathlib import Path
import struct
import numpy as np
import pytest

pytest.importorskip("ouster.sdk", reason="requires the ouster extra")
from ouster.sdk import client,pcap
from genz_lio.datasets.ouster import OusterDataset
from ouster.sdk.util.parsing import scan_to_packets

def make_capture(root,with_imu=True):
 root.mkdir(parents=True,exist_ok=True)
 info=client.SensorInfo.from_default(client.LidarMode.MODE_512x10)
 info.config.udp_port_lidar=7502
 info.config.udp_port_imu=7503
 (root/'sample.json').write_text(info.to_json_string())
 events=[]
 for i in range(4):
  scan=client.LidarScan(info)
  scan.frame_id=i
  scan.timestamp[:]=1_000_000_000+i*100_000_000+np.arange(scan.w)*100_000
  scan.measurement_id[:]=np.arange(scan.w)
  scan.status[:]=1
  scan.field(client.ChanField.RANGE)[:]=10000
  for j,p in enumerate(scan_to_packets(scan,info)):
   p.host_timestamp=1_000_000_000+i*100_000_000+j*1600000
   events.append(p)
 if with_imu:
  for t in range(990000000,1400000001,5000000):
   p=client.ImuPacket(48)
   p.buf[:]=np.frombuffer(struct.pack('<QQQffffff',t+12345,t,t,0,0,1,0,0,180),dtype=np.uint8)
   p.host_timestamp=t
   events.append(p)
 events.sort(key=lambda p:p.host_timestamp)
 pcap.record(events,str(root/'sample.pcap'))
 return root/'sample.pcap'


def test_pcap_decodes_sensor_clock_si_units_and_inter_scan_imu(tmp_path):
    dataset = OusterDataset(make_capture(tmp_path))
    frames = list(dataset)
    assert len(dataset) == len(frames) == 4
    samples = np.concatenate([frame.imu for frame in frames])
    assert np.all(np.diff(samples[:, 0]) > 0)
    np.testing.assert_allclose(samples[:, 3], 9.80665)
    np.testing.assert_allclose(samples[:, 6], np.pi)
    assert samples[0, 0] == pytest.approx(0.99)
    assert frames[1].imu[0, 0] < frames[1].begin_time
    assert frames[0].points.shape == (64 * 512, 3)
    assert frames[0].timestamps.min() == 0
    assert frames[0].timestamps.max() == pytest.approx(0.0511)
    assert len(list(dataset)) == 4


def test_pcap_without_imu_fails_clearly(tmp_path):
    dataset = OusterDataset(make_capture(tmp_path, with_imu=False))
    with pytest.raises(ValueError, match="no valid IMU packets"):
        list(dataset)
