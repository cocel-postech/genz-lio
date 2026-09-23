"""Time contracts shared by the Python bag reader and the ROS wrapper."""
from types import SimpleNamespace as NS
import sys
import numpy as np
import pytest
from genz_lio import PreprocessConfig, LidarType
from genz_lio.genz_lio_pybind import _ScanBuffer
from genz_lio.datasets.pointcloud2 import read_pointcloud2
from genz_lio.datasets.rosbag import RosbagDataset


def stamp(t):
    sec = int(t)
    return NS(stamp=NS(sec=sec, nanosec=int(round((t - sec) * 1e9))))


def cloud(times=(0, .05, .1), name='time', header=10, big=False, padded=False):
    width, height = (1, len(times)) if padded else (len(times), 1)
    row_step = width * 20 + (12 if padded else 0)
    data = bytearray(height * row_step)
    dtype = np.dtype(dict(names=['x','y','z',name], formats=[('>' if big else '<')+s
        for s in ['f4','f4','f4','f8']], offsets=[0,4,8,12],itemsize=20))
    a = np.ndarray((height,width), dtype=dtype, buffer=data, strides=(row_step,20))
    for i,t in enumerate(times):
        a[i//width,i%width] = (5+i, 1, 2, t)
    return NS(width=width,height=height,point_step=20,row_step=row_step,data=data,
        is_bigendian=big,header=stamp(header),fields=[NS(name=n,offset=o,datatype=d,count=1)
            for n,o,d in [('x',0,7),('y',4,7),('z',8,7),(name,12,8)]])


@pytest.mark.parametrize('big,padded', [(False,False),(False,True),(True,True)])
def test_pointcloud_layout(big,padded):
    p,t,_,_ = read_pointcloud2(cloud(big=big,padded=padded))
    np.testing.assert_array_equal(p[:,0], [5,6,7])
    np.testing.assert_allclose(t,[0,.05,.1])


@pytest.mark.parametrize('sensor,header,scale',[(LidarType.HESAI,10,1),
    (LidarType.ROBOSENSE,1700000000,1),(LidarType.LIVOX_PCL,1700000000,1e9)])
def test_absolute_time_anchors(sensor,header,scale):
    values=(np.array([.025,.05,.1])+header)*scale
    _,times,_,_ = read_pointcloud2(cloud(values,'timestamp',header),sensor)
    np.testing.assert_allclose(times,[.025,.05,.1],atol=4e-7,rtol=0)


def test_constant_and_nonfinite_timestamps():
    assert read_pointcloud2(cloud([-.05]*3))[1] is None
    p,t,_,_ = read_pointcloud2(cloud([-.1,np.nan,-.01]))
    assert len(p)==2 and np.isfinite(t).all()


def test_malformed_cloud_rejected():
    m=cloud();m.data=m.data[:15]
    with pytest.raises(ValueError,match='truncated'):read_pointcloud2(m)
    m=cloud();m.fields[-1].offset=19
    with pytest.raises(ValueError,match='out of bounds'):read_pointcloud2(m)


def imu(t):
    return NS(header=stamp(t),linear_acceleration=NS(x=0,y=0,z=9.81),
              angular_velocity=NS(x=0,y=0,z=0))


def dataset(events):
    class Reader:
        connections=[NS(topic='/cloud',msgtype='sensor_msgs/msg/PointCloud2'),
                     NS(topic='/imu',msgtype='sensor_msgs/msg/Imu')]
        def __init__(self,*args):pass
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def messages(self,**kwargs):
            for topic,message in events:
                yield self.connections[topic=='/imu'],0,message
        def deserialize(self,raw,*args):return raw
    result=RosbagDataset.__new__(RosbagDataset)
    result._reader_factory=Reader;result.path='unused';result.time_offset=0
    result._length=sum(topic=='/cloud' for topic,_ in events)
    result.lidar_topic='/cloud';result.imu_topic='/imu'
    result.preprocess=PreprocessConfig();result.preprocess.lidar_type=LidarType.OUSTER
    return result


def test_late_imu_waits_for_multiple_pending_scans():
    d=dataset([('/cloud',cloud(header=10)),('/cloud',cloud(header=10.1)),
        ('/imu',imu(10.05)),('/imu',imu(10.15)),('/imu',imu(10.25))])
    frames=list(d)
    assert len(frames)==2
    np.testing.assert_allclose(frames[0].imu[:,0],[10.05])
    np.testing.assert_allclose(frames[1].imu[:,0],[10.15])
    assert all(f.timing_prepared for f in frames)


def test_imu_arrives_before_cloud_and_eof_is_incomplete():
    d=dataset([('/imu',imu(10.05)),('/imu',imu(10.15)),('/cloud',cloud(header=10)),
               ('/cloud',cloud(header=10.2))])
    with pytest.warns(RuntimeWarning,match='lack IMU coverage'):
        frames=list(d)
    assert len(frames)==1
    np.testing.assert_allclose(frames[0].imu[:,0],[10.05])


def test_bag_rewind_cannot_mix_trajectories():
    d=dataset([('/imu',imu(100)),('/imu',imu(10))])
    with pytest.raises(ValueError,match='backwards'):list(d)


def test_native_buffer_reconstructs_before_pairing():
    config=PreprocessConfig();config.scan_rate=5;config.scan_line=1
    b=_ScanBuffer(config)
    yaw=np.array([0,-1,-2,-3,2,1,.1]);p=np.c_[5*np.cos(yaw),5*np.sin(yaw),np.ones(7)]
    assert b.push_scan(p,10,rings=np.zeros(7))==0
    b.push_imu(np.array([[10.11,0,0,9.81,0,0,0]]))
    assert b.pop() is None
    b.push_imu(np.array([[10.19,0,0,9.81,0,0,0],[10.21,0,0,9.81,0,0,0]]))
    _,begin,end,samples,times,_,_,_=b.pop()
    assert end-begin>.19 and times[0]==0
    np.testing.assert_allclose(samples[:,0],[10.11,10.19])
