"""CustomMsg must retain the same valid returns and timing as ROS onLivox."""
from types import SimpleNamespace as NS
import numpy as np
import pytest
from genz_lio import LidarType, PreprocessConfig
from genz_lio.datasets.livox import read_livox
from genz_lio.datasets.rosbag import RosbagDataset


def point(x=5., offset=0, tag=0, line=0):
    return NS(x=x, y=1., z=2., offset_time=offset, tag=tag, reflectivity=37, line=line)


def test_custom_returns_and_unsorted_offsets():
    msg=NS(point_num=5, points=[point(offset=100000000,tag=0x10,line=5),
        point(offset=20000000),point(offset=30000000,tag=0x20),
        point(x=float('nan')),point(offset=50000000,tag=0x30)])
    positions,times,intensities,rings=read_livox(msg)
    assert positions.shape==(2,3)
    np.testing.assert_allclose(times,[.1,.02],rtol=0,atol=1e-16)
    np.testing.assert_array_equal(intensities,[37,37]);assert rings is None


def test_custom_size_is_checked_and_empty_is_well_formed():
    with pytest.raises(ValueError,match='point_num'):read_livox(NS(point_num=2,points=[point()]))
    p,t,i,_=read_livox(NS(point_num=1,points=[point(tag=0x20)]))
    assert p.shape==(0,3) and t.shape==i.shape==(0,)


@pytest.mark.parametrize('namespace',['livox_ros_driver','livox_ros_driver2'])
def test_custom_bag_uses_header_and_shared_scan_buffer(namespace):
    header=lambda sec,ns=0:NS(stamp=NS(sec=sec,nanosec=ns))
    cloud=NS(header=header(10),timebase=99999999999,point_num=3,
        points=[point(offset=100000000),point(offset=20000000),point(offset=60000000,tag=0x20)])
    imu=lambda ns:NS(header=header(10,ns),linear_acceleration=NS(x=0,y=0,z=9.81),angular_velocity=NS(x=0,y=0,z=0))
    class Reader:
        connections=[NS(topic='/lidar',msgtype=namespace+'/msg/CustomMsg'),NS(topic='/imu',msgtype='sensor_msgs/msg/Imu')]
        def __init__(self,*args):pass
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def messages(self,**kwargs):
            for index,message in [(0,cloud),(1,imu(50000000)),(1,imu(150000000))]:yield self.connections[index],0,message
        def deserialize(self,message,*args):return message
    data=RosbagDataset.__new__(RosbagDataset);data.path='unused';data._reader_factory=Reader
    data.lidar_topic='/lidar';data.imu_topic='/imu';data.time_offset=0;data._length=1
    data.preprocess=PreprocessConfig();data.preprocess.lidar_type=LidarType.LIVOX
    frames=list(data);assert len(frames)==1
    assert abs(frames[0].begin_time-10.02)<1e-7 and abs(frames[0].end_time-10.1)<1e-7
    assert len(frames[0].points)==2 and frames[0].timing_prepared
    np.testing.assert_allclose(frames[0].imu[:,0],[10.05],rtol=0,atol=1e-9)
    assert data.pending_scans==data.empty_clouds==0
    assert RosbagDataset._pick(Reader(),(namespace+'/msg/CustomMsg',),'point cloud')=='/lidar'


@pytest.mark.parametrize('namespace', ['livox_ros_driver', 'livox_ros_driver2'])
@pytest.mark.parametrize('wire', ['ros1', 'cdr_le', 'cdr_be'])
@pytest.mark.parametrize('frame_id', ['', 'x', 'abc', 'livox_frame', '라이다'])
def test_bulk_wire_decoder_matches_rosbags(namespace, wire, frame_id):
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    from genz_lio.datasets.livox import _CUSTOM_SCHEMA, make_livox_decoder
    store = get_typestore(Stores.ROS1_NOETIC if wire == 'ros1' else Stores.ROS2_HUMBLE)
    name = namespace + '/msg/CustomMsg'
    schema = _CUSTOM_SCHEMA.format(namespace=namespace)
    if wire == 'ros1':
        schema = schema.replace('MSG: std_msgs/Header\n', 'MSG: std_msgs/Header\nuint32 seq\n')
    store.register(get_types_from_msg(schema, name))
    types = store.types
    p = types[namespace + '/msg/CustomPoint']
    header_args = [types['builtin_interfaces/msg/Time'](1234567890, 123456789), frame_id]
    if wire == 'ros1': header_args.insert(0, 19)
    msg = types[name](types['std_msgs/msg/Header'](*header_args),
        999999999999, 5, 3, np.zeros(3, dtype=np.uint8), [
            p(100000000, 1., 2., 3., 37, 0x10, 2),
            p(20000000, -1.25, 2.5, 3., 255, 0, 0),
            p(30000000, 4., 2., 3., 20, 0x20, 3),
            p(40000000, float('nan'), 2., 3., 0, 0, 0),
            p(50000000, 1., float('inf'), 3., 3, 0, 0),
            p(60000000, 9., 8., 7., 3, 0, 0)])  # beyond point_num
    raw = (store.serialize_ros1(msg, name) if wire == 'ros1' else
           store.serialize_cdr(msg, name, little_endian=wire == 'cdr_le'))
    decoded = (store.deserialize_ros1(raw, name) if wire == 'ros1' else
               store.deserialize_cdr(raw, name))
    decoder = make_livox_decoder(NS(typestore=store, is2=wire != 'ros1'), name)
    assert decoder is not None
    stamp, *arrays = decoder(raw)
    assert stamp == 1234567890 + 123456789 * 1e-9
    expected = read_livox(decoded)
    for actual, reference in zip(arrays, expected):
        if reference is None: assert actual is None
        else: np.testing.assert_array_equal(actual, reference)
    with pytest.raises(ValueError, match='truncated'):
        decoder(raw[:-1])
    msg.point_num = 7
    bad = (store.serialize_ros1(msg, name) if wire == 'ros1' else
           store.serialize_cdr(msg, name, little_endian=wire == 'cdr_le'))
    with pytest.raises(ValueError, match='point_num'): decoder(bad)
    msg.point_num = 0; msg.points = []
    empty = (store.serialize_ros1(msg, name) if wire == 'ros1' else
             store.serialize_cdr(msg, name, little_endian=wire == 'cdr_le'))
    assert decoder(empty)[1].shape == (0, 3)


def test_bulk_decoder_falls_back_for_unknown_schema_or_cdr():
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    from genz_lio.datasets.livox import _CUSTOM_SCHEMA, make_livox_decoder, read_livox_serialized
    assert make_livox_decoder(NS(), 'livox_ros_driver/msg/CustomMsg') is None
    store = get_typestore(Stores.ROS2_HUMBLE)
    name = 'livox_ros_driver/msg/CustomMsg'
    changed = _CUSTOM_SCHEMA.format(namespace='livox_ros_driver').replace('uint8 line', 'uint16 line')
    store.register(get_types_from_msg(changed, name))
    assert make_livox_decoder(NS(typestore=store, is2=True), name) is None
    assert read_livox_serialized(b'\x00\x07\x00\x00', is_cdr=True) is None
