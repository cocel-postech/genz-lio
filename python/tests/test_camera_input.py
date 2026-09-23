"""Camera display must not change LiDAR/IMU pairing or retain image history."""
from io import BytesIO
from types import SimpleNamespace as NS
import numpy as np
import pytest
from genz_lio.datasets.image import decode_image, CameraFrames
from genz_lio.datasets.rosbag import RosbagDataset
from genz_lio import PreprocessConfig, LidarType
from test_input_timing import stamp, cloud, imu


def image(encoding='rgb8', big=False):
    if encoding=='mono16':
        raw=np.array([0,65535],dtype='>u2' if big else '<u2').tobytes()+b'pad!'
        return NS(encoding=encoding,width=2,height=1,step=8,data=raw,is_bigendian=big,header=stamp(10))
    return NS(encoding=encoding,width=2,height=1,step=8,
              data=bytes([255,0,0,0,255,0,99,99]),is_bigendian=False,header=stamp(10))


@pytest.mark.parametrize('encoding,first',[('rgb8',[255,0,0]),('bgr8',[0,0,255])])
def test_color_order_and_row_padding(encoding,first):
    pixels=decode_image(image(encoding))
    assert pixels.shape==(1,2,3) and pixels.flags.c_contiguous
    np.testing.assert_array_equal(pixels[0],[first,[0,255,0]])


@pytest.mark.parametrize('big',[False,True])
def test_gray16_byte_order(big):
    np.testing.assert_array_equal(decode_image(image('mono16',big)),[[[0,0,0],[255,255,255]]])


def test_compressed_png_and_bounded_resolution():
    from PIL import Image
    pixels=np.zeros((720,1280,3),dtype=np.uint8);pixels[:,:,0]=255
    data=BytesIO();Image.fromarray(pixels).save(data,format='PNG')
    decoded=decode_image(NS(format='rgb8; png compressed bgr8',data=data.getvalue()))
    assert decoded.shape==(360,640,3)
    np.testing.assert_array_equal(decoded[0,0],[255,0,0])


def test_camera_sync_staleness_and_bounded_storage():
    camera=CameraFrames()
    camera.push(10,image());camera.push(10.2,image('bgr8'))
    first,t=camera.sample(10.1);assert t==10
    again,_=camera.sample(10.15);assert again is first
    second,t=camera.sample(10.25);assert t==10.2
    np.testing.assert_array_equal(second[0,0],[0,0,255])
    assert camera.sample(11)==(None,None)
    for i in range(100):camera.push(20+i,image())
    assert len(camera.pending)==8
    assert camera.sample(15)==(None,None)


def test_malformed_image_does_not_abort_camera_stream():
    camera=CameraFrames();m=image();m.data=b'';camera.push(10,m)
    with pytest.warns(RuntimeWarning,match='truncated'):
        assert camera.sample(10.1)[0] is None
    camera.push(10.2,image());assert camera.sample(10.3)[0] is not None


def test_images_do_not_change_scan_imu_bundles():
    events=[('/imu',imu(10.05)),('/camera',image()),('/cloud',cloud(header=10)),
            ('/imu',imu(10.15)),('/camera',image('bgr8')),('/cloud',cloud(header=10.1)),('/imu',imu(10.25))]
    connections=[NS(topic=t,msgtype='sensor_msgs/msg/'+kind) for t,kind in
                 [('/imu','Imu'),('/cloud','PointCloud2'),('/camera','Image')]]
    class Reader:
        def __init__(self,*a):self.connections=connections
        def __enter__(self):return self
        def __exit__(self,*a):pass
        def messages(self,connections):
            selected={c.topic:c for c in connections}
            for topic,msg in events:
                if topic in selected:yield selected[topic],0,msg
        def deserialize(self,raw,*a):return raw
    def dataset(camera):
        d=RosbagDataset.__new__(RosbagDataset);d._reader_factory=Reader;d.path='unused'
        d._length=2
        d.time_offset=0;d.lidar_topic='/cloud';d.imu_topic='/imu';d.image_topic='/camera' if camera else None
        d.preprocess=PreprocessConfig();d.preprocess.lidar_type=LidarType.OUSTER
        return list(d)
    off,on=dataset(False),dataset(True)
    assert len(off)==len(on)==2
    for a,b in zip(off,on):
        assert a.begin_time==b.begin_time and a.end_time==b.end_time
        for key in ['points','imu','timestamps']:
            np.testing.assert_array_equal(getattr(a,key),getattr(b,key))
        assert a.camera_image is None and b.camera_image is not None
