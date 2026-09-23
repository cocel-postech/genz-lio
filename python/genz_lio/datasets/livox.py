"""Decode Livox CustomMsg returns with the same policy as the ROS wrapper."""
import numpy as np


def read_livox(msg):
    if msg.point_num < 0 or msg.point_num > len(msg.points):
        raise ValueError("Livox point_num exceeds the available point array")
    returns = msg.points[:msg.point_num]
    points = np.asarray([(p.x, p.y, p.z) for p in returns], dtype=np.float64).reshape(-1, 3)
    tags = np.fromiter((p.tag for p in returns), dtype=np.uint8, count=msg.point_num)
    times = np.fromiter((p.offset_time for p in returns), dtype=np.float64,
                        count=msg.point_num) * 1e-9
    intensities = np.fromiter((p.reflectivity for p in returns), dtype=np.float32,
                              count=msg.point_num)
    good_return = ((tags & 0x30) == 0x00) | ((tags & 0x30) == 0x10)
    valid = good_return & np.isfinite(points).all(axis=1)
    # CustomMsg offsets are relative to its header, not timebase. The shared
    # C++ scan buffer casts offsets to float and determines their extrema.
    # ROS onLivox does not populate ring indices, so do not add them here.
    return points[valid], times[valid], intensities[valid], None


# Only this exact wire schema is eligible for the vectorized path. Other
# CustomMsg variants continue through rosbags' ordinary deserializer.
_CUSTOM_SCHEMA = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
================================================================================
MSG: {namespace}/CustomPoint
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""


def make_livox_decoder(reader, msgtype):
    """Return a bulk decoder only for a verified ROS1/CDR CustomMsg layout."""
    from rosbags.typesys import get_types_from_msg
    store = getattr(reader, 'typestore', None)
    definitions = getattr(store, 'fielddefs', None) or getattr(store, 'FIELDDEFS', {})
    namespace = msgtype.split('/')[0]
    if namespace not in ('livox_ros_driver', 'livox_ros_driver2'):
        return None
    is_cdr = getattr(reader, 'is2', None)
    if is_cdr is None:
        return None
    schema = _CUSTOM_SCHEMA.format(namespace=namespace)
    if not is_cdr:
        schema = schema.replace('MSG: std_msgs/Header\n', 'MSG: std_msgs/Header\nuint32 seq\n')
    expected = get_types_from_msg(schema, msgtype)
    if any(definitions.get(key) != fields for key, fields in expected.items()):
        return None
    return lambda raw: read_livox_serialized(raw, is_cdr=is_cdr)


def read_livox_serialized(raw, *, is_cdr):
    """Decode a schema-checked packet without constructing one object per point.

    Returns (header stamp, positions, times, intensities, rings). None requests
    the ordinary deserializer for an unsupported CDR representation. Malformed
    supported packets are rejected before NumPy accesses their point buffer.
    """
    import struct
    data = memoryview(raw)
    if is_cdr:
        if len(data) < 4:
            raise ValueError('truncated Livox CDR header')
        representation = bytes(data[:2])
        if representation not in (b'\x00\x00', b'\x00\x01'):
            return None
        endian = '<' if representation == b'\x00\x01' else '>'
        origin = offset = 4
    else:
        endian = '<'
        origin = 0
        offset = 4  # ROS1 Header.seq

    def unpack(fmt):
        nonlocal offset
        size = struct.calcsize(fmt)
        if offset + size > len(data):
            raise ValueError('truncated Livox message')
        value = struct.unpack_from(endian + fmt, data, offset)[0]
        offset += size
        return value

    sec = unpack('i')
    nanosec = unpack('I')
    length = unpack('I')
    if offset + length > len(data) or (is_cdr and (length == 0 or data[offset + length - 1] != 0)):
        raise ValueError('invalid Livox frame_id length')
    offset += length
    if is_cdr:
        offset += (-(offset - origin)) % 8
    unpack('Q')  # timebase is not the scan's header stamp
    point_num = unpack('I')
    unpack('I')  # lidar_id and the three reserved bytes
    count = unpack('I')
    if point_num > count:
        raise ValueError('Livox point_num exceeds the available point array')
    stride = 20 if is_cdr else 19
    required = (count - 1) * stride + 19 if count else 0
    if offset + required > len(data):
        raise ValueError('truncated Livox point array')
    dtype = np.dtype(dict(
        names=['offset_time', 'x', 'y', 'z', 'reflectivity', 'tag', 'line'],
        formats=[endian+'u4', endian+'f4', endian+'f4', endian+'f4', 'u1', 'u1', 'u1'],
        offsets=[0, 4, 8, 12, 16, 17, 18], itemsize=19))
    values = np.ndarray((count,), dtype=dtype, buffer=data, offset=offset,
                        strides=(stride,))[:point_num]
    points = np.column_stack((values['x'], values['y'], values['z'])).astype(np.float64)
    tags = values['tag']
    valid = (((tags & 0x30) == 0x00) | ((tags & 0x30) == 0x10)) & np.isfinite(points).all(axis=1)
    times = values['offset_time'].astype(np.float64) * 1e-9
    intensities = values['reflectivity'].astype(np.float32)
    return (float(sec) + float(nanosec) * 1e-9,
            points[valid], times[valid], intensities[valid], None)
