"""Bounded, display-only ROS camera frames; never used by the estimator."""
from collections import deque
from io import BytesIO
import warnings

import numpy as np

IMAGE_TYPES = ('sensor_msgs/msg/Image', 'sensor_msgs/Image',
               'sensor_msgs/msg/CompressedImage', 'sensor_msgs/CompressedImage')


def decode_image(message, max_size=(640, 360)):
    """Return a small, contiguous RGB8 preview, respecting ROS row padding."""
    from PIL import Image

    if hasattr(message, 'format'):
        if 'compressedDepth' in message.format:
            raise ValueError('compressed depth is not a color camera image')
        with Image.open(BytesIO(bytes(message.data))) as source:
            source.thumbnail(max_size)
            return np.array(source.convert('RGB'), dtype=np.uint8, copy=True)
    encoding = message.encoding.lower()
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4,
                'mono8': 1, '8uc1': 1, 'mono16': 1, '16uc1': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported camera image encoding: {message.encoding}')
    itemsize = 2 if encoding in ('mono16', '16uc1') else 1
    height, width, step = int(message.height), int(message.width), int(message.step)
    if min(height, width) <= 0 or step < width * channels * itemsize:
        raise ValueError('invalid camera image dimensions or row stride')
    data = memoryview(message.data)
    if data.nbytes < height * step:
        raise ValueError('truncated camera image')
    dtype = np.dtype(('>' if message.is_bigendian else '<') + 'u2') if itemsize == 2 else np.uint8
    pixels = np.ndarray((height, width, channels), dtype=dtype, buffer=data,
                        strides=(step, channels * itemsize, itemsize))
    # Limit conversion and upload work even for high-resolution raw camera topics.
    stride = max(1, int(np.ceil(max(width / max_size[0], height / max_size[1]))))
    pixels = pixels[::stride, ::stride]
    if itemsize == 2:
        pixels = (pixels / 257).astype(np.uint8)
    if channels == 1:
        pixels = np.repeat(pixels, 3, axis=2)
    elif encoding.startswith('bgr'):
        pixels = pixels[:, :, [2, 1, 0]]
    else:
        pixels = pixels[:, :, :3]
    return np.ascontiguousarray(pixels)


class CameraFrames:
    """Latest available image at/before a scan, with no future-image matching."""

    def __init__(self, max_age=.5):
        self.pending = deque(maxlen=8)
        self.max_age = max_age
        self.latest = None
        self.pixels = None
        self.warned = set()

    def push(self, stamp, message):
        if np.isfinite(stamp):
            self.pending.append((stamp, message))

    def sample(self, scan_end):
        eligible = [item for item in self.pending if item[0] <= scan_end]
        self.pending = deque((item for item in self.pending if item[0] > scan_end), maxlen=8)
        newest = max(eligible, key=lambda item: item[0]) if eligible else None
        if newest is not None and (self.latest is None or newest[0] >= self.latest[0]):
            self.latest = newest
            self.pixels = None
            if scan_end - newest[0] <= self.max_age:
                try:
                    self.pixels = decode_image(newest[1])
                except (ValueError, OSError) as error:
                    text = str(error)
                    if text not in self.warned:
                        warnings.warn('Camera preview: ' + text, RuntimeWarning)
                        self.warned.add(text)
            self.latest = (newest[0], None)  # release the full-resolution payload
        if self.latest is None or scan_end - self.latest[0] > self.max_age:
            self.pixels = None
            return None, None
        return self.pixels, self.latest[0]
