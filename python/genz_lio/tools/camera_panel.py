"""One reusable OpenGL texture for an inline ImGui camera preview."""
import ctypes as C
import ctypes.util
import numpy as np


class _Vec2(C.Structure):
    _fields_ = [('x', C.c_float), ('y', C.c_float)]


class CameraTexture:
    """Created, updated, drawn and destroyed on the visualizer's GL thread."""

    def __init__(self, gui):
        self.gui = gui
        self.texture = C.c_uint(0)
        self.shape = None
        self._native_image = None
        if not hasattr(gui, 'Image'):
            # Polyscope 2.5 (the last Python 3.8 wheel) exports ImGui::Image
            # but omits its Python binding. Resolve that exact public C++
            # signature on the Linux wheel; never guess pointers or offsets.
            import polyscope_bindings
            self._binding = C.CDLL(polyscope_bindings.__file__)
            try:
                self._native_image = getattr(self._binding, '_ZN5ImGui5ImageEyRK6ImVec2S2_S2_')
            except AttributeError as error:
                raise RuntimeError('Camera preview needs a Polyscope build with ImGui Image support') from error
            self._native_image.argtypes = [C.c_ulonglong, C.POINTER(_Vec2), C.POINTER(_Vec2), C.POINTER(_Vec2)]
            self._native_image.restype = None
        library = C.util.find_library('GL')
        if not library:
            raise RuntimeError('Camera preview requires OpenGL')
        self.gl = C.CDLL(library)
        signatures = {
            'glGetIntegerv': [C.c_uint, C.POINTER(C.c_int)],
            'glGenTextures': [C.c_int, C.POINTER(C.c_uint)],
            'glDeleteTextures': [C.c_int, C.POINTER(C.c_uint)],
            'glBindTexture': [C.c_uint, C.c_uint],
            'glBindBuffer': [C.c_uint, C.c_uint],
            'glPixelStorei': [C.c_uint, C.c_int],
            'glTexParameteri': [C.c_uint, C.c_uint, C.c_int],
            'glTexImage2D': [C.c_uint, C.c_int, C.c_int, C.c_int, C.c_int,
                             C.c_int, C.c_uint, C.c_uint, C.c_void_p],
            'glTexSubImage2D': [C.c_uint, C.c_int, C.c_int, C.c_int, C.c_int,
                                C.c_int, C.c_uint, C.c_uint, C.c_void_p],
        }
        for name, args in signatures.items():
            function = getattr(self.gl, name)
            function.argtypes, function.restype = args, None
        self.gl.glGenTextures(1, C.byref(self.texture))
        if not self.texture.value:
            raise RuntimeError('No current OpenGL context for camera preview')

    def update(self, pixels):
        pixels = np.ascontiguousarray(pixels, dtype=np.uint8)
        if pixels.ndim != 3 or pixels.shape[2] != 3:
            raise ValueError('camera preview must be RGB8')
        gl = self.gl
        old_texture, old_alignment, old_buffer, old_row, old_skip_rows, old_skip_pixels = (C.c_int() for _ in range(6))
        for parameter, value in [(0x8069, old_texture), (0x0CF5, old_alignment),
                                  (0x88EF, old_buffer), (0x0CF2, old_row),
                                  (0x0CF3, old_skip_rows), (0x0CF4, old_skip_pixels)]:
            gl.glGetIntegerv(parameter, C.byref(value))
        try:
            gl.glBindBuffer(0x88EC, 0)  # GL_PIXEL_UNPACK_BUFFER
            gl.glBindTexture(0x0DE1, self.texture.value)
            gl.glPixelStorei(0x0CF5, 1)  # GL_UNPACK_ALIGNMENT
            gl.glPixelStorei(0x0CF2, 0)  # GL_UNPACK_ROW_LENGTH
            gl.glPixelStorei(0x0CF3, 0)  # GL_UNPACK_SKIP_ROWS
            gl.glPixelStorei(0x0CF4, 0)  # GL_UNPACK_SKIP_PIXELS
            height, width = pixels.shape[:2]
            if self.shape != (height, width):
                for parameter, value in [(0x2801, 0x2601), (0x2800, 0x2601),
                                          (0x2802, 0x812F), (0x2803, 0x812F)]:
                    gl.glTexParameteri(0x0DE1, parameter, value)
                gl.glTexImage2D(0x0DE1, 0, 0x8051, width, height, 0,
                                 0x1907, 0x1401, pixels.ctypes.data)
                self.shape = (height, width)
            else:
                gl.glTexSubImage2D(0x0DE1, 0, 0, 0, width, height,
                                    0x1907, 0x1401, pixels.ctypes.data)
        finally:
            gl.glPixelStorei(0x0CF3, old_skip_rows.value)
            gl.glPixelStorei(0x0CF4, old_skip_pixels.value)
            gl.glPixelStorei(0x0CF2, old_row.value)
            gl.glPixelStorei(0x0CF5, old_alignment.value)
            gl.glBindBuffer(0x88EC, old_buffer.value)
            gl.glBindTexture(0x0DE1, old_texture.value)

    def draw(self, size):
        if self._native_image is not None:
            self._native_image(self.texture.value, C.byref(_Vec2(*size)),
                               C.byref(_Vec2(0., 0.)), C.byref(_Vec2(1., 1.)))
        else:
            reference = self.gui.ImTextureRef(self.texture.value) if hasattr(self.gui, 'ImTextureRef') else self.texture.value
            self.gui.Image(reference, size)

    def close(self):
        if self.texture.value:
            self.gl.glDeleteTextures(1, C.byref(self.texture))
            self.texture.value = 0
