# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Ignazio Vizzo, Tiziano Guadagnino, Benedikt Mersch,
# Modified by Daehan Lee, Hyungtae Lim, and Soohee Han, 2024
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""GenZ-ICP-style Polyscope controls with read-only GenZ-LIO map snapshots."""
import ctypes
import numpy as np

BACKGROUND_COLOR = [0., 0., 0.]
NON_PLANAR_COLOR = [239. / 255., 41. / 255., 41. / 255.]
PLANAR_COLOR = [10. / 255., 57. / 255., 237. / 255.]
LOCAL_MAP_COLOR = [.85, .85, .85]
TRAJECTORY_COLOR = [0., 1., 1.]


class Visualizer:
    """Starts paused; SPACE plays, N steps, G changes view, Q saves and exits.

    Map points come from the estimator's retained voxel map. Rendering neither
    trims historical regions on a timer nor modifies the estimator's map.
    """

    def __init__(self, autoplay=False, map_spacing=0., show_camera=False):
        if not np.isfinite(map_spacing) or map_spacing < 0:
            raise ValueError("map spacing must be finite and nonnegative")
        try:
            import polyscope as ps
        except ImportError as error:
            raise ImportError("visualization requires Polyscope: pip install 'genz-lio[viz]'") from error
        self._ps = ps
        self._gui = ps.imgui
        self.available = False
        self._play_mode = autoplay
        self._autoplay = autoplay
        self._step_requested = False
        self._quit_requested = False
        self._global_view = True
        self._camera_initialized = False
        self._camera_view = 'topdown'
        self._trajectory = []
        self._last_pose = np.eye(4)
        self._map_transform = np.eye(4)
        self._map_tiles = {}
        self._map_tile_counts = {}
        self._map_raw_tiles = {}
        self._map_spacing = float(map_spacing)
        self._map_density_dirty = False
        self.retained_map_points = 0
        self._last_map_display_transform = None
        self._vis_infos = {}
        self._selected_pose = ""
        self._show_camera = bool(show_camera)
        self._camera_texture = None
        self._camera_image = None
        self._camera_timestamp = None
        self._camera_error = None
        self._background_color = list(BACKGROUND_COLOR)
        # Semantic points use pixel diameters. Map points retain the approved
        # shaded gray quads with a 0.06 m world-space point radius.
        self._sizes = dict(non_planar_points=3., planar_points=3., local_map=.06)
        self._path_width = 1.
        self._display_clouds = {}
        self._enabled = dict(non_planar_points=True, planar_points=True, local_map=True)
        self._map_transparency = .2
        self._geometries = {}
        self._frame_clouds = {}
        self._frames_enabled = dict(global_frame=True, body_frame=True)
        self._processing_ms_total = 0.
        self._processed_frames = 0
        self._displayed_frames = 0
        # Replacing growing map geometry leaves large free CPU buffers in
        # glibc's arenas. Return those unused pages periodically; this does not
        # remove map points or touch live allocations. Other allocators simply
        # omit this optional maintenance operation.
        self._trim_heap = None
        try:
            trim = ctypes.CDLL(None).malloc_trim
            trim.argtypes = [ctypes.c_size_t]
            trim.restype = ctypes.c_int
            self._trim_heap = trim
        except (AttributeError, OSError):
            pass
        self.displayed_points = 0
        try:
            ps.set_program_name("GenZ-LIO Visualizer")
            ps.set_allow_headless_backends(False)
            ps.init()
            ps.set_ground_plane_mode("none")
            ps.set_up_dir("z_up")
            ps.set_background_color(BACKGROUND_COLOR)
            ps.set_verbosity(0)
            ps.set_frame_tick_limit_fps_mode("block_to_hit_target")
            ps.set_user_callback(self._main_gui_callback)
            ps.set_build_default_gui_panels(False)
        except Exception as error:
            raise RuntimeError("Polyscope could not create the GenZ-LIO window; check the display and OpenGL") from error
        self.available = True

    def update(self, points, pose, *, planar_points=None, non_planar_points=None,
               map_points=None, map_transform=None, map_updates=None, infos=None):
        """Pose and scan points use the gravity-aligned world frame.

        map_transform optionally maps retained map coordinates into that frame.
        Applying it on the geometry avoids rotating millions of points on the
        CPU and allocating another full-map array on every frame.
        """
        self._last_pose = np.array(pose, copy=True)
        self._map_transform = np.eye(4) if map_transform is None else np.array(map_transform, copy=True)
        self._trajectory.append(pose[:3, 3].copy())
        self._vis_infos = dict(infos or {})
        processing_ms = float(self._vis_infos.get('processing_time_ms', 0.))
        if np.isfinite(processing_ms) and processing_ms > 0:
            self._processing_ms_total += processing_ms
            self._processed_frames += 1
        self._vis_infos['fps'] = (1000. * self._processed_frames / self._processing_ms_total
                                  if self._processing_ms_total > 0 else 0.)
        empty = np.empty((0, 3))
        clouds = dict(non_planar_points=empty if non_planar_points is None else non_planar_points,
                      planar_points=empty if planar_points is None else planar_points)
        if map_updates is None:
            self._clear_map_tiles()
            raw_map = points if map_points is None else map_points
            self.retained_map_points = len(raw_map)
            clouds['local_map'] = self._sample_map(raw_map)
        else:
            if 'local_map' in self._geometries:
                self._ps.remove_point_cloud('local_map')
                del self._geometries['local_map']
            self._update_map_tiles(map_updates)
        colors = dict(non_planar_points=NON_PLANAR_COLOR, planar_points=PLANAR_COLOR,
                      local_map=LOCAL_MAP_COLOR)
        self._display_clouds = clouds
        for name, cloud in clouds.items():
            style = {} if name == "local_map" else {"material": "flat"}
            geometry = self._ps.register_point_cloud(name, cloud, color=colors[name],
                                                      point_render_mode="quad", **style)
            geometry.set_radius(self._sizes[name] if name == "local_map" else .005, relative=False)
            geometry.set_enabled(self._enabled[name])
            geometry.set_transparency(self._map_transparency if name == "local_map" else 1.)
            self._geometries[name] = geometry
        self.displayed_points = (len(clouds['local_map']) if map_updates is None
                                 else sum(self._map_tile_counts.values()))
        self._register_trajectory()
        self._register_frames()
        self._apply_view()
        if not self._camera_initialized:
            self._set_camera_view()
            self._camera_initialized = True
        # Always service the GUI once, even during playback. Pausing never
        # advances the dataset; a step releases exactly this frame's wait.
        while not self._quit_requested:
            self._ps.frame_tick()
            if self._ps.window_requests_close():
                self._quit_requested = True
            if self._play_mode or self._step_requested:
                self._step_requested = False
                break
        self._displayed_frames += 1
        if (self._trim_heap is not None and self.displayed_points >= 100000
                and self._displayed_frames % 100 == 0):
            self._trim_heap(0)
        return not self._quit_requested

    @staticmethod
    def _tile_name(key):
        return 'map_tile_' + '_'.join(str(value) for value in key)

    @property
    def map_spacing(self):
        return self._map_spacing

    def _clear_map_tiles(self):
        for key in self._map_tiles:
            self._ps.remove_point_cloud(self._tile_name(key))
        self._map_tiles.clear()
        self._map_tile_counts.clear()
        self._map_raw_tiles.clear()
        self.retained_map_points = 0

    def _sample_map(self, points):
        if self._map_spacing == 0.:
            return points
        from ..genz_lio_pybind import _downsample_display_map
        return _downsample_display_map(points, self._map_spacing)

    def _update_map_tiles(self, updates):
        prepared = updates.get('sampled_in_core', False)
        if updates['reset']:
            self._clear_map_tiles()
        for key in updates['removed']:
            if key in self._map_tiles:
                self._ps.remove_point_cloud(self._tile_name(key))
                del self._map_tiles[key]
                del self._map_tile_counts[key]
            self._map_raw_tiles.pop(key, None)
        if prepared:
            # The worker refreshes its cache on a spacing change. Keep using
            # this owned packet while at most one older packet is in flight.
            self._map_raw_tiles.clear()
            self.retained_map_points = updates['retained_map_points']
            changed = updates['tiles']
        else:
            self._map_raw_tiles.update(updates['tiles'])
            self.retained_map_points = sum(len(points) for points in self._map_raw_tiles.values())
            changed = self._map_raw_tiles if self._map_density_dirty else updates['tiles']
        self._map_density_dirty = False
        transform = (np.eye(4) if self._global_view else np.linalg.inv(self._last_pose)) @ self._map_transform
        for key, raw_points in changed.items():
            points = raw_points if prepared else self._sample_map(raw_points)
            if key in self._map_tiles and self._map_tile_counts[key] == len(points):
                geometry = self._map_tiles[key]
                geometry.update_point_positions(points)
            else:
                geometry = self._ps.register_point_cloud(self._tile_name(key), points,
                    color=LOCAL_MAP_COLOR, point_render_mode='quad')
                geometry.set_radius(self._sizes['local_map'], relative=False)
                geometry.set_transparency(self._map_transparency)
                geometry.set_enabled(self._enabled['local_map'])
                self._map_tiles[key] = geometry
            geometry.set_transform(transform)
            self._map_tile_counts[key] = len(points)

    def _map_geometries(self):
        if 'local_map' in self._geometries:
            yield self._geometries['local_map']
        yield from self._map_tiles.values()

    def _register_trajectory(self):
        nodes = np.asarray(self._trajectory)
        edges = np.column_stack((np.arange(len(nodes) - 1), np.arange(1, len(nodes))))
        geometry = self._ps.register_curve_network("trajectory", nodes, edges,
                                                   color=TRAJECTORY_COLOR, material="flat")
        geometry.set_radius(.005, relative=False)
        geometry.set_transparency(1.)
        self._geometries['trajectory'] = geometry

    @staticmethod
    def _frame_axes(pose, length):
        """XYZ axis segments in the display world, without arrowheads."""
        axes = []
        for axis in range(3):
            tip = np.eye(3)[axis] * length
            local = np.array([np.zeros(3), tip])
            axes.append(local @ pose[:3, :3].T + pose[:3, 3])
        return axes

    def _register_frames(self):
        edges = np.array([[0, 1]])
        for frame, pose, length in [('global_frame', np.eye(4), 2.5),
                                     ('body_frame', self._last_pose, 1.5)]:
            for axis, nodes, color in zip('xyz', self._frame_axes(pose, length), np.eye(3)):
                name = frame + '_' + axis
                geometry = self._ps.register_curve_network(name, nodes, edges,
                                                            color=color, material='flat')
                geometry.set_radius(.015, relative=False)
                geometry.set_transparency(1.)
                geometry.set_enabled(self._frames_enabled[frame])
                self._geometries[name] = geometry
                self._frame_clouds[name] = (nodes, edges)

    def _set_camera_view(self):
        self._ps.reset_camera_to_home_view()
        camera = self._ps.get_view_camera_parameters()
        center = np.asarray(self._ps.get_view_center())
        distance = max(float(np.linalg.norm(np.asarray(camera.get_position()) - center)), 10.)
        if self._camera_view == 'topdown':
            # Looking along -Z needs an explicit screen-up direction.
            root, look, up = center + [0., 0., distance], [0., 0., -1.], [0., 1., 0.]
        elif self._camera_view == 'side':
            # A horizontal side view with the vertical Z axis pointing up.
            root, look, up = center + [0., -distance, 0.], [0., 1., 0.], [0., 0., 1.]
        else:
            direction = np.ones(3) / np.sqrt(3.)
            root, look = center + distance * direction, -direction
            up = np.array([-1., -1., 2.]) / np.sqrt(6.)
        extrinsics = self._ps.CameraExtrinsics(root=root, look_dir=look, up_dir=up)
        self._ps.set_view_camera_parameters(self._ps.CameraParameters(
            intrinsics=camera.get_intrinsics(), extrinsics=extrinsics))

    @staticmethod
    def _pixel_radii(points, camera_position, look_direction, fov_degrees, height, width):
        """Convert a pixel diameter to a world radius at each point's depth."""
        depth = (points - camera_position) @ look_direction
        return np.maximum(depth, 1.e-6) * np.tan(np.deg2rad(fov_degrees) * .5) * width / max(height, 1)

    def _update_screen_sizes(self):
        if not self._geometries:
            return
        camera = self._ps.get_view_camera_parameters()
        position = np.asarray(camera.get_position())
        direction = np.asarray(camera.get_look_dir())
        height = self._ps.get_buffer_size()[1]
        fov = camera.get_fov_vertical_deg()
        transform = np.eye(4) if self._global_view else np.linalg.inv(self._last_pose)
        def radii(points, width):
            viewed = points @ transform[:3, :3].T + transform[:3, 3]
            return self._pixel_radii(viewed, position, direction, fov, height, width)
        for name in ('planar_points', 'non_planar_points'):
            points = self._display_clouds[name]
            if len(points):
                geometry = self._geometries[name]
                geometry.add_scalar_quantity('pixel_radius', radii(points, self._sizes[name]), enabled=False)
                geometry.set_point_radius_quantity('pixel_radius', autoscale=False)
        nodes = np.asarray(self._trajectory)
        geometry = self._geometries['trajectory']
        radius = radii(nodes, self._path_width)
        geometry.add_scalar_quantity('pixel_radius', radius, defined_on='nodes', enabled=False)
        geometry.set_node_radius_quantity('pixel_radius', autoscale=False)
        if len(nodes) > 1:
            geometry.add_scalar_quantity('edge_pixel_radius', (radius[:-1] + radius[1:]) * .5,
                                         defined_on='edges', enabled=False)
            geometry.set_edge_radius_quantity('edge_pixel_radius', autoscale=False)

        for name, (nodes, edges) in self._frame_clouds.items():
            geometry = self._geometries[name]
            radius = radii(nodes, 2.5)
            geometry.add_scalar_quantity('pixel_radius', radius, defined_on='nodes', enabled=False)
            geometry.set_node_radius_quantity('pixel_radius', autoscale=False)
            geometry.add_scalar_quantity('edge_pixel_radius', radius[edges].mean(axis=1),
                                         defined_on='edges', enabled=False)
            geometry.set_edge_radius_quantity('edge_pixel_radius', autoscale=False)

    @staticmethod
    def _scale_bar_position(scale, threshold, width=40):
        fraction = np.clip(scale / threshold, 0., 1.) if threshold > 0 else 0.
        return int(round(float(fraction) * width))

    def _information_panel(self):
        gui = self._gui
        if not gui.TreeNodeEx('GenZ-LIO information', gui.ImGuiTreeNodeFlags_DefaultOpen):
            return
        info = self._vis_infos
        gui.TextColored((1., 0., 0., 1.), f"# of non-planar points: {int(info.get('point_matches', 0))}")
        gui.SameLine()
        gui.TextColored((.32, .32, 1., 1.), f"# of planar points: {int(info.get('plane_matches', 0))}")
        scale = float(info.get('scale_indicator', 0.))
        threshold = float(info.get('scale_threshold', 30.))
        header_start = gui.GetCursorScreenPos()
        gui.TextUnformatted('Confined <----- ')
        gui.SameLine(0., 0.)
        gui.TextColored((0., .8, 0., 1.), f'Scale indicator: {scale:.2f}')
        gui.SameLine(0., 0.)
        gui.TextUnformatted(' -----> Open')
        # Match the actual rendered header width, including proportional fonts.
        header_end = gui.GetItemRectMax()[0]
        self._draw_scale_bar(scale, threshold, header_end - header_start[0])
        gui.TextUnformatted(f"# of target points: {info.get('setpoint', 0):.0f}")
        gui.TextUnformatted(f"Adaptive voxel size: {info.get('leaf_size', 0):.2f} m")
        gui.TextUnformatted(f"# of raw points: {int(info.get('raw_points', 0))}")
        if gui.IsItemHovered():
            gui.BeginTooltip()
            gui.TextUnformatted('Valid deskewed points before voxel downsampling.')
            gui.TextUnformatted(f"Input frame points: {int(info.get('input_points', 0))}")
            gui.EndTooltip()
        gui.TextUnformatted(f"# of voxelized points: {int(info.get('voxelized_points', 0))}")
        gui.TextUnformatted(f"Processing time: {info.get('processing_time_ms', 0):.2f} ms")
        gui.TextUnformatted(f"FPS: {info.get('fps', 0):.2f}")
        if gui.IsItemHovered():
            gui.BeginTooltip()
            gui.TextUnformatted('Mean odometry processing FPS; excludes reading, rendering and pauses.')
            gui.EndTooltip()
        gui.TextUnformatted(f"Distance traveled: {info.get('distance_traveled', 0):.2f} m")
        if 'Status' in info:
            gui.TextUnformatted(info['Status'])
        if not self._play_mode and self._global_view:
            gui.TextUnformatted('Selected Pose: ' + self._selected_pose)
        gui.TreePop()

    def _draw_scale_bar(self, scale, threshold, width):
        """Draw actual [---[]---] glyphs, without inserted space characters."""
        gui = self._gui
        x, y = gui.GetCursorScreenPos()
        left_width = gui.CalcTextSize('[')[0]
        right_width = gui.CalcTextSize(']')[0]
        marker_width = gui.CalcTextSize('[]')[0]
        track_width = max(0., width - left_width - right_width - marker_width)
        dash_count = max(1, round(track_width / max(gui.CalcTextSize('-')[0], 1.)))
        # Distribute only the sub-character width remainder over the hyphens
        # so the last ] still aligns with Open. Do not fill glyph-side bearings
        # with lines: the font's visible gaps between [-, -- and -] are intended.
        advance = track_width / dash_count
        marker = self._scale_bar_position(scale, threshold, dash_count)
        draw = gui.GetWindowDrawList()
        color = gui.GetColorU32(gui.ImGuiCol_Text)
        green = gui.GetColorU32((0., 1., 0., 1.))
        draw.AddText((x, y), color, '[')
        for index in range(dash_count):
            offset = marker_width if index >= marker else 0.
            draw.AddText((x + left_width + index * advance + offset, y), color, '-')
        draw.AddText((x + left_width + marker * advance, y), green, '[]')
        draw.AddText((x + width - right_width, y), color, ']')
        gui.Dummy((width, gui.GetTextLineHeight()))

    def _apply_view(self):
        transform = np.eye(4) if self._global_view else np.linalg.inv(self._last_pose)
        for name, geometry in self._geometries.items():
            geometry.set_transform(transform @ self._map_transform if name == 'local_map' else transform)
            if name == 'trajectory':
                geometry.set_enabled(self._global_view)
        map_transform = transform @ self._map_transform
        if not np.array_equal(map_transform, self._last_map_display_transform):
            for geometry in self._map_tiles.values():
                geometry.set_transform(map_transform)
            self._last_map_display_transform = map_transform.copy()

    def _pressed(self, label, key):
        return self._gui.Button(label) or self._gui.IsKeyPressed(getattr(self._gui, 'ImGuiKey_' + key))

    def set_camera_image(self, pixels, timestamp=None):
        """Replace the preview on the GUI thread, without retaining image history."""
        if pixels is self._camera_image and timestamp == self._camera_timestamp:
            return
        self._camera_image, self._camera_timestamp = pixels, timestamp
        if pixels is None or self._camera_error:
            return
        try:
            if self._camera_texture is None:
                from .camera_panel import CameraTexture
                self._camera_texture = CameraTexture(self._gui)
            self._camera_texture.update(pixels)
        except (RuntimeError, OSError, AttributeError) as error:
            import warnings
            self._camera_error = str(error)
            warnings.warn(self._camera_error, RuntimeWarning)

    def _camera_panel(self):
        if not self._show_camera:
            return
        gui = self._gui
        gui.Separator()
        width = max(40., gui.GetContentRegionAvail()[0])
        ratio = (self._camera_image.shape[0] / self._camera_image.shape[1]
                 if self._camera_image is not None else 9. / 16.)
        # Size the box to the image instead of shrinking a wide image into a
        # fixed-height box. Explicit padding makes the available width known.
        padding = 4.
        height = (width - 2 * padding) * ratio + 2 * padding + 2.
        gui.PushStyleVar(gui.ImGuiStyleVar_WindowPadding, (padding, padding))
        visible = gui.BeginChild('camera_preview', (width, height), True,
                                 gui.ImGuiWindowFlags_NoScrollbar)
        try:
            if visible:
                if self._camera_image is None or self._camera_error:
                    gui.TextUnformatted('Camera unavailable' if self._camera_error else 'No camera image')
                else:
                    available = gui.GetContentRegionAvail()
                    h, w = self._camera_image.shape[:2]
                    scale = max(1., available[0]) / w
                    self._camera_texture.draw((w * scale, h * scale))
        finally:
            gui.EndChild()
            gui.PopStyleVar()

    def _main_gui_callback(self):
        gui = self._gui
        if self._pressed(" PAUSE\n[SPACE]" if self._play_mode else " START\n[SPACE]", 'Space'):
            self._play_mode = not self._play_mode
        if not self._play_mode:
            gui.SameLine()
            if self._pressed("NEXT FRAME\n  [N]", 'N'):
                self._step_requested = True
        gui.SameLine()
        if self._pressed('QUIT\n [Q]', 'Q') or gui.IsKeyPressed(gui.ImGuiKey_Escape):
            self._quit_requested = True
        gui.SameLine()
        if self._pressed('LOCAL VIEW\n  [G]' if self._global_view else 'GLOBAL VIEW\n  [G]', 'G'):
            self._global_view = not self._global_view
            self._apply_view()
            self._set_camera_view()
        gui.SameLine()
        next_view = {'topdown': 'side', 'side': 'isometric', 'isometric': 'topdown'}[self._camera_view]
        if self._pressed(next_view.upper() + ' VIEW\n  [C]', 'C'):
            self._camera_view = next_view
            self._set_camera_view()
        gui.Separator()
        self._information_panel()
        gui.Separator()
        point_controls = [('non_planar_points', 'Non-planar pts (px)'),
                          ('planar_points', 'Planar pts (px)'),
                          ('local_map', 'Map pts (cm)')]
        # Reserve room for each label and checkbox instead of using ImGui's
        # default item width, which can clip the right-hand labels.
        label_width = max(gui.CalcTextSize(label)[0] for _, label in point_controls)
        gui.PushItemWidth(max(40., gui.GetContentRegionAvail()[0] - label_width
                              - gui.GetFrameHeight() - 24.))
        for name, label in point_controls:
            units = 100. if name == 'local_map' else 1.
            changed, size = gui.SliderFloat('##' + name, self._sizes[name] * units,
                                           v_min=.1 if name == 'local_map' else 1., v_max=10.)
            if changed:
                self._sizes[name] = size / units
            if changed and name == 'local_map':
                for geometry in self._map_geometries():
                    geometry.set_radius(self._sizes[name], relative=False)
            gui.SameLine()
            changed, self._enabled[name] = gui.Checkbox(label, self._enabled[name])
            if changed:
                if name == 'local_map':
                    for geometry in self._map_geometries():
                        geometry.set_enabled(self._enabled[name])
                elif name in self._geometries:
                    self._geometries[name].set_enabled(self._enabled[name])
        changed, self._map_transparency = gui.SliderFloat('Map opacity', self._map_transparency, v_min=0., v_max=1.)
        if changed:
            for geometry in self._map_geometries():
                geometry.set_transparency(self._map_transparency)
        _, self._path_width = gui.SliderFloat('Path width (px)', self._path_width, v_min=1., v_max=6.)
        for frame, label in [('global_frame', 'Global frame'), ('body_frame', 'Body frame')]:
            changed, self._frames_enabled[frame] = gui.Checkbox(label, self._frames_enabled[frame])
            if changed:
                for axis in 'xyz':
                    name = frame + '_' + axis
                    if name in self._geometries:
                        self._geometries[name].set_enabled(self._frames_enabled[frame])
            if frame == 'global_frame':
                gui.SameLine()
        gui.PopItemWidth()
        self._camera_panel()
        gui.Separator()
        self._update_screen_sizes()
        if gui.GetIO().MouseClicked[0]:
            selected = self._ps.get_selection()
            if selected.structure_name == 'trajectory':
                index = selected.structure_data.get('index', -1) if selected.structure_data.get('element_type') == 'node' else -1
                if 0 <= index < len(self._trajectory):
                    x, y, z = self._trajectory[index]
                    self._selected_pose = f'x: {x:.3f}, y: {y:.3f}, z: {z:.3f}'

    def finish(self):
        """Keep the completed map open for interactive use, unless autoplay."""
        if self._autoplay or self._quit_requested:
            return
        self._vis_infos['Status'] = 'Completed — Q to close and save'
        self._play_mode = False
        while not self._quit_requested and not self._ps.window_requests_close():
            self._ps.frame_tick()

    def close(self):
        if self.available:
            if self._camera_texture is not None:
                self._camera_texture.close()
                self._camera_texture = None
            self._ps.shutdown()
            self.available = False
