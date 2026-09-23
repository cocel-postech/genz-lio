"""Visualization must use sensor calibration without changing the saved pose."""
from types import SimpleNamespace as NS
import sys
import numpy as np
import pytest

from genz_lio import Config, GenZLIO
from genz_lio.datasets import Frame
from genz_lio.pipeline import Pipeline
from genz_lio.tools.visualizer import Visualizer
from genz_lio.genz_lio_pybind import _VisualizationMap
from genz_lio.genz_lio_pybind import _downsample_display_map


def test_display_sampling_preserves_original_points_and_negative_cells():
    points=np.array([[.01,0,0],[.09,0,0],[.21,0,0],[-.01,0,0],[-.09,0,0]])
    before=points.copy();sampled=_downsample_display_map(points,.2)
    np.testing.assert_array_equal(sampled,points[[0,2,3]])
    assert len(set(map(tuple,np.floor(sampled/.2))))==len(sampled)
    np.testing.assert_array_equal(points,before)
    full=_downsample_display_map(points,0.)
    np.testing.assert_array_equal(full,points);full[:]=100
    np.testing.assert_array_equal(points,before)
    assert _downsample_display_map(np.empty((0,3)),.2).shape==(0,3)
    for spacing in [-1.,float('nan'),float('inf')]:
        with pytest.raises(ValueError):_downsample_display_map(points,spacing)
    with pytest.raises(ValueError):_downsample_display_map(np.ones((3,2)),.2)


def test_display_density_change_restores_cached_map_and_prunes(monkeypatch):
    class DensityPolyscope(FakePolyscope):
        def __init__(self):super().__init__();self.points={}
        def register_point_cloud(self,name,points,*args,**kwargs):
            self.points[name]=points.copy()
            geometry=super().register_point_cloud(name,points,**kwargs)
            geometry.update_point_positions=lambda p:self.points.update({name:p.copy()})
            return geometry
        def remove_point_cloud(self,name):self.points.pop(name,None)
    ps=DensityPolyscope();monkeypatch.setitem(sys.modules,'polyscope',ps)
    view=Visualizer(autoplay=True,map_spacing=.2)
    points=np.array([[.01,0,0],[.09,0,0],[.21,0,0]])
    empty=np.empty((0,3));pose=np.eye(4);key=(0,0,0)
    def draw(packet):
        view.update(empty,pose,map_updates=packet,planar_points=points,non_planar_points=points)
    draw(dict(reset=True,tiles={key:points},removed=[]))
    assert view.displayed_points==2 and view.retained_map_points==3
    np.testing.assert_array_equal(ps.points['planar_points'],points)
    np.testing.assert_array_equal(ps.points['non_planar_points'],points)
    view._map_spacing=0.;view._map_density_dirty=True
    draw(dict(reset=False,tiles={},removed=[]))
    assert view.displayed_points==3
    np.testing.assert_array_equal(ps.points['map_tile_0_0_0'],points)
    draw(dict(reset=False,tiles={},removed=[key]))
    assert view.displayed_points==view.retained_map_points==0 and not view._map_raw_tiles
    draw(dict(reset=True,tiles={key:points},removed=[]))
    draw(dict(reset=True,tiles={},removed=[]))
    assert not view._map_raw_tiles and view.displayed_points==0


@pytest.mark.parametrize('voxel_size',[.5,1.,2.])
@pytest.mark.parametrize('map_range',[-1.,4.])
@pytest.mark.parametrize('hybrid',[False,True])
@pytest.mark.parametrize('spacing',[0.,.2])
def test_incremental_tiles_equal_full_map_and_leave_estimation_unchanged(voxel_size,map_range,hybrid,spacing):
    from test_bindings import _room_scan
    config=Config();config.max_threads=1
    config.mapping.voxel_size=voxel_size;config.mapping.map_range=map_range
    config.mapping.max_points_size=32;config.mapping.max_mature_points_size=16
    config.hybrid_metric.enable=hybrid;config.hybrid_metric.max_points_per_voxel=8
    engine=GenZLIO(config);reference=GenZLIO(config);view=_VisualizationMap(engine)
    tiles={}
    def check():
        delta=view.update(spacing)
        if delta['reset']:tiles.clear()
        for key in delta['removed']:tiles.pop(key,None)
        tiles.update(delta['tiles'])
        actual=np.concatenate(list(tiles.values())) if tiles else np.empty((0,3))
        expected=engine.visualization_snapshot
        assert delta['retained_map_points']==len(expected['map'])
        assert delta['sampled_in_core'] and delta['map_spacing']==spacing
        expected_map=expected['map']
        if spacing and len(expected_map):
            cells=np.column_stack((np.floor(expected_map/voxel_size),np.floor(expected_map/spacing)))
            _,indices=np.unique(cells,axis=0,return_index=True)
            expected_map=expected_map[indices]
        def ordered(points):return points[np.lexsort((points[:,2],points[:,1],points[:,0]))]
        np.testing.assert_array_equal(ordered(actual),ordered(expected_map))
        np.testing.assert_array_equal(delta['planar'],expected['planar'])
        np.testing.assert_array_equal(delta['non_planar'],expected['non_planar'])
    check()
    for index in range(30):
        begin=index*.1
        travelled=0. if begin<1. else min(begin-1.,1.)**2*.5+max(begin-2.,0.)
        points,times=_room_scan(2.+travelled)
        acceleration=1. if 1.<=begin<2. else 0.
        imu=np.array([[begin+k*.005,acceleration,0,9.81,0,0,0] for k in range(21)])
        a=engine.register_scan(points,begin,begin+.1,imu,timestamps=times)
        b=reference.register_scan(points,begin,begin+.1,imu,timestamps=times)
        assert a.valid==b.valid
        np.testing.assert_array_equal(a.pose,b.pose)
        np.testing.assert_array_equal(engine.covariance,reference.covariance)
        # Deliberately consume less often than map updates to exercise coalescing
        # and prune/recreate cases between display frames.
        if index%3==0:check()
    check()
    engine.reset();check();assert not tiles


@pytest.mark.parametrize('sign', [-1., 1.])
def test_dense_display_tile_split_preserves_all_points_and_reset(sign):
    config=Config();config.max_threads=1
    config.preprocess.blind_min=.01
    config.mapping.voxel_size=2.;config.mapping.max_layer=0
    config.mapping.max_points_size=128;config.mapping.max_mature_points_size=128
    config.mapping.down_sample_size=.25;config.adaptive_voxelization.enable=False
    axis=sign*np.linspace(2.,27.,36)
    dense=np.stack(np.meshgrid(axis,axis,axis,indexing='ij'),axis=-1).reshape(-1,3)
    engine=GenZLIO(config);view=_VisualizationMap(engine);tiles={}
    observed=[]
    for index in range(5):
        points=dense[:1000] if index<2 else dense
        begin=index*.1
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        engine.register_scan(points,begin,begin+.1,imu,timestamps=np.linspace(0,.1,len(points)))
        delta=view.update()
        if delta['reset']:tiles.clear()
        for key in delta['removed']:tiles.pop(key,None)
        tiles.update(delta['tiles'])
        actual=np.concatenate(list(tiles.values())) if tiles else np.empty((0,3))
        expected=engine.visualization_snapshot['map']
        ordered=lambda p:p[np.lexsort((p[:,2],p[:,1],p[:,0]))]
        np.testing.assert_array_equal(ordered(actual),ordered(expected))
        observed.append((len(actual),len(tiles)))
    assert observed[1][0]<32768 and observed[1][1]==1
    assert observed[-1][0]>32768 and observed[-1][1]>1
    assert not view.update()['tiles']
    engine.reset();delta=view.update()
    assert delta['reset'] and not delta['tiles'] and delta['retained_map_points']==0


def test_incremental_map_single_consumer_ownership_and_restart():
    from test_bindings import _room_scan
    config=Config();config.max_threads=1;engine=GenZLIO(config)
    for index in range(5):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        engine.register_scan(points,begin,begin+.1,imu,timestamps=times)
    view=_VisualizationMap(engine)
    with pytest.raises(RuntimeError,match='already attached'):_VisualizationMap(engine)
    first=view.update();assert first['reset'] and first['tiles']
    for points in first['tiles'].values():points[:]=np.nan
    unchanged=view.update();assert not unchanged['reset'] and not unchanged['tiles'] and not unchanged['removed']
    del view
    restarted=_VisualizationMap(engine)
    fresh=restarted.update()
    assert fresh['reset'] and all(np.isfinite(points).all() for points in fresh['tiles'].values())
    sampled=restarted.update(.2)
    assert sampled['reset'] and sampled['map_spacing']==.2
    restored=restarted.update(0.)
    assert restored['reset'] and restored['retained_map_points']==fresh['retained_map_points']
    assert restored['tiles'].keys()==fresh['tiles'].keys()
    for key,points in fresh['tiles'].items():np.testing.assert_array_equal(restored['tiles'][key],points)
    # keep_alive must retain the engine while the display cache still refers to it.
    del engine
    assert not restarted.update()['tiles']


def test_display_applies_current_extrinsic_and_gravity_without_changing_output():
    body=np.eye(4);body[:3,3]=[1,2,3]
    extrinsic=np.eye(4);extrinsic[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];extrinsic[:3,3]=[.2,.3,.4]
    gravity=np.array([[1,0,0],[0,0,-1],[0,1,0.]])
    points=np.array([[1,0,0.],[0,2,0.]])
    expected_body=body.copy();expected_points=points.copy()
    frame=Frame(np.vstack((points, np.zeros((3,3)))),0.,.1,np.array([[.1,0,0,9.81,0,0,0.]]))
    result=NS(valid=True,pose=body,timestamp=.1,deskewed=points,processing_time_ms=1.,plane_matches=1,point_matches=1,leaf_size=.3,setpoint=1000,voxelized_points=2,scale_indicator=8.)
    # Deliberately keep the configuration extrinsic at identity: rendering must
    # use the filter's current transform, including online calibration changes.
    pipeline=Pipeline([frame],Config());display=[]
    snapshot=dict(planar=np.array([[1.,2.,3.]]),non_planar=np.array([[4.,5.,6.]]),map=np.array([[7.,8.,9.]]))
    pipeline.odometry=NS(visualization_snapshot=snapshot,lidar_pose=body@extrinsic,gravity_alignment=gravity,register_scan=lambda *a,**kw:result)
    pipeline._visualizer=NS(update=lambda p,t,**kw:display.append((p.copy(),t.copy(),kw)),finish=lambda:None,close=lambda:None)
    pipeline.run()
    np.testing.assert_allclose(display[0][0],[[1.2,-3.4,3.3],[-.8,-3.4,2.3]],atol=1e-15)
    np.testing.assert_allclose(display[0][1][:3,3],[1,-3,2])
    for key, value in [("planar_points","planar"),("non_planar_points","non_planar")]:
        np.testing.assert_allclose(display[0][2][key],snapshot[value]@gravity.T)
    shown=display[0][2]
    assert shown['map_points'] is snapshot['map']
    np.testing.assert_allclose(shown['map_points']@shown['map_transform'][:3,:3].T,
                               snapshot['map']@gravity.T)
    assert display[0][2]["infos"]["raw_points"] == 2
    assert display[0][2]["infos"]["input_points"] == 5
    np.testing.assert_array_equal(pipeline.poses[0],expected_body)
    np.testing.assert_array_equal(result.pose,expected_body)
    np.testing.assert_array_equal(result.deskewed,expected_points)


def test_lidar_pose_binding_includes_nonidentity_calibration():
    from test_bindings import _room_scan
    config=Config();config.max_threads=1;config.mapping.extrinsic_est_en=False
    config.mapping.extrinsic_t=np.array([.2,-.3,.4])
    config.mapping.extrinsic_r=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    engine=GenZLIO(config);result=None
    for index in range(5):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        result=engine.register_scan(points,begin,begin+.1,imu,timestamps=times)
    assert result.valid
    calibration=np.eye(4);calibration[:3,:3]=config.mapping.extrinsic_r;calibration[:3,3]=config.mapping.extrinsic_t
    np.testing.assert_allclose(engine.lidar_pose,result.pose@calibration,atol=1e-12)
    copy=engine.lidar_pose;copy[:]=0
    assert not np.all(engine.lidar_pose==0)


class FakePolyscope:
    def __init__(self, fail=False):
        self.imgui=NS();self.fail=fail;self.ticks=0;self.callbacks=[];self.closed=False
    def init(self):
        if self.fail:raise RuntimeError('no display')
    def __getattr__(self,name):
        if name.startswith('set_') or name=='reset_camera_to_home_view':return lambda *a,**kw:None
        if name=='get_view_center':return lambda:np.zeros(3)
        if name=='get_view_camera_parameters':return lambda:NS(get_position=lambda:np.array([0.,0.,10.]),get_intrinsics=lambda:None)
        if name in ('CameraExtrinsics','CameraParameters'):return lambda **kw:NS(**kw)
        raise AttributeError(name)
    def register_point_cloud(self,*a,**kw):
        return NS(set_radius=lambda *a,**kw:None,set_enabled=lambda *a:None,set_transform=lambda *a:None,set_transparency=lambda *a:None)
    def register_curve_network(self,*a,**kw):return self.register_point_cloud(*a,**kw)
    def frame_tick(self):
        self.ticks+=1
        if self.callbacks:self.callbacks.pop(0)()
        assert self.ticks<20,'paused UI did not respond to control'
    def window_requests_close(self):return self.closed
    def shutdown(self):self.closed=True


def test_tile_geometry_reuse_removal_reset_and_body_view(monkeypatch):
    class TilePolyscope(FakePolyscope):
        def __init__(self):
            super().__init__();self.clouds={};self.positions={};self.transforms={};self.reused=0
        def register_point_cloud(self,name,points,*args,**kwargs):
            geometry=super().register_point_cloud(name,points,**kwargs)
            self.clouds[name]=geometry;self.positions[name]=points.copy()
            def update(new):
                assert len(new)==len(self.positions[name]);self.positions[name]=new.copy();self.reused+=1
            geometry.update_point_positions=update
            geometry.set_transform=lambda matrix:self.transforms.update({name:matrix.copy()})
            return geometry
        def remove_point_cloud(self,name):
            del self.clouds[name];del self.positions[name]
    ps=TilePolyscope();monkeypatch.setitem(sys.modules,'polyscope',ps)
    view=Visualizer(autoplay=True);pose=np.eye(4);points=np.ones((2,3));empty=np.empty((0,3))
    a=(-1,0,0);b=(0,0,0)
    def update(packet):
        assert view.update(empty,pose,map_updates=packet,non_planar_points=empty,planar_points=empty)
    update(dict(reset=True,tiles={a:points,b:points+1},removed=[]))
    assert view.displayed_points==4
    saved=view._map_tiles[b]
    pose[:3,3]=[2,3,4];view._global_view=False
    update(dict(reset=False,tiles={b:points+2},removed=[a]))
    assert view.displayed_points==2 and a not in view._map_tiles
    assert view._map_tiles[b] is saved and ps.reused==1
    np.testing.assert_array_equal(ps.positions[view._tile_name(b)],points+2)
    np.testing.assert_array_equal(ps.transforms[view._tile_name(b)],np.linalg.inv(pose))
    update(dict(reset=True,tiles={a:points},removed=[]))
    assert b not in view._map_tiles and view.displayed_points==2
    update(dict(reset=False,tiles={},removed=[a]))
    assert view.displayed_points==0 and not list(view._map_geometries())
    assert len(ps.positions['non_planar_points'])==0
    view.close()


def test_visualization_request_cannot_silently_skip_window_failure(monkeypatch):
    monkeypatch.setitem(sys.modules,'polyscope',FakePolyscope(fail=True))
    with pytest.raises(RuntimeError,match='could not create'):Visualizer()


def test_visualization_request_reports_missing_dependency(monkeypatch):
    monkeypatch.setitem(sys.modules,'polyscope',None)
    with pytest.raises(ImportError,match='Polyscope'):Visualizer()


def test_pause_step_play_and_window_close(monkeypatch):
    ps=FakePolyscope();monkeypatch.setitem(sys.modules,'polyscope',ps)
    view=Visualizer();points=np.zeros((2,3));pose=np.eye(4)
    # Two idle GUI ticks must not register or append another scan.
    def paused():assert len(view._trajectory)==1
    ps.callbacks=[paused,paused,lambda:setattr(view,'_step_requested',True)]
    assert view.update(points,pose);assert ps.ticks==3
    assert not view._step_requested and not view._play_mode
    ps.callbacks=[lambda:setattr(view,'_play_mode',True)]
    assert view.update(points,pose);assert len(view._trajectory)==2
    assert view.update(points,pose);assert ps.ticks==5
    ps.closed=True
    assert not view.update(points,pose)
    view.close();assert not view.available


def test_snapshot_is_owned_and_does_not_change_filter():
    from test_bindings import _room_scan
    config=Config();config.max_threads=1;engine=GenZLIO(config)
    for index in range(5):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        result=engine.register_scan(points,begin,begin+.1,imu,timestamps=times)
    assert result.valid
    pose=engine.lidar_pose.copy();covariance=engine.covariance.copy();size=engine.map_size
    snapshot=engine.visualization_snapshot
    assert len(snapshot['planar'])==result.plane_matches
    assert len(snapshot['non_planar'])==result.point_matches
    assert len(snapshot['map'])>0
    for values in snapshot.values():
        assert values.shape[1]==3 and np.isfinite(values).all()
        values[:]=np.nan
    assert np.isfinite(engine.visualization_snapshot['map']).all()
    np.testing.assert_array_equal(engine.lidar_pose,pose)
    np.testing.assert_array_equal(engine.covariance,covariance)
    assert engine.map_size==size


def test_map_transform_composes_with_global_and_body_views(monkeypatch):
    ps=FakePolyscope();monkeypatch.setitem(sys.modules,'polyscope',ps)
    view=Visualizer(autoplay=True)
    transforms={}
    def geometry(name):
        return NS(set_transform=lambda matrix:transforms.update({name:matrix.copy()}),
                  set_enabled=lambda enabled:None)
    view._geometries={name:geometry(name) for name in ('local_map','planar_points','trajectory')}
    view._map_transform=np.eye(4)
    view._map_transform[:3,:3]=[[1,0,0],[0,0,-1],[0,1,0]]
    view._last_pose=np.eye(4);view._last_pose[:3,3]=[2,3,4]
    points=np.array([[7.,8.,9.]])
    for global_view in (True,False):
        view._global_view=global_view;view._apply_view()
        expected=np.eye(4) if global_view else np.linalg.inv(view._last_pose)
        np.testing.assert_allclose(transforms['local_map'],expected@view._map_transform)
        np.testing.assert_allclose(transforms['planar_points'],expected)
        aligned=points@view._map_transform[:3,:3].T
        np.testing.assert_allclose(points@transforms['local_map'][:3,:3].T+transforms['local_map'][:3,3],
                                   aligned@expected[:3,:3].T+expected[:3,3])
    view.close()


def test_snapshot_survives_reset_and_engine_destruction():
    from test_bindings import _room_scan
    config=Config();config.max_threads=1;engine=GenZLIO(config)
    empty=engine.visualization_snapshot
    assert all(value.shape==(0,3) for value in empty.values())
    for index in range(5):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        engine.register_scan(points,begin,begin+.1,imu,timestamps=times)
    snapshot=engine.visualization_snapshot
    saved={k:v.copy() for k,v in snapshot.items()}
    engine.reset()
    assert all(value.shape==(0,3) for value in engine.visualization_snapshot.values())
    del engine
    for key,values in snapshot.items():
        np.testing.assert_array_equal(values,saved[key])


def test_quit_keeps_partial_trajectory_and_closes(monkeypatch,tmp_path):
    from test_bindings import _room_scan
    frames=[]
    for index in range(8):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        frames.append(Frame(points,begin,begin+.1,imu,timestamps=times))
    config=Config();config.max_threads=1;pipeline=Pipeline(frames,config);closed=[]
    pipeline._visualizer=NS(update=lambda *a,**kw:False,finish=lambda:None,close=lambda:closed.append(True))
    summary=pipeline.run()
    assert summary.stopped_early and summary.frames==1 and closed==[True]
    paths=pipeline.save(tmp_path)
    assert len(np.loadtxt(paths[0],ndmin=2))==1


@pytest.mark.parametrize('autoplay',[False,True])
def test_worker_pipeline_preserves_poses_and_complete_map(autoplay):
    from test_bindings import _room_scan
    import threading
    main=threading.get_ident();reader_threads=[];display=[];tiles={}
    frames=[]
    for index in range(10):
        begin=index*.1;points,times=_room_scan(2.)
        imu=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
        frames.append(Frame(points,begin,begin+.1,imu,timestamps=times))
    config=Config();config.max_threads=1
    reference=Pipeline(frames,config);reference.run()
    def reader():
        for frame in frames:
            reader_threads.append(threading.get_ident());yield frame
    pipeline=Pipeline(reader(),config)
    pipeline._map_view=_VisualizationMap(pipeline.odometry)
    def draw(points,pose,**kwargs):
        assert threading.get_ident()==main
        delta=kwargs['map_updates']
        if delta['reset']:tiles.clear()
        for key in delta['removed']:tiles.pop(key,None)
        tiles.update(delta['tiles']);display.append(pose.copy())
    pipeline._visualizer=NS(_play_mode=autoplay,update=draw,finish=lambda:None,close=lambda:None)
    pipeline.run()
    assert len(set(reader_threads))==1 and reader_threads[0]!=main
    assert len(display)==reference.summary.frames
    assert pipeline.summary.skipped==reference.summary.skipped
    np.testing.assert_array_equal(pipeline.poses,reference.poses)
    np.testing.assert_array_equal(pipeline.odometry.covariance,reference.odometry.covariance)
    def ordered(points):return points[np.lexsort((points[:,2],points[:,1],points[:,0]))]
    np.testing.assert_array_equal(ordered(np.concatenate(list(tiles.values()))),
                                  ordered(reference.odometry.visualization_snapshot['map']))


@pytest.mark.parametrize('failure',['quit','render','worker'])
def test_prefetch_is_bounded_and_worker_is_joined(failure):
    import threading
    main=threading.get_ident();second=threading.Event();release=threading.Event()
    calls=[];closed=[];reader_closed=[]
    frame=Frame(np.ones((2,3)),0.,.1,np.ones((1,7)))
    def reader():
        owner=threading.get_ident()
        try:
            yield from [frame]*5
        finally:
            assert threading.get_ident()==owner!=main
            reader_closed.append(True)
    pipeline=Pipeline(reader(),Config())
    def register(*args,**kwargs):
        assert threading.get_ident()!=main
        calls.append(len(calls)+1)
        if len(calls)==2:
            second.set()
            assert release.wait(5),'renderer failed to release worker'
            if failure=='worker':raise RuntimeError('worker failed')
        return NS(valid=True,pose=np.eye(4),timestamp=len(calls)*.1,
                  deskewed=np.ones((2,3)),processing_time_ms=1.,plane_matches=1,
                  point_matches=1,leaf_size=.3,setpoint=1000,voxelized_points=2,scale_indicator=8.)
    pipeline.odometry=NS(register_scan=register,lidar_pose=np.eye(4),gravity_alignment=np.eye(3))
    pipeline._map_view=NS(update=lambda spacing=0.:dict(reset=False,tiles={},removed=[],planar=np.empty((0,3)),non_planar=np.empty((0,3))))
    def draw(*args,**kwargs):
        assert threading.get_ident()==main and second.wait(5)
        assert calls==[1,2]  # No unbounded producer queue while the GUI is busy.
        release.set()
        if failure=='render':raise RuntimeError('render failed')
        return failure!='quit'
    def close():
        assert not any(t.name.startswith('genz-lio') for t in threading.enumerate())
        closed.append(True)
    pipeline._visualizer=NS(_play_mode=True,update=draw,finish=lambda:None,close=close)
    if failure=='quit':
        pipeline.run();assert pipeline.summary.stopped_early
    else:
        with pytest.raises(RuntimeError,match=failure+' failed'):pipeline.run()
    assert len(pipeline.poses)==1 and calls==[1,2] and closed==[True] and reader_closed==[True]


def test_pixel_sizes_and_scale_bar_have_physical_reference():
    points=np.array([[0.,0.,-10.],[0.,0.,-20.]])
    radii=Visualizer._pixel_radii(points,np.zeros(3),np.array([0.,0.,-1.]),90.,1000,3.)
    np.testing.assert_allclose(radii,[.03,.06])
    # Project the two diameters back to the same three-pixel size.
    np.testing.assert_allclose(2*radii/np.array([10.,20.])*500,[3.,3.])
    assert Visualizer._scale_bar_position(0.,30.)==0
    assert Visualizer._scale_bar_position(15.,30.)==20
    assert Visualizer._scale_bar_position(60.,30.)==40


def test_coordinate_frames_preserve_global_origin_and_follow_body_rotation():
    body=np.eye(4);body[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];body[:3,3]=[4,5,6]
    original=body.copy()
    fixed=Visualizer._frame_axes(np.eye(4),2.5)
    moving=Visualizer._frame_axes(body,1.5)
    for axis in range(3):
        np.testing.assert_array_equal(fixed[axis][0],np.zeros(3))
        np.testing.assert_allclose(fixed[axis][1],np.eye(3)[axis]*2.5)
        np.testing.assert_array_equal(moving[axis][0],body[:3,3])
        np.testing.assert_allclose(moving[axis][1]-moving[axis][0],body[:3,axis]*1.5)
        # In local view the moving body frame becomes identity; the global frame
        # remains anchored in the world and therefore moves relative to the robot.
        inverse=np.linalg.inv(body)
        local=moving[axis]@inverse[:3,:3].T+inverse[:3,3]
        np.testing.assert_allclose(local[0],np.zeros(3),atol=1e-14)
        np.testing.assert_allclose(local[1],np.eye(3)[axis]*1.5)
    np.testing.assert_array_equal(body,original)


def test_processing_fps_uses_mean_time_and_ignores_invalid_measurements(monkeypatch):
    ps=FakePolyscope();monkeypatch.setitem(sys.modules,'polyscope',ps)
    view=Visualizer(autoplay=True)
    for ms in [10.,30.,0.,float('nan')]:
        view.update(np.zeros((2,3)),np.eye(4),infos={'processing_time_ms':ms})
    assert view._vis_infos['fps']==50.
    assert view._processed_frames==2
