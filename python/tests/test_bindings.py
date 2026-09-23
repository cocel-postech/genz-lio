"""The binding surface: defaults, argument checking, and a short run."""
import numpy as np
import pytest

import genz_lio


def test_defaults_match_the_shipped_configuration():
    config = genz_lio.Config()
    av = config.adaptive_voxelization
    assert av.enable
    assert av.window_size == 5                 # N_w
    assert av.scale_threshold == 30.0          # tau_m
    assert av.setpoint_exponent == 2           # p
    assert (av.min_points, av.max_points) == (1000, 4000)
    assert av.error_sensitivity == pytest.approx(0.1)       # lambda_p
    assert av.error_rate_sensitivity == pytest.approx(0.2)  # lambda_d

    assert config.hybrid_metric.enable
    assert config.hybrid_metric.lambda_po == pytest.approx(0.05)
    assert config.mapping.voxel_size == 1.0                 # d_root
    assert config.mapping.max_layer == 4
    assert config.mapping.planar_threshold == pytest.approx(0.001)
    assert config.mapping.max_points_size == 100
    assert config.mapping.max_mature_points_size == 100
    assert config.hybrid_metric.max_points_per_voxel == 128
    assert config.hybrid_metric.reduction_ratio == 4
    assert not hasattr(config.hybrid_metric, "association")  # only Euclidean remains


def test_rejects_malformed_input():
    lio = genz_lio.GenZLIO(genz_lio.Config())
    imu = np.zeros((5, 7))

    with pytest.raises(ValueError, match=r"shape \(N, 3\)"):
        lio.register_scan(np.zeros((10, 4)), 0.0, 0.1, imu)

    with pytest.raises(ValueError, match="one entry per point"):
        lio.register_scan(np.zeros((10, 3)), 0.0, 0.1, imu, timestamps=np.zeros(5))

    with pytest.raises(ValueError, match=r"shape \(N, 7\)"):
        lio.register_scan(np.zeros((10, 3)), 0.0, 0.1, np.zeros((5, 6)))


def _room_scan(x, rings=16, azimuths=360):
    """A shell of returns inside a closed room, seen from `x`."""
    lower, upper = np.array([0.0, -4.0, 0.0]), np.array([20.0, 4.0, 3.0])
    origin = np.array([x, 0.0, 1.5])
    elevation = np.deg2rad(np.linspace(-15, 15, rings))
    azimuth = np.linspace(0, 2 * np.pi, azimuths, endpoint=False)
    E, A = np.meshgrid(elevation, azimuth, indexing="ij")
    directions = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)],
                          -1).reshape(-1, 3)
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(directions > 0, (upper - origin) / directions, (lower - origin) / directions)
    t = np.nanmin(np.where(np.abs(directions) < 1e-9, np.inf, t), axis=1)
    times = np.tile(np.linspace(0, 0.1, azimuths, endpoint=False), rings)
    return directions * t[:, None], times


def test_tracks_a_simulated_run():
    """Stationary while the IMU settles, then a metre per second forward."""
    lio = genz_lio.GenZLIO(genz_lio.Config())
    gravity = 9.81
    last = None

    for index in range(40):
        begin, end = index * 0.1, index * 0.1 + 0.1
        travelled = 0.0 if begin < 1.0 else min(begin - 1.0, 1.0) ** 2 * 0.5 + max(begin - 2.0, 0.0)
        points, times = _room_scan(2.0 + travelled)
        acceleration = 1.0 if 1.0 <= begin < 2.0 else 0.0
        imu = np.array([[begin + k * 0.005, acceleration, 0.0, gravity, 0.0, 0.0, 0.0]
                        for k in range(21)])
        result = lio.register_scan(points, begin, end, imu, timestamps=times)
        if result.valid:
            last = result

    assert last is not None and last.valid
    assert lio.initialized
    assert lio.map_size > 0
    assert np.isfinite(last.pose).all()
    # Roughly 2.5 m of travel; the pipeline reports motion from wherever it
    # started, so only the displacement is meaningful.
    assert 1.5 < last.pose[0, 3] < 3.5
    assert last.plane_matches > 0
    assert last.leaf_size > 0.0
    assert last.timing.update > 0.0


def test_reset_clears_the_map():
    lio = genz_lio.GenZLIO(genz_lio.Config())
    gravity = 9.81
    for index in range(5):
        begin = index * 0.1
        points, times = _room_scan(2.0)
        imu = np.array([[begin + k * 0.005, 0.0, 0.0, gravity, 0.0, 0.0, 0.0] for k in range(21)])
        lio.register_scan(points, begin, begin + 0.1, imu, timestamps=times)

    assert lio.map_size > 0
    lio.reset()
    assert lio.map_size == 0
    assert not lio.initialized


def test_empty_arrays_and_diagnostics():
    config=genz_lio.Config();config.max_threads=1
    lio=genz_lio.GenZLIO(config)
    result=lio.register_scan(np.empty((0,3)),0,.1,np.empty((0,7)),
        timestamps=np.empty(0),intensities=np.empty(0),rings=np.empty(0))
    assert not result.valid and not result.update_rejected
    assert np.isfinite(result.median_range)
    assert lio.covariance.shape==(23,23)


@pytest.mark.parametrize('name', ['intensities','rings'])
def test_optional_arrays_require_exact_lengths(name):
    lio=genz_lio.GenZLIO(genz_lio.Config())
    with pytest.raises(ValueError,match='one entry per point'):
        lio.register_scan(np.ones((3,3)),0,.1,np.empty((0,7)),**{name:np.ones(2)})


def test_nonfinite_points_and_timing_removed_together():
    from genz_lio.genz_lio_pybind import _ScanBuffer
    config=genz_lio.PreprocessConfig();config.lidar_type=genz_lio.LidarType.OUSTER
    b=_ScanBuffer(config)
    p=np.ones((4,3));p[1,0]=np.nan
    # Noncontiguous float64 intensity / uint16 ring data require temporary casts.
    intensity=np.arange(8,dtype=float)[::2]
    rings=np.arange(8,dtype=np.uint16)[::2]
    assert b.push_scan(p,10,np.array([0,.01,np.nan,.09]),intensity,rings)==0
    b.push_imu(np.array([[10.11,0,0,9.81,0,0,0]]))
    points,_,_,_,times,values,indices,_=b.pop()
    assert len(points)==2
    np.testing.assert_array_equal(values,[0,6]);np.testing.assert_array_equal(indices,[0,6])
    np.testing.assert_allclose(times,[0,.09])


@pytest.mark.parametrize('bad', [np.nan,-1,65535,.5])
def test_invalid_ring_values(bad):
    lio=genz_lio.GenZLIO(genz_lio.Config())
    with pytest.raises(ValueError,match='integer indices'):
        lio.register_scan(np.ones((1,3)),0,.1,np.empty((0,7)),rings=np.array([bad]))


def test_latest_core_bias_noise_and_reset_are_used():
    def pair(lio,base):
        points=np.array([[5.,1.,.5]])
        for offset in [0.,.1]:
            begin=base+offset
            samples=np.array([[begin+k*.005,0,0,9.81,0,0,0] for k in range(21)])
            lio.register_scan(points,begin,begin+.1,samples,timestamps=np.array([.08]))
            if offset==0:before=lio.covariance.copy()
        return lio.covariance-before
    for noise in [1e-6,1e-2]:
        config=genz_lio.Config();config.max_threads=1
        config.noise_model.b_gyr_cov=noise;config.noise_model.b_acc_cov=noise
        lio=genz_lio.GenZLIO(config)
        for base in [100.,10.]:
            if base==10:lio.reset()
            delta=pair(lio,base)
            np.testing.assert_allclose(np.diag(delta)[15:21],20*.005**2*noise,rtol=0,atol=1e-12)


def test_future_imu_rejected_before_mutating_filter():
    config=genz_lio.Config();config.max_threads=1
    lio=genz_lio.GenZLIO(config);before=lio.covariance.copy()
    with pytest.raises(ValueError,match='after scan end'):
        lio.register_scan(np.ones((10,3)),10,10.1,
            np.array([[10.11,0,0,9.81,0,0,0]]),timestamps=np.linspace(0,.09,10))
    np.testing.assert_array_equal(lio.covariance,before)
    assert not lio.initialized
