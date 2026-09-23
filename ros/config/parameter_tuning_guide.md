# GenZ-LIO parameter guide

Use one complete YAML from [default/](default/) for a new platform, or select
an [experiment configuration](../README.md#benchmark-configurations) for a known
sequence. ROS 1, ROS 2, and Python read the same estimator parameters. Defaults
are starting points, not a promise of the paper's sequence-specific results.

## Before tuning

1. Verify `common.lidar_topic`, `common.imu_topic`, message types, scan geometry,
   and continuous, overlapping sensor timestamps.
2. Set `mapping.extrinsic_r` and `mapping.extrinsic_t` so that
   `p_imu = extrinsic_r * p_lidar + extrinsic_t`. Rotation is a row-major 3×3
   matrix and translation is in meters. Keep `extrinsic_est_en: false` when
   using a known calibration.
3. Check per-point timestamps and deskewing. Azimuth-based timing reconstruction
   for supported spinning sensors needs correct `scan_line` and `scan_rate`;
   it is not a substitute for valid timing on every sensor type.
4. Verify the IMU and LiDAR noise conventions below. Do not copy a noise-density
   specification into a covariance parameter without conversion.
5. Establish a baseline with the intended build and complete input. Score with
   the dataset's ground-truth convention before comparing parameter changes.

`common.time_offset` is a fixed offset added to LiDAR timestamps, in seconds.
The wrappers do not perform automatic clock alignment. Provide synchronized
timestamps or a calibrated fixed offset. Neither approach repairs missing samples.

## Three spatial resolutions

| Parameter | Role |
|---|---|
| `mapping.down_sample_size` | Initial adaptive voxel size; fixed scan resolution when adaptation is disabled |
| Adaptive voxel size | Per-scan resolution controlled by `adaptive_voxelization` |
| `mapping.voxel_size` | Root-cell edge length of the retained octree map |

With adaptive voxelization enabled, the map-insertion cloud uses half the current
adaptive voxel size, and estimation points are downsampled from that cloud at
the full adaptive size. With adaptation disabled, both use `down_sample_size`.
These resolutions are distinct from map root size and rendered point size.
`Map pts (cm)` in the visualizer changes only appearance.

## Scale-aware adaptive voxelization

The scale indicator is a moving mean of per-scan median ranges from a downsampled
probe. It determines a target number of points between `min_points` and
`max_points`; a PD controller adjusts the estimation voxel size to track it.
The target is not a hard bound on the actual point count.

| Parameter under `adaptive_voxelization` | Starting value | Effect |
|---|---|---|
| `enable` | `true` | Enable adaptive scan downsampling |
| `window_size` | `5` | Number of scan medians averaged; larger smooths and delays response |
| `scale_threshold` | `30.0` m | Scale at which the target reaches `max_points` |
| `setpoint_exponent` | `2` | Shape of target interpolation |
| `min_points` / `max_points` | `1000` / `4000` | Target counts at the confined/open ends |
| `p_gain_min` / `p_gain_max` | `5e-6` / `5e-5` | Scheduled proportional gain bounds |
| `d_gain_min` / `d_gain_max` | `5e-8` / `5e-7` | Scheduled derivative gain bounds |
| `error_sensitivity` | `0.1` | Error normalization for gain scheduling |
| `error_rate_sensitivity` | `0.2` | Error-rate normalization for gain scheduling |

For normalized scale `t` between 0 and 1, interpolation uses `1 - (1 - t)^p`.
Increasing `p` raises the target at intermediate scales and smooths its approach
to the maximum for `p > 1`. If voxel size oscillates, inspect the scheduled gains
and point-count error. Change gain bounds together, then replay a full transition
sequence to check both tracking and computation time.

## Map structure and correspondence budgets

| Parameter | Meaning and tradeoff |
|---|---|
| `mapping.voxel_size` | Root edge length in meters; changes geometry grouping and point-to-point discretization covariance |
| `mapping.max_layer` | Maximum subdivision level; level `l` has cell edge `voxel_size / 2^l` |
| `mapping.layer_point_size` | Point-count threshold for plane initialization at each level |
| `mapping.planar_threshold` | Maximum smallest covariance eigenvalue, in m², for a planar cell; lower is stricter |
| `mapping.max_points_size` | Limit used for plane covariance updates and retained support |
| `mapping.max_mature_points_size` | Threshold controlling when mature plane updates stop |
| `hybrid_metric.max_points_per_voxel` | Root-level candidate budget for point-to-point matching |
| `hybrid_metric.reduction_ratio` | Reduction of that candidate budget as cells subdivide |

Tune `voxel_size`, `max_layer`, `max_points_per_voxel`, and `reduction_ratio`
together. A smaller cell or deeper tree changes support density; aggressively
reducing child budgets can leave too few candidates even if the root budget
looks generous. More retained points can improve support but increase memory
and search cost. Check the planar/non-planar match counts and full-sequence error,
not just how the map looks.

### Map retention

`mapping.map_range <= 0` retains all root voxels. A positive value is a **radius**
in meters around the current estimated IMU position: root voxels whose centers
are at or beyond that distance are removed. It is not a map diameter, LiDAR
maximum-range parameter, or display setting.

Reducing this radius can lower retained memory and search/update work, but it
also removes previous geometry and can change later correspondences and revisit
accuracy. A LiDAR's maximum measurement range alone does not establish a safe
map-retention radius. Keep each benchmark's configured value when reproducing
results; validate both accuracy and wall time before changing it.

## Hybrid metric and gates

`hybrid_metric.enable: true` combines point-to-plane and point-to-point updates.
`hybrid_metric.lambda_po` scales point-to-point covariance; reducing it increases
the relative weight of those constraints. Start at `0.05` and assess changes on
both planar and unstructured scenes.

`mapping.sigma_num` gates point-to-plane residuals, while
`hybrid_metric.sigma_num` gates point-to-point residuals. Smaller values reject
more matches and can leave an update with insufficient support.

The adaptive gate lives **inside `hybrid_metric`**:

```yaml
hybrid_metric:
    adaptive_threshold:
        initial_threshold: 0.5
        max_range_motion: 100.0
```

`initial_threshold` sets the initial/minimum threshold scale.
`max_range_motion` converts rotational corrections to displacement at a chosen
range. The gate responds to accepted pose corrections and point range; rejected
updates do not replace its correction history.

## Noise and iteration count

| Parameter under `noise_model` | Implementation convention |
|---|---|
| `ranging_cov` | Range-noise scale in meters; squared when forming point covariance |
| `angle_cov` | Angular-noise scale in degrees; converted to angular variance |
| `acc_cov`, `gyr_cov` | Values placed in the IMU process-noise covariance |
| `b_acc_cov`, `b_gyr_cov` | Bias process-noise covariance values |

The `_cov` suffix is not a uniform declaration of units across all six fields.
Consult the sensor calibration and the estimator's discretization rather than
assuming every value is a datasheet noise density. Treat changes as model
changes and score complete sequences.

`mapping.max_iteration` bounds iterated filter updates per scan. The supplied
benchmark protocol uses `4`. Raising it costs computation and does not guarantee
better convergence.

## Benchmark tuning protocol

For comparable tuning trials, keep the following fixed:

- `preprocess.point_filter_num: 1` and `mapping.max_iteration: 4`.
- Adaptive voxelization enabled, window `5`, scale threshold `30.0`, exponent `2`,
  and point targets `1000` / `4000`.
- Hybrid metric enabled.
- `mapping.down_sample_size` at the sequence's selected `0.25` or `0.4`.
- `mapping.map_range: -1.0` except the configured GEODE Waterways retention limits.

First search map root sizes `0.5`, `1.0`, or `2.0` and maximum layers `3` or `4`,
along with plane thresholds, gates, candidate/support budgets, adaptive gate
parameters, and noise parameters. Prefer keeping `lambda_po` and the adaptive
controller gains/sensitivities at the values above until evidence justifies a
change. Re-evaluate related sequences when tuning a shared YAML.

## Threads, transport, and visualization

`runtime.max_threads: 0` chooses the detected physical-core count as the OpenMP
worker limit; a positive number gives an explicit limit. It does not pin work
to P-cores or select a particular CPU affinity. More workers are not necessarily
faster. Record CPU, compiler, Eigen version, and build options with results.

The filter uses a local observation-product reduction policy documented in
[the vendored patch notes](../../cpp/genz_lio/3rdparty/PATCHES.md). This addresses
cache-dependent reduction order, not all floating-point differences across
architectures. Avoid fast-math for reproducibility and recheck accuracy after
dependency or compiler changes.

ROS 2 `common.qos_reliability` must match the publisher and `common.qos_depth`
sets a finite history for both LiDAR and IMU. Buffering affects memory and
message delivery under load. See [ROS transport and diagnostics](../README.md#ros-2-transport-setup).
Keep `publish.dense_publish_en: false` to display voxelized registered scans.
The Python `--visualize-map-spacing` option samples display geometry only;
`mapping.map_range` changes estimator retention.

## Evaluate a change

1. Save the YAML and build revision, and use the same input and ground truth.
2. Check complete input consumption and valid trajectory coverage. Missing IMU
   samples can change accuracy even when the number of output poses looks right.
3. Compare the appropriate trajectory/control-point/endpoint error against the
   baseline. Comparing ROS with offline C++ output checks interface agreement;
   it is not a separate ground-truth accuracy metric.
4. Measure whole-run wall time and peak memory with visualization enabled if it
   will be used. The displayed FPS excludes input, transport, and rendering.
5. Replay other sequences sharing the YAML, then retain or reject the change.

Useful symptoms include rotation-dependent drift (calibration/timing), unstable
point counts (adaptive control), and rising frame time with map size (retention,
search work, or rendering). Diagnose input completeness and the slow stage before
changing parameters to compensate for a transport or display problem.
