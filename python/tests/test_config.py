"""Reading and writing the YAML configuration."""
from pathlib import Path

import numpy as np
import pytest
import yaml

import genz_lio
from genz_lio import load_config, save_config


def test_omitted_keys_keep_their_defaults(tmp_path):
    path = tmp_path / "partial.yaml"
    path.write_text("hybrid_metric:\n    lambda_po: 0.2\n")

    config = load_config(path)
    assert config.hybrid_metric.lambda_po == pytest.approx(0.2)
    assert config.adaptive_voxelization.max_points == 4000   # untouched
    assert config.mapping.voxel_size == genz_lio.Config().mapping.voxel_size


def test_sensor_file_layers_over_a_base(tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text("hybrid_metric:\n    lambda_po: 0.07\n")
    sensor = tmp_path / "sensor.yaml"
    sensor.write_text("preprocess:\n    lidar_type: ouster\n    scan_line: 64\n")

    config = load_config(sensor, load_config(base))
    assert config.preprocess.lidar_type == genz_lio.LidarType.OUSTER
    assert config.preprocess.scan_line == 64
    # The sensor file must not disturb tuning it says nothing about.
    assert config.hybrid_metric.lambda_po == pytest.approx(0.07)


def test_unknown_keys_warn_rather_than_fail(tmp_path):
    path = tmp_path / "stale.yaml"
    path.write_text("mapping:\n    fov_degree: 360\nlegacy:\n    a: 1\n")

    with pytest.warns(UserWarning, match="unrecognised keys"):
        config = load_config(path)
    assert config.mapping.voxel_size == genz_lio.Config().mapping.voxel_size   # still usable


def test_bad_values_are_rejected(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("preprocess:\n    lidar_type: sick\n")
    with pytest.raises(ValueError, match="unknown lidar_type"):
        load_config(path)

    path.write_text("mapping:\n    extrinsic_t: [1.0, 2.0]\n")
    with pytest.raises(ValueError, match="expects 3 values"):
        load_config(path)


def test_extrinsic_is_row_major(tmp_path):
    path = tmp_path / "extrinsic.yaml"
    path.write_text(
        "mapping:\n"
        "    extrinsic_t: [0.1, 0.2, 0.3]\n"
        "    extrinsic_r: [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0]\n"
    )
    config = load_config(path)
    assert np.allclose(config.mapping.extrinsic_t, [0.1, 0.2, 0.3])
    assert np.allclose(np.asarray(config.mapping.extrinsic_r).diagonal(), [1.0, -1.0, -1.0])


def test_round_trip(tmp_path):
    config = genz_lio.Config()
    config.hybrid_metric.lambda_po = 0.11
    config.preprocess.lidar_type = genz_lio.LidarType.HESAI
    config.mapping.extrinsic_t = np.array([0.4, 0.5, 0.6])

    path = tmp_path / "out.yaml"
    save_config(config, path)
    assert yaml.safe_load(path.read_text())["preprocess"]["lidar_type"] == "hesai"

    reloaded = load_config(path)
    assert reloaded.hybrid_metric.lambda_po == pytest.approx(0.11)
    assert reloaded.preprocess.lidar_type == genz_lio.LidarType.HESAI
    assert np.allclose(reloaded.mapping.extrinsic_t, [0.4, 0.5, 0.6])


@pytest.mark.parametrize("sensor_name,lidar_type,scan_line", [
    ("avia", "livox", 6), ("hesai", "hesai", 32),
    ("mid360", "livox", 4), ("ouster", "ouster", 64),
    ("robosense", "robosense", 16), ("velodyne", "velodyne", 16),
])
def test_complete_sensor_defaults(sensor_name, lidar_type, scan_line, tmp_path):
    import warnings

    root = Path(__file__).resolve().parents[2] / "ros" / "config"
    path = root / "default" / (sensor_name + ".yaml")
    assert path.is_file()
    document = yaml.safe_load(path.read_text())

    # Every core field must be explicit, not an accidental compiled fallback.
    core_path = tmp_path / "core.yaml"
    save_config(genz_lio.Config(), core_path)
    schema = yaml.safe_load(core_path.read_text())
    schema.update({
        "common": dict.fromkeys(["lidar_topic", "imu_topic",
                                 "time_offset", "qos_reliability", "qos_depth"]),
        "publish": dict.fromkeys(["terminal_status_en", "path_en", "scan_publish_en",
                                  "dense_publish_en", "scan_bodyframe_pub_en",
                                  "scan_semantic_publish_en", "voxel_map_en",
                                  "voxel_map_max_layer"]),
        "pcd_save": dict.fromkeys(["enable", "interval", "directory"]),
    })

    def keys(tree, prefix=""):
        result = set()
        for key, value in tree.items():
            name = prefix + key
            if isinstance(value, dict):
                result.update(keys(value, name + "."))
            else:
                result.add(name)
        return result

    assert keys(document) == keys(schema)
    assert document["preprocess"]["lidar_type"] == lidar_type
    assert document["preprocess"]["scan_line"] == scan_line
    assert document["common"]["qos_depth"] == 1000
    assert document["common"]["qos_reliability"] == "reliable"

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        config = load_config(path)  # One file, no base and no overlay.

    assert config.mapping.voxel_size == pytest.approx(1.0)
    assert config.mapping.max_layer == 4
    assert config.mapping.planar_threshold == pytest.approx(0.001)
    assert config.hybrid_metric.max_points_per_voxel == 128
    assert config.hybrid_metric.lambda_po == pytest.approx(0.05)
    assert config.adaptive_voxelization.scale_threshold == 30.0
    assert config.preprocess.scan_line == scan_line
    assert config.preprocess.blind_max == document["preprocess"]["blind_max"]
    for key, value in document["noise_model"].items():
        assert value > 0
        assert getattr(config.noise_model, key) == pytest.approx(value)

    # The public serializer must preserve every supplied core parameter.
    save_config(config, core_path)
    saved = yaml.safe_load(core_path.read_text())
    for section, values in saved.items():
        assert values == document[section]


def test_single_file_launch_contract():
    import ast
    import xml.etree.ElementTree as ET

    root = Path(__file__).resolve().parents[2] / "ros"
    assert {p.name for p in (root / "launch").iterdir() if p.is_file()} == {
        "odometry.launch", "odometry.launch.py",
    }
    launch = ET.parse(str(root / "launch" / "odometry.launch")).getroot()
    arguments = {arg.attrib["name"]: arg.attrib for arg in launch.findall("arg")}
    assert arguments["config"]["default"] == "default/velodyne.yaml"
    assert set(arguments) == {"config", "rviz", "lidar_topic", "imu_topic"}
    node = next(n for n in launch.findall("node") if n.attrib["type"] == "genz_lio_node")
    params = {p.attrib["name"]: p.attrib["value"] for p in node.findall("param")}
    assert "arg('config').startswith('/')" in params["config_path"]
    assert "find('genz_lio') + '/config/'" in params["config_path"]
    assert "sensor_config_path" not in params

    # ROS2 is also single-file: no silently applied sensor_config_path.
    module = ast.parse((root / "launch" / "odometry.launch.py").read_text())
    strings = [n.s for n in ast.walk(module) if isinstance(n, ast.Str)]
    assert "sensor_config_path" not in strings
    assert "default/velodyne.yaml" in strings
    assert "sensor" not in strings
    assert "config_path" in strings
