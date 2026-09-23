"""The CLI must pass sensor configuration to the bag reader."""
from types import SimpleNamespace as NS
from genz_lio import cli
from genz_lio.pipeline import RunSummary
from pathlib import Path
import pytest
import yaml
from typer.testing import CliRunner


def test_yaml_reader_options_and_explicit_topic_override(tmp_path,monkeypatch):
    base=tmp_path/'base.yaml'; sensor=tmp_path/'sensor.yaml'; bag=tmp_path/'data.bag'
    base.write_text('common:\n  lidar_topic: /base/cloud\n  imu_topic: /base/imu\n  time_offset: 0.125\npreprocess:\n  scan_rate: 5\n')
    sensor.write_text('common:\n  imu_topic: /sensor/imu\n  time_offset: 0.25\n')
    bag.touch(); captured={}
    class Data:
        sequence_id='test'
        def __len__(self):return 0
    def open_data(path,**options):
        captured.update(options);return Data()
    class Pipe:
        def __init__(self,*args,**kwargs):pass
        def run(self,**kwargs):return RunSummary('test')
        def save(self,*args,**kwargs):return []
    monkeypatch.setattr(cli,'open_dataset',open_data);monkeypatch.setattr(cli,'Pipeline',Pipe)
    result = CliRunner().invoke(cli.app, [
        'run', str(bag), '--config', str(base), '--sensor-config', str(sensor),
        '--output', str(tmp_path), '--lidar-topic', '/explicit/cloud', '--max-threads', '1',
    ])
    assert result.exit_code == 0, result.output
    assert captured['lidar_topic']=='/explicit/cloud'
    assert captured['imu_topic']=='/sensor/imu'
    assert captured['time_offset']==.25
    assert captured['preprocess'].scan_rate==5


@pytest.mark.parametrize('sensor_name', ['avia', 'hesai', 'mid360', 'ouster', 'robosense', 'velodyne'])
@pytest.mark.parametrize('cli_variant', ['installed', 'source'])
@pytest.mark.parametrize('override_topics', [False, True])
def test_complete_config_needs_no_sensor_overlay(sensor_name, cli_variant, override_topics, tmp_path, monkeypatch):
    import importlib.util
    import inspect

    cli_under_test = cli
    if cli_variant == 'source':
        source = Path(__file__).resolve().parents[1] / 'genz_lio' / 'cli.py'
        spec = importlib.util.spec_from_file_location('genz_lio.cli_layout_check', source)
        cli_under_test = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cli_under_test)
    path = Path(__file__).resolve().parents[2] / 'ros' / 'config' / 'default' / (sensor_name + '.yaml')
    document = yaml.safe_load(path.read_text())
    bag = tmp_path / 'data.bag'
    bag.touch()
    captured = {}

    class Data:
        sequence_id = 'test'
        def __len__(self):
            return 0

    def open_data(path, **options):
        captured.update(options)
        return Data()

    class Pipe:
        # Keep the installed Pipeline signature: the default CLI invocation
        # must not require the newer optional map-spacing argument.
        def __init__(self, dataset, settings, visualize=False, visualize_autoplay=False):
            captured['settings'] = settings
        def run(self, **kwargs):
            return RunSummary('test')
        def save(self, *args, **kwargs):
            return []

    monkeypatch.setattr(cli_under_test, 'open_dataset', open_data)
    monkeypatch.setattr(cli_under_test, 'Pipeline', Pipe)
    # Exercise relative filesystem paths as well as absolute paths.
    monkeypatch.chdir(path.parent)
    config_path = Path(path.name) if override_topics else path
    options = dict(data=bag, config=config_path, sensor_config=None, output=tmp_path,
                   formats=['tum'], lidar_topic='/override/cloud' if override_topics else None,
                   imu_topic='/override/imu' if override_topics else None, visualize=False,
                   max_threads=None, visualize_autoplay=False, image_topic=None)
    # Older installed CLIs have no display-only map-spacing option.
    if 'visualize_map_spacing' in inspect.signature(cli_under_test.run).parameters:
        options['visualize_map_spacing'] = 0.
    cli_under_test.run(**options)
    assert captured['lidar_topic'] == ('/override/cloud' if override_topics else document['common']['lidar_topic'])
    assert captured['imu_topic'] == ('/override/imu' if override_topics else document['common']['imu_topic'])
    assert captured['settings'].mapping.voxel_size == 1.0
    assert captured['settings'].mapping.planar_threshold == 0.001
    assert captured['settings'].mapping.max_layer == 4
    assert captured['settings'].hybrid_metric.max_points_per_voxel == 128
