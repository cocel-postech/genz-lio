"""An installed wheel must carry complete, unchanged, safely exportable YAMLs."""
from pathlib import Path

import pytest
from typer.testing import CliRunner

from genz_lio.cli import app
from genz_lio.packaged_configs import available_configs, export_config


def test_packaged_yamls_match_repository(tmp_path):
    root = Path(__file__).resolve().parents[2] / "ros" / "config"
    expected = sorted(str(p.relative_to(root)) for sub in ("default", "experiments")
                      for p in (root / sub).rglob("*.yaml"))
    assert expected and available_configs() == expected
    for index, name in enumerate(expected):
        target = tmp_path / f"{index}.yaml"
        export_config(name, target)
        assert target.read_bytes() == (root / name).read_bytes()


@pytest.mark.parametrize("name", ["../LICENSE", "/etc/passwd", "missing.yaml"])
def test_export_rejects_unlisted_paths(name, tmp_path):
    with pytest.raises(ValueError):
        export_config(name, tmp_path / "out.yaml")
    assert not (tmp_path / "out.yaml").exists()


def test_cli_lists_exports_and_preserves_existing_file(tmp_path):
    runner = CliRunner()
    result = runner.invoke(app, ["list-configs"])
    assert result.exit_code == 0
    assert "default/velodyne.yaml" in result.stdout
    output = tmp_path / "config.yaml"
    result = runner.invoke(app, ["export-config", "default/velodyne.yaml", str(output)])
    assert result.exit_code == 0
    output.write_text("user calibration\n")
    result = runner.invoke(app, ["export-config", "default/velodyne.yaml", str(output)])
    assert result.exit_code != 0
    assert output.read_text() == "user calibration\n"
