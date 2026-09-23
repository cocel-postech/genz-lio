"""Read-only configurations shipped with the installed package."""
from importlib.metadata import distribution
from pathlib import Path
from typing import List


def _config_root() -> Path:
    # CMake installs YAMLs here even when Python modules use editable redirects.
    return Path(distribution("genz-lio").locate_file("genz_lio/configs"))


def available_configs() -> List[str]:
    root = _config_root()
    return sorted(path.relative_to(root).as_posix() for path in root.rglob("*.yaml"))


def export_config(name: str, output: Path) -> None:
    """Copy a packaged YAML without modifying package files or overwriting a file."""
    if name not in available_configs():
        raise ValueError(f"unknown packaged configuration: {name}; use list-configs")
    contents = (_config_root() / name).read_bytes()
    with Path(output).open("xb") as handle:
        handle.write(contents)
