"""Command line entry point.

    genz_lio_pipeline run sequence.bag --config ros/config/default/velodyne.yaml
    genz_lio_pipeline dump-config my_config.yaml
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional

import typer
import yaml
from rich.console import Console
from rich.progress import BarColumn, Progress, TextColumn, TimeRemainingColumn
from rich.table import Table

from .config import load_config, save_config
from .datasets import open_dataset
from .genz_lio_pybind import Config
from .pipeline import Pipeline
from .packaged_configs import available_configs, export_config as copy_packaged_config

app = typer.Typer(add_completion=False, help="GenZ-LIO: LiDAR-inertial odometry.")
console = Console()


@app.command()
def run(
    data: Path = typer.Argument(..., help="rosbag file, rosbag2 directory, Ouster pcap, "
                                          "or a directory of scans with an IMU file"),
    config: Optional[Path] = typer.Option(None, "--config", "-c",
                                          help="Complete YAML path (absolute or relative to current directory); omitted uses compiled core defaults"),
    sensor_config: Optional[Path] = typer.Option(None, "--sensor-config", "-s",
                                                 help="Optional legacy overlay; not needed for config/default/*.yaml"),
    output: Path = typer.Option(Path("results"), "--output", "-o",
                                help="where to write the trajectory"),
    formats: List[str] = typer.Option(["tum"], "--format", "-f", help="tum and/or kitti"),
    lidar_topic: Optional[str] = typer.Option(None, "--lidar-topic",
                                             help="Override YAML LiDAR topic; omitted keeps YAML"),
    imu_topic: Optional[str] = typer.Option(None, "--imu-topic",
                                           help="Override YAML IMU topic; omitted keeps YAML"),
    visualize: bool = typer.Option(False, "--visualize", "-v",
                                   help="draw the map as it is built; needs the 'viz' extra"),
    max_threads: Optional[int] = typer.Option(None, help="0 uses one thread per physical core"),
    visualize_autoplay: bool = typer.Option(False, "--visualize-autoplay",
                                            help="open the visualizer playing and close it at sequence end"),
    visualize_map_spacing: float = typer.Option(0., "--visualize-map-spacing", min=0.,
                                                help="display-only map sampling cell size in meters; 0 keeps all points"),
    image_topic: Optional[str] = typer.Option(None, "--image-topic",
        help="bag Image/CompressedImage topic shown in the visualizer camera panel"),
) -> None:
    """Run the odometry over a recorded sequence."""
    settings = Config()
    if config is not None:
        settings = load_config(config)
    if sensor_config is not None:
        settings = load_config(sensor_config, settings)
    if max_threads is not None:
        settings.max_threads = max_threads

    reader_options = {}
    is_bag = data.suffix == ".bag" or (data.is_dir() and (data / "metadata.yaml").exists())
    if image_topic:
        if not is_bag or not (visualize or visualize_autoplay):
            raise typer.BadParameter("--image-topic requires a rosbag and --visualize or --visualize-autoplay")
        reader_options['image_topic'] = image_topic
    if is_bag:
        for path in (config, sensor_config):
            if path is None:
                continue
            common = (yaml.safe_load(path.read_text()) or {}).get("common") or {}
            for key in ("lidar_topic", "imu_topic", "time_offset"):
                if key in common:
                    reader_options[key] = common[key]
        reader_options["preprocess"] = settings.preprocess
    if lidar_topic:
        reader_options["lidar_topic"] = lidar_topic
    if imu_topic:
        reader_options["imu_topic"] = imu_topic

    try:
        dataset = open_dataset(data, **reader_options)
    except (ImportError, ValueError, FileNotFoundError) as error:
        console.print(f"[red]{error}[/red]")
        raise typer.Exit(1)

    pipeline_options = {}
    if visualize_map_spacing > 0:
        pipeline_options["visualize_map_spacing"] = visualize_map_spacing
    pipeline = Pipeline(dataset, settings, visualize=visualize or visualize_autoplay,
                        visualize_autoplay=visualize_autoplay, **pipeline_options)
    total = len(dataset) if hasattr(dataset, "__len__") else None

    with Progress(TextColumn("[bold blue]{task.description}"), BarColumn(),
                  TextColumn("{task.completed}/{task.total}"), TimeRemainingColumn(),
                  console=console) as progress:
        task = progress.add_task(dataset.sequence_id, total=total)
        summary = pipeline.run(progress=lambda: progress.advance(task))

    written = pipeline.save(output, formats=tuple(formats))
    if summary.stopped_early:
        console.print("Stopped by visualizer; saved the trajectory processed so far.")

    table = Table(title=f"GenZ-LIO — {summary.sequence_id}", show_header=False)
    table.add_row("frames", f"{summary.frames}")
    if summary.skipped:
        table.add_row("skipped", f"{summary.skipped}")
    table.add_row("path length", f"{summary.path_length:.2f} m")
    table.add_row("frame time", f"{summary.mean_frame_time_ms:.1f} ms mean, "
                                f"{summary.p95_frame_time_ms:.1f} ms p95")
    table.add_row("wall time", f"{summary.wall_time:.1f} s")
    console.print(table)
    for path in written:
        console.print(f"wrote [green]{path}[/green]")

    if summary.skipped > summary.frames * 0.1:
        console.print(
            "[yellow]A large share of frames was skipped. That usually means the IMU topic "
            "is wrong, or the scans and the IMU do not overlap in time.[/yellow]"
        )


@app.command("dump-config")
def dump_config(
    output: Path = typer.Argument(..., help="where to write the YAML"),
    base: Optional[Path] = typer.Option(None, "--base", "-b",
                                        help="start from this file instead of the defaults"),
) -> None:
    """Write a core-only template; use export-config for a complete sensor YAML."""
    settings = load_config(base) if base else Config()
    save_config(settings, output)
    console.print(f"wrote [green]{output}[/green]")


@app.command("list-configs")
def list_configs() -> None:
    """List the complete YAML configurations included in this installation."""
    for name in available_configs():
        console.print(name, markup=False, highlight=False)


@app.command("export-config")
def export_config(
    name: str = typer.Argument(..., help="Name from list-configs"),
    output: Path = typer.Argument(..., help="New YAML file to create"),
) -> None:
    """Copy a packaged sensor or experiment YAML to an editable local file."""
    try:
        copy_packaged_config(name, output)
    except (ValueError, OSError) as error:
        console.print(f"[red]{error}[/red]")
        raise typer.Exit(1)
    console.print(f"wrote [green]{output}[/green]")


@app.command()
def inspect(
    data: Path = typer.Argument(..., help="sequence to describe"),
) -> None:
    """Report what a sequence contains, without running anything."""
    try:
        dataset = open_dataset(data)
    except (ImportError, ValueError, FileNotFoundError) as error:
        console.print(f"[red]{error}[/red]")
        raise typer.Exit(1)

    frame = next(iter(dataset))
    table = Table(title=str(data), show_header=False)
    table.add_row("frames", f"{len(dataset)}")
    table.add_row("points per scan", f"{len(frame.points)}")
    table.add_row("per-point timing", "yes" if frame.timestamps is not None else
                  "no (offsets will be reconstructed from azimuth)")
    if frame.timestamps is not None:
        table.add_row("timing range", f"{frame.timestamps.min():+.5f} .. "
                                      f"{frame.timestamps.max():+.5f} s")
    table.add_row("ring index", "yes" if frame.rings is not None else "no")
    table.add_row("IMU samples in first scan", f"{len(frame.imu)}")
    console.print(table)


if __name__ == "__main__":
    app()
