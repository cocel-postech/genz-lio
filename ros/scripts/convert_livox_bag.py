#!/usr/bin/env python3
"""Convert a ROS 1 bag to ROS 2, mapping the official Livox driver message types."""

import argparse
from importlib.metadata import version
from pathlib import Path
import shutil
import sys

# Official layouts: https://github.com/Livox-SDK/livox_ros_driver2/tree/master/msg
# Compare parsed fields, including widths and array sizes, before mapping names.
POINT_FIELDS = """uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
MSG_FIELDS = """std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
{package}/CustomPoint[] points
"""
OLD_PACKAGE = "livox_ros_driver"
NEW_PACKAGE = "livox_ros_driver2"


def livox_types(package, parse):
    types = parse(POINT_FIELDS, package + "/msg/CustomPoint")
    types.update(parse(MSG_FIELDS.format(package=package), package + "/msg/CustomMsg"))
    return types


def convert(source, destination):
    if version("rosbags") != "0.9.23":
        raise RuntimeError("Use the tested converter dependency: pip install rosbags==0.9.23")
    from rosbags.convert.converter import upgrade_connection
    from rosbags.rosbag1 import Reader
    from rosbags.rosbag2 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    source = Path(source).expanduser()
    destination = Path(destination).expanduser()
    if not source.is_file() or source.suffix != ".bag":
        raise ValueError("source must be an indexed ROS 1 .bag file")
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(f"destination already exists: {destination}")
    count = mapped = 0
    created = False
    try:
        with Reader(source) as reader:
            source_types = get_typestore(Stores.EMPTY)
            header = "std_msgs/msg/Header"
            source_types.register({header: get_typestore(Stores.ROS2_FOXY).FIELDDEFS[header]})
            expected = livox_types(OLD_PACKAGE, get_types_from_msg)
            for connection in reader.connections:
                definitions = get_types_from_msg(connection.msgdef, connection.msgtype)
                for name, definition in definitions.items():
                    if name in expected and definition != expected[name]:
                        raise ValueError(f"unsupported Livox schema: {name}; refusing a type-only rename")
                definitions.pop(header, None)
                source_types.register(definitions)
            output_types = get_typestore(Stores.EMPTY)
            output_types.register(source_types.FIELDDEFS)
            output_types.register(livox_types(NEW_PACKAGE, get_types_from_msg))
            destination.parent.mkdir(parents=True, exist_ok=True)
            with Writer(destination) as writer:
                created = True
                connections, unique = {}, {}
                for connection in reader.connections:
                    msgtype = connection.msgtype
                    if msgtype in expected:
                        msgtype = msgtype.replace(OLD_PACKAGE + "/", NEW_PACKAGE + "/", 1)
                    qos = upgrade_connection(connection).ext.offered_qos_profiles
                    key = (connection.topic, msgtype, qos)
                    if key not in unique:
                        unique[key] = writer.add_connection(
                            connection.topic, msgtype, typestore=output_types,
                            offered_qos_profiles=qos,
                        )
                    connections[connection.id] = unique[key]
                for connection, timestamp, data in reader.messages():
                    # ROS 1 Header.seq is removed during CDR conversion. Livox
                    # payload layouts are identical after the schema check above.
                    cdr = source_types.ros1_to_cdr(data, connection.msgtype)
                    writer.write(connections[connection.id], timestamp, cdr)
                    count += 1
                    mapped += connection.msgtype in expected
    except BaseException:
        # Only remove output opened by this invocation, never an existing bag.
        if created:
            shutil.rmtree(destination)
        raise
    return count, mapped


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="ROS 1 .bag file")
    parser.add_argument("--dst", type=Path, required=True, help="New rosbag2 output directory")
    args = parser.parse_args(argv)
    try:
        count, mapped = convert(args.source, args.dst)
    except KeyboardInterrupt:
        print("Conversion interrupted; partial output removed.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Conversion failed: {exc}", file=sys.stderr)
        return 1
    print(f"Wrote {count} messages to {args.dst}; mapped {mapped} Livox messages to {NEW_PACKAGE}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
