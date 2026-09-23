"""The ROS2 workspace default must preserve user-supplied DDS profiles."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest


class FastDdsEnvironmentTests(unittest.TestCase):
    def source_hook(self, overrides, prefix_variable="AMENT_CURRENT_PREFIX"):
        template = Path(__file__).resolve().parents[1] / "env-hooks/genz_lio_fastdds.sh.in"
        with tempfile.TemporaryDirectory(prefix="genz hook ") as directory:
            hook = Path(directory) / "hook.sh"
            hook.write_text(template.read_text().replace("@PROJECT_NAME@", "genz_lio"))
            env = dict(os.environ)
            env.pop("AMENT_CURRENT_PREFIX", None)
            env.pop("COLCON_CURRENT_PREFIX", None)
            env[prefix_variable] = directory
            for name in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
                env.pop(name, None)
            env.update(overrides)
            result = subprocess.run(
                ["bash", "-c", '. "$1"; . "$1"; printf "%s\\n%s\\n" '
                 '"${FASTRTPS_DEFAULT_PROFILES_FILE-}" "${FASTDDS_DEFAULT_PROFILES_FILE-}"',
                 "hook-test", str(hook)], env=env, check=True, capture_output=True, text=True,
            )
            return result.stdout.splitlines(), directory

    def test_default_and_repeat_source_with_spaces(self):
        values, prefix = self.source_hook({})
        self.assertEqual(values, [prefix + "/share/genz_lio/config/dds/fastdds_local.xml", ""])

    def test_explicit_legacy_profile(self):
        values, _ = self.source_hook({"FASTRTPS_DEFAULT_PROFILES_FILE": "/custom/profile.xml"})
        self.assertEqual(values, ["/custom/profile.xml", ""])

    def test_colcon_dsv_prefix(self):
        values, prefix = self.source_hook({}, prefix_variable="COLCON_CURRENT_PREFIX")
        self.assertEqual(values, [prefix + "/share/genz_lio/config/dds/fastdds_local.xml", ""])

    def test_colcon_dsv_ignores_stale_ament_underlay(self):
        values, prefix = self.source_hook(
            {"AMENT_CURRENT_PREFIX": "/opt/ros/humble"},
            prefix_variable="COLCON_CURRENT_PREFIX",
        )
        self.assertEqual(values, [prefix + "/share/genz_lio/config/dds/fastdds_local.xml", ""])

    def test_explicit_new_profile(self):
        values, _ = self.source_hook({"FASTDDS_DEFAULT_PROFILES_FILE": "/custom/new.xml"})
        self.assertEqual(values, ["", "/custom/new.xml"])

    def test_explicit_both_profiles(self):
        values, _ = self.source_hook({"FASTRTPS_DEFAULT_PROFILES_FILE": "/old.xml",
                                      "FASTDDS_DEFAULT_PROFILES_FILE": "/new.xml"})
        self.assertEqual(values, ["/old.xml", "/new.xml"])


if __name__ == "__main__":
    unittest.main()
