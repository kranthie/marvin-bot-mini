"""Convert the generated URDF to the USD that MARVIN_BOT_MINI_CFG loads.

Runs Isaac Sim's URDF importer, so it needs the isaacsim extra. Run it again after
tools/build_urdf.py changes the URDF. The output (sim/isaaclab/usd/) is git-ignored.

Usage (from an Isaac Lab 3.0 checkout, with this package installed; see README.md):
  uv run --no-sync python <repo>/sim/isaaclab/convert_urdf.py
"""

import argparse
import shutil
import sys
from pathlib import Path

from isaaclab.app import add_launcher_args, launch_simulation

parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
add_launcher_args(parser)
args = parser.parse_args()

from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from marvin_bot_mini.robot_cfg import URDF_PATH, USD_DIR, USD_PATH  # noqa: E402


def main() -> None:
    # the importer needs Isaac Sim (Kit); the PhysX backend launches it
    urdf, usd_path = URDF_PATH, USD_PATH
    with launch_simulation(SimulationCfg(physics=PhysxCfg()), args):
        out_dir = usd_path.parent.parent
        shutil.rmtree(out_dir, ignore_errors=True)  # the importer numbers repeat outputs
        cfg = UrdfConverterCfg(
            asset_path=str(urdf),
            usd_dir=str(out_dir),
            fix_base=False,
            merge_fixed_joints=True,
            force_usd_conversion=True,
            # explicit actuator models compute the torques, so the USD drives carry no gains
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
            ),
        )
        usd = Path(UrdfConverter(cfg).usd_path)
        if usd.resolve() != usd_path.resolve():
            sys.exit(f"importer wrote {usd}, expected {usd_path}")
        print(f"wrote {usd_path.relative_to(USD_DIR.parents[2])}")


if __name__ == "__main__":
    main()
