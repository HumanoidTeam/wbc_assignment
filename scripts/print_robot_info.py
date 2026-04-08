from __future__ import annotations

import argparse
from pathlib import Path

import pybullet as p


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=["3dof", "4dof"],
        default="3dof",
        help="Robot model to inspect.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    urdf_path = root / "assets" / "urdf" / f"vertical_arm_{args.robot}.urdf"

    client = p.connect(p.DIRECT)
    try:
        robot_id = p.loadURDF(str(urdf_path), useFixedBase=True)
        print(f"Robot: {urdf_path.name}")
        print(f"Number of joints: {p.getNumJoints(robot_id)}")
        print()
        for joint_index in range(p.getNumJoints(robot_id)):
            joint_info = p.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = joint_info[2]
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            link_name = joint_info[12].decode("utf-8")
            print(
                f"joint {joint_index}: name={joint_name}, type={joint_type}, "
                f"limits=({lower_limit:.2f}, {upper_limit:.2f}), child_link={link_name}"
            )
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
