from __future__ import annotations

import argparse
import time
from pathlib import Path

import pybullet as p
import pybullet_data


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=["3dof", "4dof"],
        default="3dof",
        help="Robot model to load.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    urdf_path = root / "assets" / "urdf" / f"vertical_arm_{args.robot}.urdf"

    client = p.connect(p.GUI)
    if client < 0:
        raise RuntimeError("Failed to open PyBullet GUI.")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0.0, 0.0, 0.0],
        useFixedBase=True,
    )

    home_configuration = [-0.3, -0.8, 1.1] if args.robot == "3dof" else [-0.2, -0.7, 1.0, 0.4]
    for joint_index, joint_value in enumerate(home_configuration):
        p.resetJointState(robot_id, joint_index, joint_value)

    joint_sliders: list[int] = []
    for joint_index in range(len(home_configuration)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")
        lower_limit = joint_info[8]
        upper_limit = joint_info[9]
        slider_id = p.addUserDebugParameter(
            paramName=joint_name,
            rangeMin=lower_limit,
            rangeMax=upper_limit,
            startValue=home_configuration[joint_index],
        )
        joint_sliders.append(slider_id)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=55.0,
        cameraPitch=-18.0,
        cameraTargetPosition=[0.45, 0.0, 0.30],
    )

    print(f"Loaded robot from: {urdf_path}")
    print("Joint names:")
    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        print(f"  {joint_index}: {joint_info[1].decode('utf-8')}")

    print("\nUse the sliders to move the arm.")
    print("Close the PyBullet window to stop.")
    try:
        while p.isConnected():
            for joint_index, slider_id in enumerate(joint_sliders):
                joint_value = p.readUserDebugParameter(slider_id)
                p.resetJointState(robot_id, joint_index, joint_value)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
