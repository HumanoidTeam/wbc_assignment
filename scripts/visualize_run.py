from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import pybullet as p
import pybullet_data


CONTROL_DT = 1.0 / 60.0
SIM_DT = 1.0 / 240.0
SIM_STEPS_PER_CONTROL = int(CONTROL_DT / SIM_DT)
TARGET_HOLD_TIME = 3.0  # seconds


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--robot",
        choices=["3dof", "4dof"],
        default="3dof",
        help="Robot model to load.",
    )
    return parser.parse_args()


def load_targets(csv_path: Path) -> list[tuple[float, float]]:
    with csv_path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        return [(float(row["x"]), float(row["z"])) for row in reader]


def get_joint_state(robot_id: int, joint_indices: list[int]) -> tuple[list[float], list[float]]:
    joint_states = p.getJointStates(robot_id, joint_indices)
    joint_positions = [joint_state[0] for joint_state in joint_states]
    joint_velocities = [joint_state[1] for joint_state in joint_states]
    return joint_positions, joint_velocities


def get_ee_position(robot_id: int, ee_link_index: int) -> tuple[float, float]:
    link_state = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
    ee_world = link_state[4]
    return float(ee_world[0]), float(ee_world[2])


def candidate_impedance_controller(
    robot_id: int,
    joint_indices: list[int],
    ee_link_index: int,
    target_position: tuple[float, float],
    control_dt: float,
) -> tuple[list[float], list[float], list[float], list[float]]:
    """Controller hook used by the runner."""
    del ee_link_index, target_position, control_dt
    joint_positions, _ = get_joint_state(robot_id, joint_indices)
    zero = [0.0] * len(joint_indices)
    return joint_positions, zero, zero, zero


def main() -> None:
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    urdf_path = root / "assets" / "urdf" / f"vertical_arm_{args.robot}.urdf"
    targets_path = root / "assets" / "targets" / "fixed_targets.csv"
    targets = load_targets(targets_path)

    client = p.connect(p.GUI)
    if client < 0:
        raise RuntimeError("Failed to open PyBullet GUI.")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setTimeStep(SIM_DT)
    p.setGravity(0.0, 0.0, -9.81)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0.0, 0.0, 0.0],
        useFixedBase=True,
    )

    joint_indices = [
        joint_index
        for joint_index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, joint_index)[2] != p.JOINT_FIXED
    ]
    ee_link_index = p.getNumJoints(robot_id) - 1
    home_configuration = [-0.3, -0.8, 1.1] if args.robot == "3dof" else [-0.2, -0.7, 1.0, 0.4]
    for joint_index, joint_value in enumerate(home_configuration):
        p.resetJointState(robot_id, joint_index, joint_value)

    p.resetDebugVisualizerCamera(
        cameraDistance=1.2,
        cameraYaw=55.0,
        cameraPitch=-18.0,
        cameraTargetPosition=[0.45, 0.0, 0.30],
    )

    target_visual_shape = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.03,
        rgbaColor=[0.9, 0.1, 0.1, 0.9],
    )
    target_body = p.createMultiBody(
        baseMass=0.0,
        baseVisualShapeIndex=target_visual_shape,
        basePosition=[targets[0][0], 0.0, targets[0][1]],
    )

    print(f"Loaded robot from: {urdf_path}")
    print(f"Loaded {len(targets)} EE targets from: {targets_path}")
    print(f"Robot: {args.robot}")
    print(f"Target hold time: {TARGET_HOLD_TIME:.1f}s")
    print("Close the PyBullet window to stop.")

    target_index = 0
    current_target = targets[target_index]
    last_target_switch_time = time.time()
    last_debug_update = time.time()
    previous_ee_position: tuple[float, float] | None = None

    try:
        while p.isConnected():
            now = time.time()
            if now - last_target_switch_time >= TARGET_HOLD_TIME:
                target_index = (target_index + 1) % len(targets)
                current_target = targets[target_index]
                last_target_switch_time = now

            p.resetBasePositionAndOrientation(
                target_body,
                [current_target[0], 0.0, current_target[1]],
                [0.0, 0.0, 0.0, 1.0],
            )

            desired_joint_positions, desired_joint_velocities, kp_gains, kd_gains = candidate_impedance_controller(
                robot_id=robot_id,
                joint_indices=joint_indices,
                ee_link_index=ee_link_index,
                target_position=current_target,
                control_dt=CONTROL_DT,
            )

            if len(desired_joint_positions) != len(joint_indices):
                raise ValueError(
                    "candidate_impedance_controller() must return one desired position per joint."
                )
            if len(desired_joint_velocities) != len(joint_indices):
                raise ValueError(
                    "candidate_impedance_controller() must return one desired velocity per joint."
                )
            if len(kp_gains) != len(joint_indices) or len(kd_gains) != len(joint_indices):
                raise ValueError(
                    "candidate_impedance_controller() must return one kp and one kd per joint."
                )

            if max(abs(gain) for gain in kp_gains + kd_gains) == 0.0:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=joint_indices,
                    controlMode=p.VELOCITY_CONTROL,
                    forces=[0.0] * len(joint_indices),
                )
            else:
                p.setJointMotorControlArray(
                    bodyUniqueId=robot_id,
                    jointIndices=joint_indices,
                    controlMode=p.POSITION_CONTROL,
                    targetPositions=desired_joint_positions,
                    targetVelocities=desired_joint_velocities,
                    forces=[60.0, 50.0, 40.0],
                    positionGains=kp_gains,
                    velocityGains=kd_gains,
                )

            for _ in range(SIM_STEPS_PER_CONTROL):
                p.stepSimulation()
                time.sleep(SIM_DT)

            ee_position = get_ee_position(robot_id, ee_link_index)
            if previous_ee_position is not None:
                p.addUserDebugLine(
                    [previous_ee_position[0], 0.0, previous_ee_position[1]],
                    [ee_position[0], 0.0, ee_position[1]],
                    lineColorRGB=[0.2, 0.8, 0.2],
                    lineWidth=2.0,
                    lifeTime=2.0,
                )
            previous_ee_position = ee_position

            target_error = ((ee_position[0] - current_target[0]) ** 2 + (ee_position[1] - current_target[1]) ** 2) ** 0.5
            if now - last_debug_update > 0.25:
                p.addUserDebugText(
                    text=(
                        f"target_index={target_index + 1}/{len(targets)}\n"
                        f"target=({current_target[0]:.2f}, {current_target[1]:.2f})\n"
                        f"ee=({ee_position[0]:.2f}, {ee_position[1]:.2f})\n"
                        f"error={target_error:.3f}"
                    ),
                    textPosition=[-0.05, 0.02, 0.85],
                    textColorRGB=[1.0, 1.0, 1.0],
                    textSize=1.2,
                    lifeTime=0.3,
                )
                last_debug_update = now
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
