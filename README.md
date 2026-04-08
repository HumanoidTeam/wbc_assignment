# EE Tracking Assignment

Small starter repo for a vertical-plane tracking task with a fixed-base 3-DoF arm.

## Contents

- `assets/urdf/vertical_arm_3dof.urdf`: robot model
- `assets/urdf/vertical_arm_4dof.urdf`: optional 4-DoF variant
- `assets/targets/fixed_targets.csv`: end-effector targets in the `x-z` plane
- `scripts/print_robot_info.py`: prints joint names and limits
- `scripts/view_robot.py`: opens the robot with joint sliders
- `scripts/visualize_run.py`: main runner

## Task

Track the provided end-effector targets in the vertical plane using two approaches:

- one classical controller
- one reinforcement learning controller

Assumptions:

- gravity is on
- the task is position-only
- targets are sampled from `assets/targets/fixed_targets.csv`
- the low-level interface is joint impedance

`scripts/visualize_run.py` handles the simulation loop and target switching. The controller hook is `candidate_impedance_controller()`.

The controller is expected to return:

- desired joint positions
- desired joint velocities
- `kp`
- `kd`

Returning zero gains leaves the arm passive under gravity.

## Deliverables

Please submit:

- one classical solution for the tracking task
- one RL-based solution for the tracking task
- a short document summarizing tracking performance
- a comparison between the two approaches

The comparison should include at least:

- tracking accuracy
- stability of motion
- behavior across the provided targets
- any clear failure modes

The write-up does not need to be long, but it should be complete enough to explain:

- how each method was implemented
- how each method was evaluated
- where each method worked well
- where each method struggled

## Stretch Goal

There is also a 4-DoF version of the same arm in `assets/urdf/vertical_arm_4dof.urdf`.

As a stretch goal, compare the 3-DoF and 4-DoF models for both:

- the classical controller
- the RL controller

The comparison can be qualitative or quantitative, but it should address how the extra degree of freedom changes:

- tracking behavior
- ease of control design
- stability or smoothness
- any differences in RL training behavior

## Quick Start

```bash
python -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

```bash
python scripts/print_robot_info.py
python scripts/view_robot.py
python scripts/visualize_run.py
```

To use the 4-DoF model:

```bash
python scripts/print_robot_info.py --robot 4dof
python scripts/view_robot.py --robot 4dof
python scripts/visualize_run.py --robot 4dof
```

## Notes

Keep the scope tight. No orientation tracking, obstacle avoidance, or extra infrastructure is needed.
