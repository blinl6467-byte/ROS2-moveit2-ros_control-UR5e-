# UR5e Kinematics

Self-developed C++ kinematics for the UR5e model used in this workspace. The
core implementation does not call MoveIt, KDL, IKFast, or Pinocchio.

详细中文推导、代码映射和面试问答见
[`docs/kinematics_interview_guide.md`](../../docs/kinematics_interview_guide.md)。

## Implemented algorithms

- Forward kinematics from the fixed transforms in the workspace URDF/Xacro.
- 6x6 spatial geometric Jacobian in `base_link`.
- SO(3) logarithm and six-dimensional pose error.
- Damped least-squares numerical inverse kinematics.
- Adaptive singularity damping, step limiting, and joint-limit enforcement.

## Mathematical definition

For each revolute joint, the implementation composes its fixed URDF origin
with a local Z-axis rotation:

```text
T_base_tool(q) = T_base_internal * product(T_origin_i * Rz(q_i)) * T_wrist_tool0
```

The spatial geometric Jacobian is assembled column by column:

```text
Jv_i = z_i x (p_tool - p_i)
Jw_i = z_i
```

Orientation error is represented by the SO(3) logarithm of
`R_target * R_current.transpose()`. The inverse solver uses the weighted,
damped least-squares update

```text
dq = Jw.transpose() * (Jw * Jw.transpose() + lambda^2 * I)^-1 * error
```

where damping increases as the minimum singular value falls below a configured
threshold. Every update is norm-limited and optionally clamped to UR5e joint
limits.

The reported end-effector frame is `tool0`, matching this workspace's chain:

```text
base_link -> base_link_inertia -> six revolute joints -> wrist_3_link
          -> flange -> tool0
```

## Build and test

```bash
cd /home/ubuntu22/ur5e_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur5e_kinematics
colcon test --packages-select ur5e_kinematics --event-handlers console_direct+
colcon test-result --verbose
```

## Random reachable-pose benchmark

```bash
source install/setup.bash
ros2 run ur5e_kinematics kinematics_benchmark 1000
```

The benchmark generates reachable target poses from random joint states, adds
noise to each seed, solves IK, and reports convergence, residuals, iteration
count, and solve time. It measures local numerical IK robustness; it is not a
claim of global completeness.

## Validation strategy

- Rotation orthonormality and determinant checks.
- Analytical Jacobian against central finite differences.
- IK forward-backward consistency on random reachable poses.
- Joint-limit enforcement.
- A fixed zero-configuration reference derived from the URDF chain.
- FK and Jacobian comparison against an independently parsed KDL chain over
  hundreds of random configurations.

KDL is deliberately used only in the test target, never inside the
self-developed solver. Pinocchio can be added later as a second dynamics and
kinematics reference.
