# Configurable Gazebo bringup and MoveIt pick/lift

One task implementation supports FR3 and UR3e with Robotiq 2F-85 assemblies.
Robot description packages retain their URDF/Xacro, SRDF and robot-specific
integration. The shared package generates the world, starts the assembled robot,
activates controllers in order, and starts MoveIt.

## Build

From `~/gripper_grasp_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_pick_lift fr3_2f85_description ur3e_2f85_description
source install/setup.bash
```

Source ROS and the workspace in each terminal. Run only one ordinary simulation
at a time: supplied configurations use global controller/service names.

## Bring up a robot, then execute the task

Terminal 1:

```bash
ros2 launch robot_pick_lift simulation.launch.py robot:=fr3
```

This starts Gazebo with the configured table, green block and robot, publishes
robot state and simulation clock, loads the joint-state, arm, and gripper
controllers, then starts MoveIt. Controller activation is sequential and checked;
a failed spawn or controller activation shuts down bringup. For a server-only
simulation add `gui:=false`. Replace `fr3` with `ur3e` to select UR3e.

After MoveIt prints `You can start planning now!`, terminal 2:

```bash
ros2 launch robot_pick_lift pick_lift.launch.py robot:=fr3
```

The task checks all target poses before motion, then runs pregrasp, open,
Cartesian descent, close, lift, hold, lower, release, and retreat.
The task exits on failure; the separately launched simulation stays running.

The executable also accepts model selection directly:

```bash
ros2 run robot_pick_lift moveit_pick_lift --ros-args -p robot:=fr3 -p use_sim_time:=true
```

## Three configuration files

| File | Owner | Settings |
| --- | --- | --- |
| `config/arm.yaml` | Selected description package | Assembly/SRDF resources, arm pose, Xacro arguments, resource paths, controllers, kinematics, OMPL settings, task planning group/tool link and arm tuning |
| `config/gripper.yaml` | Selected description package | Gripper mount, grasp transform, controller source/name, action and driver joint, open/close targets, timeout and effort |
| `config/environment.yaml` | `robot_pick_lift`, or user-supplied file | World name, physics/gravity, floor height, table/block dimensions, poses, colors, block mass/friction, lift distance and hold time |

The FR3 and UR3e description packages each provide their own `arm.yaml` and
`gripper.yaml`. Resource references use `{package: package_name, path: relative/path}`;
an absolute filename is also accepted. Controller YAMLs are referenced by the
arm/gripper files and merged into a generated configuration containing the
selected controllers. Gazebo's ros2_control plugin owns the controller manager;
the launcher does not start a second manager.

Override any file using `arm_config`, `gripper_config`, or `environment_config`:

```bash
ros2 launch robot_pick_lift simulation.launch.py robot:=fr3 \
  environment_config:=/absolute/path/my_environment.yaml

ros2 launch robot_pick_lift pick_lift.launch.py robot:=fr3 \
  environment_config:=/absolute/path/my_environment.yaml
```

**Use the same files for simulation and task.** The task reads the environment
file and builds its MoveIt collision geometry from those values. Editing a file
after simulation starts does not change already-spawned objects; restart bringup
to apply changes. Configured object poses are initial poses, not live perception
or tracking of objects moved during simulation.

The task's launch also accepts explicit `block_x`, `block_y`, `block_z`,
`grasp_yaw`, `lift_distance`, and `hold_seconds` overrides. Empty defaults retain
the environment values. Block overrides change only the task target, not Gazebo;
prefer editing the environment file when changing the simulation.

### Pose conventions

Positions and box dimensions use metres. Angles use radians. Table/block poses
refer to their geometric centres in the Gazebo `world` frame. Arbitrary box
rotations are reflected in both Gazebo and MoveIt. The grasp remains a world
vertical, downward-facing grasp, aligned to block yaw plus `grasp_yaw_offset`;
large block tilts may require another grasp strategy.

`arm.pose` locates the arm base assembly in world coordinates. It changes the
world-to-base fixed joint in the generated URDF; the same URDF is used by Gazebo,
state publishing, and MoveIt. Do not additionally offset the spawned model.

`gripper.mount` is the gripper mounting frame relative to the planning tool link.
`gripper.grasp.position` and `.quaternion` locate the finger grasp frame relative
to that mounting frame. Quaternion order is `[x, y, z, w]`. The loader composes
both transforms to derive `tool_to_grasp_xyz` and `tool_to_grasp_quaternion`.
Changing a mount therefore updates the task's tool-to-grasp offset as well as
the assembled robot. Supplied Xacro adapters expose `gripper_xyz`/`gripper_rpy`.

Block inertia is computed from its configured box dimensions and mass. The
floor is at `floor_z`; only the arm profile's mounting links may touch it in
MoveIt. The block proxy protects it during the first motion and is removed
before contact; physical grasping is handled by Gazebo contacts/friction. The
block is not attached in MoveIt during the lift.

## Reachability and failure behavior

Before sending any arm/gripper goals, the task queries the current robot state
and requests collision-free IK for pregrasp, grasp, lift and release. It uses the
configured open-finger state for these checks, temporarily removes the block
proxy to permit intended contact, then restores the proxy for the initial plan.
An unavailable service, missing robot state, or unsuccessful IK raises an error
with the phase, target coordinates and MoveIt error code, and exits nonzero.
“No collision-free IK found” can mean an unreachable target or collision; it
is not a mathematical proof that no IK solution exists.

During execution, pose planning/execution failures and incomplete Cartesian
paths also stop the task. Successful IK does not guarantee a connecting motion
plan. Controller/task completion does not verify physical block capture. Arm
settling delays use wall time and do not measure actual standstill.

## Add another robot or gripper

1. Provide and install a description package with a combined arm/gripper Xacro,
   SRDF, kinematics and controller resources. The simulation hardware plugin
   remains part of that assembly's Xacro.
2. Supply `config/arm.yaml` and `config/gripper.yaml`, following the existing
   examples. The assembly must accept `xyz`, `rpy`, `gripper_xyz`, `gripper_rpy`,
   and `simulation_controllers` Xacro arguments. Set any other model/plugin
   switches in `arm.xacro_arguments`.
3. Use `description_package:=my_arm_description` in both launch commands.
   No Python robot registry is required. `robot:=name` is shorthand for
   `description_package:=name_2f85_description`.
4. Validate startup, collision geometry, reachable poses, and physical grasping.

An explicit `arm_config` can also be used without `robot` or
`description_package`; by default `gripper.yaml` is resolved alongside that file.
The supplied assembly adapters include a Robotiq 2F-85 macro. Merely changing
`gripper.model` does not install a different gripper: another model needs a
compatible description adapter and a `GripperCommand` controller (or an action
adapter). Arm control currently uses `FollowJointTrajectory`.

## Existing workflows

Legacy robot-specific task packages and launches remain available. The former
`config/pick_lift.yaml` profiles remain usable through explicit `profile:=...`,
or directly with `--params-file`. The new three-file configuration is the default.
`start_moveit:=true` on the task launcher remains available for an already-running
simulator; do not enable it if simulation.launch.py already started MoveIt.

The FR3 profile preserves its calibrated gripper close target, absolute jump
limit, settling delay, and replacement clearance. UR3e retains its relative jump
limit and replacement clearance. Both use a 0.0 rad opening and ten-second
GripperCommand wait.

## Validation

Tests cover real Xacro assembly for both robots, base/gripper mounting poses,
controller/MoveIt consistency, environment/world/scene values, box inertia,
configuration validation, target preflight failures, no motion on unreachable
targets, collision-rule preservation, task sequence and custom package loading.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOG_DIR=/tmp/robot_pick_lift_test_logs python3 -m pytest -q src/gripper_grasp_task/robot_pick_lift/test
```

Headless isolated runtime checks reached controller activation and MoveIt ready
for both robots. An FR3 target at x=5 m was rejected with NO_IK_SOLUTION before
motion. Nominal target preflight passed for both robots without sending motion commands.
Full physical pick/release repeatability after this change remains to be checked.
The installed MoveIt build can segfault during interrupted shutdown, also observed
in the earlier FR3 workflow; this is separate from successful bringup/preflight.
