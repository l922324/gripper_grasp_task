# UR3e MoveIt pick and lift

Keep the existing Gazebo command. Source `/opt/ros/humble/setup.bash` and
`~/gripper_grasp_ws/install/setup.bash` in each terminal.

Build only the custom packages:

```bash
colcon build --packages-select ur3e_2f85_description ur3e_pick_lift
```

After Gazebo is running, start MoveIt in another terminal:

```bash
ros2 launch ur3e_2f85_description moveit.launch.py
```

Then run the new task instead of `fixed_pick_lift`:

```bash
ros2 run ur3e_pick_lift moveit_pick_lift --ros-args -p use_sim_time:=true
```

The arm first plans to a downward-facing clearance pose at tool0 world position
(0.25, 0, 0.35) m. This is a configurable candidate, not a runtime-validated
waypoint. The initial segment allows reorientation from the starting pose.
It then plans to pregrasp with a downward orientation path constraint (0.15 rad
rotation-vector tolerance on each axis). Descent, lift, replacement, and retreat
use collision-checked Cartesian paths. The gripper uses
the existing GripperCommand controller. No fixed arm joint targets are used.
The original fixed task remains available.

Default block center in world coordinates: (0.45, 0, 0.04) m, matching the SDF.
The grasp orientation points tool0 +Z downward. `tcp_offset_z` defaults to
0.145 m along tool0 +Z from tool0 to the finger grasp center; this is an initial
estimate that needs Gazebo calibration, not a measured value. Adjust with e.g.
`-p tcp_offset_z:=0.14`. Other parameters: block_x, block_y, block_z,
grasp_yaw (radians), lift_distance (default 0.10 m), hold_seconds (default 3).

The planning scene includes the table at its SDF dimensions and an infinite
floor plane at world z=0. Only the fixed base_link/base_link_inertia mounting
links may contact that floor; moving links remain collision-checked. Existing
collision rules are preserved. The node refuses movement from an invalid
starting state and reports collision pairs when available. Restart Gazebo after
a previous collision before trying this workflow.

Pose moves use plan-only requests followed by trajectory checks and a separate
execution action. Both pose and Cartesian trajectories reject empty/nonfinite
positions, non-increasing timing, joint steps above 0.75 rad, and cumulative
travel above 3.14 rad per joint per segment. These are conservative rejection
limits, not a guarantee of a particular path or of physical tracking accuracy.
RRTConnect can still produce different accepted paths. A rejected plan stops
the task without executing that segment.

Additional parameters: clearance_x, clearance_y, clearance_z,
orientation_tolerance, max_joint_step, max_joint_travel. Clearance coordinates
refer to tool0, not the fingertip. Clearance z must be at least the pregrasp
height. Calibrate the waypoint and TCP offset in Gazebo before relying on them.

 The block is not
added or attached in MoveIt; it is moved only by Gazebo finger contact and
friction. Task success confirms motion/controller completion, not block pickup.
The local SRDF retains upstream UR arm collision exclusions and excludes internal
Robotiq linkage pairs, which mechanically contact during closure. Gripper versus
arm/table collision checks remain enabled except the adjacent wrist/base pair.

Validation: orientation regression checks, trajectory rejection cases,
collision-matrix preservation, and plan-before-execute request checks. Earlier
launch generation/xacro checks passed; the new motion still needs runtime QA. Physical grasp, reachability, and full simulation execution still need
runtime validation. Do not run the old and new task simultaneously.

## Wait for the arm before closing

After MoveIt reports success, `moveit_pick_lift` now checks fresh `/joint_states`
against the final arm trajectory point. Every commanded joint must be within
`arm_position_tolerance` (0.005 rad) and below `arm_velocity_tolerance`
(0.01 rad/s) for `arm_settle_time` (0.25 wall seconds). A missing or unsettled
state times out after `arm_settle_timeout` (15 wall seconds) and aborts the task
before the gripper-close command. This also verifies the other arm moves.
The Cartesian arm trajectory must contain exactly the six UR arm joints.

The descent logs `DESCEND`, then `Arm reached target and stopped`, followed by
`GRASP REACHED: closing gripper`. The task command and detected-pose parameters
are unchanged. This check verifies arm standstill; it does not verify physical
block capture or prevent passive finger movement caused by simulated contacts.

The MoveIt task now commands 0.0 rad for fully open jaws. The fixed-angle task's
0.1 rad opening target is deliberately left unchanged. Using 0.0 avoids a small
closing motion toward 0.1 during descent when starting from fully open fingers.
The MoveIt sequence also allows ten wall seconds for each gripper action,
matching the slower simulator response rather than the fixed task's four-second
wait. A timeout still cancels the action and stops the sequence.
