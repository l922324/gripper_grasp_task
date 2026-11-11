# FR3 + Robotiq 2F-85 MoveIt pick and lift

This package ports the current working `ur3e_pick_lift` task with the same
pose-based workflow, table and green block, gripper action, 100 mm lift,
three-second hold, and replacement 6 mm above the original grasp pose before
release (the small clearance avoids forcing the held block into the table). The arm uses `fr3_arm`, flange `fr3_link8`, and
`/fr3_arm_controller/follow_joint_trajectory` for its seven joints.
No UR3e task or official package was modified. The task has no dependency on
`robot_pick_lift` or `ur3e_pick_lift`.

Build from the workspace:

```bash
cd ~/robospec_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fr3_2f85_description fr3_pick_lift
source install/setup.bash
```

Stop the UR3e simulation and its MoveIt/task nodes before running FR3: the two
workflows share global controller/action/topic names. Source ROS and the workspace
in each terminal.

Terminal 1 — simulation with the same table and block:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch fr3_2f85_description fr3_sim.launch.py
```

Terminal 2 — MoveIt (wait until simulation controllers are active):

```bash
ros2 launch fr3_2f85_description moveit.launch.py
```

Terminal 3 — task:

```bash
ros2 run fr3_pick_lift moveit_pick_lift --ros-args -p use_sim_time:=true
```

The task parameters are the same as the current UR3e task: `block_x` (0.45 m),
`block_y` (0 m), `block_z` (0.04 m), `tcp_offset_z` (0.145 m), `grasp_yaw`
(0 radians), `lift_distance` (0.10 m), and `hold_seconds` (3 seconds).
The offset is measured from the mounting flange along local +Z to the grasp
center, and may need calibration. The gripper is mounted with zero translation
and rotation at `fr3_link8`, matching the existing FR3 description.

The FR3 gripper controller uses a 0.01 rad/s stall velocity threshold and a
2-second stall timeout, so slow finger movement is not immediately reported as
contact. The gripper action wait is 10 seconds to allow contact detection to complete.
The FR3 task commands a fully open gripper (0.0 rad) for descent clearance,
with a 0.005 rad controller goal tolerance. Closing uses a calibrated 0.47 rad target for this 40 mm block; the original
0.75 rad command overclosed the simulated fingers and expelled the block.
Opening still requires the controller to report `reached_goal`.

The arm controller uses explicit joint position tolerances and MoveIt allows
additional time for the simulated controller to settle at each target.

The initial FR3 configuration comes from the upstream simulation hardware macro;
arm task targets are Cartesian poses, not fixed joint arrays. RRTConnect is
retained from the current UR3e node. The planning scene includes the table, the floor (only the fixed base may touch
it), and a temporary block proxy so the first
move avoids knocking the block away. The proxy is removed before descent so
Gazebo handles physical grasp contact. A 0.25-second settling interval precedes
arm planning to let the simulated state stream catch up after contact. Cartesian
paths retain collision checking and use an absolute 0.2 rad per-joint jump limit
for the seven-joint arm. Controller
success alone does not verify that the block was physically picked up.

For a headless simulation, override `gz_args`:

```bash
ros2 launch fr3_2f85_description fr3_sim.launch.py \
  "gz_args:=$(ros2 pkg prefix --share fr3_2f85_description)/worlds/mvp3a_table_cube.sdf -r -s"
```

## Validation

Both custom packages build successfully. Eight regression checks cover model and
semantic consistency, controller joint mapping, identical scene geometry,
downward orientation, task sequence, and preservation of collision rules.

An isolated headless Gazebo + MoveIt run completed the full task successfully.
Recorded block-center height: initial 0.040 m, peak 0.140034 m, final 0.040 m.
The arm reached the grasp/lift/release targets within approximately 0.1 mm in
that run. These are observations from one complete calibrated run, not a
repeatability guarantee for every randomized OMPL plan.

The installed MoveIt build also emitted a teardown segmentation fault on some
interrupted validation runs. This occurred during shutdown, after task execution;
no upstream files were changed to address that separate issue.
