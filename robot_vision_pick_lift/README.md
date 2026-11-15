# Robot vision pick and lift

`robot_vision_pick_lift` observes a green block with two simulated D455-style
RGB-D cameras, saves/logs its measured pose, and immediately executes the shared
`robot_pick_lift` task using that pose. FR3 and UR3e use the same perception and
motion workflow. There is no manual JSON-to-command step or inference server
requirement. The existing `fr3_vision_pick_lift` package remains available with
its separate GraspGen integration.

## Build

From `~/gripper_grasp_ws`:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select robot_pick_lift robot_vision_pick_lift fr3_2f85_description ur3e_2f85_description
source install/setup.bash
```

Source ROS and the workspace in every terminal. Run one robot workflow at a time
because the supplied robot interfaces use global controller/service names.

## Simulate, detect, and pick automatically

Terminal 1, for FR3:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch robot_vision_pick_lift vision_sim.launch.py robot:=fr3
```

After controllers are active and MoveIt prints `You can start planning now!`,
run in terminal 2:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=fr3
```

For UR3e, use `robot:=ur3e` in **both** commands. Simulation brings up the robot,
gripper, table, randomized green block, camera bridges/extrinsics, controllers,
and MoveIt through the same lifecycle used by `robot_pick_lift`.

The task requires fresh observations from both cameras, segments/fuses green
points in world coordinates, fits the block footprint, and writes:

- `/tmp/robot_observation/observation.json`: frame, robot model, measured centre,
  yaw in radians, and fused point count.
- `/tmp/robot_observation/object_cloud.npy`: measured fused point cloud.

It confirms the pose with another fresh pair, then applies the detected centre
and yaw to the shared task. Reachability, collision checking, gripper settings,
lift distance, and release clearance come from `robot_pick_lift`. A camera,
perception, freshness, reachability, or planning failure aborts the task; no
configured/spawned block pose is substituted for a failed detection. The
separately launched simulation remains available for inspection.

To observe without moving:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=fr3 \
  perception_only:=true output_directory:=/tmp/fr3_observation
```

`execute:=false` also writes perception output without motion. By default,
`perception_only` is false and `execute` is true. Square-block yaw is reported
modulo 90 degrees; rectangular footprint yaw is modulo 180 degrees.

## Configuration

The layout follows `robot_pick_lift`:

```text
robot_vision_pick_lift/
  config/environment.yaml       # Table/block geometry and task settings
  config/vision.yaml            # Cameras, randomization, perception and output
  launch/vision_sim.launch.py
  launch/vision_task.launch.py
  robot_vision_pick_lift/
    configuration.py            # Shared configuration composition/validation
    scene.py                    # Randomized sensor-world generation
    perception.py               # Point-cloud decoding, crop, fusion and fit
    vision_pick_lift.py          # Observe -> output -> shared task
  test/
```

`arm.yaml` and `gripper.yaml` remain in the selected robot's description package.
The additional `environment.yaml` follows the shared package's schema: table and
block sizes, centre positions, roll/pitch/yaw, colors, block mass/friction, floor,
physics, lift distance and hold time. Gazebo and MoveIt use the same geometry.
The crop uses the configured table dimensions/pose, not fixed world XY bounds.

`vision.yaml` controls camera positions and look-at targets, image dimensions,
rate, field of view, clipping distances, cloud thresholds, output directory,
and randomization. Camera topics and static transforms derive from the same
camera names/poses used by Gazebo. These are ideal RGB-D approximations, not
Intel stereo/noise or interference models; no physical D455 driver is launched.

Randomization bounds are **offsets in the table's local XY frame**, not world
coordinates. A conservative footprint check keeps every yawed corner on the
table. The block rests on the table using its configured height. The default
seed `-1` changes the pose each launch; `seed:=17` makes it repeatable. Set
`randomization.enabled: false` to use the exact block pose in `environment.yaml`.
Randomized spawn truth stays in simulation generation and is not passed to the
perception node.

Use matching configuration files in both terminals:

```bash
ros2 launch robot_vision_pick_lift vision_sim.launch.py robot:=ur3e \
  environment_config:=/absolute/path/environment.yaml \
  vision_config:=/absolute/path/vision.yaml
```

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=ur3e \
  environment_config:=/absolute/path/environment.yaml \
  vision_config:=/absolute/path/vision.yaml
```

`arm_config`, `gripper_config`, and `description_package` work as in
`robot_pick_lift`. Another compatible description package can therefore use the
same vision code without a robot registry. The camera positions/targets must
cover its configured workspace, and the gripper must be calibrated for the
configured block dimensions. Geometry configuration does not automatically
calibrate contact targets or guarantee reachability.

For this world-vertical grasp strategy, the table must be horizontal and the
block must lie flat. Table yaw, translations, rectangular block dimensions and
block yaw are supported. Tilted tables/blocks fail configuration validation.
The block must remain green for the supplied segmenter.

`gui:=false` disables the GUI but sensor rendering still requires graphics.
On systems with EGL support, add `egl:=true` for display-free rendering. Image
bridges are optional through `bridge_images:=true`; point clouds and camera info
are always bridged.

## Maintenance and validation

The vision node subclasses the shared `MoveItPickLift` and only supplies the
measured pose. Robot bringup is reused through `robot_pick_lift.simulation`;
there are no FR3/UR3e branches or duplicated arm/gripper execution sequences.
The old package remains unchanged so its GraspGen experiments are preserved.

```bash
ROS_LOG_DIR=/tmp/robot_vision_tests python3 -m pytest -q \
  src/gripper_grasp_task/robot_vision_pick_lift/test \
  src/gripper_grasp_task/robot_pick_lift/test
```

Tests cover moved/rotated tables, rectangular blocks, seeded/fixed worlds,
sensor generation, padded cloud layouts, pose fitting, robot selection, output,
stale/missing observations and measured-pose handoff. Controller completion
confirms the motion sequence, not independent proof of physical block capture.
