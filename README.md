# gripper_grasp_task

Simulation environment for a robot arm grasping task: a **UR3e** or **Franka FR3** arm fitted with a **Robotiq 2F-85** gripper picks up a green block from a table top in Gazebo, lifts it, holds it, and puts it back down.

`robot_pick_lift` provides one configuration-driven workflow for both arms. It
brings up the Gazebo world, robot and gripper, activates their controllers, and
starts MoveIt. The same package then executes the pick-and-lift task using the
selected robot's configuration. Robot geometry and integration remain in the
individual description packages.

Three configuration files control the workflow:

- `arm.yaml` in the selected description package: robot assembly, base pose,
  controllers, planning group, tool link, and planning settings.
- `gripper.yaml` in the selected description package: mounting and grasp
  transforms, controller, open/close targets, and action settings.
- [environment.yaml](robot_pick_lift/config/environment.yaml) in `robot_pick_lift`:
  table and block sizes/poses, colors, physics, and task settings.

Gazebo and the task's MoveIt collision scene use the same environment geometry.
Before motion, the task checks target reachability and collision validity; an
unsuccessful check or motion plan stops the task with an error. See the
[package guide](robot_pick_lift/README.md) for configuration details and adding
other robot models.

`robot_vision_pick_lift` extends this workflow with two simulated D455-style
RGB-D cameras. It measures the green block pose, saves the observation, and
passes that pose directly to the shared pick-and-lift task. Both packages support
FR3 and UR3e through the same robot description and gripper configuration.

## Development Environment

| Item       | Version      |
| ---------- | ------------ |
| OS         | Ubuntu 22.04 |
| ROS 2      | Humble       |
| Simulator  | Gazebo (via `ur_simulation_gz` / `gz_ros2_control`) |

## Models Used

- **UR3e** – robot arm
- **Franka FR3** – alternative robot arm
- **Robotiq 2F-85** – gripper, attached at the tip (tool flange) of the selected arm
- **Two D455-style RGB-D cameras** – simulated colored depth observations for the vision workflow

## Repository Structure

| Package / Folder | Description |
| ---------------- | ----------- |
| `ur3e_2f85_description` | Combined UR3e + 2F-85 URDF/xacro, controller config (`ur3e_2f85_controllers.yaml`) and the Gazebo world (`worlds/mvp3a_table_cube.sdf`) |
| `robot_vision_pick_lift` | Configurable two-camera perception and automatic shared pick/lift task; [usage](robot_vision_pick_lift/README.md) |
| `robot_pick_lift` | Shared configuration-driven Gazebo bringup and MoveIt task; [usage and adding robots](robot_pick_lift/README.md) |
| `ur3e_pick_lift` | Legacy UR3e task nodes, retained for compatibility |
| `fr3_2f85_description` | FR3 + 2F-85 robot description and Gazebo and MoveIt launch files |
| `fr3_pick_lift` | Legacy FR3 task node, retained for compatibility |
| `demo.repos` | Third-party repos imported with `vcs import` (see Setup) |

Third-party repositories imported via `demo.repos`:

| Repository | Branch / Tag | Description |
| ---------- | ------ | ----------- |
| [Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation) | `humble` | UR Gazebo simulation launch files |
| [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `humble` | UR robot description |
| [robotiq-ros](https://github.com/robotiq/robotiq-ros) | `main` | Robotiq gripper description, controllers and driver |
| [franka_ros2](https://github.com/frankarobotics/franka_ros2) | `humble` | Franka ROS 2 hardware interfaces, controllers and Gazebo integration |
| [franka_description](https://github.com/frankarobotics/franka_description) | `2.8.1` | Franka robot descriptions, meshes and end-effector models |
| [libfranka](https://github.com/frankarobotics/libfranka) | `0.20.4` | C++ library for communicating with and controlling Franka robots |
| [olvx_descriptions_module](https://github.com/olive-robotics/olvx_descriptions_module) | `main` | Olive component URDF descriptions, meshes and collision geometry for sensors and other modules |

## Task Flow

1. A 40 mm green block is placed at a configured or randomized pose on a table top.
2. Move the robot arm to a pre-grasp position, 100 mm above the green block.
3. Open the gripper.
4. Lower the robot arm to the grasp position.
5. Close the gripper around the block.
6. Lift the block to the pre-grasp position.
7. Hold for 3 seconds.
8. Put down the green block.
9. Open the gripper to release the block.
10. Move the robot arm back to the pre-grasp position.
11. Task complete.

The shared `robot_pick_lift` task executes this sequence for either arm. In the
vision workflow, `robot_vision_pick_lift` first detects the block from both cameras
and supplies its measured position and yaw to that same sequence. The
40 mm block, 100 mm lift, and three-second hold are defaults and can be changed
through configuration. Task completion confirms motion/controller completion;
it does not independently verify physical block capture.

## Setup

### 0. Clone into a ROS 2 workspace and import third-party repos

The third-party UR, Robotiq, Franka and Olive packages are not stored in this repository; they are listed in `demo.repos` and imported with [vcstool](https://github.com/dirk-thomas/vcstool).

```bash
sudo apt install python3-vcstool
mkdir -p ~/gripper_grasp_ws/src
cd ~/gripper_grasp_ws/src
git clone <this-repo-url> gripper_grasp_task
vcs import ~/gripper_grasp_ws/src < gripper_grasp_task/demo.repos
```

### 1. Install dependencies

```bash
cd ~/gripper_grasp_ws
rosdep update
rosdep install --ignore-src --from-paths src -y
sudo apt install ros-humble-gz-ros2-control
```

### 2. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
source ~/gripper_grasp_ws/install/setup.bash
```

### 3. Start the simulation with `robot_pick_lift`

Run one robot workflow at a time. In each terminal, source ROS 2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/gripper_grasp_ws/install/setup.bash
```

In terminal 1, choose one robot and leave the simulation running.

For **UR3e + Robotiq 2F-85**:

```bash
ros2 launch robot_pick_lift simulation.launch.py robot:=ur3e
```

For **Franka FR3 + Robotiq 2F-85**:

```bash
ros2 launch robot_pick_lift simulation.launch.py robot:=fr3
```

The launcher creates the configured world, spawns the robot, activates the
joint-state, arm, and gripper controllers, and starts MoveIt. No separate
controller-spawner or MoveIt command is needed. Wait until MoveIt prints
`You can start planning now!` before running the task.

For a headless simulation, add `gui:=false`. If needed on your graphics setup,
prefix the launch command with `LIBGL_DRI3_DISABLE=1`.

### 4. Execute the pick-and-lift task

In terminal 2, run the command matching the robot selected in step 3.

For **UR3e**:

```bash
ros2 launch robot_pick_lift pick_lift.launch.py robot:=ur3e
```

For **FR3**:

```bash
ros2 launch robot_pick_lift pick_lift.launch.py robot:=fr3
```

The task checks pregrasp, grasp, lift, and release targets before sending arm or
gripper commands. If no collision-free IK solution is found, or a subsequent
motion plan fails, the task exits with an error. The separately launched
simulation stays running for inspection.

### 5. Use custom configuration files

To change table/block geometry or poses, copy and edit
[environment.yaml](robot_pick_lift/config/environment.yaml). Supply the same file
to both launch commands, using the same robot selection:

```bash
ros2 launch robot_pick_lift simulation.launch.py robot:=fr3 \
  environment_config:=/absolute/path/my_environment.yaml
```

In another terminal, after MoveIt is ready:

```bash
ros2 launch robot_pick_lift pick_lift.launch.py robot:=fr3 \
  environment_config:=/absolute/path/my_environment.yaml
```

Use `robot:=ur3e` in both commands for UR3e. Custom arm and gripper files can be
supplied with `arm_config:=/absolute/path/arm.yaml` and
`gripper_config:=/absolute/path/gripper.yaml`; pass the same files to simulation
and task. Restart the simulation after editing configuration so its spawned
objects match the task's collision scene.

For another description package, use `description_package:=my_arm_description`
in both commands. See [adding a robot or gripper](robot_pick_lift/README.md#add-another-robot-or-gripper)
for the required description and configuration interfaces.

## Vision-guided pick and lift

[`robot_vision_pick_lift`](robot_vision_pick_lift/README.md) brings up the selected
robot with two simulated D455-style RGB-D cameras, a table, and a randomized
green block. The task fuses fresh green-block observations from both cameras,
saves/logs the detected pose, and immediately runs `robot_pick_lift` using that
pose. No manual JSON-to-command step or GraspGen server is required.

Use this simulation instead of the non-vision simulation in setup step 3. Run
one workflow at a time, and source ROS 2 and the workspace in every terminal.

### Start the vision simulation

In terminal 1, choose one robot.

For **FR3**:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch robot_vision_pick_lift vision_sim.launch.py robot:=fr3
```

For **UR3e**:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch robot_vision_pick_lift vision_sim.launch.py robot:=ur3e
```

The launcher starts the world, cameras and bridges, robot state publisher,
controllers, and MoveIt. Wait for `You can start planning now!` before running
the task. Add `seed:=17` for a reproducible block pose. `gui:=false` disables the
GUI; sensor rendering still needs graphics, or `egl:=true` on systems supporting
EGL.

### Detect and execute automatically

In terminal 2, use the same robot selection as the simulation.

For **FR3**:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=fr3
```

For **UR3e**:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=ur3e
```

By default, the task detects the block, saves the observation, confirms the pose
with another fresh camera pair, and executes the shared pick/lift sequence.
Missing/stale observations, block movement between captures, or unsuccessful
reachability/planning checks stop the task with an error.

### Inspect perception without moving

To save observations only, replace the automatic task command with:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=fr3 \
  perception_only:=true output_directory:=/tmp/fr3_observation
```

Use `robot:=ur3e` if running the UR3e simulation. `execute:=false` also disables
motion. The default output directory is `/tmp/robot_observation`; the example
above overrides it. The output contains:

- `observation.json`: world frame, robot model, detected centre `[x, y, z]` in
  metres, yaw in radians, and fused point count.
- `object_cloud.npy`: the measured fused green-block point cloud.

For example:

```bash
cat /tmp/fr3_observation/observation.json
```

Square-block yaw is observable modulo 90 degrees. The automatic task uses the
measured pose directly; reading or copying the JSON values is optional.

### Configure the vision environment and cameras

The vision package reuses `arm.yaml` and `gripper.yaml` from the selected
robot's description package, with two additional files:

| Configuration | Settings |
| --- | --- |
| [environment.yaml](robot_vision_pick_lift/config/environment.yaml) | Table/block dimensions and centre poses, colors, mass/friction, floor, physics, lift distance and hold time |
| [vision.yaml](robot_vision_pick_lift/config/vision.yaml) | Camera positions/targets, image settings, randomization bounds/seed, perception thresholds, execution mode and output directory |

Randomization bounds are offsets in the table's local XY frame. Set
`randomization.enabled: false` to use the exact block pose in the environment
file. The supplied perception strategy supports horizontal tables, table yaw and
translation, and flat green blocks with configurable dimensions and yaw.
Gripper contact settings must remain calibrated for the chosen block dimensions.

Pass the same custom files to simulation and task:

```bash
ros2 launch robot_vision_pick_lift vision_sim.launch.py robot:=fr3 \
  environment_config:=/absolute/path/environment.yaml \
  vision_config:=/absolute/path/vision.yaml
```

In another terminal, after MoveIt is ready:

```bash
ros2 launch robot_vision_pick_lift vision_task.launch.py robot:=fr3 \
  environment_config:=/absolute/path/environment.yaml \
  vision_config:=/absolute/path/vision.yaml
```

Both commands also accept `arm_config`, `gripper_config`, and
`description_package` for other compatible robot descriptions. Restart the
simulation after editing its configuration. Camera models approximate colored
RGB-D observations; they do not simulate Intel active-stereo noise/interference
or launch physical camera drivers.

The original `fr3_vision_pick_lift` package remains available separately for its
GraspGen integration. See the [vision package guide](robot_vision_pick_lift/README.md)
for its structure, configuration contract, and validation instructions.
