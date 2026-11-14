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

## Repository Structure

| Package / Folder | Description |
| ---------------- | ----------- |
| `ur3e_2f85_description` | Combined UR3e + 2F-85 URDF/xacro, controller config (`ur3e_2f85_controllers.yaml`) and the Gazebo world (`worlds/mvp3a_table_cube.sdf`) |
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

1. A 40 mm x 40 mm green block is placed at a fixed position on a table top.
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

The shared `robot_pick_lift` task executes this sequence for either arm. The
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
