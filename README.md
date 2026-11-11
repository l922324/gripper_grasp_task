# gripper_grasp_task

Simulation environment for a robot arm grasping task: a **UR3e** or **Franka FR3** arm fitted with a **Robotiq 2F-85** gripper picks up a green block from a table top in Gazebo, lifts it, holds it, and puts it back down.

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
| `ur3e_pick_lift` | Python nodes `fixed_pick_lift` and `moveit_pick_lift` that execute the pick-and-place task |
| `fr3_2f85_description` | FR3 + 2F-85 robot description and Gazebo and MoveIt launch files |
| `fr3_pick_lift` | Python node `moveit_pick_lift` that executes the FR3 pick-and-lift task |
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

The selected robot's task script executes this sequence using its configured poses and end-effector frame. The UR3e fixed-angle script also verifies the lift distance using the `world -> tool0` TF before and after lifting, and fails if it is outside 100 mm ± 10 mm. On success it logs `PICKUP SIM PASS`.

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
source ~/gripper_grasp_ws/install/ur3e_2f85_description/share/ur3e_2f85_description/local_setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/gripper_grasp_ws/install/robotiq_description/share:$GZ_SIM_RESOURCE_PATH
```

### 3. Run with fixed joint angles (`fixed_pick_lift`)

This workflow uses fixed joint angles for the pre-grasp and grasp poses.

Start Gazebo:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3e \
  description_package:=ur3e_2f85_description \
  description_file:=ur3e_2f85.urdf.xacro \
  runtime_config_package:=ur3e_2f85_description \
  controllers_file:=ur3e_2f85_controllers.yaml \
  world_file:=$(ros2 pkg prefix --share ur3e_2f85_description)/worlds/mvp3a_table_cube.sdf \
  launch_rviz:=false
```

In a new terminal (after sourcing the workspace as in step 2):

```bash
ros2 run controller_manager spawner robotiq_gripper_controller --controller-manager /controller_manager
ros2 run ur3e_pick_lift fixed_pick_lift
```

### 4. Run the UR3e with MoveIt (`moveit_pick_lift`)

Use this workflow to plan the pick-and-lift motions with MoveIt. Run one workflow at a time. In each terminal, source ROS 2 and the workspace before running the commands:

```bash
source /opt/ros/humble/setup.bash
source ~/gripper_grasp_ws/install/setup.bash
```

In terminal 1, start Gazebo and leave it running:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3e \
  description_package:=ur3e_2f85_description \
  description_file:=ur3e_2f85.urdf.xacro \
  runtime_config_package:=ur3e_2f85_description \
  controllers_file:=ur3e_2f85_controllers.yaml \
  world_file:=$(ros2 pkg prefix --share ur3e_2f85_description)/worlds/mvp3a_table_cube.sdf \
  launch_rviz:=false
```

In terminal 2, start MoveIt and leave it running:

```bash
ros2 launch ur3e_2f85_description moveit.launch.py
```

Once Gazebo and MoveIt are ready, use terminal 3 to spawn the gripper controller. After the spawner succeeds, execute the MoveIt task:

```bash
ros2 run controller_manager spawner \
  robotiq_gripper_controller \
  --controller-manager /controller_manager
ros2 run ur3e_pick_lift moveit_pick_lift --ros-args -p use_sim_time:=true
```

### 5. Run the FR3 with the 2F-85 gripper and MoveIt

This workflow executes the same pick-and-lift task using the Franka FR3 and Robotiq 2F-85 gripper. Run one robot workflow at a time. In each terminal, source ROS 2 and the workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/gripper_grasp_ws/install/setup.bash
```

In terminal 1, start the FR3 simulation and leave it running:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch fr3_2f85_description fr3_sim.launch.py
```

In terminal 2, start MoveIt and leave it running:

```bash
ros2 launch fr3_2f85_description moveit.launch.py
```

Once the simulation and MoveIt are ready, execute the task in terminal 3:

```bash
ros2 run fr3_pick_lift moveit_pick_lift --ros-args -p use_sim_time:=true
```
