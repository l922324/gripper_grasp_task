# gripper_grasp_task

Simulation environment for a robot arm grasping task: a **UR3e** arm fitted with a **Robotiq 2F-85** gripper picks up a green block from a table top in Gazebo, lifts it, holds it, and puts it back down.

## Development Environment

| Item       | Version      |
| ---------- | ------------ |
| OS         | Ubuntu 22.04 |
| ROS 2      | Humble       |
| Simulator  | Gazebo (via `ur_simulation_gz` / `gz_ros2_control`) |

## Models Used

- **UR3e** – robot arm
- **Robotiq 2F-85** – gripper, attached at the tip (tool flange) of the UR3e

## Repository Structure

| Package / Folder | Description |
| ---------------- | ----------- |
| `ur3e_2f85_description` | Combined UR3e + 2F-85 URDF/xacro, controller config (`ur3e_2f85_controllers.yaml`) and the Gazebo world (`worlds/mvp3a_table_cube.sdf`) |
| `ur3e_pick_lift` | Python node `fixed_pick_lift` that executes the pick-and-place task |
| `demo.repos` | Third-party repos imported with `vcs import` (see Setup) |

Third-party packages imported via `demo.repos`:

| Repository | Branch | Description |
| ---------- | ------ | ----------- |
| [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) | `humble` | UR ROS 2 driver and controllers |
| [Universal_Robots_ROS2_GZ_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation) | `humble` | UR Gazebo simulation launch files |
| [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | `humble` | UR robot description |
| [robotiq-ros](https://github.com/robotiq/robotiq-ros) | `main` | Robotiq gripper description, controllers and driver |

## Task Flow

1. A 40 mm x 40 mm green block is placed at a fixed position on a table top.
2. Move the UR3e to a pre-grasp position, 100 mm above the green block.
3. Open the 2F-85.
4. Lower the UR3e to the grasp position.
5. Close the 2F-85.
6. Lift up to the pre-grasp position.
7. Hold for 3 seconds.
8. Put down the green block.
9. Open the 2F-85.
10. Lift up to the pre-grasp position again.
11. Job DONE!

The task script also verifies the lift distance by reading the `world -> tool0` TF before and after lifting, and fails if it is outside 100 mm ± 10 mm. On success it logs `PICKUP SIM PASS`.

## Setup

### 0. Clone into a ROS 2 workspace and import third-party repos

The UR and Robotiq packages are not stored in this repository; they are listed in `demo.repos` and imported with [vcstool](https://github.com/dirk-thomas/vcstool).

```bash
sudo apt install python3-vcstool
mkdir -p ~/robospec_ws/src
cd ~/robospec_ws/src
git clone <this-repo-url> gripper_grasp_task
vcs import ~/robospec_ws/src < gripper_grasp_task/demo.repos
```

### 1. Install dependencies

```bash
cd ~/robospec_ws
rosdep update
rosdep install --ignore-src --from-paths src -y
sudo apt install ros-humble-gz-ros2-control
```

### 2. Build the workspace

```bash
source /opt/ros/humble/setup.bash
colcon build
source ~/robospec_ws/install/setup.bash
source ~/robospec_ws/install/ur3e_2f85_description/share/ur3e_2f85_description/local_setup.bash
export GZ_SIM_RESOURCE_PATH=$HOME/robospec_ws/install/robotiq_description/share:$GZ_SIM_RESOURCE_PATH
```

### 3. Run Gazebo

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

### 4. Execute the task script

In a new terminal (after sourcing the workspace as in step 2):

```bash
ros2 run controller_manager spawner robotiq_gripper_controller --controller-manager /controller_manager
ros2 run ur3e_pick_lift fixed_pick_lift
```
