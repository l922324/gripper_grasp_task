# Randomized FR3 pick/lift with two RGB-D views and GraspGenX

This is a separate extension package. The working `fr3_pick_lift`, the existing
FR3 description, and all official packages remain unchanged.

The new path is:

1. Generate a fresh world with a random green-block XY position and yaw.
2. Receive colored point clouds from **both** simulated cameras, transform to
   `world`, segment green points, fuse them, and fit the cube footprint.
3. Send that observed object cloud to a real GraspGenX server for `robotiq_2f_85`.
4. Filter returned grasps for score, downward approach, table clearance, proximity
   to the observed object, and jaw alignment suitable for the calibrated cube grip.
5. In score order, ask MoveIt to preview pregrasp and a complete Cartesian approach.
6. Execute the first feasible candidate, lift 100 mm, hold three seconds, replace,
   open, and retreat, using the existing FR3 motion and gripper helpers.

The inference machine is intentionally left for later configuration. There is
**no generated-pose fallback** when GraspGenX is missing or returns no valid grasp.
Neural inference and the new vision-guided physical grasp cannot be validated
until a compatible server and its gripper-frame calibration are available.

## Build and observe now

Source ROS and this workspace in every terminal. Stop the old simulation/MoveIt
before starting the new one, since both use the same global robot action names.

```bash
cd ~/robospec_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select fr3_vision_pick_lift
source install/setup.bash
```

Terminal 1 starts the randomized simulation, camera bridges/extrinsics, and MoveIt:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch fr3_vision_pick_lift vision_sim.launch.py
```

Terminal 2 can test perception without inference or any controller commands:

```bash
ros2 launch fr3_vision_pick_lift vision_task.launch.py \
  perception_only:=true output_directory:=/tmp/fr3_observation
```

The observation directory contains `object_cloud.npy` and `observation.json`.
Use `seed:=17` on the simulation launch for a reproducible scene. The default
`seed:=-1` samples a different pose each launch. The generated world and its
truth pose are logged only for simulation inspection/offline QA; the task never
reads them or receives that pose through parameters.

`headless:=true` disables the Gazebo GUI but still uses the existing display for
sensor rendering. On a machine supporting EGL, `headless:=true egl:=true` selects
Gazebo's display-free renderer. These are rendering options, separate from the
GPU needed for learned inference. Camera bridges start after an eight-second
startup interval so sensor rendering does not compete with controller creation.
The generated world selects DART's Bullet collision detector: the default
detector missed randomized cube/table contacts during Fortress validation.

## Camera arrangement and perception

The idealized D455-style RGB-D sensors have a configured 86-degree horizontal
field of view, 320x240 images, a 0.4–6 m clip range, and a 2 Hz update rate. They
model colored depth observation, **not** Intel's active-stereo hardware/noise or
inter-camera interference. No physical D455 driver is launched.

Two camera positions in metres are configured in `config/vision.yaml`:

| Camera | World position | Aim point |
|---|---|---|
| d455_left | (0.75, -0.50, 0.65) | (0.45, 0, 0.04) |
| d455_right | (0.75, 0.50, 0.65) | (0.45, 0, 0.04) |

The opposing elevated views provide complementary top/side observations while
keeping mounts away from the low grasp motion. Frustum tests cover the default
sampling region. These positions are an initial simulation layout, not a claimed
global camera-placement optimum.

Default XY randomization is x=[0.35,0.55], y=[-0.15,0.15], a reachable subset of
the original table. The cube remains 40 mm, lies flat at center z=0.04, and has
random yaw over a full turn. Roll/pitch stay zero. All rotated corners retain
edge clearance. Expanding the region requires validating FR3 reachability.

Green segmentation uses RGB channel separation, not spawn coordinates. The
world-frame tabletop crop, voxel fusion, connected-component rejection and
minimum-area footprint fit estimate position and yaw. The center height comes
from the observed top surface and known cube size. A square cube's yaw is
observable only modulo 90 degrees; this is sufficient for equivalent face grasps.
Both cameras must supply enough points with recent, closely matched timestamps.
The simulation defaults allow clouds up to five simulation seconds old and
up to two seconds apart, accommodating asynchronous software rendering on this
VM. These bounds assume a stationary cube; tighten them for moving objects or
hardware cameras. Both cameras must still provide new frames for reobservation.
The task also reobserves before execution and rejects a block that moved.

Fortress cloud coordinates use +X-forward sensor axes. The cloud bridge labels
them with each camera's `camera_frame`; image/CameraInfo topics retain the
+Z-forward `optical_frame`. Both extrinsics are published explicitly.

ROS topics include `/d455_left/points`, `/d455_right/points`, and corresponding
`camera_info`. Set `bridge_images:=true` to also bridge RGB and depth images.
While the task is running it publishes `/fr3_vision/object_cloud`, `block_pose`,
`grasp_candidates`, and `selected_grasp`, all in `world`. Use RViz fixed frame
`world`. One-shot perception exits after writing its optional artifacts.

## Configure GraspGenX later

Use the [official GraspGenX setup](https://github.com/NVlabs/GraspGenX) on the
chosen inference machine and its [standalone server instructions](https://github.com/NVlabs/GraspGenX/blob/main/client-server/README.md).
The released model supports `robotiq_2f_85`; the legacy name-based `infer` action
returns scored base-frame grasp transforms in the input cloud's coordinate frame.
Our adapter uses that documented ZMQ/msgpack protocol and sends points in metres.

Example server commands after installing its GPU dependencies and model assets:

```bash
# On the inference machine, in the GraspGenX checkout/environment:
uv sync --extra serve
uv run python client-server/graspgenx_server.py \
  --config ext/graspgenx_checkpoints/release \
  --default_gripper robotiq_2f_85 --port 5556
```

Ensure the checkpoint path and gripper asset path match that installation. Do not
substitute Panda or Robotiq 2F-140 checkpoints/embodiments for the 2F-85.

The ROS client needs only `numpy`, `pyzmq`, `msgpack`, and `msgpack-numpy`; it does
not need torch, CUDA, or the model. One way to isolate these additions is:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-vision
.venv-vision/bin/python -m pip install -r src/fr3_vision_pick_lift/requirements-client.txt
```

The installed ROS console script normally uses `/usr/bin/python3`. To use the
isolated environment explicitly, after sourcing ROS and the workspace:

```bash
.venv-vision/bin/python -c \
  'from fr3_vision_pick_lift.vision_pick_lift import main; main()' \
  --ros-args -p use_sim_time:=true -p execute:=false \
  -p graspgen_endpoint:=tcp://INFERENCE_HOST:5556
```

Alternatively install these light client dependencies in the ROS Python
environment through its normal package management. They are not installed into
an existing environment automatically by this package.

## Preview and execute after server setup

The configuration contains `model_base_to_flange_xyz` / `_rpy`, representing
`T_model_gripper_base_from_fr3_flange`. Identity assumes matching base origins
and axes: +Z along the fingers, +X across the jaws. Verify this against the
server's actual Robotiq asset before execution. A model's base pose is not a
fingertip TCP pose: the adapter applies this mount transform once and does **not**
add the old 0.145 m TCP offset again.

Preview with the server connected and client dependencies available:

```bash
ros2 launch fr3_vision_pick_lift vision_task.launch.py \
  graspgen_endpoint:=tcp://INFERENCE_HOST:5556 execute:=false \
  output_directory:=/tmp/fr3_grasp_preview
```

This writes raw model candidates and the selected flange pose for inspection,
and previews plans without arm/gripper execution. It cannot invent a result
without the server. After verifying the frame calibration, execute:

```bash
ros2 launch fr3_vision_pick_lift vision_task.launch.py \
  graspgen_endpoint:=tcp://INFERENCE_HOST:5556
```

Edit the shared YAML for thresholds or mount calibration and pass the same
`config:=/absolute/path/vision.yaml` to both launches. It includes scene metadata,
so use these launch files rather than passing that combined YAML directly to
`ros2 run --params-file`.

"Best" here means highest model score among the filtered candidates whose
pregrasp and approach pass MoveIt, up to `max_candidates`. It is not a guarantee
of the globally best physical grasp. Predicted poses are filtered, never snapped
to vertical or replaced by the detected centroid. The existing 0.0/0.47 rad
Robotiq calibration and 6 mm release clearance remain specific to this 40 mm cube.

## Validation status

The package builds on ROS 2 Humble / Gazebo Fortress. Geometry and motion-gating
tests pass (12 tests), and both GraspGenX wire-contract tests pass in the existing
client-capable Python environment. These protocol fixtures do not run a model.
A live seed-17 scene produced 210 fused points and center
(0.454432, 0.091998, 0.040000) m, within 0.1 mm of the simulation pose;
this measures ideal simulated sensors, not real D455 accuracy. A second scene
(seed 22), with MoveIt and all controllers active, produced 225 fused points
and a center error of 0.34 mm.

This VM delivered live clouds using the normal Gazebo GUI renderer. Headless
rendering was inconsistent here; use the default launch when first validating.
Learned inference, model-frame calibration, and the complete new grasp execution
remain to be validated once the inference machine is configured.

## Simple cube workflow without GraspGenX

For this known 40 mm cube, keep `vision_sim.launch.py` running (it already starts
MoveIt), run perception as above, and manually copy the latest `center` and `yaw`
from `observation.json` into the existing task parameters:

```bash
ros2 run fr3_pick_lift moveit_pick_lift --ros-args \
  -p use_sim_time:=true \
  -p block_x:=0.5476459554778785 \
  -p block_y:=0.03329176383335662 \
  -p block_z:=0.039999983757734295 \
  -p grasp_yaw:=-0.13864798943069978
```

Those numbers are one example observation; replace them after every randomized
launch or whenever the block moves. Coordinates are metres in `world`, yaw is
radians. The cube's yaw aligns the jaws; the tool still uses its calibrated
**downward** orientation, not the cube's upright roll/pitch. The existing flange
height offset of 0.145 m is applied internally: enter the observed cube center
without adding that offset yourself. A square's equivalent yaw modulo 90 degrees
is sufficient for this face-aligned grasp.

This command executes pregrasp, descent, close, 100 mm lift, three-second hold,
replacement, open, and retreat. It does not require GraspGenX, its server, CUDA,
or the inference-client dependencies. Do not launch a second simulation or
MoveIt instance alongside `vision_sim.launch.py`.

## Select FR3 or UR3e with the same vision launch

The `robot_arm` launch argument accepts `fr3` (default) or `ur3e`:

```bash
LIBGL_DRI3_DISABLE=1 ros2 launch fr3_vision_pick_lift vision_sim.launch.py robot_arm:=fr3
LIBGL_DRI3_DISABLE=1 ros2 launch fr3_vision_pick_lift vision_sim.launch.py robot_arm:=ur3e
```

Run only one of these at a time. Each starts the selected robot, its Robotiq
controller, the matching MoveIt configuration, the randomized world, and the
same two cameras. The UR3e route includes the official simulation launcher with
our description/controller files; it does not modify that official package.
The package name remains `fr3_vision_pick_lift` to avoid restructuring it.

The perception command is unchanged for either arm:

```bash
ros2 launch fr3_vision_pick_lift vision_task.launch.py \
  perception_only:=true output_directory:=/tmp/fr3_observation
```

Then manually pass the latest observed center and yaw to the selected arm's
existing node: `ros2 run ur3e_pick_lift moveit_pick_lift` for UR3e, or
`ros2 run fr3_pick_lift moveit_pick_lift` for FR3, with the same `--ros-args`
parameters shown above. GraspGenX is not required for this manual workflow.
Do not launch an additional MoveIt instance. The non-perception GraspGenX task
in this package still targets FR3; UR3e uses its own `ur3e_pick_lift` motion node.

Cloud subscriptions use reliable delivery with a one-message queue, matching the
Gazebo bridges so large fragmented point clouds can be retransmitted on the VM.
The UR3e selector was checked live with MoveIt and all three controllers active;
seed 17 produced 210 fused object points and a center estimate within 0.1 mm of
the simulated pose. This check did not execute a UR3e pick/lift motion.
