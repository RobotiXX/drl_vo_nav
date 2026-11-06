# DRL-VO for ROS 2 and RWD

This workspace contains the Nav2 controller adaptation of the pretrained
[DRL-VO](https://doi.org/10.1109/TRO.2023.3257549) policy. The local RWD
integration keeps semantic planning, task lifecycle, cancellation, door and
elevator behavior, and experiment logging in the RWD stack. DRL-VO replaces
only Nav2's `FollowPath` implementation; Theta* remains the global planner.

The repository includes the upstream pretrained archive at
`nav2py_drl_vo_controller/model/drl_vo.zip` (SHA-256
`78a6bc9918b092e2bd26782664248d9ce06a5498e7cd086260dbfe6548f99645`). No
additional checkpoint download is required.

## Important interpretation boundary

The policy has three inputs: a short lidar history, a robot-frame subgoal, and
a two-channel pedestrian velocity map. The original Humble adapter replaced
the pedestrian map with zeros. Such execution tests the learned lidar/subgoal
policy mechanics, but it is **not** a test of the paper's social DRL-VO method.

This adaptation exposes two explicit modes:

- `social` requires fresh pedestrian velocity maps and fails closed when they
  are unavailable or stale.
- `lidar_ablation` deliberately supplies the checkpoint with a zero pedestrian
  map. Use it only for controller plumbing, static-obstacle simulation, and
  regression testing. Do not report it as the social-navigation condition.

The social input is a `std_msgs/msg/Float32MultiArray` on
`/drl_vo/pedestrian_velocity_map`. It must contain 12,800 finite values: first
the flattened 80-by-80 robot-frame longitudinal-velocity channel, then the
lateral-velocity channel. The grid covers `x=[0,20] m` and `y=[-10,10] m` at
`0.25 m/cell`, matching the released training code. A replaceable perception
adapter must transform tracked people and their velocities into the robot
frame before rasterizing this message. That adapter is not supplied by this
external repository. The parent ScoutMini/RWD repository uses the same source
as the original DRL-VO hardware deployment: native ZED2 `ObjectsStamped`
position and velocity are converted to `rwd_interfaces/TrackedPersons`, then
the reusable `rwd_perception` rasterizer produces the policy input.

## Build

Ubuntu 22.04, ROS 2 Humble, Python 3.10, and a platform-appropriate PyTorch
installation are required. PyTorch is intentionally not downloaded by the
package build. This lets an x86 workstation use its normal wheel and a Jetson
use NVIDIA's JetPack-compatible wheel.

Verify PyTorch first:

```bash
python3 -c "import torch; print(torch.__version__)"
```

Then build the external overlay:

```bash
cd /path/to/ScoutMini/drl_vo_nav/nav2py_drl_vo
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  ament_cmake_venv nav2py nav2py_drl_vo_controller
source install/setup.bash
```

The controller package creates a local virtual environment that inherits the
platform PyTorch installation. It installs Stable-Baselines3 1.1.0, Gym
0.23.1, Cloudpickle, and PyYAML there. Gym 0.23.1 is used because the upstream
Gym 0.18.0 pin has no Python 3.10 wheel; the released checkpoint loads without
state-dictionary changes under Stable-Baselines3 1.1.0.

## RWD simulation

Source the RWD workspace, ScoutMini adapter workspace, and this workspace in
that order. To exercise the social-controller input and freshness checks in the
current static Fuse simulation:

```bash
cd /path/to/ScoutMini
source rwd_ws/install/setup.bash
source ros2_ws/install/setup.bash
source drl_vo_nav/nav2py_drl_vo/install/setup.bash
DRL_VO_MODE=social ./scripts/sim_byobu
```

This command renders the normal `rwd_config` `nav2_sim` profile, replaces only
`FollowPath`, records the selected mode plus checkpoint/configuration hashes in
the task experiment context, and leaves the dashboard/task-executor workflow
unchanged. `DoorTraversal` and `ElevatorAdaptiveNoSpin` retain their existing
specialized controllers. The default
`DRL_VO_SIM_PEOPLE_SOURCE=empty` starts a simulation-only publisher of fresh
empty `TrackedPersons` plus the production rasterizer. This path produces a
fresh zero pedestrian map and validates the interface watchdog, but it is not a
human-perception or social-navigation experiment.

Use `DRL_VO_SIM_PEOPLE_SOURCE=external DRL_VO_MODE=social` when another
simulation node supplies a validated `/drl_vo/pedestrian_velocity_map`. Use
`DRL_VO_MODE=lidar_ablation` for controller mechanics without any pedestrian
topic; that mode deliberately fills the observation internally with zeros.

For ScoutMini hardware, `nav2_byobu` enables ZED2 SDK object detection and
tracking, starts the ZED-to-RWD adapter, and starts the rasterizer:

```bash
NAVIGATION_MODE=drl_vo ./scripts/nav2_byobu
```

The upstream `deploy` branch used a 10 m by 10 m, 0.125 m/cell raster even
though the simulation/training branch used the 20 m by 20 m, 0.25 m/cell
raster documented above. The RWD integration adopts the original native ZED
tracking boundary but keeps the training-branch raster convention consistently
across simulation and hardware.

The current Fuse simulation contains no moving-person sensor pipeline. The
empty mock supports end-to-end controller testing only; launch a validated
simulated `TrackedPersons` provider before claiming a social-navigation result.

Hardware study selection uses the mutually exclusive
`NAVIGATION_MODE=dwb|drl_vo|vint` contract because the learned methods replace
the same `FollowPath` slot. `DRL_VO_MODEL_FILE` can override the installed
policy checkpoint. Simulation retains `DRL_VO_MODE` so its social and
lidar-ablation development variants remain selectable. The hardware wrapper
records SHA-256 hashes for the effective Nav2 profile, DRL-VO policy,
controller configuration, and ZED perception configuration in the
task-associated experiment context.

## Training-compatible preprocessing

The adapter converts arbitrary full-circle `LaserScan` resolution into the
policy's fixed input without assuming a 1080-ray sensor:

1. The `[-120, 120] degree` field of view is nearest-neighbor resampled to 720
   values. Invalid rays become zero, matching the original data pipeline.
2. Ten **new sensor messages**, rather than ten controller ticks, form the
   history. Each frame is reduced to 80 minimum and 80 mean bins of nine rays.
3. The 1,600 scan features are repeated four times, scaled from `[0,30] m` to
   `[-1,1]`, and concatenated with the 12,800-value pedestrian map and 2-D
   subgoal.
4. Pedestrian velocities and the 2 m subgoal are scaled from `[-2,2]` to
   `[-1,1]`, as in the original ROS 1 inference code.

The resulting observation has 19,202 values and the action has two values. The
action is converted to forward speed in `[0,max_linear_speed]` and yaw rate in
`[-max_angular_speed,max_angular_speed]`.

## Safety and Nav2 behavior

The Python policy returns velocity values to the C++ Nav2 controller over a
loopback socket. Large policy arrays use base64-encoded, big-endian float32
storage, and the receiver reads buffered chunks; the original byte-at-a-time
YAML transport could exceed the controller deadline. One discarded policy
inference warms the model before Nav2 activates the controller. The C++ side
enforces five fail-closed checks:

1. Scan geometry, history length, freshness, and finite values must be valid.
2. Social mode requires a fresh, correctly sized pedestrian map.
3. Python must answer before `command_timeout`.
4. The result must be finite and is clamped to the configured speed limits,
   including Nav2 speed-limit updates.
5. A constant-command rollout checks the padded robot footprint against the
   live local costmap. A lethal, unknown, or off-map footprint stops the robot.

The rollout is a final safety filter, not a proof of collision-free learned
navigation. It checks the returned command for the configured horizon and does
not evaluate an uncertainty distribution or a sequence of future policy
outputs. Supervised simulation, hardware e-stop coverage, conservative speed
limits, and a validated pedestrian provider remain required before a user
study around people.

The rotation shim remains outside DRL-VO. Consequently, normal destinations
retain the RWD goal-checker behavior and typed door approaches can request a
tighter final yaw without changing the learned policy. The shim can perform a
costmap-checked in-place alignment before the learned controller is called;
therefore, an absent pedestrian raster prevents DRL-VO translation but does not
prohibit that initial shim rotation.

The RWD Nav2 controller server runs at **10 Hz** for controller comparisons.
The released ROS 1 deployment generates the observation history and triggers
policy inference at 10 Hz, even though its launch argument, velocity smoother,
and Humble Nav2 example contain 20 Hz settings. The RWD profile follows the
actual policy-command rate. MPPI profiles use `model_dt: 0.1`; their
`time_steps` values are reduced correspondingly so changing the update period
does not double their prediction horizons. The velocity smoother can continue
publishing between controller updates at 20 Hz.

The policy device is configurable. `model_device: auto` selects CUDA when the
installed PyTorch runtime exposes it and otherwise falls back to CPU. Freeze it
to `cuda` or `cpu` when a study condition requires an explicit device; always
measure checkpoint warmup and sustained inference on the deployment machine
against `command_timeout` and the controller period.

## Controller parameters

The RWD profile is
`nav2py_drl_vo_controller/config/rwd_controller.yaml`. Important parameters
are:

- `lookahead_dist`: robot-frame path subgoal distance; `2.0 m` matches training.
- `model_scan_angle_min/max`: the policy's fixed 240-degree lidar view.
- `scan_timeout` and `pedestrian_timeout`: maximum sensor age.
- `require_pedestrian_data`: true for the social policy, false only for the
  named lidar ablation.
- `max_linear_speed` and `max_angular_speed`: physical action scaling and hard
  output clamps.
- `min_obstacle_distance`: forward lidar emergency-stop threshold.
- `collision_check_horizon` and `collision_check_step`: local-costmap rollout.
- `command_timeout`: maximum synchronous Python inference wait.
- `model_device`: Stable-Baselines3 inference device (`auto`, `cpu`, or a CUDA device accepted by PyTorch).

## Tests

```bash
cd /path/to/ScoutMini/drl_vo_nav/nav2py_drl_vo
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=87
colcon test --packages-select nav2py_drl_vo_controller \
  --event-handlers console_cohesion+
colcon test-result --verbose
```

The package tests the lidar geometry conversion, observation size and scaling,
action scaling, obstacle stop, malformed input handling, and non-destructive
RWD profile composition. Checkpoint load and inference should also be timed on
each deployment machine because PyTorch performance is platform dependent.

## Remaining work for a valid social-navigation condition

- Add synchronized pedestrian-map timestamps; `Float32MultiArray` currently
  records receipt freshness but has no source timestamp.
- Validate ZED2 position, velocity, identity continuity, camera mounting TF,
  and occlusion behavior with recorded and annotated scenes.
- Run controlled moving-person simulation and then supervised low-speed robot
  trials before the participant study.

## Citation

```bibtex
@article{xie2023drl,
  author={Xie, Zhanteng and Dames, Philip},
  journal={IEEE Transactions on Robotics},
  title={{DRL-VO}: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles},
  year={2023},
  volume={39},
  number={4},
  pages={2700--2719},
  doi={10.1109/TRO.2023.3257549}
}
```
