# vista_sim

Simulation package for UUV kinematics with Dubins path planning and drift dynamics.

## Quick Start

### Build

```bash
cd ~/projects/VISTA_ws
colcon build --packages-select uuv_interfaces vista_sim
source install/setup.bash
```

### Launch

```bash
# Launch with default flat seabed
ros2 launch vista_sim vista_sim_launch.py

# Launch with procedural rolling seabed
ros2 launch vista_sim vista_sim_launch.py environment:=environment_seabed_basic

# Launch with hill/valley terrain
ros2 launch vista_sim vista_sim_launch.py environment:=environment_hill_valley

# Launch without RViz
ros2 launch vista_sim vista_sim_launch.py start_rviz:=false

# Override parameters
ros2 launch vista_sim vista_sim_launch.py drift_velocity:=0.1 constant_velocity:=1.0 time_step:=0.05
```

The `environment` argument selects a config yaml from `sensor_model/config/` (omit the `.yaml` extension).

| Environment | Description |
|---|---|
| `environment_basic` | Flat 200x200m seabed (default) |
| `environment_seabed_basic` | Procedural rolling seabed with roughness |
| `environment_hill_valley` | Valley at box_0 (0,10), hill at box_1 (0,20) |

### Send a Goal (separate terminal)

```bash
# Source first
source ~/projects/VISTA_ws/install/setup.bash

# Send goal via action client node
ros2 run vista_sim dubins_pose_to_pose_action_client \
  --ros-args -p goal_x:=5.0 -p goal_y:=5.0 -p goal_z:=0.0 -p goal_yaw:=0.0
```

### Verify Drift Service

```bash
# Check TF is broadcasting (vehicle should be moving)
ros2 run tf2_ros tf2_echo ned base_link
```

## Nodes

| Node | Description |
|------|-------------|
| `dubins_pose_to_pose_action_server` | Plans and executes Dubins paths to goal poses; sole owner of `ned→base_link` TF |
| `dubins_pose_to_pose_action_client` | Sends a parameterized goal to the action server |

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `pose_to_pose` | Action (`PoseToPose`) | Navigate to a goal pose |

## Parameters

### Launch-controlled (set via launch file or command line)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_id` | `ned` | TF parent frame for `base_link` |
| `time_step` | `0.1` | Simulation dt (seconds) |
| `constant_velocity` | `0.5` | Navigation speed along Dubins paths (m/s) |
| `drift_velocity` | `0.25` | Idle drift speed between goals (m/s) |
| `log_level` | `info` | ROS log level for all Python nodes (debug/info/warn/error/fatal) |

### Hardcoded vehicle dynamics (in `execute_cb`)

These govern the `SimpleVehicleModel` and `DubinsAirplanePath` and require a code change to tune.

| Parameter | Value | Description |
|-----------|-------|-------------|
| `turn_radius_m` | `0.5` | Minimum Dubins turn radius (m); also used by the planner |
| `look_ahead_dist` | `1.0` | LOS follower look-ahead distance (m); affects path tracking tightness |
| `speed_time_constant` | `2.0` | First-order lag on surge speed response |
| `yaw_time_constant` | `0.2` | First-order lag on yaw response |
| `pitch_time_constant` | `1.5` | First-order lag on pitch response |
| `max_acceleration_mps2` | `1.0` | Surge acceleration limit (m/s²) |
| `max_pitch_deg` | `15.0` | Pitch angle limit (degrees); also used by the planner |
| `pitch_proportional_gain` | `0.5` | Proportional gain for depth/pitch control |
| `roll_ratio` | `0.2` | Roll coupling ratio during turns |

### Log level usage

```bash
# Default (info)
ros2 launch vista_sim vista_sim_launch.py

# Debug all Python nodes
ros2 launch vista_sim vista_sim_launch.py log_level:=debug

# Quiet (warnings/errors only)
ros2 launch vista_sim vista_sim_launch.py log_level:=warn
```

When launched via `demo_behaviors/demo_mission_launch.py`, the `log_level` arg is forwarded from the parent, so a single launch arg controls verbosity across the whole stack.
