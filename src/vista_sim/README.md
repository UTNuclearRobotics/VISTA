# vista_sim

Simulation package for UUV kinematics with Dubins path planning and drift dynamics.

## Quick Start

### Build

```bash
cd ~/projects/VISTA_ws
colcon build --packages-up-to demo_behaviors --symlink-install
source install/setup.bash
```

### Launch

```bash
# Launch with default flat seabed
ros2 launch vista_sim vista_sim_launch.py

# Launch with a specific environment
ros2 launch vista_sim vista_sim_launch.py environment:=environment_50x50

# Launch without RViz
ros2 launch vista_sim vista_sim_launch.py start_rviz:=false

# Override velocity and time step
ros2 launch vista_sim vista_sim_launch.py drift_velocity:=0.1 constant_velocity:=1.0 time_step:=0.05
```

The `environment` argument selects a config yaml from `sensor_model/config/` (omit the `.yaml` extension).

| Environment | Description |
|---|---|
| `environment_basic` | Flat seabed, no objects (default) |
| `environment_seabed_basic` | Procedural rolling seabed with roughness |
| `environment_50x50` | Flat 50×50m arena, four boxes at corners |
| `env_50x50_centroidBox` | Flat 50×50m arena, single box offset from centroid |
| `env_50x50_cluster` | Flat 50×50m arena, three boxes clustered mid-arena + one corner |
| `env_50x50_cluster_seabed` | Procedural seabed with divot at cluster site, same box layout |

### Send a Goal (separate terminal)

```bash
source ~/projects/VISTA_ws/install/setup.bash
ros2 action send_goal --feedback /pose_to_pose uuv_interfaces/action/PoseToPose \
  "{goal_pose: {header: {frame_id: 'ned'}, pose: {position: {x: 25.0, y: 25.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Verify TF broadcast

```bash
ros2 run tf2_ros tf2_echo ned base_link
ros2 run tf2_ros tf2_monitor ned base_link
```

Expected: ~10 Hz, single publisher (`vehicle_sim_server`).

## Nodes

| Node | Description |
|------|-------------|
| `dubins_pose_to_pose_action_server` | Plans and executes Dubins paths to goal poses; sole owner of `ned→base_link` TF |
<<<<<<< HEAD
=======
| `dubins_pose_to_pose_action_client` | Sends a parameterized goal to the action server |
>>>>>>> 06bc889fddc8f1418d2771b78d6614e29308a2cd

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