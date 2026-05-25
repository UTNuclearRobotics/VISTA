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
| `drift_service` | Propagates vehicle at constant velocity, exposes pause/resume services |
| `dubins_pose_to_pose_action_server` | Plans and executes Dubins paths to goal poses |
| `dubins_pose_to_pose_action_client` | Sends a parameterized goal to the action server |

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `pose_to_pose` | Action (`PoseToPose`) | Navigate to a goal pose |
| `pause_drift` | Service (`PauseDrift`) | Stop drift, return current state |
| `resume_drift` | Service (`ResumeDrift`) | Resume drift from given state |

## Parameters

| Parameter | Default | Used by | Description |
|-----------|---------|---------|-------------|
| `frame_id` | `ned` | Both | TF parent frame |
| `time_step` | `0.1` | Both | Simulation dt (seconds) |
| `drift_velocity` | `0.25` | Drift service | Idle drift speed (m/s) |
| `constant_velocity` | `0.5` | Action server | Navigation speed for Dubins paths (m/s) |
| `log_level` | `info` | All Python nodes | ROS log level (debug/info/warn/error/fatal) applied to drift_service, vehicle_sim_server, meshes_rviz, depth_publisher |

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
