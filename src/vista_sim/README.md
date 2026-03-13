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
# Launch all nodes (drift service, action server, sensor model, RViz)
ros2 launch vista_sim vista_sim_launch.py

# Launch without RViz
ros2 launch vista_sim vista_sim_launch.py start_rviz:=false

# Override parameters
ros2 launch vista_sim vista_sim_launch.py drift_velocity:=0.1 constant_velocity:=1.0 time_step:=0.05
```

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
| `max_runtime` | `300.0` | Action server | Goal timeout (seconds) |
