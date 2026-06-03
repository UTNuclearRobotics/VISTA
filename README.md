# ROSMOSIS

**Robot Operating System — Multiple Object Surface Inspection Survey** — a ROS 2 Humble simulation workspace for UUV-based NBV inspection and Bayesian search missions using BT.cpp v4 behavior trees.

The system combines:
- Dubins-path vehicle simulation with idle drift dynamics
- Next-Best-View (NBV) inspection of detected targets on a helical viewpoint manifold
- Bayesian occupancy search to find targets in unexplored space
- FLS (Forward-Looking Sonar) sensor simulation via ray casting

## Repository Structure

```
src/
├── demo_behaviors/       # BT.cpp v4 nodes, behavior trees, and full mission launch
├── vista_sim/            # Dubins action server, vehicle kinematics, environment launch
├── sensor_model/         # FLS ray-cast sensor publisher and environment configs
├── uuv_interfaces/       # ROS 2 message, service, and action definitions
├── helix_generator/      # Python service: helix-sampled viewpoint generation
├── eca_a9_description/   # URDF/xacro for the ECA A9 UUV model
├── bayesian_search/      # Bayesian occupancy map and search pose server
├── nbv_cpp/              # TSDF + NBV scoring server (submodule)
├── sample_nbv_behaviors/ # NBV behavior tree nodes (submodule, rosmosis branch)
└── nrg_behaviors/        # Utility BT nodes: PublishTransform, RepeatWhile, etc. (submodule)
```

## Prerequisites

Clone the three external submodules into `src/`:

```bash
cd ~/projects/VISTA_ws/src

# NBV behaviors — use rosmosis branch (main requires non-public magellan_interfaces)
git clone git@github.com:UTNuclearRobotics/sample_nbv_behaviors.git
cd sample_nbv_behaviors && git checkout rosmosis && cd ..

# NRG utility behaviors
git clone git@github.com:UTNuclearRobotics/nrg_behaviors.git

# NBV TSDF server
git clone git@github.com:UTNuclearRobotics/nbv_cpp.git
```

Follow each submodule's README for any additional setup.

## Build

```bash
cd ~/projects/VISTA_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`-DCMAKE_BUILD_TYPE=Release` is required for `nbv_cpp` — its TSDF integration and ray-cast loops are significantly slower in debug mode. The Python packages (`vista_sim`, `helix_generator`, `sensor_model`) are unaffected.

For a partial build (sim + mission stack only, skipping nbv_cpp):

```bash
colcon build --packages-up-to demo_behaviors --symlink-install
source install/setup.bash
```

## Launch

### Full mission

```bash
ros2 launch demo_behaviors demo_mission_launch.py
```

Starts: vehicle sim, FLS sensor, NBV server, helix service, Bayesian search server, behavior tree runner, RViz, rqt_console, rqt_graph.

### With a specific environment

```bash
ros2 launch demo_behaviors demo_mission_launch.py environment:=env_50x50_cluster_seabed
```

| Environment | Description |
|---|---|
| `environment_basic` | Flat seabed, no objects (default) |
| `environment_seabed_basic` | Procedural rolling seabed |
| `environment_50x50` | Flat 50×50m arena, four boxes at corners |
| `env_50x50_centroidBox` | Flat 50×50m arena, single box offset from centroid |
| `env_50x50_cluster` | Flat 50×50m arena, three boxes clustered mid-arena + one corner |
| `env_50x50_cluster_seabed` | Procedural seabed with divot at cluster site, same box layout |

### Key launch parameters

```bash
ros2 launch demo_behaviors demo_mission_launch.py \
    environment:=env_50x50_cluster_seabed \
    drift_velocity:=0.1 \
    constant_velocity:=0.8 \
    log_level:=debug
```

| Parameter | Default | Description |
|---|---|---|
| `environment` | `environment_basic` | Environment yaml from `sensor_model/config/` |
| `start_rviz` | `true` | Launch RViz |
| `drift_velocity` | `0.25` | Idle drift speed between goals (m/s) |
| `constant_velocity` | `0.5` | Navigation speed along Dubins paths (m/s) |
| `time_step` | `0.1` | Simulation dt (s) |
| `log_level` | `info` | ROS log level for all nodes (debug/info/warn/error/fatal) |

See all args: `ros2 launch demo_behaviors demo_mission_launch.py --show-args`

## Mission Architecture

### Behavior Trees

**MainTree** — top-level mission loop (4 cycles):
- `DetectAndSortQueue` runs continuously in parallel, accumulating detected targets sorted by distance
- `ReactiveFallback` switches between two branches each tick:
  - **Inspection branch**: if queue is non-empty, run `NBVOnTarget` on the front target
  - **Search branch**: if queue is empty and within time budget, run `BayesianSearch` to drive toward high-probability unexplored regions

**NBVOnTarget** — per-target inspection:
1. Publish ROI frame at target position
2. Activate NBV policy, generate helix viewpoints, score with TSDF
3. Iterate: drive to best view → re-score → repeat until saturation
4. Conclude policy, mark target complete

**BayesianSearch** — occupancy-driven exploration:
- Queries `bayesian_search_server` for the next high-utility pose
- Drives there via `DubinsClient`
- Sensor observations update the occupancy map, reducing uncertainty

### Vehicle Simulation

`vehicle_sim_server` is the sole publisher of `ned→base_link`. It:
- Plans Dubins paths to action goals and follows them with an LOS controller
- Propagates idle drift between goals at `drift_velocity`
- Declares a goal complete when within 1m + 0.3 rad, or when the path is fully consumed and the vehicle is receding from closest approach (miss detection)

### Sensor Model

Ray-cast FLS sensor in `sensor_model`. Publishes:
- `/image_raw` + `/camera_info` — synthetic depth image in `sonar_optical` frame
- `/detected_boxes` — detected object poses with geometry IDs
- Mesh markers for RViz visualization

### Live Monitoring

The BT runner publishes its state on TCP port 1667 via `BT::Groot2Publisher`:

1. Download Groot2 AppImage from [behaviortree.dev](https://www.behaviortree.dev/groot/)
2. Monitor → Connect to `localhost:1667`
3. Watch nodes light up live as the tree ticks

## Related Package READMEs

- [vista_sim/README.md](src/vista_sim/README.md) — vehicle sim parameters, environments, TF verification
- [demo_behaviors/README.md](src/demo_behaviors/README.md) — behavior node reference, custom behavior table, architecture details