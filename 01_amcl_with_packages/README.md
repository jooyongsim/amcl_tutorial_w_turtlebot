# AMCL + navigation using packages (ROS 2 + Nav2)

The "use the existing packages" track. Here the AMCL algorithm itself is provided
by **`nav2_amcl`** — your job is to *configure* it (the parameter files), *launch*
it, and *integrate* it into a navigation pipeline. Contrast with
`../02_amcl_from_scratch/`, where you implement the filter by hand.

## Requirements

- **ROS 2** Humble or Jazzy
- **Nav2**: `sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup`
- A robot or simulator that publishes a `LaserScan` on `/scan` and the
  `odom -> base_link` TF. Easiest: **TurtleBot3** in Gazebo
  (`sudo apt install ros-$ROS_DISTRO-turtlebot3*`).

## Package layout

```
amcl_nav_pkg/
├── package.xml / setup.py / setup.cfg   # ament_python package metadata
├── config/
│   ├── amcl.yaml          # AMCL params, heavily commented  <-- you edit this
│   └── nav2_params.yaml   # full Nav2 stack (incl. the amcl block)
├── launch/
│   ├── localization.launch.py   # map_server + amcl + lifecycle manager
│   └── navigation.launch.py     # AMCL + full Nav2 (planner/controller/BT)
├── maps/
│   ├── tutorial_map.pgm / .yaml  # the same OGM used by the from-scratch track
├── amcl_nav_pkg/
│   └── waypoint_navigator.py     # drive a patrol via nav2_simple_commander
└── rviz/                          # (drop your .rviz configs here)
```

## Build

```bash
# In your ROS 2 workspace (e.g. ~/ros2_ws/src):
ln -s /home/jysim/claude/nav_mecha_2026/01_amcl_with_packages/amcl_nav_pkg .
cd ~/ros2_ws
colcon build --packages-select amcl_nav_pkg
source install/setup.bash
```

## Run

### A) Localization only ("Where am I?")

```bash
# Terminal 1: a robot/sim publishing /scan and the odom->base_frame TF.
#   The full step-by-step setup (install, launch, verify, build a matching map)
#   is in SIMULATION.md. Quick version (TurtleBot3 + Gazebo):
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: localization (pass a map that MATCHES the simulated world):
ros2 launch amcl_nav_pkg localization.launch.py \
    map:=<path-to-your-world-map>.yaml use_sim_time:=true

# Terminal 3 (RViz): set Fixed Frame=map; add Map, LaserScan, ParticleCloud;
# click "2D Pose Estimate" to seed AMCL, then drive — watch the cloud converge.

# Terminal 4: drive the robot so AMCL has motion to converge on:
ros2 run turtlebot3_teleop teleop_keyboard
```

> **See [SIMULATION.md](SIMULATION.md)** for the detailed Terminal 1 walkthrough —
> installing TurtleBot3/Gazebo for Humble *and* Jazzy, verifying `/scan` and the
> TF tree, **building a map that matches the world** (the bundled
> `tutorial_map` does *not* match the Gazebo world), and the `base_footprint`
> vs. `base_link` frame gotcha.

### B) Full navigation ("Get me there")

```bash
ros2 launch amcl_nav_pkg navigation.launch.py

# In RViz: "2D Pose Estimate" to localize, then "Nav2 Goal" to send a goal.
# Or drive a programmatic patrol:
ros2 run amcl_nav_pkg waypoint_navigator
```

## What to tune (and why)

Open [`config/amcl.yaml`](amcl_nav_pkg/config/amcl.yaml) — it is commented param-by-param. The high-leverage knobs:

| Parameter | Effect | Slide |
|-----------|--------|-------|
| `min_particles` / `max_particles` | KLD-sampling bounds | 05 |
| `alpha1..alpha4` | odometry noise — too low = overconfident, too high = never converges | 04 |
| `laser_model_type` | `likelihood_field` (fast) vs. `beam` | 04 |
| `z_hit` / `z_rand` / `sigma_hit` | sensor model shape | 04 |
| `recovery_alpha_slow/fast` | kidnapped-robot recovery (set both 0 to disable) | 05 |
| `update_min_d` / `update_min_a` | how far to move before updating (CPU) | 03 |

## Suggested exercises

1. Set `recovery_alpha_slow/fast` to `0.0`, drive the robot, then manually move it
   in the sim ("kidnap") — observe that it can't recover. Re-enable and repeat.
2. Crank `alpha1..alpha4` way up, then way down — watch the particle cloud either
   never converge or collapse and lose track.
3. Switch `laser_model_type` to `beam` and compare CPU usage / robustness.
4. Compare the converged cloud here with your from-scratch `03_run_amcl.py` output —
   same algorithm, same behavior.
