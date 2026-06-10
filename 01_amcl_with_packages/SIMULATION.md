# Terminal 1 — bringing up a robot that publishes `/scan` and `odom → base_link`

AMCL doesn't drive a robot or read sensors directly — it consumes a `LaserScan`
and the `odom → base_frame` TF, and produces the `map → odom` correction. So
before launching our localization (Terminal 2), something has to provide that
sensor + odometry stream. The easiest source is the **TurtleBot3 Gazebo**
simulation. This guide sets it up end to end.

> **Two gotchas this guide handles up front**
> 1. **The map must match the world.** AMCL localizes *in a map*. The synthetic
>    `maps/tutorial_map.*` in this package does **not** match the Gazebo world,
>    so you must use a map of the world you actually simulate (built with SLAM,
>    or the prebuilt TurtleBot3 map). See [§4](#4-get-a-map-that-matches-the-world).
> 2. **TurtleBot3's base frame is `base_footprint`, not `base_link`.** Set
>    `base_frame_id: base_footprint` in [`config/amcl.yaml`](amcl_nav_pkg/config/amcl.yaml)
>    (it's flagged there too).

---

## 1. Install the simulator

Pick the block for your ROS 2 distro. (`$ROS_DISTRO` is `humble`, `jazzy`, etc.)

### Humble (Gazebo **Classic**)
```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-cartographer \
  ros-humble-nav2-bringup
```

### Jazzy (new **Gz** / Gazebo Harmonic)
```bash
sudo apt update
sudo apt install \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox
```
On Jazzy the world launch file is the same name but uses `ros_gz_bridge` to
bridge `/scan`, `/odom`, and `/tf` from Gz into ROS 2 — the topic/TF names you
care about are identical, so the rest of this guide is unchanged.

---

## 2. Environment setup

Add these to your `~/.bashrc` (or run them in every new terminal):
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
export TURTLEBOT3_MODEL=burger        # or 'waffle' / 'waffle_pi'

# Humble / Gazebo Classic only — let Gazebo find the TurtleBot3 models:
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models
```
Then `source ~/.bashrc`.

---

## 3. Launch the Gazebo world (this is "Terminal 1")

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

This single launch starts:
- **Gazebo** with the TurtleBot3 "world" (a walled arena with cylinders),
- the **differential-drive plugin** → publishes `/odom` and the `odom → base_footprint` TF,
- the **laser plugin** → publishes `/scan` (`sensor_msgs/LaserScan`),
- **`robot_state_publisher`** → the rest of the robot TF tree.

### Verify it's actually publishing what AMCL needs
In a second terminal:
```bash
ros2 topic hz /scan                       # should report ~5 Hz
ros2 topic echo /scan --once | head        # see ranges/angle_min/angle_max
ros2 run tf2_ros tf2_echo odom base_footprint   # the odom->base TF AMCL consumes
ros2 run tf2_tools view_frames             # writes frames.pdf of the whole TF tree
```
If `/scan` is flowing and `tf2_echo odom base_footprint` prints transforms, the
robot side is ready.

### Drive the robot (so AMCL has motion to converge on)
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Use `w/x` (linear) and `a/d` (angular) to move. AMCL only updates after the robot
moves `update_min_d` / `update_min_a` (see `amcl.yaml`), so a stationary robot's
cloud won't converge.

---

## 4. Get a map that matches the world

AMCL needs an occupancy grid of **this** Gazebo world. Two ways:

### Option A — build it yourself with SLAM (recommended; distro-agnostic)
With the Gazebo world running (§3):
```bash
# Humble:
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
# Jazzy (slam_toolbox):
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# In another terminal, drive around with teleop until the map looks complete,
# then save it:
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_world_map
```
That writes `~/turtlebot3_world_map.pgm` + `.yaml`. Copy them into this package
and point our launch at them:
```bash
cp ~/turtlebot3_world_map.* \
   /home/jysim/claude/nav_mecha_2026/01_amcl_with_packages/amcl_nav_pkg/maps/
```

### Option B — use the prebuilt TurtleBot3 map (quick start)
`nav2_bringup` / `turtlebot3_navigation2` ship a ready-made OGM of the
**`turtlebot3_world`** arena — use it directly, no SLAM needed.

**Confirmed prebuilt maps on this machine (Humble):**
```
/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml      <- recommended
/opt/ros/humble/share/turtlebot3_navigation2/map/map.yaml          <- identical copy
```
Both are the same map (resolution 0.05 m/cell, origin [-10, -10, 0]). Pass it as
the `map:=` argument in Terminal 2:
```bash
ros2 launch amcl_nav_pkg localization.launch.py \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    use_sim_time:=true
```

> **Only `turtlebot3_world` has a prebuilt map.** There is **no** bundled OGM for
> `turtlebot3_house` — for the house world you must build the map yourself with
> SLAM (Option A above). To re-discover prebuilt maps on any system:
> ```bash
> find /opt/ros/$ROS_DISTRO/share -name "*.pgm"   # each .pgm has a sibling .yaml
> ```

**World ↔ map pairing** (the laser senses the *world*; AMCL matches it to the *map* — they must depict the same place):

| Gazebo world launch | Matching OGM |
|---------------------|--------------|
| `turtlebot3_world.launch.py` | `nav2_bringup/maps/turtlebot3_world.yaml` ✅ prebuilt |
| `turtlebot3_house.launch.py` | none bundled — build with SLAM (Option A) |
| `empty_world.launch.py` | n/a — no walls, AMCL cannot localize |

---

## 5. Putting the terminals together

| Terminal | Command | Provides |
|----------|---------|----------|
| **1** | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` | `/scan`, `/odom`, `odom→base_footprint` TF |
| **2** | `ros2 launch amcl_nav_pkg localization.launch.py map:=<your_map>.yaml` | AMCL → `map→odom` TF |
| **3** | `rviz2` | visualize Map / LaserScan / ParticleCloud |
| **4** | `ros2 run turtlebot3_teleop teleop_keyboard` | drive the robot |

For Terminal 2, remember to set `base_frame_id: base_footprint` in
`config/amcl.yaml`, and pass a map that matches the world. Quickest is the
prebuilt `turtlebot3_world` OGM:
```bash
ros2 launch amcl_nav_pkg localization.launch.py \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    use_sim_time:=true
```
(Or, once you've built your own with SLAM, point `map:=` at
`amcl_nav_pkg/maps/turtlebot3_world_map.yaml` instead.)
In RViz: set **Fixed Frame** to `map`, add **Map**, **LaserScan** (topic `/scan`),
and **ParticleCloud** (topic `/particle_cloud`) displays, click **2D Pose Estimate**
to seed AMCL near the robot's true position, then drive — the particle cloud
should tighten around the robot.

---

## Alternative sources for `/scan` + odom (no Gazebo)

- **A recorded rosbag:** `ros2 bag play my_run.bag --clock` (the `--clock` flag
  publishes `/clock`; keep `use_sim_time:=true`). Make sure the bag contains
  `/scan`, `/tf`, and `/odom`.
- **A real robot:** any platform whose driver publishes a `LaserScan` and the
  `odom → base_link` TF works — just match `scan_topic` and `base_frame_id` in
  `amcl.yaml` to that robot.
- **Other simulators:** Gazebo with your own URDF, Webots (`webots_ros2`), or
  Isaac Sim — the contract is the same: publish `/scan` and `odom → base_frame`.
