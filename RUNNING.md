# Running the rover — start here

The one file to open before touching a terminal. It's a sequence of copy-paste
commands, nothing more — for *why* things are built this way, see
[`rover_control/README.md`](rover_control/README.md) and
[`rover_hardware/README.md`](rover_hardware/README.md).

Two kinds of command below:
- **[LEAVE RUNNING]** — starts a long-lived program. Keep its terminal open;
  closing it or `Ctrl+C` stops that program and whatever depends on it.
- **[RUNS ONCE]** — does one thing and exits. Safe to reuse any terminal
  that's already finished its previous one-shot command.

Terminal numbers reset at the start of each section — "Terminal 1" in
Simulation and "Terminal 1" in Real hardware are not the same window.

## 0. One-time setup

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```
Open a fresh terminal after this (or `source ~/.bashrc`) so `ros2`/`colcon`
work. The second line errors harmlessly until step 1 below has built
something once.

**If `ros2 launch` ever says a file "was not found" for a package that
clearly exists**, your install is stale — see
[Troubleshooting](#troubleshooting) below before anything else.

## 1. Simulation

**Terminal 1 — build** `[RUNS ONCE]`
```bash
cd ~/ros2_ws
colcon build --packages-select rover_control --symlink-install
source install/setup.bash
```
Always build from `~/ros2_ws`, not from inside `src/`. Checkpoint: no red
`Failed` lines.

**Terminal 1 — launch** `[LEAVE RUNNING]`
```bash
ros2 launch rover_control rover_sim.launch.py
```
Checkpoint: Gazebo + RViz windows open, no `[ERROR]` lines. Leave this
running for everything else in this section.

**Terminal 2 — confirm the lidar** `[RUNS ONCE]`
```bash
ros2 topic echo /scan --once
```
Real numbers near the interior boxes, not all `.inf`.

**Terminal 3 — drive it** `[LEAVE RUNNING while driving]`
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i`/`,` forward/back, `j`/`l` turn, `k` stop.

### Build a map

Launch args are fixed at startup, so restart Terminal 1 to add `slam:=true`:
```bash
# Terminal 1: Ctrl+C, then:
ros2 launch rover_control rover_sim.launch.py slam:=true
```
Drive a full loop back near your start point (Terminal 3), watching the map
fill in in RViz. Then save it, Terminal 2:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/rover_room --ros-args -p use_sim_time:=true
```
If this fails with "Failed to spin map subscription": `slam:=true` isn't
running, or you haven't driven ~0.5m yet.

### View the saved map

No need to relaunch anything above — one command, any time:
```bash
ros2 launch rover_control view_map.launch.py map:=~/maps/rover_room.yaml
```

### Nav2

Needs `sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup` once.
Alongside Terminal 1 (still running with `slam:=true`):

**Terminal 4** `[LEAVE RUNNING]`
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$(ros2 pkg prefix rover_control)/share/rover_control/config/nav2_params.yaml \
  use_sim_time:=true
```
Sanity-check first: `echo $(ros2 pkg prefix rover_control)` should print a
real path.

### Autonomy FSM

Restart Terminal 1 with both flags:
```bash
# Terminal 1: Ctrl+C, then:
ros2 launch rover_control rover_sim.launch.py slam:=true autonomy:=true
```
With Terminal 4 (Nav2) also running, Terminal 2:
```bash
ros2 service call /autonomy/start_explore std_srvs/srv/Trigger
ros2 topic echo /autonomy/state
```
Safety behaviors, also Terminal 2:
```bash
ros2 topic pub /autonomy/emergency_stop std_msgs/msg/Bool "{data: true}" --once
ros2 service call /autonomy/clear_emergency std_srvs/srv/Trigger
ros2 service call /autonomy/return_home std_srvs/srv/Trigger
```
Don't run teleop on `/cmd_vel` at the same time as the FSM — see
`rover_control/README.md` for why.

## 2. Real hardware

Full wiring/pin/calibration details are in
[`rover_hardware/README.md`](rover_hardware/README.md) — this is just the
command sequence. **Wheels off the ground on blocks, every time, until you
trust this.**

**Terminal 1 — one-time: install lgpio, find your GPIO chip**
```bash
git clone https://github.com/joan2937/lg && cd lg && make && sudo make install && sudo ldconfig
gpiodetect   # note the chip number labeled pinctrl-rp1 - don't assume 0
```

Fill in every `TODO` in
`rover_hardware/urdf/rover_hardware.ros2_control.xacro` (pins,
`encoder_counts_per_rev`, `gpio_chip`) before building — full parameter
reference is in `rover_hardware/README.md`.

**Terminal 1 — build both packages** `[RUNS ONCE]`
```bash
cd ~/ros2_ws
colcon build --packages-select rover_control rover_hardware --symlink-install
source install/setup.bash
```

**Terminal 1 — bring up the hardware interface** `[LEAVE RUNNING]`
```bash
ros2 launch rover_hardware rover_hardware.launch.py
```

**Terminal 2 — confirm it claimed everything, check wheel signs** `[RUNS ONCE, then inspect]`
```bash
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states   # spin each wheel by hand; negative = swap that wheel's encoder_a/b_pin
```
**Don't move on until every wheel's sign is correct.**

**Terminal 2 — first commanded motion, still on blocks** `[RUNS ONCE]`
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --rate 10
```
Leave `velocity_kp`/`velocity_ki` at `0.0` until direction/speed look sane.

**Wheels down:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3 — lidar** `[LEAVE RUNNING]`
```bash
ros2 launch sllidar_ros2 sllidar_c1_launch.py
ros2 topic echo /scan --once   # sanity check, same as sim
```

**SLAM / Nav2 / autonomy — identical to sim, `use_sim_time:=false`:**

**Terminal 4**
```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file $(ros2 pkg prefix rover_control)/share/rover_control/config/slam_params.yaml \
  -p use_sim_time:=false
```

**Terminal 5**
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$(ros2 pkg prefix rover_control)/share/rover_control/config/nav2_params.yaml \
  use_sim_time:=false
```

**Terminal 6**
```bash
ros2 run rover_control autonomous_decision_maker --ros-args -p use_sim_time:=false
```

Map saving/viewing: identical commands to the sim section, just drop
`use_sim_time:=true`.

**Not yet built:** the ICM-20948 IMU driver — doesn't block anything above,
wheel odometry alone drives SLAM/Nav2/the FSM fine.

## Troubleshooting

Work down this ladder, stop at the first thing that looks wrong:
```bash
ros2 pkg prefix rover_control      # (1) is this terminal sourced, package built?
ros2 node list                     # (2) is the process actually running?
ros2 topic list                    # (3) does the topic exist?
ros2 topic hz /scan                # (4) is data flowing?
ros2 topic echo /scan --once       # (5) does it look sane?
```

**A package's `ros2 pkg prefix` succeeds but `ros2 launch ... some_file.launch.py`
says "file not found"** — your install is stale relative to the source
(usually after moving/renaming a package directory; colcon doesn't fix old
installs on its own). Fix:
```bash
cd ~/ros2_ws
rm -rf build/rover_control install/rover_control log
colcon build --packages-select rover_control --symlink-install
source install/setup.bash
```

**Gazebo won't open / crashes (WSL2)** — `export LIBGL_ALWAYS_SOFTWARE=1`
before launching.

**`map_saver_cli`: "Failed to spin map subscription"** — `slam:=true` isn't
running, or the robot hasn't moved ~0.5m yet.

**TF "extrapolation into the future/past"** — a `use_sim_time` mismatch
between nodes; sim session needs it `true` everywhere, real-hardware session
needs it `false` everywhere.

For the full symptom → cause → fix catalog and architecture reference, see
the two package READMEs.
