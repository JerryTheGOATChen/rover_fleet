# rover_control

4-wheel differential drive rover with a planar lidar, simulated in Gazebo
Harmonic (gz-sim) and driven through native gz-sim system plugins +
`ros_gz_bridge`. Includes SLAM (slam_toolbox), Nav2 configs, and a
high-level autonomous decision-making FSM.

## Layout

```
urdf/           rover.xacro (robot geometry), ros2_control.xacro (Gazebo
                DiffDrive + JointStatePublisher plugins, gpu_lidar sensor)
worlds/         rover_world.sdf (enclosed room + Sensors system plugin)
config/         rover_bridge.yaml, slam_params.yaml, nav2_params.yaml,
                controllers.yaml (legacy, see note below)
launch/         rover_sim.launch.py
rviz/           rover.rviz
src/            lidar_safety_node.cpp, autonomous/autonomous_decision_maker.cpp
include/        matching headers
```

## Why the lidar didn't work originally

Two stacked WSL2 gz-sim gotchas:
1. **The Sensors system plugin wasn't loaded.** gz-sim only auto-loads
   Physics/UserCommands/SceneBroadcaster for a plugin-free world; any custom
   world needs `gz-sim-sensors-system` spelled out explicitly or sensors
   silently produce nothing. Fixed in `worlds/rover_world.sdf`.
2. **OGRE2 (the default renderer) is known to crash under WSL2's WSLg
   OpenGL layer**, even with a real GPU behind it. `gpu_lidar` computes
   ranges through the render engine, so a dead renderer means a dead lidar.
   `rover_world.sdf` forces the legacy `ogre` renderer instead. If that's
   still not enough: `export LIBGL_ALWAYS_SOFTWARE=1` before launching.

## Architecture note: two control paths, one description

`config/controllers.yaml` and `src/rover_control_node.cpp` (a Twist→
TwistStamped converter) are **not** used by the simulation - the sim drives
wheels with Gazebo's native `DiffDrive` system plugin instead (see
`urdf/ros2_control.xacro`), which needs no `controller_manager` at all.

They're both very much used now by the real robot, via the sibling
`rover_hardware` package - see its README for the full story. Short version:
same `controllers.yaml`, same wheel names/groupings, so
SLAM/Nav2/the autonomous FSM don't need to know or care which one is
actually running underneath.

## Build

```bash
cd ~/ros2_ws
rm -rf build/rover_control install/rover_control log
colcon build --packages-select rover_control
source install/setup.bash
```

**Always run `colcon build` from `~/ros2_ws`** (the workspace root, one level
above `src/`) - never from inside `src/` or a package directory. Doing it
from the wrong place creates a second, parallel `build`/`install` there
instead of updating the real one, and you'll end up sourcing whichever one
you happen to `cd` into next.

If you ever move or rename a package's source directory, `colcon build`
won't retroactively fix an install that was built before the move - it only
touches what you ask it to build, when you ask it to. The symptom is
confusing: `ros2 pkg prefix rover_control` succeeds (the *old* install is
still there), but `ros2 launch rover_control <file>.launch.py` fails with
"file not found," because that install's symlinks point at a source
directory that no longer exists. Fix: the `rm -rf build/rover_control
install/rover_control log` + rebuild above, from `~/ros2_ws`.

## Run the sim

```bash
ros2 launch rover_control rover_sim.launch.py
```

Verify the lidar: `ros2 topic hz /scan` / `ros2 topic echo /scan --once`
(should show finite ranges near the interior boxes, not all `inf`).

Drive it: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

## Mapping workflow

The world is an enclosed ~8x8m room with interior obstacles specifically so
there's structure worth mapping.

```bash
ros2 launch rover_control rover_sim.launch.py slam:=true
```
Starts `async_slam_toolbox_node` against `config/slam_params.yaml` unchanged.
RViz's Map display is wired to `/map`, Fixed Frame defaults to `map` (switch
back to `odom` in Global Options if you launch without `slam:=true`).

Drive a full loop back near your start point — that's the real test of
whether the map is self-consistent, not just "looks fine while driving."
Watch for walls doubling up after the loop (odometry drift outrunning
`do_loop_closing`; drive slower).

Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/rover_room --ros-args -p use_sim_time:=true
```

## View a saved map

No need to relaunch the sim/robot, SLAM, or Nav2 just to look at a map you
already saved - this brings up only `map_server` (auto-activated) + RViz:
```bash
ros2 launch rover_control view_map.launch.py map:=~/maps/rover_room.yaml
```
`map` defaults to `~/maps/rover_room.yaml`, so if you saved there you can
drop the argument entirely.

## Real hardware

The sim's chassis/lidar description, `slam_params.yaml`, `nav2_params.yaml`,
and `controllers.yaml` are all shared with the physical robot (Raspberry Pi
5, TB6612FNG, JGA25-371 gearmotors with encoders, RPLIDAR C1, ICM-20948 IMU)
via the sibling **`rover_hardware`** package, which provides the actual
`ros2_control` `SystemInterface` plugin. It's kept as a separate package
deliberately - it needs the `lgpio` system library and a real Pi 5 GPIO
chip, neither of which this package (or your dev machine) needs. See
`rover_hardware/README.md` for wiring, safety notes, and calibration.

## Nav2

Bring up Nav2 with the existing config once you either have `slam:=true`
running or have saved a map and are running AMCL against it:
```bash
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=$(ros2 pkg prefix rover_control)/share/rover_control/config/nav2_params.yaml \
  use_sim_time:=true
```
This is what the autonomous FSM below delegates all actual driving to
during EXPLORE/RETURN_HOME.

## Autonomous decision maker

`src/autonomous/autonomous_decision_maker.cpp` is a supervisory FSM with
five states:

- **IDLE** — doing nothing, waiting for a mission to start.
- **EXPLORE** — periodically sends Nav2 a `navigate_to_pose` goal to a
  random point within `explore_radius` of home. This is a placeholder
  wander policy, not frontier exploration — it doesn't look at `/map` for
  unexplored space. Swap in a frontier-based explorer (e.g. a package like
  `m-explore-ros2`) if you want the robot to prioritize unmapped area
  instead of wandering.
- **AVOID_OBSTACLE** — reactive, bypasses Nav2 entirely: cancels the active
  goal and directly publishes `cmd_vel` to back up and turn toward
  whichever side the lidar sees more clearance. Returns to whichever of
  EXPLORE/RETURN_HOME it interrupted once the frontal sector is clear for
  `avoid_min_duration` seconds.
- **RETURN_HOME** — sends one `navigate_to_pose` goal to the pose recorded
  (via TF, `map` -> `body_link`) the first time that transform became
  available after startup. Transitions to IDLE on success.
- **EMERGENCY_STOP** — reachable from any state, publishes zero `cmd_vel`
  every tick, cancels any Nav2 goal, and can only be left via the
  `clear_emergency` service (a topic blip can't self-clear it).

### Interfaces

Topics:
| Topic | Type | Direction |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | in — obstacle distance + which side has more clearance |
| `/autonomy/emergency_stop` | `std_msgs/Bool` | in — `true` latches EMERGENCY_STOP |
| `/autonomy/state` | `std_msgs/String` | out — current state name, published every tick |
| `/cmd_vel` | `geometry_msgs/Twist` | out — **only** during AVOID_OBSTACLE / EMERGENCY_STOP |

Services (all `std_srvs/Trigger`):
`/autonomy/start_explore`, `/autonomy/return_home`, `/autonomy/stop`,
`/autonomy/clear_emergency`.

Action client: `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) — the
node waits (and logs a warning, doesn't crash) if Nav2 isn't up yet.

### Running it

```bash
ros2 launch rover_control rover_sim.launch.py slam:=true autonomy:=true
# separately, once slam_toolbox has published a map -> body_link transform:
ros2 launch nav2_bringup navigation_launch.py params_file:=... use_sim_time:=true

ros2 service call /autonomy/start_explore std_srvs/srv/Trigger
ros2 topic echo /autonomy/state
```

**Don't run raw teleop on `/cmd_vel` at the same time as the FSM** — during
EXPLORE/RETURN_HOME, Nav2 owns `/cmd_vel`; during AVOID_OBSTACLE/
EMERGENCY_STOP, the FSM does. A third publisher fighting for the same topic
just produces jittery, unpredictable motion. Use the services above to
control the mission instead, and `/autonomy/emergency_stop` to override it.

### Tuning parameters

All declared with defaults in the node (override via a params file or
`--ros-args -p name:=value`): `evaluation_rate_hz` (10.0),
`obstacle_distance` (0.6m), `obstacle_frontal_angle_deg` (40°),
`avoid_backup_speed` (-0.15 m/s), `avoid_turn_speed` (0.6 rad/s),
`avoid_min_duration` (2.0s), `explore_radius` (2.5m),
`explore_goal_timeout` (60s).
