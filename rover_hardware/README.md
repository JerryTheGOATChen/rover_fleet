# rover_hardware

`ros2_control` `SystemInterface` for the physical rover: a TB6612FNG driving
two JGA25-371 gearmotor pairs (left side, right side), each wheel with its
own quadrature encoder, accessed via `lgpio` on the Raspberry Pi 5's RP1 GPIO
chip. Reads/writes at whatever rate `controller_manager` calls `read()`/
`write()` (100 Hz per `config/controllers.yaml` in `rover_control`).

Kept as its own package, separate from `rover_control`, on purpose: this is
the only part of the whole project that needs the `lgpio` system library and
an actual Pi 5 GPIO chip to build/run. `rover_control` (sim, FSM, safety
node) stays buildable on your WSL2 dev machine without either.

## Why ros2_control instead of the sim's native DiffDrive plugin

The sim drives wheels straight through Gazebo's `DiffDrive` system plugin -
there's no real hardware underneath for it to abstract. On the real robot,
`ros2_control`'s job is exactly that abstraction: `diff_drive_controller`
and `joint_state_broadcaster` (both already in `rover_control/config/
controllers.yaml`, unused until now) don't know or care that a
`RoverSystemHardware` plugin is what's actually toggling GPIO underneath -
same as they wouldn't care if it were a UR5 arm. That's what lets
`slam_toolbox`/Nav2/the autonomous FSM stay identical between sim and real
robot: they only ever see `/odom`, `/tf`, `/joint_states`, `/scan`.

## Before you build: install lgpio

Not a ROS package - it's a plain system C library:
```bash
git clone https://github.com/joan2937/lg
cd lg && make && sudo make install && sudo ldconfig
```

## Before you run: fill in `urdf/rover_hardware.ros2_control.xacro`

Every `TODO` in that file is a deliberately invalid placeholder - it'll fail
loudly at `on_configure()` rather than silently doing something to the wrong
pin. You need, from your KiCad schematic:

- `standby_pin` — TB6612FNG STBY (shared across both channels)
- `left_pwm_pin` / `left_dir1_pin` / `left_dir2_pin` — the left-side channel
- `right_pwm_pin` / `right_dir1_pin` / `right_dir2_pin` — the right-side channel
- Per wheel joint (`wheel1_joint`..`wheel4_joint`): `encoder_a_pin`,
  `encoder_b_pin`, `encoder_counts_per_rev`

And **run `gpiodetect` first** to confirm `gpio_chip`. On Pi 5 this has
actually changed across kernel/OS image versions - some report the 40-pin
header's chip as `gpiochip4`, current images as `gpiochip0`. Look for the
line labeled `pinctrl-rp1`, that's the one; don't assume `0`.

`encoder_counts_per_rev` is ticks-per-output-shaft-revolution under this
plugin's decoding (X2: both edges of channel A only, not full X4 - simpler
and more reliably correct on a first pass, at half the resolution of a full
quadrature decode). If it's not on your motor's datasheet, get it
empirically: mark the wheel, turn it exactly one full revolution by hand,
and read the tick delta with the hardware interface configured (motors not
yet driving):
```bash
ros2 topic echo /joint_states
```

If a wheel's reported velocity is negative while it's visibly turning
forward, that wheel's `encoder_a_pin`/`encoder_b_pin` are swapped relative
to this plugin's convention - swap them in the xacro rather than touching
code.

## Safety notes - this moves a real robot

- **STBY is the hardware kill switch.** It's only driven high in
  `on_activate()` and immediately driven low in `on_deactivate()` - so if
  `controller_manager` ever deactivates the hardware component (shutdown, a
  lifecycle transition, a crash that takes the process down cleanly), the
  H-bridge is disabled at the hardware level regardless of whatever
  software state anything else is in.
- **Prop the rover up on blocks for first bring-up.** Wheels off the ground,
  always, until you've confirmed direction and rough speed are sane at low
  commands.
- **`velocity_kp`/`velocity_ki` default to `0.0` on purpose** - pure
  open-loop feedforward (commanded rad/s divided by
  `max_wheel_angular_velocity` gives a PWM duty fraction directly) rather
  than a guessed closed-loop tuning. Open-loop is predictable and boring,
  which is what you want for the first power-up of code driving real
  motors. Bring `velocity_kp` up from `0.0` in small steps only after
  you've confirmed direction/rough speed are correct and the robot is still
  propped up - a poorly-tuned `ki` in particular can wind up and produce a
  sudden jump in output.
- **`max_wheel_angular_velocity`** (rad/s at 100% duty) needs calibrating
  for your actual motors/battery voltage/load - it's a placeholder `10.0`.
  Command a small known velocity, compare against the encoder-reported
  velocity in `/joint_states`, and adjust.

## Parameter reference

Hardware-level (in `<hardware><param>`):
`gpio_chip`, `standby_pin`, `pwm_frequency_hz` (default 1000),
`max_wheel_angular_velocity`, `velocity_kp`, `velocity_ki`,
`left_pwm_pin`/`left_dir1_pin`/`left_dir2_pin`,
`right_pwm_pin`/`right_dir1_pin`/`right_dir2_pin`.

Per-joint (in each `<joint><param>`):
`side` (`left`/`right` - must match which physical channel that wheel's
motor is wired to), `encoder_a_pin`, `encoder_b_pin`,
`encoder_counts_per_rev`.

## Run

```bash
cd ~/ros2_ws
colcon build --packages-select rover_hardware
source install/setup.bash
ros2 launch rover_hardware rover_hardware.launch.py
```
(Always build from `~/ros2_ws`, the workspace root - see the note in
`rover_control/README.md`'s Build section if a launch file ever fails to
resolve after moving/renaming a package.)

In another terminal:
```bash
ros2 control list_hardware_interfaces   # confirm all 4 wheels show up
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --rate 10
```

## Not built yet

RPLIDAR C1 (`sllidar_ros2`, confirmed to support the C1 - see its
`sllidar_c1_launch.py`) and the ICM-20948 IMU driver aren't part of this
package. Both are independent of this hardware interface - `/scan` and
`/imu` show up as their own topics same as in sim, nothing here needs to
change when you add them.
