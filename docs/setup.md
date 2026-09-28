# Set up and first run

This page continues from [Set up](../README.md#set-up) in the README. Do the sections in order.

## The reference bench

A number in these docs is a measurement on this setup, unless the text says mock hardware, worked
out, model or inferred. Values from the code, the protocol or the example are not measurements.

- Four Waveshare ST3025 servos (model 6410, firmware 3.20): ids 1 and 2 position, 3 and 4 wheels.
- A Waveshare Bus Servo Adapter (A) on USB (vendor id `1a86`, `/dev/ttyACM0`) at 1000000 baud, and
  a 12 V servo supply (`scan` reads 12.3 V).
- Ubuntu 24.04 on x86_64 in the devcontainer image without `--ulimit` (no real-time scheduling).
- A `PREEMPT_DYNAMIC` host kernel, not `PREEMPT_RT`.
- ros2_control 4.48.0 and ros2_controllers 4.42.1, in September 2026.

## Serial port access

- WARNING: Do not make the adapter device world-writable. Every local user can then drive the
  servos, and udev resets the mode when you connect the adapter again.
- WARNING: Do not use `sudo` for the driver, the tools or a serial monitor. Root ignores `TIOCEXCL`,
  and `screen` and `minicom` ignore `flock`. Thus two programs can use the bus at the same time.

The adapter device is in the group `dialout` (mode 0660). After the `usermod` step in
[Set up](../README.md#set-up), make sure that `id -nG` shows `dialout`. Find the adapter with
`ls -l /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/`. For `port`, use the `/dev/serial/by-id/`
name. It stays the same after a reconnection. For one limit, see "Busy-port holder with a by-id
path" in [Known issues](operation.md#known-issues).

## Devcontainer

WARNING: Use the devcontainer only on a computer that you control. It runs `--privileged` with the
host `/dev` and `/etc/udev` mounted read-write, and its user `ubuntu` has `sudo` without a
password. Thus anything in the container can become root on the host.

To start it, open the package directory in VS Code with the "Remote Development" extension pack,
and select "Reopen in container". At each start, `docker/setup.sh` runs `rosdep install` and
`colcon build --symlink-install`. A rebuild deletes `build/`, `install/`, `log/` and the
[EEPROM journal](bench-check.md#eeprom-journal). If `stat -c %g /dev/ttyACM0` on the host does not
show 20 (the container `dialout`), add `"--group-add", "<that gid>"` to `runArgs` (not tested).
Then rebuild the container.

## Try it without hardware

WARNING: Disconnect the adapter or switch off the servo supply for this step. `ros2 launch` does
not check argument names, and the example default is real hardware. Thus a misspelt
`use_mock_hardware` starts the real driver. An empty value (`use_mock_hardware:=`) or a value of
only spaces also starts it, because xacro then uses `false`. The driver can write a mode register
and switch torque on in milliseconds.

Run `ros2 launch waveshare_servos example.launch.py use_mock_hardware:=true gui:=false`. The log
must show `Loaded hardware 'example_ws_ros2_control' from plugin 'mock_components/GenericSystem'`.
If it shows `waveshare_servos/WaveshareServos`, stop the launch. `ros2 control list_controllers`
shows three active controllers and `diff_drive_controller` inactive.

`mock_components/GenericSystem` opens no port and copies each command to its state. It ignores
`id`, `type` and `offset`. `effort`, `temperature`, wheel positions and odometry stay at zero. A
mock run proves nothing about the port, the baud rate or the ids. Do not add `calculate_dynamics`:
it refuses a joint with position and velocity, so `joint_trajectory_position_controller` fails.

## Launch arguments

<!-- reference:launch-arguments:begin -->

| argument | default | effect |
|---|---|---|
| `port` | `/dev/ttyACM0` | serial port of the adapter. Mock hardware ignores it. |
| `baudrate` | `1000000` | bus baud rate. Mock hardware ignores it. |
| `use_mock_hardware` | `false` | `true` swaps in `mock_components/GenericSystem`. Accepts `true`, `false`, `1` or `0` (any case). An empty value, or only spaces, gives `false`. Other values stop the launch. |
| `gui` | `true` | `false` starts no RViz. |

<!-- reference:launch-arguments:end -->

A misspelt name (for example `prot:=`) has no effect. The `bus configuration:` line shows the port
in use ([Hardware parameters](configuration.md#hardware-parameters)).

## Find your servos

Supply power to the servos, stop other programs on the port, and run
`ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0`. It only reads. Put the `id` and
`type` columns in your description ([scan](tools.md#scan)). If no servo answers, examine the
supply, the wiring and the adapter jumper (on B for USB), then try all seven baud rates:

```bash
for b in 1000000 500000 115200 57600 38400 19200 9600; do
  ros2 run waveshare_servos scan --ros-args -p baudrate:=$b; echo "baud $b: exit $?"; done
```

Exit status 3 means no servo at that rate. On the bench, the loop took about 100 s (43 s at 9600
baud).

## Run the example on the bench

- The example is for the reference bench. With `allow_missing_servos` `false`, it needs ids 1-4, or
  `ros2_control_node` exits ([Start-up](operation.md#start-up-and-missing-servos)).
- WARNING: Do not connect a servo that must keep its mode. If its mode does not agree with its joint
  `type`, the driver writes the EEPROM mode register, and the new mode stays (log
  `motor id '<N>' mode changed from <old> to <new>`). To undo it, use a `pos` joint (mode 0).
- WARNING: Keep clear of the joints, and keep the servo supply switch near you. Torque comes on
  before the first goal, so a joint can move toward an old goal (-π/2 after power-on). Only a clean
  Ctrl-C stops the wheels, and torque stays on after shutdown ([Safety](operation.md#safety)).

Then run `ros2 launch waveshare_servos example.launch.py gui:=false`. The controllers have the same
states as on mock hardware.

## Move a joint

These commands move `joint1` to 0.6 rad in 2 s. Send `positions: [0.0]` to move it back.

```bash
ros2 topic pub -t 3 -r 10 /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1], points: [{positions: [0.6], time_from_start: {sec: 2}}]}"
ros2 topic echo --once /joint_states
```

Use `-t 3 -r 10`, not `--once`. A new `ros2 topic pub` process can lose its first message and still
exit 0 (9 of 700 `--once` commands on mock hardware). Each copy starts the 2 s move again. Make
sure that the joint moved ([A joint ignores commands](operation.md#a-joint-ignores-commands)).

## Drive the wheels

WARNING: On a robot, lift the wheels off the ground. The wheels turn until the zero command.

```bash
ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, -0.5]}"
ros2 param get /joint_velocity_controller joints
ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0]}"
```

`data` holds one velocity in rad/s for each joint, in the order that `ros2 param get` shows
(`[joint3, joint4]`). Each servo keeps its last speed, so send zeros at the end and make sure that
the wheels stopped. A command of the wrong length stops the wheels and deactivates the controller.
Activate it again with `ros2 control switch_controllers --activate joint_velocity_controller`.

## Drive a differential base

`diff_drive_controller` is an optional demo that drives `joint3` (left) and `joint4` (right) as a
differential base. Only one of it and `joint_velocity_controller` can own the wheel command
interfaces, so the launch loads it inactive (it claims nothing). Without its package, only its
spawner fails (`Failed loading controller diff_drive_controller`).

WARNING: Lift the wheels or clear the floor. The `ros2 topic pub` line drives the base until Ctrl-C.

```bash
ros2 control switch_controllers --strict \
  --deactivate joint_velocity_controller --activate diff_drive_controller
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}'
ros2 topic echo --once /diff_drive_controller/odom
for s in inactive unconfigured inactive active; do
  ros2 control set_controller_state diff_drive_controller $s; done
ros2 control switch_controllers --strict \
  --deactivate diff_drive_controller --activate joint_velocity_controller
```

- Without `--deactivate`, the activation fails: `is currently claimed by another controller`.
- Run `ros2 topic pub` in its own terminal. The controller accepts only `TwistStamped`, brakes 0.5 s
  after the last message and then logs `Velocity command timed out. Braking.` each second (normal).
- `wheel_radius` 0.05 m and `wheel_separation` 0.20 m are example values. Measure your robot.
- To disable a body limit, remove it. Do not add the deprecated `has_*_limits` keys. The maximum
  `linear.x` and `angular.z` together ask 12.0 rad/s of one wheel, and the driver clamps it.
- At the first activation after a load, the odometry pose jumps by the whole unwrapped wheel
  travel. The `for` loop sets the pose to zero.
- Do not run this launch with another launch that spawns controllers: all spawners race for a lock.

## Real-time scheduling

This section is optional. Real-time scheduling decreases loop jitter, but the driver works without
it. `ros2_control_node` tries `SCHED_FIFO` priority 50 for its control loop (`thread_priority`).
If this fails, it logs `Could not enable FIFO RT scheduling policy ...` and uses normal priority.

### Resource limits

For a user that is not root, `ulimit -r` must be 50 or more. `ulimit -l` must be `unlimited` if the
controller manager locks its memory. It does this by default only on `PREEMPT_RT`. On other kernels,
add `lock_memory: true` under `controller_manager: ros__parameters:` in your controller YAML. If
the kernel refuses the lock, the controller manager logs `Unable to lock the memory`.

### Host setup

WARNING: The change is permanent and applies to all members of the group. Their processes can use
real-time priority 99 and lock all memory, so a runaway process can stop the computer. On a shared
computer, use a dedicated user for the robot. The `tee` line replaces a file with the same name.

```bash
sudo groupadd realtime && sudo usermod -aG realtime "$USER"
printf '@realtime - rtprio 99\n@realtime - memlock unlimited\n' | \
  sudo tee /etc/security/limits.d/99-realtime.conf
```

Log out fully and log in again. Do not add the ros2_control `priority 99` lines: they set nice 19.

### Devcontainer limits

`.devcontainer/devcontainer.json` passes `--ulimit rtprio=99 --ulimit memlock=-1`. Pass the same
flags to your own `docker run`. `--privileged` and `--cap-add=sys_nice` help only root.

### Check real-time scheduling

```bash
# While the stack runs, in the shell that you launch from:
ulimit -r; ulimit -l   # must show 99 and unlimited
pid=$(pgrep -o -f lib/controller_manager/ros2_control_node)
ps -L -o tid,cls,rtprio,comm -p "$pid"   # control-loop thread FF, 50 (do not add -e)
grep VmLck /proc/"$pid"/status           # with lock_memory: true, not 0 kB
# The log must show: Successful set up FIFO RT scheduling policy with priority 50.
```
