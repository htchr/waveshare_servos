# waveshare_servos

A [ros2_control](https://github.com/ros-controls/ros2_control) `SystemInterface` for Waveshare/Feetech ST and STS serial bus servos (the SMS/STS protocol) on ROS 2 Jazzy.
It is written for the [Waveshare ST3025 servo](https://www.waveshare.com/product/st3025-servo.htm) and the Waveshare [Bus Servo Adapter (A)](https://www.waveshare.com/product/bus-servo-adapter-a.htm), and ships the hardware plugin, four command-line tools, a four-joint example and a test suite.
The Humble version is 0.1.0, on the [`humble` branch](https://github.com/htchr/waveshare_servos/tree/humble).

**Contents**

- [Features](#features)
- [Requirements and tested hardware](#requirements-and-tested-hardware)
- [Install](#install)
- [Quick start](#quick-start)
- [Using the driver in your robot](#using-the-driver-in-your-robot)
- [Running it](#running-it)
- [Sizing the bus](#sizing-the-bus)
- [Command-line tools](#command-line-tools)
- [Troubleshooting](#troubleshooting)
- [Testing](#testing)
- [Known issues and limitations](#known-issues-and-limitations)
- [Versioning and changes](#versioning-and-changes)
- [License and third-party code](#license-and-third-party-code)
- [Contributing](#contributing)
- [Appendix: engineering notes](#appendix-engineering-notes)


## Features

- Up to 253 servos on one bus (ids 1..253, unique within a `<ros2_control>` block). The practical limit is the timing budget: at 100 Hz the bus model allows about twelve ([Sizing the bus](#sizing-the-bus); nothing above four servos was measured). Several buses as several `<ros2_control>` blocks, each with its own `port`.
- Per joint, position control (servo mode 0) or velocity control (wheel mode 1), inferred from the joint's command interfaces.
- Wheel positions are unwrapped into a multi-turn count, so `diff_drive_controller` odometry does not jump once per revolution.
- Nine state interfaces (position, velocity, effort, current, voltage, temperature, load, status and the deprecated torque), declared per joint in any subset and any order.
- An exclusive port lock (`TIOCEXCL` and `flock`): while the driver holds the port, another program that opens it is refused (unless it runs as root), and so is one that asks for its lock; a program that already had the port open is not stopped, but the driver names it in a warning.
- One sync read per control cycle and broadcast sync writes: for four servos on the reference bench, `read()` plus `write()` take 1.66 ms of each 10 ms cycle.
- Four command-line tools: `scan`, `set_id`, `calibrate_midpoint` and `factory_reset`.
- A test suite that needs no motors, and a bench test for the reference bench.


## Requirements and tested hardware

- **Software:** Ubuntu 24.04 and ROS 2 Jazzy. Built and tested against ros2_control 4.48.0 and ros2_controllers 4.42.1; the oldest versions that work were not determined.
- **The reference bench.** The measurements in this README were made on this setup, some of them (such as `factory_reset`'s) with only part of it on the bus, and the rest of the README calls it "the reference bench". A measurement made on mock hardware instead, such as the command count in Quick start [step 4](#4-move-a-joint-and-the-wheels), says so. Values that come from the code, the protocol or the example's configuration, and numbers marked worked out, a model or inferred, are not measurements:
  - four Waveshare ST3025 servos (model number 6410, firmware 3.20) at ids 1-4; ids 1 and 2 run as position servos, ids 3 and 4 as wheels;
  - a Waveshare Bus Servo Adapter (A) on USB (vendor id `1a86`, "USB Single Serial", seen as `/dev/ttyACM0`), at 1 000 000 baud;
  - a 12 V servo supply (`scan` reads 12.3 V);
  - an x86_64 machine running Ubuntu 24.04 in a container of the project's devcontainer image, started without the `--ulimit` flags the devcontainer passes (RLIMIT_RTPRIO 0, so without real-time scheduling), on host kernel 7.0.0-30-generic (`PREEMPT_DYNAMIC`, not `PREEMPT_RT`);
  - ros2_control 4.48.0 and ros2_controllers 4.42.1, in September 2026.
- **Expected to work, untested:** other Waveshare and Feetech servos that speak the SMS/STS protocol.
- **Not supported:** the SCS/SCSCL series. `protocol` `scscl` is refused when the description loads.
- **Hardware notes:**
  - The servos need their own supply; USB does not power them.
  - For USB control, the jumper cap on the Bus Servo Adapter (A) must be on **B**. Waveshare's [wiki page for the adapter](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) says "the jumper cap of the serial bus driver board should be at the B position".
  - The driver and the tools support seven baud rates: 9600, 19200, 38400, 57600, 115200, 500000 and 1000000. Every timing default is sized for 1 000 000 (see [Hardware parameters](#hardware-parameters)).
- 0.1.0, the Humble version, was also used on a Jetson Orin Nano (ROS 2 Humble and Isaac ROS in Docker).


## Install

This package is source-only: there is no binary release, so there is no `ros-jazzy-waveshare-servos` apt package.
Build it from source in a colcon workspace.

### From source

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b jazzy https://github.com/htchr/waveshare_servos.git
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo rosdep init        # once per machine; skip it if it says the sources list already exists
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --packages-select waveshare_servos
source install/setup.bash
```

- `-b jazzy` selects the Jazzy code. The repository's default branch holds 0.1.0, the Humble version.
- The `rosdep install` line passes `--rosdistro jazzy`, so it resolves every dependency even in a shell where ROS is not sourced; without that option, such a shell stops with `ROS distro is not set`.
- It deliberately leaves out `-r`: that option turns a dependency rosdep cannot resolve into success (exit status 0), which hides the one failure this step exists to show.
- Plain `colcon build` installs copies of the package's files. `--symlink-install` is only for developing the package itself ([Developing and releasing](#developing-and-releasing)).
- Source `install/setup.bash` in every new terminal before using the package.

### Serial port access

The adapter's device belongs to group `dialout` with mode 0660, so your user needs to be in `dialout`:

```bash
sudo usermod -aG dialout "$USER"      # then log out and back in
id -nG                                 # must list dialout
ls -l /dev/ttyACM0 /dev/serial/by-id/
```

- Group membership takes effect in a new login session: log out completely and back in.
- Do not make the device world-writable instead. That lets every local user drive the servos, and udev resets the mode whenever the adapter is plugged in again.
- Do not use `sudo` instead either, for the driver, the tools, or a serial monitor. The kernel lets a process running as root open a port that another program has marked exclusive (`TIOCEXCL`). Only the advisory lock (`flock`) then keeps it off the bus, and only programs that take that lock respect it: this package's driver and tools do; `screen` and `minicom` do not. The driver and the tools print a `running as root` warning when started as root ([Messages quoted in this README](#messages-quoted-in-this-readme)).
- To find the adapter, list the candidates with `ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/`. On the reference bench it is `/dev/ttyACM0`, which is also reachable as `/dev/serial/by-id/usb-1a86_USB_Single_Serial_<serial>-if00`, where `<serial>` is the adapter's own serial number.
- The `/dev/serial/by-id/` name stays the same when the adapter is re-plugged or other USB serial devices come and go, while the `ttyACM` number can change, so it is the better value for `port:=` and `<param name="port">`. One caveat: with a `/dev/serial/by-id/` path, the driver's "port busy" error cannot name the process that holds the port (the tools can); see [Known issues and limitations](#known-issues-and-limitations), item 4.

### Devcontainer

The repository carries a VS Code devcontainer for Jazzy (`.devcontainer/devcontainer.json`, `docker/Dockerfile`, `docker/setup.sh`). The container runs `--privileged` with the host's `/dev` and `/etc/udev` mounted read-write, and its user (`ubuntu`) has passwordless sudo, so anything run in it can become root on the host without a password: use it only on a machine you control. To use it:

1. Install the "Remote Development" extension pack for VS Code.
2. Open the directory in VS Code.
3. Select "Reopen in container".

What it does:

- It runs the container `--privileged`, on the host network, with the host's `/dev` and `/etc/udev` bind-mounted, which among other things makes the adapter visible inside.
- It forwards X11 through `DISPLAY` for RViz; the host must allow local X clients.
- It passes `--ulimit rtprio=99 --ulimit memlock=-1`. These flags set RLIMIT_RTPRIO to 99 and RLIMIT_MEMLOCK to unlimited, the limits that real-time scheduling and memory locking need ([Real-time scheduling](#real-time-scheduling)); no container was built to confirm them. It also passes `--cap-add=sys_nice`, which does nothing for the container user (same section).
- The Dockerfile puts the container user (`ubuntu`) in `dialout`, and the image build fails if that did not work.
- It installs `bam`, Rhoban's BAM ("Better Actuator Models"), a friction-model identification pipeline for servos. It is there for the user's own experiments; this package does not use it.
- **Of the workspace and the home directory, only the package directory is bind-mounted** (`workspaceMount`; the other bind mounts are the host's `/dev`, `/etc/udev`, `/tmp/.X11-unix`, `/etc/timezone` and `/etc/localtime`). The workspace's `build/`, `install/` and `log/`, and the bench test's EEPROM journal under `~/.local/state/waveshare_servos/`, live inside the container and are lost when it is rebuilt.
- `docker/setup.sh` is the `postStartCommand`, so **every** start of the container runs `rosdep install` and a `colcon build --symlink-install` of the workspace; the first start takes minutes.
- `setup.sh` runs under `set -eo pipefail`, and its `rosdep install` does not pass `-r`, so a dependency that cannot be resolved (or downloaded, while offline) stops the start-up before the build, with rosdep's error.
- At every start it prints the container's real-time limits and warns when the user is not in `dialout`.
- Rebuild the container after changing `runArgs`.
- If the host's `/dev/ttyACM0` does not belong to group id 20 (the container's `dialout`), membership of the container's `dialout` grants nothing. `stat -c %g /dev/ttyACM0` prints the group id; add `"--group-add", "<that gid>"` to `runArgs`. This is from Docker's documentation and was not tested with this package.


## Quick start

### 1. Try it without hardware

Launch the example on mock hardware with `ros2 launch waveshare_servos example.launch.py use_mock_hardware:=true gui:=false` and, in a second terminal, `ros2 control list_controllers`.

`ros2 launch` does not check argument names. It ignores a name it does not know, without a warning, and the example's default is real hardware. A misspelt `use_mock_hardware` therefore runs the real driver on `/dev/ttyACM0`, which can rewrite a servo's mode register and switch its torque on within milliseconds of starting. For this step, leave the adapter unplugged or the servo supply off, so that a mistake fails at configure instead of moving anything. To confirm that mock hardware is in use, look for `Loaded hardware 'example_ws_ros2_control' from plugin 'mock_components/GenericSystem'` in the log. If the log shows `from plugin 'waveshare_servos/WaveshareServos'` or a `bus configuration:` line instead, the real driver is loaded and may already have configured and activated the servos: stop the launch, and remember that torque stays on after shutdown ([Shutdown](#shutdown)).

- `use_mock_hardware:=true` swaps the driver for `mock_components/GenericSystem`, which opens no serial port and mirrors every command straight back into its state, so this is how to try the example with no hardware at all.
- `list_controllers` shows `joint_state_broadcaster`, `joint_trajectory_position_controller` and `joint_velocity_controller` active, and `diff_drive_controller` inactive ([step 5](#5-drive-the-wheels-as-a-differential-base) says why).
- One mock-only quirk is worth knowing before reporting it as a bug: a velocity-commanded joint's `position` never advances, so the wheels stand still in RViz and the odometry of step 5 stays at zero. That is `mock_components/GenericSystem`, not the driver.
- `effort` and `temperature` likewise read a constant `0.0`.

### 2. Find your servos

With the servos powered and nothing else using the port, run `ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0`.

`scan` pings every id from 0 to 253 at one baud rate (1 000 000 unless you pass `baudrate`) and prints one row per servo that answers. It only reads.
Its `id` and `type` columns are what your description needs.
If it finds nothing, check the servo supply, the wiring and the adapter's jumper (B for USB). If the servos may be set to another rate, try every rate:

```bash
for b in 1000000 500000 115200 57600 38400 19200 9600; do
  ros2 run waveshare_servos scan --ros-args -p baudrate:=$b; echo "baud $b: exit $?"
done
```

A rate at which no servo answers exits with status 3. On the reference bench the whole loop took about 100 s: `scan` took about 4 s at 1 000 000 and 500 000 baud, 5.5 s at 115 200 and 43 s at 9600. [Command-line tools](#command-line-tools) describes `scan` and the other tools.

### 3. Run the example on the reference bench

> **Before you launch the hardware example.** It is written for the reference bench.
>
> - It declares servos at ids 1-4 and leaves `allow_missing_servos` at `false`, so with fewer servos the controller manager exits at start-up ([Start-up](#start-up-and-servos-that-do-not-answer)).
> - It drives ids 1 and 2 as position servos within ±π/2 of the example's joint zero, servo tick 1024 (`offset` 1.570796), and ids 3 and 4 as wheels.
> - **It can write EEPROM.** A servo whose mode register differs from its joint's `type` gets that register rewritten, at configure and at an activation that adds back a servo that was missing or dropped. The driver then logs `motor id '<N>' mode changed from <old> to <new>`. On other hardware this turns a position servo at id 3 or 4 into a wheel. The new mode stays after the example exits and is meant to survive a power cycle. To get such a servo back, configure it under your own description with a `pos` joint at its id: the driver then writes mode 0 back ([Joint parameters](#joint-parameters)). `factory_reset` also sets mode 0, but it resets the offset, limits and gains as well.
> - Torque comes on at activation, before the first goal is written ([Known issues and limitations](#known-issues-and-limitations), item 1), and **stays on after shutdown** ([Shutdown](#shutdown)). So a position joint can start toward a stale goal for about one control period, for example after `calibrate_midpoint`, or right after the servos are switched on, when that goal is 0 (servo tick 0: -π/2 rad for `joint1` and `joint2`, the lower end of their range). Keep clear of the joints when you launch.
> - Only a clean shutdown (Ctrl-C) stops the wheels; after a kill, a crash or a lost adapter they keep turning, so keep the servo supply switch within reach ([Shutdown](#shutdown)).
> - `gui` defaults to `true`; without a display, pass `gui:=false`.

Launch it with `ros2 launch waveshare_servos example.launch.py gui:=false`, and check the controllers with `ros2 control list_controllers`: the same four as in step 1, in the same states.

The launch file takes four arguments, as `name:=value`:

<!-- reference:launch-arguments:begin -->

| argument | default | effect |
|---|---|---|
| `port` | `/dev/ttyACM0` | serial port of the bus servo adapter; ignored under mock hardware |
| `baudrate` | `1000000` | bus baud rate; ignored under mock hardware |
| `use_mock_hardware` | `false` | `true` swaps in `mock_components/GenericSystem`, which opens no serial port. Accepts `true`, `false`, `1` or `0` in any case; any other value aborts the render on purpose |
| `gui` | `true` | `false` starts no RViz |

<!-- reference:launch-arguments:end -->

Argument names are not checked: a misspelt name (for example `prot:=`) is ignored and the argument keeps its default, so `port` stays `/dev/ttyACM0`. The `bus configuration:` line shows the port the driver uses.

To run the driver on other hardware, copy `description/`, `bringup/config/example_controllers.yaml` and `bringup/launch/example.launch.py` into your own robot package and edit the copies, not the installed example.
Point the copied launch file at your package, and in the copied `description/urdf/example.urdf.xacro` change `$(find waveshare_servos)` in the `xacro:include` line to your package. Otherwise the copy keeps including the installed example's `ros2_control` block (ids 1-4, with 3 and 4 as wheels), and your edits to the copied one are ignored without any warning.
Set the ids, types and offsets of your servos ([Using the driver in your robot](#using-the-driver-in-your-robot)), and when you remove a joint, remove it from the controllers YAML too: the controllers name their joints, and a spawner fails on a joint the hardware does not have.

### 4. Move a joint and the wheels

The trajectory controller `joint_trajectory_position_controller` drives `joint1` and `joint2`:

```bash
ros2 topic pub -t 3 -r 10 /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1], points: [{positions: [0.6], time_from_start: {sec: 2}}]}"
ros2 topic echo --once /joint_states
```

This moves `joint1` to 0.6 rad and prints one `/joint_states` message. Send the same command with `positions: [0.0]` to go back.

**Why `-t 3 -r 10` and not `--once`.** `-t 3 -r 10` publishes the same message three times, 0.1 s apart, and exits. The first message that a new `ros2 topic pub` process sends can be dropped with nothing logged: the process still prints `publishing #1` and exits with status 0, the controller never gets that message (the trajectory controller's `reference` on `/joint_trajectory_position_controller/controller_state` does not move), and the joint or the wheels keep doing what they did. Both controllers subscribe with the system-default QoS, which with Jazzy's default middleware (Fast DDS) is best-effort, so nothing sends a dropped message again. With `--once` this happened on the reference bench and on mock hardware; why the message is dropped was not found below the ROS layer. The copies do no harm: each copy of a trajectory restarts the 2 s move from where `joint1` is, so it reaches 0.6 rad about 2.2 s after the command starts publishing, not 2.0 s, and each copy of a wheel command sets the same speeds. Measured on mock hardware on 2026-09-24, each command sent from a new process in the form printed here: with `--once`, 9 of 700 commands to the two controllers had no effect; with `-t 3 -r 10`, all 700 took effect, 7 of them only through the second copy. On the reference bench on 2026-09-24, in one run, each of 111 commands in this form whose first send was checked took effect without a resend (59 trajectory commands and 52 wheel commands); one trajectory command took effect only through its second copy. Neither result proves that a command is never lost, so once the move is over, run the `ros2 topic echo` line again and check that `joint1` moved, and below check that the wheels turn and stop; if a command had no effect, send it again, and if it still has none, see [Troubleshooting](#troubleshooting).

`joint_velocity_controller` (a `velocity_controllers/JointGroupVelocityController`) drives the wheels, `joint3` and `joint4`. The wheels start turning at the first line of the block below and keep turning until its last line sends zeros; on a robot, lift the wheels off the ground first:

```bash
ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, -0.5]}"
ros2 param get /joint_velocity_controller joints
ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0]}"
```

- The message carries no joint names. `data` holds one velocity (rad/s) per joint, in the order of the `joints` list of `joint_velocity_controller` in `example_controllers.yaml`, not in servo id order or URDF order.
- The shipped list is `[joint3, joint4]`, so `{data: [1.0, -0.5]}` commands `joint3` to 1.0 rad/s and `joint4` to -0.5 rad/s. The `ros2 param get` line prints the order on a running system.
- On the reference bench the two wheels then turned at about 0.99 and -0.46 rad/s, not exactly 1.0 and -0.5: an ST3025 wheel appears to run only in steps of about 0.077 rad/s (inferred; [Command interfaces and limits](#command-interfaces-and-limits)).
- For example, if your copy of the YAML listed the joints the other way round (a hypothetical, not what ships):

  ```yaml
  joint_velocity_controller:
    ros__parameters:
      joints:
        - joint4
        - joint3
  ```

  then the same `{data: [1.0, -0.5]}` would command `joint4` to 1.0 rad/s and `joint3` to -0.5 rad/s.
- Each servo keeps turning at its commanded velocity until a new command arrives, or until the servo supply is switched off, which is why the block ends by sending a zero for every joint; a killed stack sends no zero ([Shutdown](#shutdown)).
- After the last line, check that the wheels stopped. A stop that was lost (above) leaves them turning at their previous speed; send the zeros again.
- A command whose length does not match the `joints` list stops all the velocity-controlled servos and deactivates the controller. Reactivate it with `ros2 control switch_controllers --activate joint_velocity_controller`.

### 5. Drive the wheels as a differential base

`example_controllers.yaml` also configures a `diff_drive_controller` over the two wheels.
It ships in its own apt package, `ros-jazzy-diff-drive-controller`, which the package declares as a dependency, so `rosdep install` installs it.
Without it the launch still brings up the other three controllers, but the `diff_drive_controller` spawner exits non-zero with `Failed loading controller diff_drive_controller`. The controller manager logs the reason: `Loader for controller 'diff_drive_controller' (type 'diff_drive_controller/DiffDriveController') not found.`, followed by the list of controller types it can load, and that list has no `diff_drive_controller/DiffDriveController`.

The launch file loads it but leaves it **inactive**, because it commands the same `joint3/velocity` and `joint4/velocity` interfaces as `joint_velocity_controller`, and ros2_control hands each command interface to one controller only.
Swap the two atomically; activating `diff_drive_controller` first, without the deactivate, is refused, and the controller manager logs that the interface is currently claimed by another controller.
The `ros2 topic pub` line drives the base until you press Ctrl-C; the controller brakes 0.5 s after the last message. Run it in its own terminal. On a robot, lift the wheels off the ground or give it clear floor. With the example's `wheel_radius` (0.05 m), the base moves at 0.1 m/s only if its wheels really have that radius.

```bash
ros2 control switch_controllers --strict \
  --deactivate joint_velocity_controller --activate diff_drive_controller
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}'
ros2 topic echo --once /diff_drive_controller/odom
for s in inactive unconfigured inactive active; do
  ros2 control set_controller_state diff_drive_controller $s
done
ros2 control switch_controllers --strict \
  --deactivate diff_drive_controller --activate joint_velocity_controller
```

- The first command swaps the controllers. The last one swaps them back.
- The controller subscribes to `geometry_msgs/msg/TwistStamped` (4.42.1 has no unstamped option) and brakes 0.5 s after the last message, so publish continuously (`-r 20`) rather than with `--once`, and read the odometry from a second terminal while it runs.
- `wheel_radius` (0.05 m) and `wheel_separation` (0.20 m) are **example values for a bench with no chassis**; measure them on a real robot. They are picked so the arithmetic checks by eye: `linear.x: 0.1` commands 2.0 rad/s on both wheels, and `angular.z: 1.0` commands -2.0 rad/s on the left wheel (`joint3`, `left_wheel_names`) and +2.0 rad/s on the right (`joint4`); on the reference bench each wheel then turned at about 1.99 rad/s in magnitude ([step 4](#4-move-a-joint-and-the-wheels)). Same sign on both wheels for `linear.x`, opposite signs for `angular.z`; a positive `angular.z` drives the right wheel forward.
- `/diff_drive_controller/odom` integrates the wheels' **unwrapped** multi-turn `position` state (`position_feedback: true`), so it does not jump once per revolution.
- The pose takes one step the first time the controller activates after it is loaded. The controller starts its previous-wheel-position memory at zero, so its first update adds the wheels' whole unwrapped travel (times `wheel_radius`). A later activation steps only by however far the wheels turned since the controller last ran, for example under `joint_velocity_controller`. A cleanup or configure zeroes the pose but not that memory; a plain deactivate and activate keeps the pose. 4.42.1 has no reset service. (From the ros2_controllers 4.42.1 source; of this bullet, only the zeroing by the loop below was measured.)
- The `for` loop zeroes the pose. It cycles the controller through `inactive`, `unconfigured`, `inactive` and `active`; the CLI does not go from `active` to `unconfigured` in one step. From the reactivation on, the pose again counts every change of the wheel positions since the controller last ran before the loop (the step of the previous bullet, then ordinary odometry), so it reads exactly zero only if both wheels still read the positions they had then. A wheel standing still can flicker between two adjacent encoder ticks (2π/4096 rad). One tick of one wheel works out to 0.000038 m of x and 0.00038 rad of yaw on the example's geometry (`wheel_radius` × tick / 2 and `wheel_radius` × tick / `wheel_separation`). Measured on the reference bench on 2026-09-24, with the wheels commanded to zero, both outcomes occurred. In one run the pose read x -0.098 m, y -0.595 m and yaw 2.549 rad before the loop and exactly zero (x, y and yaw 0.0) after it. In another it read x -0.000038 m, y 7e-9 m and yaw 0.00038 rad after the loop, one tick of `joint3`: that wheel read the same while the controller was inactive, and flickered between two adjacent ticks from 0.09 s after the reactivation. Both read the same pose again 2 s later.
- While nothing publishes `cmd_vel`, the controller logs `Velocity command timed out. Braking.` about once a second. That is expected.
- Under `use_mock_hardware:=true` the controller activates and commands the wheels, but the odometry stays at zero (step 1).


## Using the driver in your robot

### A minimal description

One position servo and one wheel on one bus:

```xml
<ros2_control name="my_bus" type="system">
  <hardware>
    <plugin>waveshare_servos/WaveshareServos</plugin>
    <param name="port">/dev/ttyACM0</param>
    <param name="baudrate">1000000</param>
  </hardware>
  <joint name="pan">
    <param name="id">1</param>
    <param name="offset">3.141593</param>
    <command_interface name="position">
      <param name="min">-1.5708</param>
      <param name="max">1.5708</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="wheel">
    <param name="id">3</param>
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

- The plugin is `waveshare_servos/WaveshareServos`. `port` and `baudrate` are shown at their default values; every hardware parameter is optional ([Hardware parameters](#hardware-parameters)).
- Neither joint sets `type`. `pan` has a position command interface, so it is a position joint (`pos`, servo mode 0); `wheel` has only a velocity command interface, so it is a wheel (`vel`, mode 1).
- Loading it can write EEPROM. At configure, and at an activation that adds back a servo that was missing or dropped, the driver rewrites the mode register of any servo that differs from its joint's type: servo 1 to position mode (0), servo 3 to wheel mode (1). A position servo at id 3 becomes a wheel. `scan`'s `type` column shows each servo's current mode. Change the ids to match your bus first ([Joint parameters](#joint-parameters), "`type` can write EEPROM").
- `offset` 3.141593 makes servo tick 2048, the middle of the servo's turn, joint zero. A servo calibrated with `calibrate_midpoint` reads 0 where it was calibrated, and the limits ±1.5708 rad map to ticks 1024 and 3072: `lround((±1.5708 + 3.141593) × 4096 / 2π)`.
- With `offset` 0 the same block is refused when it loads, because -1.5708 rad maps to tick -1024, outside the servo's single turn. The driver logs `joint 'pan': position limits [-1.5708, 1.5708] rad with offset 0.0000 rad and inverted=false map to servo ticks [-1024, 1024], outside the servo's single-turn range [0, 4095]; change the offset, the limits, or 'inverted'`.
- The URDF around the block needs `<joint>` elements with the same names (`pan` a revolute joint with `<limit>` values to match, `wheel` a continuous one).

### Hardware parameters

All eleven are optional. Each goes in the `<hardware>` block as `<param name="...">value</param>`.

<!-- reference:hardware-parameters:begin -->

| parameter | type | default | legal values | what it does |
|---|---|---|---|---|
| `port` | string | `/dev/ttyACM0` | any non-empty path | Serial device of the bus adapter. Opened (exclusively: `TIOCEXCL` + `flock`) in `on_configure`, never in `on_init`, so a path that does not exist is reported at configure time. Several `<ros2_control>` blocks may use different ports. |
| `baudrate` | integer | `1000000` | `9600`, `19200`, `38400`, `57600`, `115200`, `500000`, `1000000` | Line rate. Must equal the rate stored in the servos (scan's `baud` column). The timing defaults below are sized for 1 Mbaud; see the note under this table. |
| `io_timeout_ms` | integer (ms) | `5` | `2`..`1000` | Budget for one bus transaction. A servo that goes silent mid-run makes every read take about `io_timeout_ms` + 2 ms on the sync-read path (the default: the full timeout, then a 2 ms drain; worked out from the code, not measured), or about `io_timeout_ms` on the per-servo path, until `max_read_fails` drops it. Interacts with `feedback_mode` and the joint count ([`io_timeout_ms` and `feedback_mode`](#io_timeout_ms-and-feedback_mode)). |
| `ping_attempts` | integer | `3` | `1`..`10` | Pings per servo in `on_configure`, and per absent servo in `on_activate`. An absent servo costs up to `ping_attempts x io_timeout_ms` at each of those. |
| `max_read_fails` | integer | `50` | `1`..`1000000` | How many **consecutive** failed reads drop a servo from the read cycle (one ERROR). Counted in read cycles, so the wall-clock time stretches with a lower `rw_rate`: 0.5 s at 100 Hz, 1 s at 50 Hz. Deactivate and activate the hardware to look for the servo again ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost): keep clear of the joint first). |
| `allow_missing_servos` | bool | `false` | `true` / `false` (any case) | What `on_configure` does when a servo does not answer its ping: `false` fails the configure, `true` continues without it. Only applies at configure time. It has no effect on a servo dropped later. |
| `protocol` | string | `sms_sts` | `sms_sts` (any case) | Servo protocol family. `scscl` is recognised but refused, because it is not implemented. |
| `feedback_mode` | string | `auto` | `auto`, `sync_read`, `per_servo` (any case) | How `read()` fetches feedback. `auto` uses one `INST_SYNC_READ` per cycle and falls back to one read per servo if the activation probe goes unanswered. `sync_read` never falls back: activation fails instead. `per_servo` never sends a sync read. |
| `encoder_steps` | integer | `4096` | even, `2`..`32768` | Steps per revolution. It enters every angle and rate conversion. The **register** defaults of `max_speed` and `max_accel` do not scale with it ([Joint parameters](#joint-parameters)). |
| `current_per_count_a` | double (A/count) | `0.006` | `> 0` and `<= 1` | Scale of the current register (69-70). Not verified with an ammeter. |
| `torque_constant_nm_per_a` | double (N m/A) | `0.8825985` | `> 0` and `<= 100` | `effort = current x torque_constant_nm_per_a`. The default is 9.0 kgf cm/A. |

<!-- reference:hardware-parameters:end -->

**How values are read** (this applies to every hardware and joint parameter):

- Leading and trailing whitespace is stripped.
- An **absent** parameter keeps its default. An **empty** one (`<param name="x"></param>`) is an error; it never falls back to the default.
- Numbers go through `hardware_interface::stod` / `stoi_generic`, which are locale-independent and reject any trailing characters: `20ms`, `1.0f`, `1,5`, `1e6` for an integer, `1000000.0` for an integer. `nan` and `inf` are also refused.
- Integers accept a leading `+` and leading zeros, so `+001` is `1`. Hex is refused.
- Booleans are exactly `true` / `false`, in any case. `1`, `0`, `yes`, `no`, `on` and `off` are refused.
- An **unknown** hardware parameter name logs `hardware parameter '<name>' is not used by this driver; ignoring it` (one WARN per name, sorted) and loading continues. A misspelt name therefore leaves its parameter at the default. The configuration line (below) is how you check what was applied.
- A `<param>` repeated in the same block keeps the **last** value without any diagnostic; that is the URDF parser's doing.
- Parameters are checked in this order: port, baudrate, protocol, feedback_mode, io_timeout_ms, ping_attempts, max_read_fails, allow_missing_servos, encoder_steps, current_per_count_a, torque_constant_nm_per_a, then the timeout floor. All hardware parameters are checked before any joint.
- The **first** bad value logs one FATAL, and `on_init` returns `ERROR`. Nothing after it is parsed. The port is never opened.

**What an `on_init` failure looks like under `ros2_control_node`** (measured on the reference bench): the driver's FATAL, then `Failed to initialize hardware '<name>'`, then `Could not load and initialize hardware. ... try to publish robot description again.`
The node keeps running with **no hardware** and logs `Waiting for data on 'robot_description' topic to finish initialization` once a second.
The controller manager creates its services only after the hardware initialises, so, from the controller_manager 4.48.0 code (not measured), `/controller_manager/list_controllers` never appears. One of the example's two spawners, which have no `--controller-manager-timeout`, keeps waiting for it; the other gives up after about two minutes. The launch does not end by itself ([Troubleshooting](#troubleshooting)).
Contrast this with a **configure** failure (a missing servo, a busy port), where `ros2_control_node` exits at start-up ([Start-up](#start-up-and-servos-that-do-not-answer)).

**Rejection messages.** A bad hardware parameter produces one of three messages, where `<E>` is the parameter's own expectation from the table below:

- `hardware parameter '<name>' is empty; expected <E>`
- `hardware parameter '<name>' is '<value>', which is not <E>` (malformed)
- `hardware parameter '<name>' is '<value>', which is out of range; expected <E>`

| parameter | `<E>` | special messages |
|---|---|---|
| `port` | `a device path such as '/dev/ttyACM0'` | |
| `baudrate` | `an integer` | any well-formed but unmapped value (including 0 and negatives): `hardware parameter 'baudrate' is '<value>'; the servo library maps only 9600, 19200, 38400, 57600, 115200, 500000 and 1000000, and silently falls back to 115200 for anything else` |
| `protocol` | `'sms_sts'` | `scscl`: `hardware parameter 'protocol' is 'scscl'; only 'sms_sts' is implemented, the SCS/SCSCL series is not supported yet`. Any other value: `... which is not a known protocol; expected 'sms_sts'` |
| `feedback_mode` | `'auto', 'sync_read' or 'per_servo'` | any other value: `... which is not a known feedback mode; expected 'auto', 'sync_read' or 'per_servo'` |
| `io_timeout_ms` | `an integer between 2 and 1000 (milliseconds); 1 ms is not enough for a batched feedback read, which needs about 0.48 ms plus 0.29 ms per servo` | the `sync_read` floor refusal in the next section |
| `ping_attempts` | `an integer between 1 and 10` | |
| `max_read_fails` | `an integer between 1 and 1000000` | |
| `allow_missing_servos` | `'true' or 'false'` | |
| `encoder_steps` | `an even integer between 2 and 32768 (encoder steps per revolution)` | an odd value uses the out-of-range template |
| `current_per_count_a` | `a number greater than 0 and at most 1 (amperes per current count)` | |
| `torque_constant_nm_per_a` | `a number greater than 0 and at most 100 (newton metres per ampere)` | |

**The effective configuration.** One INFO per load shows the values the driver will actually use, after the timeout floor (next section) has raised or demoted anything. Read it after every change to the description. The stock example, as measured on the reference bench:

```text
bus configuration: port '/dev/ttyACM0', 1000000 baud, protocol 'sms_sts', io timeout 5 ms, 3 ping attempt(s), drop a servo after 50 consecutive read failures, allow_missing_servos false, feedback_mode 'auto', 4096 encoder steps per revolution, 0.006 A per current count, 0.8825985 N m/A
```

**Baud rate and the timeout** (worked out from wire time, not measured).
The 5 ms default, the floor in the next section and every number in [Sizing the bus](#sizing-the-bus) were measured at 1 Mbaud.
The driver does **not** scale `io_timeout_ms` with `baudrate`; the tools do, with `max(5, ceil(510000 / baud) + 2)` ms.
Wire time alone, at 10 bits per byte: a ping is 6 + 6 bytes, one feedback read 8 + 21 bytes, and a sync read of 4 servos about 12 + 84 bytes. So at the 5 ms default:

- **115200:** a 4-servo sync read (about 8.3 ms) cannot complete. `auto` falls back to per-servo reads at activation, with a WARN. `sync_read` fails activation.
- **57600 and below:** a single feedback read does not fit (about 5.04 ms at 57600, already just over the budget before any USB or servo turnaround time; 7.6 ms at 38400).
- **19200 and below:** not even a ping (6.25 ms at 19200, 12.5 ms at 9600) can complete, so configure refuses with "did not answer".

Below 1 Mbaud, set `io_timeout_ms` to at least the tools' value for that rate: 7 at 115200, 11 at 57600, 16 at 38400, 29 at 19200, 56 at 9600. Add the sync-read burst on top, or use `feedback_mode` `per_servo`.

### `io_timeout_ms` and `feedback_mode`

`io_timeout_ms` is the per-transaction budget the driver hands to the vendored serial layer (`SCSerial::IOTimeOut`). Its range is 2..1000 and its default 5.
1 is refused because at 1 ms a sync read of four servos sometimes returns the previous cycle's frames, with valid headers, ids, lengths and checksums: 8 of 3000 reads did exactly that on the reference bench. No flush can prevent it, so the value is refused instead.
If you used a pre-release `jazzy` snapshot, see [CHANGELOG.rst](CHANGELOG.rst): the range and the default were different there.

A sync read carries a whole burst of replies, so the timeout it needs grows with the number of servos.
Let `K = min(number of <joint> elements in the <ros2_control> block, 30)`: the joint count **declared**, not the number of servos that answer (absent servos count too).
The driver's floor is `ServoBus::min_io_timeout_ms(K)`, roughly `0.48 + 0.29 x K` ms scaled by 1.19 for the tail, plus a millisecond:

| K (joints declared) | 1 | 2-4 | 5-7 | 8-9 | 10-12 | 13-15 | 16-18 | 19-21 | 22-24 | 25-27 | 28-30 (and above) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| floor (ms) | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |

| `feedback_mode` | `io_timeout_ms` | below the floor | above `max(8, floor)` |
|---|---|---|---|
| `per_servo` | any | nothing | nothing |
| `auto` or `sync_read` | **not set** (default 5) | raised to the floor, one INFO: `io_timeout_ms raised from 5 to <F> ms for a sync read of <K> servos` (from K = 10) | cannot happen |
| `auto` | set | one WARN: `io_timeout_ms <T> is below the <F> ms a sync read of <K> servos needs here; using one feedback read per servo`; the mode is **latched to `per_servo`** and the configuration line shows `'per_servo'` | one WARN: `io_timeout_ms is <T> ms and a failed read pays the 2 ms drain on top of it; ...` (the timeout is not wrong, just expensive when a servo goes silent) |
| `sync_read` | set | FATAL, `on_init` ERROR: `hardware parameter 'io_timeout_ms' is '<T>', below the <F> ms a sync read of <K> servos needs; feedback_mode is 'sync_read', which rules out the per-servo path that would survive it, so raise io_timeout_ms to at least <F> or use 'auto'` | same WARN as above |

So the 5 ms default is silent up to nine declared joints.

**At activation**, `auto` and `sync_read` send one sync read (a retry is allowed). The test is whether every servo that answered its activation feedback read also answers the sync read:

- On success, one INFO: `feedback for <n> servos travels in one sync read per cycle (INST_SYNC_READ)`.
- If a servo does not answer it, `auto` logs `sync read went unanswered by motor id(s) <ids>; falling back to one feedback read per servo for this activation`.
- `sync_read` logs a FATAL instead. `on_activate` returns `ERROR`, `on_error` parks the servos and closes the port, and the component returns to `unconfigured`.
- The decision is taken again at every activation.

The measurements behind the floor and the default are in [`io_timeout_ms`, and how often a transaction fails](#io_timeout_ms-and-how-often-a-transaction-fails).

### Joint parameters

Each goes in a `<joint>` block as `<param name="...">value</param>`. Only `id` is required.

<!-- reference:joint-parameters:begin -->

| parameter | type | default | legal values | what it does |
|---|---|---|---|---|
| `id` | integer | **required** | `1`..`253`, unique within the `<ros2_control>` block | Bus id of the servo (scan's `id` column). 0 is not usable (the tools can reach it; `set_id` moves a servo off it). 254 is the broadcast id and 255 the packet header. Two blocks on one port are kept apart by the port lock, not by an id check. |
| `type` | `pos` or `vel` | inferred | exactly `pos` or `vel` (**case-sensitive**) | `pos` runs the servo in mode 0 and drives it to a goal position. `vel` runs it in mode 1 as a closed-loop wheel. If absent: a joint with a velocity command interface and **no** position command interface is `vel`, everything else is `pos`. It must agree with the command interfaces: `pos` needs a `position` command interface, and `vel` must not have one. The mode register (EEPROM 33) is written only when it differs, at configure and at an activation that adds a servo back. |
| `offset` | double (rad) | `0.0` | any finite number. On a `pos` joint the position limits must map inside the servo's single turn (see below). | The servo angle that is joint zero: `position = sign x (tick x 2 pi / encoder_steps - offset)` and `goal tick = (sign x command + offset) x encoder_steps / (2 pi)`. It also shifts a `vel` joint's reported position, and a `vel` joint is not range-checked. |
| `inverted` | bool | `false` | `true` / `false` (any case) | Flips the `position`, `velocity` and `load` states and the `position` and `velocity` commands. It does **not** flip `effort`, `current`, `torque`, `max_speed` or `max_accel`. Written as `<param name="inverted">true</param>`, not as an attribute. |
| `max_speed` | double (rad/s), a magnitude | 6000 steps/s = **9.2039 rad/s** at 4096 | `> 0`. Values that round to 0 steps/s (below 0.000767 rad/s) are refused. Values above 32767 steps/s (**50.2639 rad/s**) are capped, with a WARN. | `pos` joint: the ceiling of the goal speed the driver sends. `vel` joint: the velocity command is clamped to plus or minus this. Also used as the "could have turned" bound of the unwrap-gap warning. The default is a register value (6000 steps/s), so in rad/s it changes with `encoder_steps` (36.8 rad/s at 1024). |
| `max_accel` | double (rad/s^2), a magnitude | register value 150 = **23.0097 rad/s^2** at 4096 | `0` (no ramp), or a value that rounds to at least 1 register count (0.0767 rad/s^2 or more). Values above 255 counts (**39.1165 rad/s^2**) are capped, with a WARN. | Acceleration register 41. A `pos` joint sends it in every goal record. A `vel` joint's is written at configure, at every activation, when a servo answers again after missed reads, and when a fault clears. The scale (100 steps/s^2 per count) is from the Feetech table and **not verified** on the ST3025. An absent `max_accel` keeps register 150 and never goes through that scale. |
| `unwrap` | bool | `true` for `vel`, `false` for `pos` | `true` / `false` (any case). `true` on a `pos` joint is refused. | Makes `position` a continuous multi-turn count instead of wrapping at one revolution. One INFO per joint that unwraps: `joint '<j>' reports an unwrapped, multi-turn position`. |

<!-- reference:joint-parameters:end -->

**The single-turn check** applies to `pos` joints only.
Each limit is mapped with the write path's own formula, `lround((sign x q + offset) x encoder_steps / 2 pi)`, and the result must lie in `[0, encoder_steps - 1]`. With `inverted` the two ends swap.

- **Both limits finite:** both must fit.
- **One limit finite:** a WARN (below), and that one limit must fit.
- **No limits:** a WARN, and joint zero, `lround(offset x encoder_steps / 2 pi)`, must fit.

The WARN reads `joint '<j>' has type 'pos' but no finite position command limits, so its offset cannot be checked against the servo's single-turn range [0, <steps-1>] ticks; add <param name="min"> and <param name="max"> to its position command interface`.

**`offset`, worked through** (at the default 4096 steps). `position = sign x (tick x 2π / encoder_steps - offset)`, so `offset` names the servo tick that is joint zero:

- `offset` 1.570796: tick 1024 is joint zero. That is the example's choice, which fits the reference bench's existing calibration. There, `scan` put the servos of `joint1` and `joint2` at tick 1026, and right after activation both joints read 0.00307 rad, which is (1026 - 1024) × 2π / 4096 (measured).
- `offset` 3.141593: tick 2048 is joint zero. That is what `calibrate_midpoint` produces: a servo reads 0 where it was calibrated.
- So with the example's `offset`, a freshly calibrated servo reads +π/2 at the position where it was calibrated, the example's upper limit. After `calibrate_midpoint`, use `offset` 3.141593 for that joint.

**`type` can write EEPROM.** At configure, and at an activation that adds back a servo that was missing or dropped, the driver reads the servo's mode register and rewrites it when it differs from the joint's `type` (a position servo at a `vel` joint becomes a wheel, and the other way round). The register is in EEPROM, so the new mode is meant to survive a power cycle, although the driver does not check the EEPROM unlock and lock around the write ([Known issues and limitations](#known-issues-and-limitations), item 7). The driver logs `motor id '<N>' mode changed from <old> to <new>`; expect that line once after you change a joint's `type`.

**More about individual parameters:**

- A joint without `id` is refused with `joint '<j>' has no <param name="id">; every joint needs the bus id of its servo (1..253)`.
- `inverted` is a `<param>` child of the `<joint>`, and it takes `true` or `false` only (not `1` or `0`). An `inverted="true"` attribute on the `<joint>` element is read by nothing.
- `max_speed`: the refusal of a tiny value says "less than one encoder step per second", but the test is whether the value rounds to 0 steps/s: 0.5 to 1 step/s is accepted as 1 step/s.
- An unknown joint parameter logs `joint '<j>' parameter '<name>' is not used by this driver; ignoring it`, and loading continues. So `invert` instead of `inverted` gives a servo that turns the wrong way, with this WARN as the only sign (measured on the reference bench).

**Joint rejection messages.** Each is one FATAL, and `on_init` returns ERROR. `<j>` is the joint's name.

| cause | message |
|---|---|
| `id` absent or empty | `joint '<j>' has no <param name="id">; every joint needs the bus id of its servo (1..253)` |
| `id` not an integer | `joint '<j>' has an id that is not a whole number: '<v>'` |
| `id` out of 1..253 | `joint '<j>' has id <v>, outside the range 1..253; 254 is the broadcast id the sync writes use and 255 is the packet header byte` |
| `id` repeated | `joint '<j>' has id <n>, which joint '<k>' already uses; ids must be unique within a <ros2_control> block` |
| `type` not `pos`/`vel` (includes empty and `position`) | `joint '<j>' has type '<v>'; it must be 'pos' or 'vel', or left out so the driver infers it from the command interfaces` |
| `pos` without a position command | `joint '<j>' has type 'pos' but declares no position command interface; a position joint needs <command_interface name="position"> (a velocity command interface only paces the move)` (the parenthesis is inaccurate: [Command interfaces and limits](#command-interfaces-and-limits)) |
| `vel` with a position command | `joint '<j>' has type 'vel' but declares a position command interface; a velocity joint runs its servo in wheel mode and takes only <command_interface name="velocity">` |
| `offset` not finite | `joint '<j>' has an offset that is not a finite number: '<v>'` |
| limits outside the single turn | `joint '<j>': position limits [<min>, <max>] rad with offset <o> rad and inverted=<b> map to servo ticks [<lo>, <hi>], outside the servo's single-turn range [0, <steps-1>]; change the offset, the limits, or 'inverted'` (plus one-limit and no-limit variants) |
| `inverted` not a bool | `joint '<j>' has inverted='<v>'; it must be 'true' or 'false'` |
| `max_speed` not finite / `<= 0` / rounds to 0 | `... has a max_speed that is not a finite number: '<v>'` / `... has max_speed <v> rad/s; it must be greater than 0` / `... which is less than one encoder step per second (<x> rad/s); raise it` |
| `max_accel` not finite / `< 0` / rounds to 0 | `... has a max_accel that is not a finite number: '<v>'` / `... it must be 0 (no acceleration limit) or greater` / `... which rounds to 0 acceleration-register counts; the smallest step is <x> rad/s^2, and 0 means 'no acceleration limit'` |
| `unwrap` not a bool / `true` on `pos` | `joint '<j>' has unwrap='<v>'; it must be 'true' or 'false'` / `joint '<j>' has unwrap=true with type pos; only a vel joint has a multi-turn position` |

The two caps are WARNs, and loading continues: `joint '<j>' has max_speed <v> rad/s, above the largest the goal speed register can hold (<cap> rad/s); using that instead` (the cap is 50.2639 rad/s at 4096 steps), and the same shape for `max_accel` (39.1165 rad/s^2).

One INFO per successful load summarises the joints: `parsed <n> joints: <p> position (mode 0), <v> velocity (mode 1)`.

### Command interfaces and limits

| command interface | allowed on | unit | driver reads its `min`/`max`? | driver-side bound | framework bound (only with `enforce_command_limits: true`) |
|---|---|---|---|---|---|
| `position` | `pos` joints (required there) | rad, joint frame | **yes**. Both are optional. | Every goal is clamped to `[min, max]`. A joint found more than one encoder step outside them at activation, or at a shutdown/error park, is **held where it is** until it is commanded to a position inside them. They are also the input of the single-turn check. INFO `joint '<j>' position commands clamped to [<min>, <max>] rad`. | `[max(min, URDF lower), min(max, URDF upper)]` for `revolute`/`prismatic`. A `continuous` joint has no URDF position limits. |
| `velocity` | both types; the only command interface of a `vel` joint | rad/s, joint frame | **no**, ignored by the driver | `vel` joint: clamped to plus or minus `max_speed`, then rounded to whole steps/s (1 step/s = 0.00153 rad/s); the ST3025 runs it more coarsely still (below the table). `pos` joint: **not used** once a position has been measured, which is always after activation. The goal speed is paced as `abs(goal - measured) / period`, clamped to `[1, max_speed]` steps/s. | symmetric: `min(abs(min), max, URDF <limit velocity>)` |
| anything else (`effort`, `acceleration`, ...) | refused | | | FATAL `a joint is using a command interface that isn't position or velocity` | |
| none at all | refused | | | FATAL `a joint does not have a command interfaces` | |

- **A position joint's `velocity` command is not used.** Once a position has been measured, which is always the case after activation, the driver paces the servo's goal speed itself: `|goal - measured| / period`, clamped to `[1, max_speed]` steps/s. The velocity command is only a fallback for a cycle with no measurement.
- So a raw position step through a controller that does not interpolate (a forward position controller, say) gets a goal speed of up to the joint's `max_speed`, 9.2 rad/s by default, and the servo's own acceleration limit, not the driver, then shapes the move. Measured on the reference bench on 2026-09-24: a 0.5 rad step of `joint1`, sent as a trajectory point with a `time_from_start` of 10 ms, peaked at 3.14 rad/s and was within 0.005 rad of its target 0.30 s after it was sent, while the same joint's 0.6 rad move over a 2 s trajectory peaked at about 0.4 rad/s (0.38 rad/s in one run and 0.46 rad/s in another; every velocity reading there was a multiple of 0.0767 rad/s, [State interfaces](#state-interfaces)). The step's two numbers are consistent with the default `max_accel` (register 150, 23 rad/s^2 by the unverified scale in [Joint parameters](#joint-parameters)): accelerating and then braking at that rate over 0.5 rad peaks at about 3.4 rad/s and takes about 0.3 s (worked out, not measured; the registers were not read during the step). For slow moves, use a trajectory controller (as the example does) or lower `max_speed`.
- **What a wheel actually turns at.** The driver writes a `vel` joint's goal speed in whole steps/s, but the ST3025 does not run every such value. On the reference bench, commands of 0.5, 1.0 and 2.0 rad/s, which the driver writes as 326, 652 and 1304 steps/s, turned the wheels at about 0.46, 0.99 and 1.99 rad/s, and -0.5 rad/s at about -0.46 rad/s. That fits the servo rounding the goal speed toward zero to a multiple of 50 steps/s (0.0767 rad/s at 4096 steps): 300, 650 and 1300 steps/s are 0.460, 0.997 and 1.994 rad/s. The rule is inferred from those four commands, not read from the firmware; by it, a command below 0.0767 rad/s would not turn the wheel at all (worked out, not measured).
- The driver's own FATAL for a `pos` joint without a position command says a velocity command interface "only paces the move"; that is inaccurate ([Known issues and limitations](#known-issues-and-limitations), item 5).
- **The out-of-limit hold.** A position joint that starts outside its limits (moved by hand with the torque off, say) is not run to the nearest limit. The driver logs `joint '<j>' starts at <x> rad, outside its limits [<min>, <max>]; holding it there until it is commanded to a position inside them` and keeps it where it is until a command inside the limits arrives.
- `enforce_command_limits` is a **controller-manager** parameter, `false` by default in 4.48, and the example sets it to `false` explicitly. It is read when the hardware is loaded, so setting it on a running stack has no effect. The second line of the block in [Start-up](#start-up-and-servos-that-do-not-answer) prints it.
- With it off, the driver's position clamp and `max_speed` are the only bounds. The `min`/`max` of a `velocity` command interface and the URDF `<limit>` then do nothing at all.
- With it on, the framework clamps first, using the merged limits, and the driver clamps again with its own. One side effect: a position joint whose **measured** position is more than 0.0087 rad outside its limits (the merged URDF `<limit>` and `<command_interface>` min/max) makes the framework's limiter throw `Joint position is out of bounds for the joint ...` on the first cycle after a controller claims that joint's position command. Nothing in the controller manager's loop catches it, so, worked out from the ros2_control 4.48.0 source and not measured, expect `ros2_control_node` to abort rather than one controller to be deactivated. That pre-empts the driver's "hold where it started".
- A `<limits enable="false"/>` child element disables the framework's limits for that interface or joint. It does **not** disable the driver's position clamp.
- A command interface declared twice on one joint is reported by the resource manager (`already existing key`); on any joint but the last, the description is refused.
- Commands start as NaN, or as the command interface's `initial_value` if it declares one. `on_activate` sets every position command to the measured position (0.0 for an absent servo) and every velocity command to 0.0, so a controller activated next starts from where the robot is.

### State interfaces

A joint may declare any subset of the state interfaces below, in any order, and a joint that declares none is legal too.

The driver exports what it was asked for, in **description order**: joint by joint as the `<ros2_control>` block lists them, and within each joint in the order its `<state_interface>` elements appear.
That is the order `ros2 control list_hardware_components -v` prints.

It is **not** a promise about `/joint_states` or `/dynamic_joint_states`.
The ordering on those topics is `joint_state_broadcaster`'s, produced downstream of the driver's export, and it need not match the description; on the reference bench it reliably does not.
**Read interfaces by name, never by index.**

**`/joint_states` carries only `position`, `velocity` and `effort`.** Every other declared interface (`current`, `voltage`, `temperature`, `load`, `status`, `torque`) reaches only `/dynamic_joint_states`, which `ros2 topic echo --once /dynamic_joint_states` prints.

<!-- reference:state-interfaces:begin -->

| name | unit | frame / sign | source and scale | notes |
|---|---|---|---|---|
| `position` | rad | joint frame, flipped by `inverted` | reg 56, `sign x (ticks x 2 pi / encoder_steps - offset)` | unbounded when `unwrap` is on |
| `velocity` | rad/s | joint frame, flipped | reg 58, `sign x speed_steps x 2 pi / encoder_steps` | on the reference bench every reading, of a position servo and of a wheel, was a multiple of 50 steps/s (0.0767 rad/s), so a still servo can read 0.0767 rad/s in a single message |
| `effort` | N m | a **magnitude**, not flipped | `current x torque_constant_nm_per_a` | **approximate, scale unverified** (the current scale is unverified) |
| `current` | A | a magnitude, not flipped | reg 69, `counts x current_per_count_a` | **approximate, scale unverified**; the firmware never sets the sign bit |
| `voltage` | V | none | reg 62, `raw x 0.1` | **approximate, scale unverified**: the 0.1 V/count scale is from a third-party table; the reference bench reads 12.3 V on a 12 V supply |
| `temperature` | deg C | none | reg 63, raw | |
| `load` | fraction of full PWM | joint frame, flipped | reg 60, `raw / 1000`, sign on bit 10 | about -1.023..+1.023, not clamped; the only direction-bearing member of the effort family |
| `status` | bitmask 0..255 | none | status byte of the reply frame | NaN while there is no reply (see below) |
| `torque` | kg cm | a magnitude | `current x torque_constant_nm_per_a / 0.0980665` | **deprecated** alias of `effort`; **approximate, scale unverified**. At the default constant this is exactly `current x 9.0`. If you change the constant, `torque` follows it. |

<!-- reference:state-interfaces:end -->

- `torque` still works and still reports kg cm; declare `effort` instead, which reports N m. A joint that declares `torque` logs one WARN when the description loads: `joint '<j>' declares the deprecated state interface 'torque' (kg cm); it keeps working and keeps reporting kg cm, but declare 'effort' instead, which reports N m`. 0.1.0 truncated `torque` to whole amperes before scaling it, so it read 0 below 1 A.
- **Rejected at load** (FATAL):
  - a name that is not one of the nine, including `moving`, which is deliberately not served: `joint '<j>' declares the unsupported state interface '<name>'; supported names are position, velocity, effort, current, voltage, temperature, load, status and the deprecated torque`;
  - a `data_type` other than `double`: `joint '<j>' declares the state interface '<name>' with data_type '<type>'; only 'double' is supported`;
  - the same name twice: `joint '<j>' declares the state interface '<name>' more than once`.
- A `<gpio>` or `<sensor>` in the same `<ros2_control>` block is refused by the resource manager ("Discrepancy between robot description file (urdf) and actually exported HW interfaces"). The driver exports joint interfaces only.
- **Values over the lifecycle:**
  - Every state is NaN from load until the first reading.
  - `<param name="initial_value">` on a state interface is visible from load only until the driver first publishes that joint's states: at the first `read()` after configure (the controller manager calls `read()` on an inactive component too) or at `on_activate`, whichever comes first.
  - A servo that is absent at configure time, or dropped after `max_read_fails`: only a `pos` joint's `position` **mirrors its position command**; a `vel` joint's `position` holds its last reading if the joint unwraps (the default) and returns to the value it read at activation if `unwrap` is false; it reads 0.0 if the servo never answered, and also after a deactivate/activate that does not find the servo; `velocity`, `effort`, `current`, `voltage`, `temperature`, `load` and `torque` read `0.0`; `status` reads NaN.
  - A failed read keeps the last good sample.

#### What `inverted` flips

`<param name="inverted">true</param>` flips **three** state interfaces, `position`, `velocity` and `load`, and both command interfaces. It does **not** flip `effort`, `current` or `torque`.

The servo's protocol allows a signed current (`ReadCurrent` decodes a sign bit exactly as `ReadSpeed` does), but the ST3025's firmware never sets it: a recording with the wheels driven in both directions contains no negative current sample at all.
`current` is therefore a **magnitude**, and so are the `effort` and `torque` derived from it.
Negating a magnitude would make every mirrored joint read uniformly negative, which tells you less than leaving it unsigned.

**So, on an inverted joint, `effort` and `current` are unsigned magnitudes and do not agree in sign with `velocity`. If you need to know which way a joint is working, read `load`: it is the direction-bearing signal.** Its sign is real (the servo reports it in bit 10 of the load word) and the driver flips it with the joint.

`effort == current * torque_constant_nm_per_a` holds for every joint, inverted or not.

#### The `status` interface and its bits

`status` is the servo's status byte, published as a plain bitmask (`0.0` .. `255.0`), or **NaN** while the driver has no reply from that servo (never pinged, or dropped after `max_read_fails`).
`0.0` means "the servo answered and reported no fault". Guard it with `if (std::isfinite(v)) { bits = static_cast<uint8_t>(v); }`.
When a servo's status byte changes to a non-zero value, the driver logs `motor id '<N>' reports status 0x<hh> (<bits>)` with the names below, and when the fault clears, `motor id '<N>' cleared its fault; re-enabling torque`.
The re-enable needs no action from you, and the driver does not keep a tripped servo off. It goes on sending the joint's command every cycle, so once torque is back on, a position joint moves toward its command again and a wheel turns at its command again. If the cause of the trip is still there (a jam, a stall), the joint pushes against it again and can trip again. If a joint trips against an obstruction, deactivate the hardware component: deactivation parks each position joint where it is and stops the wheels. Clear the cause before you activate it again, and keep clear of the joint: from the first cycle after the activation, the controller, which stays active, drives the joint toward the controller's last command again at up to `max_speed` ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost)). Whether and when the ST3025 clears a status bit while its cause persists was not measured.

<!-- reference:status-bits:begin -->

| bit | mask | name the driver prints | source |
|---|---|---|---|
| 0 | 0x01 | `voltage` | FEETECH SDK `ERRBIT_VOLTAGE` |
| 1 | 0x02 | `angle` | FEETECH SDK `ERRBIT_ANGLE` |
| 2 | 0x04 | `overheat` | FEETECH SDK `ERRBIT_OVERHEAT` |
| 3 | 0x08 | `overcurrent` | FEETECH SDK `ERRBIT_OVERELE` |
| 4 | 0x10 | `bit4 (meaning unverified)` | none |
| 5 | 0x20 | `overload` | FEETECH SDK `ERRBIT_OVERLOAD` |
| 6 | 0x40 | `bit6 (meaning unverified)` | none |
| 7 | 0x80 | `bit7 (meaning unverified)` | none |

<!-- reference:status-bits:end -->

The names come from FEETECH's own Python SDK ([FTServo_Python](https://github.com/ftservo/FTServo_Python), `scservo_sdk/protocol_packet_handler.py`, retrieved 2026-09-16), which decodes the same packet byte this driver captures.
An STS3215 memory table (V3.6) names status bits 0-5 voltage, sensor, temperature, current, angle and overload, which disagrees with the SDK's names on bits 1 and 4.
Neither source was verified on the ST3025's firmware, and **nothing in this package establishes which of these bits the firmware actually raises.**
That is why the raw byte is published, why three bits are printed as unverified, and why this table says only what the driver prints.
If you confirm a bit on real hardware, please open an issue with what you did and what you saw ([Contributing](#contributing)).

### Other ros2_control settings

| element / attribute | read by | effect on this driver |
|---|---|---|
| `<ros2_control name="...">` | framework | Component name and logger name (`<cm logger>.hardware_component.system.<name>`). |
| `<ros2_control type="system">` | framework | The plugin is a `SystemInterface`. |
| `<hardware><plugin>waveshare_servos/WaveshareServos</plugin>` | framework | |
| `rw_rate="N"` attribute on `<ros2_control>` | framework (4.48) | Reads and writes at N Hz. The driver does not read it. Pacing uses the real period since the last write, and the `max_read_fails` budget is in cycles. `list_hardware_components` shows `read/write rate: N Hz`. Absent or 0 means `update_rate`. A value that does not divide `update_rate` runs at the nearest integer decimation. A value above `update_rate` is treated as `update_rate`. See [Running the bus slower, or off the control thread](#running-the-bus-slower-or-off-the-control-thread). |
| `is_async="true"` attribute | framework | Separate thread. The driver does not read it. Its thread's priority is set by `<properties><async thread_priority="N"/></properties>` inside `<ros2_control>` (the 4.48 form; default 50). The older `thread_priority` attribute on `<ros2_control>` is still read when `is_async` is true. What 4.48 deprecates is the C++ field `HardwareInfo::thread_priority`, in favour of `async_params`. |
| an attribute the framework does not know | nobody | Dropped silently: a typo looks like nothing happened. |
| `<param name="min"/"max">` inside `<command_interface>` | driver (position) and framework (both) | [Command interfaces and limits](#command-interfaces-and-limits). |
| `<limits enable="false"/>` | framework only | [Command interfaces and limits](#command-interfaces-and-limits). |
| `<param name="initial_value">` inside an interface | framework | [State interfaces](#state-interfaces). |
| `data_type` attribute of `<state_interface>` | driver checks it | Must be `double` (the default). |
| URDF `<joint><limit>` | framework only | Merged most-restrictive-wins with the command interface `min`/`max` when `enforce_command_limits` is on. The driver never reads it. |
| xacro arguments `port`, `baudrate`, `use_mock_hardware` (and launch argument `gui`) | the example only | The launch-argument table in [Quick start](#3-run-the-example-on-the-reference-bench). |

### Several buses

Use one `<ros2_control>` block per adapter, each with its own `port`.
Ids must be unique within a block; the same id may appear in two blocks on different ports.
A second block on the same port is refused at configure by the port lock, not by an id check.


## Running it

### Start-up, and servos that do not answer

At start-up the driver pings every servo the description declares (`ping_attempts` tries each).
A servo that does not answer gets one WARN, `unable to ping motor id '<N>'; joint '<j>' will be skipped on the bus`. Then:

- With `allow_missing_servos` `false` (the default), the driver logs one FATAL naming every missing id and joint, `<n> of <m> servos did not answer: <list>; refusing to configure because 'allow_missing_servos' is false`, releases the port, and the configure fails.
  At controller-manager start-up the component's target state is `active`, so a refused configure makes the controller manager fail to set the component's initial state, and **`ros2_control_node` exits**. That is because the controller-manager parameter `hardware_components_initial_state.shutdown_on_initial_state_failure` defaults to `true` (ros2_control 4.48.0; the parameter is read-only).
- With `allow_missing_servos` `true`, the driver configures anyway and logs one summary WARN after the per-servo ones: `continuing without <n> of <m> servos because 'allow_missing_servos' is true: <list>; their joints mirror their commands into their states until the servos answer`. Despite that wording, only a `pos` joint's `position` mirrors its command: the other states of a missing servo read as [State interfaces](#state-interfaces) lists.

On a running controller manager, this block prints the two controller-manager defaults this README relies on (`True` and `False`):

```bash
ros2 param get /controller_manager hardware_components_initial_state.shutdown_on_initial_state_failure
ros2 param get /controller_manager enforce_command_limits
```

If you would rather the node came up with the component left unconfigured, set that **controller-manager** parameter; it is not a driver parameter:

```yaml
controller_manager:
  ros__parameters:
    hardware_components_initial_state:
      shutdown_on_initial_state_failure: false
```

With it, the refusal is logged as an ERROR, the node keeps running, the component stays `unconfigured`, and the port is released. Measured on the reference bench with one declared servo missing: the controller manager logged `Failed to set the initial state of the component : 'example_ws_ros2_control' to 'active'`, and a request to go `inactive` exited with status 1 while the servo was still missing.
Worked out from the ros2_control 4.48.0 and spawner code, not measured: no controller that uses the component's interfaces can activate, because an unconfigured component offers none of them; the example's spawner stops at the first controller that fails (`joint_state_broadcaster`), so the other two it lists are not loaded.
Once the bus is fixed, bring the component up with the last two lines of the block in [Recovery after a servo is lost](#recovery-after-a-servo-is-lost), after reading the warning above that block (a servo that was just switched on holds goal 0); while a servo is still missing, the request to go `inactive` fails again.
The controllers then have to be started again, because the example's spawner has already exited; that step was not measured.

Use the controller-manager parameter when you want the failure to be loud but survivable; use `allow_missing_servos` when you are deliberately running with part of the robot absent.
Without the escape hatch, `ros2_control_node` exits but the rest of the launch keeps running (one spawner waits for the controller manager with no timeout), so the launch does not end by itself ([Troubleshooting](#troubleshooting)).

### Recovery after a servo is lost

A servo that stops answering mid-run is dropped from the read cycle after `max_read_fails` consecutive failed reads (50 by default: half a second at 100 Hz), with one ERROR:

`motor id '<N>' stopped answering after <n> attempts; dropping it from the read cycle until the hardware is re-activated`

Until then, each failed read keeps the servo's last good sample, and the first failed read in a row (and every 200th) logs `read failed for motor id '<N>' (<k> in a row)`.
While a servo is dropped, its joint's states read as for a missing servo ([State interfaces](#state-interfaces)).
`allow_missing_servos` has no say in the drop, and none in whether a dropped joint is re-armed: it is a configure-time policy only.

To look for the servo again, deactivate and activate the hardware component, with the block below.

**Before you run it, keep clear of the joint.** Activation switches the servo's torque on before the driver writes a goal, so a position joint can start toward a stale goal for about one control period ([Known issues and limitations](#known-issues-and-limitations), item 1). Which goal that is depends on what reached the servo last. While the component is active, the driver writes each position joint's goal every cycle, also to a servo that came back limp and to one that was dropped. The `inactive` line parks each servo that answers at its measured position (a controller that is still active can replace that park with its own command, and a dropped servo is sent that command), and no goal is written between that line and the activation. So the stale goal is normally the last goal the driver sent, not 0. That a servo with its torque off keeps a goal written to it was measured for a single-servo write of its present position; the driver's sync write, and a goal other than the present position, were not measured. The stale goal is 0 for a servo that was switched on after the `inactive` line, while the component was unconfigured, or behind a lost adapter (below): a servo powers up with torque 0 and goal 0 (the measured power-up state). Servo tick 0 is the joint angle `-offset` (`+offset` with `inverted`): -π/2 rad, the lower end of the range, for the example's `joint1` and `joint2`.

The controllers stay active through the recovery. So from the first cycle after the activation, the driver sends the command of the controller that drives each joint, not the position the joint stopped at. For the example's trajectory controller, that command is the end point of its current trajectory, which the joint may not have reached if it went limp mid-move. A joint that is not there (it sagged or was moved while limp) is driven to it with a goal speed of up to `max_speed` (9.2 rad/s by default), like a raw step ([Command interfaces and limits](#command-interfaces-and-limits)). The out-of-limit hold does not prevent this move, because it lets go as soon as a command inside the limits arrives. The wheels turn at their velocity controller's last speed again. That the controllers stay active was measured on the reference bench; the rest of this paragraph is worked out from the code, not measured. Keep hands and objects out of the joint's range of motion before you activate. The drop ERROR does not say why a servo stopped answering, so treat every recovery this way.

```bash
ros2 control list_hardware_components
ros2 control set_hardware_component_state example_ws_ros2_control inactive
ros2 control set_hardware_component_state example_ws_ros2_control active
```

`example_ws_ros2_control` is the example's component name, the `name` of its `<ros2_control>` block; the first line lists the names on your system.
On activation the driver pings every servo that is missing or dropped, adds back each one that answers, with the INFO `motor id '<N>' answered on activation; adding it back`, and switches its torque on. The port stays open throughout.

| route | `allow_missing_servos` | servo answers again? | outcome |
|---|---|---|---|
| `active -> inactive -> active` (deactivate/activate) | either | yes | re-pinged in `on_activate`, added back with the INFO above; the port is never released |
| `active -> inactive -> active` | either | no | activates anyway; the joint stays absent (a `pos` joint's `position` mirrors its command, its other states read `0.0` and `status` NaN); the drop ERROR already in the log is the record |
| `unconfigured -> inactive` (a fresh configure) | `true` | no | configures, with one WARN per missing servo and the summary WARN |
| `unconfigured -> inactive` | `false` | no | FATAL, the port is closed, the configure fails; the component stays `unconfigured` and the port is free |
| `unconfigured -> inactive` | either | yes | configures normally |

So: a **re-configure** after a drop is a fresh configure and is subject to the gate; a **deactivate/activate** is not.

**A lost adapter.** If the USB adapter itself disconnects, every servo is dropped and the component stays active; nothing reports it as a failure ([Known issues and limitations](#known-issues-and-limitations), items 2 and 14).
The wheels keep their last speed. Neither route below can stop them over the lost adapter, because a shutdown's or cleanup's stop writes go to the dead descriptor. Route 1 below stops them when the new launch activates: its freshly started velocity controller holds no command, and activation sets every velocity command to 0. Route 2 does not: the velocity controller stays active through it and sends its last command again from the first cycle after `active`. So in route 2, publish a zero to the wheels before the `active` step, with the last line of the wheel block in [step 4](#4-move-a-joint-and-the-wheels). Switching off the servo supply stops them too. This is from the code: of it, only that the controllers stay active was measured (with the adapter left plugged in), and the unplug itself was not measured.
Deactivate/activate does **not** help: neither deactivating nor activating closes the port, so the activation pings over the dead descriptor and, by the second route of the table, activates anyway.
Re-plugging the adapter while the old descriptor is still held usually enumerates it as a new `/dev/ttyACMn`. Two ways out:

1. Always works: stop the launch with Ctrl-C, re-plug the adapter, and start the launch again.
2. Without a restart: set the component `unconfigured` (which closes the port), re-plug the adapter, then set it `active` (configure opens the port again and pings every servo). The `active` step also activates, so keep clear of the joints first, as the warning before the block above says: a servo that lost power as well holds goal 0, and from the first cycle the still-active controllers drive the joints to their last commands and the wheels at their last speeds. Publish the wheels' zero before that step (above):

```bash
ros2 control set_hardware_component_state example_ws_ros2_control unconfigured
# re-plug the adapter here
ros2 control set_hardware_component_state example_ws_ros2_control active
```

Address the adapter by its `/dev/serial/by-id/` name ([Serial port access](#serial-port-access)) so the re-plugged device keeps its name.
Setting the component `unconfigured` runs the driver's cleanup, which closes the port, and `active` configures it again, which opens the port and pings every servo.
The port release and the re-activation were measured on the reference bench, with the adapter left plugged in: after `unconfigured` no process held the port (checked 2 s later) and the driver logged its `bus totals:` line; after `active` the controller manager held the port again, no controller had been deactivated, and `joint1` and the wheels followed their next commands. The unplug itself was not measured.

### Shutdown

On SIGINT or SIGTERM (Ctrl-C on the launch), the controller manager deactivates the hardware and then shuts it down.
The driver stops the wheels, parks the position joints where they are, and closes the port.
Each deactivation logs one INFO line with the failed-transaction counts of the activation that just ended:

`bus totals: transactions <n>, failed <n> (<x> per million), worst consecutive <n>, dropped <n> [<per-servo counts>]`

To read your own run's counts, stop the launch with Ctrl-C and look for the line: `grep 'bus totals:' <your launch log>`.

**Only a clean shutdown over a working bus stops the wheels.** Nothing on the servo stops a wheel when the driver stops writing to it: a wheel keeps turning at its last commanded speed and a position servo keeps holding its goal with torque on. The bench test measures this: after the controller manager is killed with SIGKILL while the wheels turn at 2 rad/s, the wheels still turn at about 2 rad/s and the goal-speed register still holds the command. A crash or an out-of-memory kill runs no driver code either, so it leaves the servos the same way (from the code, not measured). Switching off the servo supply is the only stop that always works; keep it within reach whenever the wheels can move.

**Torque stays on.** The driver switches torque on at activation and does not switch it off at deactivation or at shutdown; of the command-line tools, only `calibrate_midpoint` and `factory_reset` switch it off.
So after shutdown the servos hold their pose and draw current until their supply is switched off. On the reference bench, after a Ctrl-C shutdown of the example with the wheels turning, the launch exited with status 0, the wheels had stopped, and the torque register of all four servos read 1 (on).

At Ctrl-C the log can also show two ERRORs, `[controller_manager.pal_statistics]: Exception in publisher thread: context cannot be slept with because it's invalid!`. They come from an upstream shutdown race in the controller manager's statistics publisher, after the driver has deactivated, not from the driver.

### Real-time scheduling

At start-up, `ros2_control_node` tries to move its control loop to `SCHED_FIFO` at priority 50.
That priority is the controller manager's `thread_priority` parameter, which is current in 4.48. The hardware component's async-thread priority is a different setting (`<properties><async thread_priority="N"/>`, or the older `thread_priority` attribute on `<ros2_control>`), read only when `is_async` is true.
Without permission, the controller manager logs one WARN and runs at normal priority:

`[controller_manager]: Could not enable FIFO RT scheduling policy: with error number <1>(Operation not permitted). ...`

The driver works either way, and **every measurement in this README was made without real-time scheduling**.
Real-time scheduling reduces loop jitter when the machine is busy.
Without it, a missed deadline shows up as the controller manager's `Overrun might occur, Total time : ... (Expected < ...)` WARN. A measurement of an early development version on the reference bench logged occasional overruns of 12 to 30 ms that coincided with command-line tools starting. The example's stack, run on the reference bench for 191.5 s without real-time scheduling, through dozens of `ros2` command-line calls, a refused `scan`, the hardware component taken to `inactive` and back and to `unconfigured` and back, and wheel runs of up to 2 rad/s, logged none.

For a normal (non-root) user, two resource limits decide it, not capabilities:

- `ulimit -r` (max real-time priority) must be at least 50.
- `ulimit -l` (max locked memory) matters only if the controller manager locks its memory. It does that by default only on a `PREEMPT_RT` kernel. On a stock kernel, add `lock_memory: true` to the controller manager's parameters, as in the block below. If the lock is refused, the log has `Unable to lock the memory: 'No proper privileges to lock the memory!'`. Use `unlimited` for the limit, not a size: the lock also covers every later allocation, including each new thread's stack.

```yaml
controller_manager:
  ros__parameters:
    lock_memory: true
```

**On the host**, create a `realtime` group with both limits and join it.
This change lasts and covers every member of the group: from their next login, any process they start may run at real-time priority up to 99 and lock any amount of memory. A runaway real-time process can then make the machine unresponsive and starve lower-priority real-time threads, the controller manager's loop among them, and a leaking one can pin memory that the rest of the system needs. On a shared machine, run the robot under a dedicated user. The `tee` line below replaces any existing file of that name:

```bash
sudo groupadd realtime
sudo usermod -aG realtime "$USER"
printf '@realtime - rtprio 99\n@realtime - memlock unlimited\n' | \
  sudo tee /etc/security/limits.d/99-realtime.conf
```

Then log out completely and back in; a new terminal is not enough.
The ros2_control documentation also lists `priority 99` lines; leave them out. In `limits.conf`, `priority` is not a real-time limit: it sets the nice value every login process starts with, and 99 means nice 19, the lowest priority.

**In the devcontainer**, `.devcontainer/devcontainer.json` passes `--ulimit rtprio=99 --ulimit memlock=-1`, which is what the container user needs; rebuild the container after changing `runArgs`. The reference-bench measurements were made in a container started without these flags.

- `--privileged` does not give real-time scheduling to the `ubuntu` user: capabilities go to root in the container, a process started as `ubuntu` has none, and so the kernel checks the ulimits.
- `--cap-add=sys_nice` is there for parity with the ros2_control documentation and only matters for a process run as root.
- A container's limits come from Docker, not from `limits.conf`, so a realtime group inside the image would do nothing.
- For your own `docker run`, pass `--ulimit rtprio=99 --ulimit memlock=-1`.
- From Docker's documentation, not tested with this package: with rootless Docker or Podman the ulimits cannot exceed the host user's own, so set up the host as well; and on a host with cgroup v1 and real-time group scheduling, a container also needs `--cpu-rt-runtime`.

**Check that it took effect**, while the stack runs, from the shell you launch from:

```bash
ulimit -r; ulimit -l
ps -L -o tid,cls,rtprio,comm -p "$(pgrep -o -f lib/controller_manager/ros2_control_node)"
grep VmLck /proc/"$(pgrep -o -f lib/controller_manager/ros2_control_node)"/status
```

1. `ulimit -r` prints `99` and `ulimit -l` prints `unlimited`.
2. The launch log has `Successful set up FIFO RT scheduling policy with priority 50.` and no `Could not enable FIFO RT scheduling policy`.
3. The `ps` line lists the controller manager's threads: its control-loop thread shows class `FF` and priority 50, while most others stay `TS`. Keep `-L ... -p`; do not add `-e`, which lists every process (and on a host some kernel threads are `FF` anyway).
4. Only with `lock_memory: true`: no `Unable to lock the memory` line, and the `VmLck` line is not `0 kB`. A successful lock prints nothing, so `VmLck` is the only evidence.


## Sizing the bus

The measurements in this section were made on the [reference bench](#requirements-and-tested-hardware), at `update_rate: 100` and without real-time scheduling; the model fitted to them, and the figures worked out from that model, are not measurements.
These are that bench's numbers, not a promise about your robot. The cost model below is the part that is meant to travel; the totals are not.

### What a cycle costs

| per 100 Hz control cycle, four servos | one read per servo (pre-release path) | one sync read (current driver) |
|---|---|---|
| `read()` -- position, velocity, load, voltage, temperature, current, status | 3.064 ms | **1.645 ms** |
| `write()` -- two position goals and two wheel speeds | 1.447 ms | **0.014 ms** |
| both, out of the 10 ms period | 4.51 ms (45 %) | **1.66 ms (16.6 %)** |

- The read saving is a round trip removed per extra servo: one `INST_SYNC_READ` covers all four servos instead of one `FeedBack()` round trip per joint.
- The write saving comes from the wheels' acceleration register. The per-servo path wrote it every cycle with a call that blocks until that servo acknowledges; the current driver writes it at four edges instead, and what is left is two unacknowledged broadcast packets the kernel accepts in microseconds.

How the table was measured, and why the published values did not change, is in the [appendix](#how-the-cost-table-was-measured).

### Measuring your own bus

The controller manager publishes its own timing on `/diagnostics`; `ros2 topic echo --once /diagnostics` prints one message.

- Per hardware component, `<component>.read_cycle.execution_time` and `<component>.write_cycle.execution_time`, each as `Avg: <us> [<min> - <max>] us, StdDev: <us>`.
- For the control loop, `periodicity.average`, `periodicity.standard_deviation`, `periodicity.min` and `periodicity.max`, in Hz.

The read average plus the write average, against the period (10 ms at 100 Hz), is the budget the model below predicts.
On the reference bench, about 20 s after the example activated, one message read `example_ws_ros2_control.read_cycle.execution_time` `Avg: 1649.45 [1561.73 - 2061.39] us, StdDev: 56.42`, `example_ws_ros2_control.write_cycle.execution_time` `Avg: 13.04 [7.34 - 87.36] us, StdDev: 2.52`, and a `periodicity.average` of 100.0004 Hz (standard deviation 0.28 Hz, minimum 95.5, maximum 104.9): four servos read in about 1.65 ms, as in the table above, and read plus write take about 1.66 ms of each 10 ms cycle.
With more than four servos, measure here first, and read your failure counts from the `bus totals:` line ([Shutdown](#shutdown)).

### How the cost scales with servo count

Measured per transaction, by least squares over one to four servos:

```text
sync read of n servos        t(n) = 0.476 + 0.290 n   ms
one FeedBack() per servo     t(n) = 0.750 n           ms
```

The crossover is n = 1.03, so the sync read wins from two servos up, and every servo after the first costs **0.29 ms** instead of 0.75 ms.
Above four servos this is a model, not a measurement: it is a straight line through four points on one bench with one adapter, and nothing has been run with more.

Broadcast sync writes are unacknowledged, so they cost the caller almost nothing regardless of servo count (0.004 ms measured at a 100 Hz cadence), but they do occupy the bus for `(8 + n * (nLen + 1)) * 10 us`, which is exactly wire time at 1 Mbaud, with `nLen` 7 bytes for a position record and 2 for a wheel speed.

**How many joints fit at 100 Hz.** Allowing half the 10 ms period for bus work and scaling the model by the measured 1.19 tail factor:

```text
1.19 * (0.476 + 0.290 n) + 0.02 <= 5   ->   n <= 12.8
```

So **twelve joints at 100 Hz with 50 % headroom**, on a p99 basis. On a mean basis the same budget allows fifteen or sixteen, and that is a mean-based figure; do not size a robot with it.
Two cross-checks, both worked out rather than measured, agree that twelve is the honest number: pure wire time would allow sixteen, and at 1 Mbaud a twelve-servo cycle is 3.76 ms of wire time, which the adapter's 1 ms USB framing (below) rounds up to a 4 ms floor.
For comparison, the per-servo path holds **five** joints in the same budget, so the sync read slightly more than doubles the joint count a 100 Hz loop can carry.

**What is actually binding** is that 0.29 ms of marginal cost per servo. It is not something a faster baud rate could fix: the adapter is USB CDC and the host polls it once per 1 ms USB frame, so at 1 Mbaud a back-to-back read loop bottoms out at 2.000 ms per cycle (499.7 Hz with four servos). 1 Mbaud is the fastest rate the servos and the driver support, so the line rate cannot be raised; a slower rate adds wire time and only raises every number in this section (worked out from wire time, not measured; see the baud-rate note under [Hardware parameters](#hardware-parameters)).
It is also not the packet size limits: a sync write chunks at 30 position records or 82 wheel-speed records per packet, and a bus that large is far past the timing ceiling above.

### `io_timeout_ms`, and how often a transaction fails

`io_timeout_ms` ([its reference](#io_timeout_ms-and-feedback_mode)) is the budget of one transaction, and one transaction may be a whole sync-read burst.

**Size it for the burst, not for one transaction.** A single `FeedBack()` survives a 1 ms timeout comfortably; a sync read of four ids fails 98.55 % of the time at 1 ms and 0.00 % at 2 ms and above.
The floor the driver checks against (the driver's formula, fitted on one to four servos) is roughly `0.48 + 0.29 * K` milliseconds for K declared joints, scaled by about 1.2 for the tail, plus a millisecond:

| joints declared (K) | 1 | 4 | 8 | 9 | 10 | 12 | 16 | 30 |
|---|---|---|---|---|---|---|---|---|
| advisory floor (ms) | 2 | 3 | 5 | 5 | 6 | 6 | 8 | 12 |

Below that floor the driver says so. A value you did not set is raised to the floor with one INFO; a value you did set is either warned about and demoted to the one-read-per-servo path, or refused outright if you pinned `feedback_mode` to `sync_read`. The 5 ms default is silent up to nine declared joints.

**What a silent servo costs.** On the sync-read path (`feedback_mode` `auto` or `sync_read`): one full timeout plus the driver's 2 ms drain, once per cycle, not once per servo, and it does not matter where the dead id sits in the list. On the `per_servo` path it is one timeout, with no drain. The first two rows below were measured with a library-level sync read (a probe of the vendored library, without the driver's error path), against a clean read at the same timeout (1.60-1.63 ms); the last row adds the driver's 2 ms drain and is worked out, not measured:

| `io_timeout_ms` | 2 | 3 | 5 | 10 | 20 |
|---|---|---|---|---|---|
| cost of one absent servo, library-level sync read (measured) | +0.46 ms | +1.47 ms | +3.56 ms | +8.57 ms | +18.60 ms |
| read, relative to a clean one, library-level sync read (measured) | 1.29x | 1.92x | 3.23x | 6.31x | 12.44x |
| the driver's read with one absent servo (worked out: measured + 2 ms drain) | about 4.1 ms | about 5.1 ms | about 7.2 ms | about 12.2 ms | about 22.2 ms |

At 100 Hz that is the whole argument for a small value. With one servo dead, 5 ms keeps the loop at 100 Hz, but the failed read (about 5.2 ms, plus the driver's 2 ms drain) takes about 7 of its 10 ms. At 10 ms the failed read takes about 12 ms, so every cycle overruns. The Jazzy controller manager then waits for the next period boundary (its overrun handling is on by default), so the loop drops to about 50 Hz. The vendored library's own 100 ms default would cost about ten missed cycles on every read. These rates are worked out from the costs above and the controller manager's loop, not measured with a dead servo.
A servo that is dead *at configure time* never enters the sync-read list at all and costs nothing; this is the price of one that goes quiet mid-run, for the `max_read_fails` cycles before it is dropped.

**It costs nothing when the bus is healthy.** Mean read latency is flat at 1.60-1.63 ms for every timeout from 2 ms to 20 ms, so there is no reason to keep a large value "for safety". Running the driver with the timeout set to 20 ms moved the measured read average by 0.003 ms, against a run-to-run spread of 0.008 ms. A healthy sync read never comes near its deadline.

**Sub-millisecond values are meaningless** on this adapter. It is USB CDC and the host polls it once per 1 ms frame: a 1 ms read window collects, on average, 0.0 of the 84 reply bytes, and all 84 are waiting 2 ms later. Design your timing constants on a 1 ms granularity.

**How often a transaction fails, on the reference bench.**

> Over three ten-minute soaks at 100 Hz (four servos at 1 Mbaud, both wheels turning at 1 rad/s and the arm holding position, `io_timeout_ms: 5`), the driver recorded **0 failed transactions in 182 994**, with a worst consecutive failing run of 0, no servo dropped, and zero failures on each of the four ids individually.
>
> Zero events is not a failure rate of zero. With nothing observed, the honest statement is an upper bound: by the rule of three, the failed-transaction rate on the reference bench is **below 17 per million at 95 % confidence** (3/182 994 = 16.4), under about six failures per hour of continuous operation at 100 Hz. The true rate may be far lower, and this measurement cannot tell you.

A fourth clean ten-minute soak inside the full bench test brings the pool to 0 failures in 244 158 transactions, which tightens the same bound to 13 per million (3/244 158 = 12.3).
To measure your own bus, run your own stack for as long as you like, stop it with Ctrl-C, and read the `bus totals:` line ([Shutdown](#shutdown)): it carries the same counters, with a per-servo breakdown.

**What a failure actually does.** The driver keeps the last good sample for that joint rather than publishing the `-1` a timed-out read returns, counts the failure against that servo, and after `max_read_fails` consecutive failures drops the servo from the read cycle with one ERROR. See [Recovery after a servo is lost](#recovery-after-a-servo-is-lost) for the message, what happens next and how to get the joint back.

### Running the bus slower, or off the control thread

`ros2_control` 4.48 can read and write the hardware at a lower rate than the controller manager's `update_rate`, and can run it on its own thread. Both are **attributes of the `<ros2_control>` element**, not `<param>` children of `<hardware>`, which is the mistake that costs an afternoon:

```xml
<!-- optional, ros2_control 4.48: read/write the hardware at a different rate than the
     controller manager's update_rate, and/or on its own thread
<ros2_control name="${name}" type="system" rw_rate="50" is_async="true">
-->
```

This package sets neither, for the reasons below. `ros2 control list_hardware_components` prints `read/write rate:` and `is_async:` unconditionally; that is how you check an attribute took effect.

Measured on the reference bench at `update_rate: 100` with four servos:

- `rw_rate="50"` does exactly what it says. Bus occupancy halves (49.98 against 99.96 transactions per second, so 8.2 % of wall clock instead of 16.5 %), with the per-read cost unchanged at about 1.65 ms and no failed transactions. The unwrapper, the command pacing and the drop policy are unaffected.
- `is_async="true"` changed no measurable number at four servos: read 1.648 ms against the synchronous 1.653 ms, `/joint_states` unchanged at 100 Hz. A 120 s recording found nothing torn, which is a negative result over one window, not a proof ([appendix](#rw_rate-is_async-and-the-bench-test)).

The caveats are worth more than the results:

- **`/joint_states` keeps publishing at the controller manager's `update_rate`.** A lower `rw_rate` does not slow the topic down; it makes each value repeat. Anything differentiating `position` numerically will read zero apparent motion on alternate samples.
- **Pick an `rw_rate` that divides the `update_rate`.** `rw_rate="60"` against `update_rate: 100` is reported by `list_hardware_components` as `60 Hz` and actually runs at 50 Hz: the controller manager decimates the loop by an integer, and nothing logs the difference. The CLI shows the rate you asked for, not the rate you got.
- **A value above the `update_rate`, and `rw_rate="0"`, are silently treated as the `update_rate`.** No warning, no error, no mention in the log, so a typo is invisible except through `list_hardware_components`.
- **The bench test assumes a 100 Hz component**, so `rw_rate` is outside what the bench test checks, which is why the package does not set it ([appendix](#rw_rate-is_async-and-the-bench-test)).
- **An async component asks for a real-time thread.** Without real-time privileges the controller manager warns once per component (`Could not enable FIFO RT scheduling policy`) and runs it at normal priority, which is the configuration in which async jitter is least predictable. If you use it, set up [Real-time scheduling](#real-time-scheduling).
- For an async component's thread priority, use `<properties><async thread_priority="N"/></properties>`, the form the 4.48 documentation describes. The older `thread_priority` attribute on `<ros2_control>` still works; the deprecation in 4.48 is on the C++ field `HardwareInfo::thread_priority` (in favour of `async_params`), not on the XML.

On this driver `is_async` buys nothing, because `read()` is the only expensive thing in the control thread, and in the reference-bench runs behind the numbers above (the bench test's controllers at `update_rate: 100`) the controller manager logged no `Overrun might occur` WARN. Use it if you have something else in that thread; use `rw_rate` if the bus, not the CPU, is your bottleneck.

### What this section does not cover

Four things a reader could reasonably expect from the numbers above, and which nobody has measured:

- **More than four servos.** The cost model, the twelve-joint ceiling and the timeout floor table are all fitted on one to four servos, on one bench, through one adapter. Nothing above four has been run. Treat the model as a model.
- **A servo that loses power mid-run.** The power-up state was measured: after a power cycle, every servo of the reference bench came back with torque 0, goal 0 and the EEPROM lock set. What follows from it is inferred, not measured: a servo that loses power mid-run and answers again before it is dropped comes back with torque off, and the driver re-enables torque only at activation and when a status fault clears, so, unless it reported a fault just before or after the outage, the joint stays limp until the hardware is deactivated and activated ([Known issues and limitations](#known-issues-and-limitations), item 8). The wheel acceleration register (SRAM) is rewritten at four edges (when the groups are built, on activation, when a servo starts answering again, and when a fault clears), and whether those edges catch every brown-out is untested. A mid-run brown-out was not measured.
- **More than 30 position servos.** Their goal positions split across two packets about 2.5 ms apart (worked out from wire time at 1 Mbaud). Whether that skew is visible in motion cannot be answered on a four-servo bench.
- **`is_async` and torn samples.** The 120 s recording above found nothing torn, which is a negative result over one window and not a proof that the framework copies the driver's state arrays under a lock.


## Command-line tools

Four command-line tools talk to the servos directly: `scan`, `set_id`, `calibrate_midpoint` and `factory_reset`.
Each takes its parameters after `--ros-args`, as the commands below show, and opens the bus exclusively, exactly as the hardware interface does.
So **stop the controller manager first**: while another process holds the port exclusively or holds its lock (a controller manager, or another of these tools), a tool refuses with exit code 1, sends nothing, and names the holder:

`port '<port>' is held by another process<holders>; refusing to share the bus -- stop that process first (a running controller manager holds the port for as long as its hardware component is configured). Nothing was sent to the servos.`

`<holders>` is ` (pid <N> <name>, pid <N> <name>, ...)`, one `pid <N> <name>` for each process that has the port open, where `<name>` is the process name cut to 15 characters (it is left out if it cannot be read). When the holder belongs to another user, `<holders>` is ` (no holder visible in /proc; it may belong to another user)`. A `/dev/serial/by-id/` path works for this, because the tools resolve it to the device.

**A program that has the port open without taking the lock (screen, minicom, a plain serial script) is not refused.** The tool prints `process(es) <list> had '<port>' open before this tool took it; they hold no lock and can still write to the bus`, where `<list>` is `pid <N> <name>, ...`, and runs anyway, so close such programs before using `set_id`, `calibrate_midpoint` or `factory_reset`; [Troubleshooting](#troubleshooting) shows how to list them.

Every message a tool writes itself starts with the tool's name, for example `scan: `. Three kinds of stderr line do not: the usage line printed after a refusal (exit code 64), rcl's own errors about a malformed `--ros-args` line (for example `[ERROR] ... [rcl]: Failed to parse global arguments`), and output from the vendored serial layer. That output is the `serial speed <N>` line printed on every open, plus a `perror` line such as `open:: ...` or `tcsetattr:: ...` if the port cannot be opened or configured.

<!-- reference:tool-parameters:begin -->

| tool | parameters | required | ranges |
|---|---|---|---|
| `scan` | `port`, `baudrate` | none | `port`: a device path; `baudrate`: one of the seven rates |
| `set_id` | `start_id`, `new_id`, `port`, `baudrate` | `start_id` and `new_id` | `start_id` 0..253; `new_id` 1..253, different from `start_id` and not already answering |
| `calibrate_midpoint` | `id`, `port`, `baudrate` | `id` | `id` 0..253 |
| `factory_reset` | `id`, `port`, `baudrate` | `id` | `id` 0..253 |

<!-- reference:tool-parameters:end -->

| parameter | type | default | legal values | notes |
|---|---|---|---|---|
| `port` | string | `/dev/ttyACM0` | non-empty | A `/dev/serial/by-id` link works: the tool resolves it to name the holder of a busy port. |
| `baudrate` | integer | `1000000` | the same seven rates as the hardware parameter | Must be the servos' current rate. `factory_reset` may leave the servo at 1 000 000 and says so. |
| `start_id` | integer | required | `0`..`253` | The servo to renumber; 0 is reachable here. |
| `new_id` | integer | required | `1`..`253`, different from `start_id` | Must not already answer (checked on the bus; exit 4). |
| `id` | integer | required | `0`..`253` | |

Every tool refuses, with exit code 64 and without opening the port:

- a value of the wrong type: there is **no type coercion**, so `-p port:=0` is an integer and `-p baudrate:=1e6` a double, and both are refused; ids must be integers;
- an **unknown** parameter name (the message lists the accepted ones);
- the 0.1.0 names `device_port` and `baud_rate`, even next to their replacements `port` and `baudrate`;
- a parameter override addressed to another node name, which rclcpp would drop silently (overrides with no node prefix, for `/**`, or for the tool's own node name are accepted; `/*` too when the node is not namespaced; the same holds for `--params-file` keys);
- a positional argument;
- `start_id` equal to `new_id`;
- an id out of range (checked as a 64-bit integer, before any narrowing).

Every parameter error is printed at once, followed by the usage line: first any refusal of overrides addressed to another node, then the rest sorted by parameter name. A positional argument, or an argument rcl cannot parse (such as `-p port:=`), is reported on its own, before any parameter is checked, so fix it first and run again.
Fixed, not parameters: 3 ping attempts per id, the scan range 0..253, and the tools' io timeout, `max(5, ceil(510000 / baudrate) + 2)` ms: 5 at 1 000 000 and 500 000, 7 at 115 200, 11 at 57 600, 16 at 38 400, 29 at 19 200 and 56 at 9600.

### Find the servos on a bus

`ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0`

Pings every id from 0 to 253 (about 4 s at 1 Mbaud) and prints one row per servo that answers: id, type, mode, model (registers 3-4), baud-rate register (and the rate it stands for), position, supply voltage, temperature, status byte and position offset.
Read-only. Use the `id` column as `<param name="id">` and the `type` column (`pos` for a mode 0 servo, `vel` for mode 1) as `<param name="type">`.
The hardware interface cannot use id 0; after the table, scan prints a note for it on stderr: `scan: id 0: the hardware interface accepts ids 1..253; give this servo another id with set_id before putting it in a URDF`.
When no servo answers, it says `check servo power (USB does not power the servos), the wiring, and the baud rate: a servo set to another rate answers only at that rate` and exits with status 3; [step 2 of the quick start](#2-find-your-servos) loops over every rate.

### Change a servo's id

`set_id` writes a new id into the servo's EEPROM.
New servos all ship with the same id, and two servos on one id answer on top of each other (no tool can tell them apart), so connect only the servo you are renumbering (power off before plugging it in), then run:

```bash
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
```

Both ids are required. `new_id` must be 1..253 and must not already answer, or set_id refuses and changes nothing.
It checks that the servo answers on the new id and no longer on the old one, and that its other settings did not change.
The id is stored in EEPROM: power-cycle the servo and run `scan` to confirm it survived.

### Set the midpoint

`calibrate_midpoint` writes the servo's position offset into its EEPROM, and it switches the servo's torque off first and leaves it off, so an arm may sag; the hardware interface turns it back on when it activates.
Position-mode (mode 0) servos only; a wheel is refused.

The tool does not move the servo. It makes the position where the servo settles, with its torque off, the new tick 2048. So first bring the joint to the angle you want as its centre, for example by commanding it there and then stopping the stack (the driver leaves the torque on). Then hold the joint there while the tool runs: the tool switches the torque off, waits until the position holds within 1 tick for 100 ms, and calibrates wherever it settled, so an arm that sags is calibrated where it sagged. If the servo is still moving after 2 s, the tool refuses with exit 4 (`servo <id> is still moving (<a> -> <b>); hold it still at the intended midpoint and run again. Its torque is now OFF; no EEPROM byte was written.`).

```bash
ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<id>
```

`id` is required. Makes the servo's present position read tick 2048 (π rad, 180 degrees) and checks that it does.
The servo's goal register keeps the value it had before the calibration, which then names a different angle, so the first activation afterwards can briefly move the joint toward it until the hardware interface's first command replaces it ([Known issues and limitations](#known-issues-and-limitations), item 1); on the reference bench the old goal was about a quarter turn away.
Before the joint is next commanded, set that joint's `offset` to 3.141593 so the calibrated position is joint zero; with the example's 1.570796 it reads +π/2 ([Joint parameters](#joint-parameters)).

### Reset a servo to its factory settings

`factory_reset` puts one servo's EEPROM settings, all but the id, back to their factory values, and switches its torque off.
On the ST3025 a reset puts the EEPROM settings back to their factory values but keeps the id: the baud rate goes back to 1 Mbaud, the offset to 0 (undoing `calibrate_midpoint`) and the mode to 0 (position). This was measured on one servo (firmware 3.20). There the return delay, both angle limits, the temperature limit and three control gains (registers 7, 9, 11, 13, 21, 37 and 39) were also changed and came back after the reset. The other settings, including the remaining protection limits and gains, were not tried: that they are reset too comes from the protocol's description of RESET ("reset control table to factory value"), not from a measurement. The tool lists every register that changed.
The torque is switched off first and left off.
If the servo was at another baud rate, the tool follows it to 1 Mbaud to check it, and says so: `scan` and the hardware interface then need `baudrate` 1000000 for that servo.

Before you run it:

- The torque goes off and stays off. The driver leaves torque on when the controller manager stops, so a loaded arm may sag or fall: support it first.
- The old settings are kept nowhere else. Save the tool's output: it lists each changed register with its old and new value, and that list is the only record of the old offset, angle limits, protection limits and gains.
- With the offset back at 0, the joint's `offset` parameter and its limits land at other physical angles. Run `calibrate_midpoint` again (and use `offset` 3.141593), or work out the offset again, before commanding the joint.
- Any angle or protection limit you tightened is back at its factory value.
- Afterwards the servo's torque and goal registers read 0, as after a power-up, so [Known issues and limitations](#known-issues-and-limitations), item 1, applies to the next activation.

```bash
ros2 run waveshare_servos factory_reset --ros-args -p id:=<id>
```

`id` is required. Sends the protocol's RESET instruction (0x06) to that one servo and checks the result by reading the servo back.
The servo keeps its id, so a reset cannot rescue a servo that answers at no id: `scan` has to find it first.
Only the addressed servo is reset, never the whole bus, and the tool refuses if two servos may share that id.
A joint declared `vel` gets its mode back when the hardware interface configures it.

### Exit codes

Shared by all four tools:

<!-- reference:exit-codes:begin -->

- `0`: done and verified.
- `1`: the port is held by another process. Nothing was sent.
- `2`: the port cannot be opened, or it disappeared before anything was written.
- `3`: the servo did not answer (scan: none did).
- `4`: refused before any EEPROM write; calibrate_midpoint says so if it had already switched the torque off.
- `5`: a write (or factory_reset's reset) did not take, and the servo's EEPROM is as it was.
- `6`: the servo's state changed or is unknown. Read the message and run `scan`.
- `7` (scan only): a servo answered oddly, for example two servos on one id, or unreadable registers.
- `64`: bad parameters. The port was not opened.
- `70`: an internal error. Please report it with the message.
- `130`: interrupted. scan stops between ids and prints what it found. set_id stops only before its first write, calibrate_midpoint only before it opens the EEPROM lock, and factory_reset only before it sends the reset (both say so if they had already switched the torque off); a signal after that is held until the change has been made and checked, and the EEPROM lock is closed.

<!-- reference:exit-codes:end -->


## Troubleshooting

Each symptom starts with the text you see. `<...>` marks what the program fills in.

- **`port '<port>' is already held exclusively by another process<holder>; refusing to share the bus.`** Another program holds the port, usually another controller manager or one of this package's tools; the driver's own open failed with `EBUSY`. `<holder>` is ` (pid <N>)` when the driver can see the holder, and ` (no holder visible in /proc; it may belong to another user)` when it cannot. The variant `another process holds the lock on port '<port>'<holder>; refusing to share the bus.` means the holder took only the advisory lock. A tool in the same situation exits with status 1 and names `pid <N> <name>`. Stop the other program. `find /proc/[0-9]*/fd -lname /dev/ttyACM0` lists every process of your own user that has the port open (the number after `/proc/` is its pid). It cannot look inside other users' processes, root's included: `find` prints a `Permission denied` line for each process it cannot read, whether or not that process holds the port, and exits with status 1. Add `2>/dev/null` to hide those lines. A holder that belongs to another user, for example a controller manager started with sudo, shows up only if you run the same command as that user or as root.
- **`not allowed to open port '<port>': <reason>. The port usually belongs to group 'dialout'; ...`** See [Serial port access](#serial-port-access).
- **`port '<port>' does not exist: <reason>. ...`** The adapter is unplugged or has another name: list the candidates with `ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/`.
- **`<n> of <m> servos did not answer: ...`** Check the servo supply, the wiring, the ids (`scan`) and the baud rate ([step 2](#2-find-your-servos)), or use the escape hatch of [Start-up](#start-up-and-servos-that-do-not-answer).
- **`scan` finds nothing** and says `check servo power (USB does not power the servos), the wiring, and the baud rate: ...`: also check that the adapter's jumper cap is on B for USB control ([Requirements and tested hardware](#requirements-and-tested-hardware)).
- **`motor id '<N>' mode changed from <old> to <new>`**: expected once after you change a joint's `type`; it is an EEPROM write ([Joint parameters](#joint-parameters)).
- **`Could not enable FIFO RT scheduling policy ...`** or **`Unable to lock the memory ...`**: see [Real-time scheduling](#real-time-scheduling). Neither stops the driver.
- **`hardware parameter '<name>' is not used by this driver; ignoring it`**, or the joint form `joint '<j>' parameter '<name>' is not used by this driver; ignoring it`: a typo, and the parameter you meant is at its default. Read the `bus configuration:` line ([Hardware parameters](#hardware-parameters)).
- **`a joint does not have a command interfaces`** or **`a joint is using a command interface that isn't position or velocity`**: these two name no joint, so check the command interfaces of every `<joint>` in the block. A joint needs at least one, and only `position` and `velocity` are accepted ([Command interfaces and limits](#command-interfaces-and-limits)).
- **The launch does not end by itself after a FATAL.** It happens in two ways; either way, press Ctrl-C.
  - A FATAL at **configure** (the port missing or busy, servos missing) makes `ros2_control_node` exit, but the rest of the launch keeps running: `robot_state_publisher` (and RViz), and the spawner that got the spawner lock, which waits for the controller manager with no timeout. The other spawner gives up after about two minutes with `Failed to acquire lock after multiple attempts.`
  - A FATAL at **load** (a bad parameter in `on_init`, or the minimal description with `offset` 0) leaves `ros2_control_node` running with no hardware, logging `Waiting for data on 'robot_description' topic to finish initialization` once a second; its controller services never appear, so the spawners behave as after a configure FATAL (worked out from the code, not measured).
- **`read failed for motor id '<N>' (<k> in a row)`** (WARN): a missed read. After `max_read_fails` of them in a row the servo is dropped ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost)).
- **`motor id '<N>' reports status 0x<hh> (<bits>)`** (WARN): a protection bit ([the status bits](#the-status-interface-and-its-bits)). **`motor id '<N>' cleared its fault; re-enabling torque`** (INFO): the bit cleared, the driver switched torque back on by itself, and the joint is moving toward its command again; if the cause is still there, deactivate the hardware ([the status bits](#the-status-interface-and-its-bits)).
- **`Overrun might occur, Total time : ... (Expected < ...)`** (controller manager WARN): the loop missed its period. Without real-time scheduling this happens ([Real-time scheduling](#real-time-scheduling)).
- **`Velocity command timed out. Braking.`** (diff drive, while idle) and the two `pal_statistics` ERRORs at Ctrl-C: expected ([step 5](#5-drive-the-wheels-as-a-differential-base), [Shutdown](#shutdown)).
- **A joint does not move after start-up**: look for the out-of-limit hold, `joint '<j>' starts at <x> rad, outside its limits [<min>, <max>]; ...`, and command it to a position inside its limits ([Command interfaces and limits](#command-interfaces-and-limits)).
- **A joint or the wheels ignore a `ros2 topic pub` command** that printed `publishing #1` and exited with status 0: the first message of a new `ros2 topic pub` process can be dropped before it reaches the controller, so a command sent once (`--once`) can be lost. Send commands the way [step 4](#4-move-a-joint-and-the-wheels) prints them, three copies 0.1 s apart (`-t 3 -r 10`), and send a command that had no effect again. If the second send has no effect either, run `ros2 control list_controllers`. While inactive, `joint_trajectory_position_controller` and `joint_velocity_controller` accept messages on their topics, so `ros2 topic pub` still prints `publishing #1` and exits with status 0, but they ignore the messages and log nothing. `joint_velocity_controller` is inactive after a command of the wrong length ([step 4](#4-move-a-joint-and-the-wheels)) and while `diff_drive_controller` is active ([step 5](#5-drive-the-wheels-as-a-differential-base)). A command sent while a controller was inactive is not applied when the controller is reactivated, so send it again afterwards. (From the ros2_controllers 4.42.1 source; on the reference bench, the second and third copies of a wrong-length wheel command, published after `joint_velocity_controller` had deactivated, left no log line, and the controller was reactivated without applying them.)
- **A joint goes limp mid-run after a power glitch**: its servo came back with torque off ([Known issues and limitations](#known-issues-and-limitations), item 8). Activation switches torque on before it writes a goal (item 1), so the joint can start toward a stale goal (the last goal the driver sent) for about one control period, and then it is driven to its controller's last command at up to `max_speed`, a fast move if the joint sagged while limp ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost)): keep clear of it, then deactivate and activate the hardware.

### Messages quoted in this README

Every message of the driver and the tools that this README quotes, as the program prints it, with `<...>` where it fills in a value and `...` where text is left out.
Messages from ROS itself (the controller manager's FIFO, overrun and initialisation lines, `diff_drive_controller`'s braking line) are not in this list.

<!-- reference:log-lines:begin -->

- `bus configuration: port '<port>', <baud> baud, protocol '<protocol>', io timeout <ms> ms, <n> ping attempt(s), drop a servo after <n> consecutive read failures, allow_missing_servos <bool>, feedback_mode '<mode>', <steps> encoder steps per revolution, <scale> A per current count, <constant> N m/A`
- `hardware parameter '<name>' is not used by this driver; ignoring it`
- `hardware parameter '<name>' is empty; expected <E>`
- `hardware parameter '<name>' is '<value>', which is not <E>`
- `hardware parameter '<name>' is '<value>', which is out of range; expected <E>`
- `hardware parameter '<name>' is '<value>'; the servo library maps only 9600, 19200, 38400, 57600, 115200, 500000 and 1000000, and silently falls back to 115200 for anything else`
- `hardware parameter '<name>' is ... only 'sms_sts' is implemented, the SCS/SCSCL series is not supported yet`
- `hardware parameter '<name>' is '<T>', below the <F> ms a sync read of <K> servos needs; feedback_mode is 'sync_read', which rules out the per-servo path that would survive it, so raise io_timeout_ms to at least <F> or use 'auto'`
- `io_timeout_ms raised from <from> to <F> ms for a sync read of <K> servos`
- `io_timeout_ms <T> is below the <F> ms a sync read of <K> servos needs here; using one feedback read per servo`
- `io_timeout_ms is <T> ms and a failed read pays the <d> ms drain on top of it; ...`
- `feedback for <n> servos travels in one sync read per cycle (INST_SYNC_READ)`
- `sync read went unanswered by motor id(s) <ids>; falling back to one feedback read per servo for this activation`
- `parsed <n> joints: <p> position (mode 0), <v> velocity (mode 1)`
- `joint '<j>' parameter '<name>' is not used by this driver; ignoring it`
- `joint '<j>' has no <param name="id">; every joint needs the bus id of its servo (1..253)`
- `joint '<j>' has an id that is not a whole number: '<v>'`
- `joint '<j>' has id <v>, outside the range 1..253; 254 is the broadcast id the sync writes use and 255 is the packet header byte`
- `joint '<j>' has id <n>, which joint '<k>' already uses; ids must be unique within a <ros2_control> block`
- `joint '<j>' has type '<v>'; it must be 'pos' or 'vel', or left out so the driver infers it from the command interfaces`
- `joint '<j>' has type 'pos' but declares no position command interface; a position joint needs <command_interface name="position"> (a velocity command interface only paces the move)`
- `joint '<j>' has type 'vel' but declares a position command interface; a velocity joint runs its servo in wheel mode and takes only <command_interface name="velocity">`
- `joint '<j>' has an offset that is not a finite number: '<v>'`
- `joint '<j>': position limits [<min>, <max>] rad with offset <o> rad and inverted=<b> map to servo ticks [<lo>, <hi>], outside the servo's single-turn range [0, <steps-1>]; change the offset, the limits, or 'inverted'`
- `joint '<j>' has type 'pos' but no finite position command limits, so its offset cannot be checked against the servo's single-turn range [0, <steps-1>] ticks; add <param name="min"> and <param name="max"> to its position command interface`
- `joint '<j>' has inverted='<v>'; it must be 'true' or 'false'`
- `joint '<j>' has a max_speed that is not a finite number: '<v>'`
- `joint '<j>' has max_speed <v> rad/s; it must be greater than 0`
- `joint '<j>' has max_speed <v> rad/s, which is less than one encoder step per second (<x> rad/s); raise it`
- `joint '<j>' has max_speed <v> rad/s, above the largest the goal speed register can hold (<cap> rad/s); using that instead`
- `joint '<j>' has a max_accel that is not a finite number: '<v>'`
- `joint '<j>' has max_accel <v> rad/s^2; it must be 0 (no acceleration limit) or greater`
- `joint '<j>' has max_accel <v> rad/s^2, which rounds to 0 acceleration-register counts; the smallest step is <x> rad/s^2, and 0 means 'no acceleration limit'`
- `joint '<j>' has max_accel <v> rad/s^2, above the largest the acceleration register can hold (<cap> rad/s^2); using that instead`
- `joint '<j>' has unwrap='<v>'; it must be 'true' or 'false'`
- `joint '<j>' has unwrap=true with type pos; only a vel joint has a multi-turn position`
- `joint '<j>' reports an unwrapped, multi-turn position`
- `a joint does not have a command interfaces`
- `a joint is using a command interface that isn't position or velocity`
- `joint '<j>' position commands clamped to [<min>, <max>] rad`
- `joint '<j>' starts at <x> rad, outside its limits [<min>, <max>]; holding it there until it is commanded to a position inside them`
- `joint '<j>' declares the unsupported state interface '<name>'; supported names are position, velocity, effort, current, voltage, temperature, load, status and the deprecated torque`
- `joint '<j>' declares the state interface '<name>' with data_type '<type>'; only 'double' is supported`
- `joint '<j>' declares the state interface '<name>' more than once`
- `joint '<j>' declares the deprecated state interface 'torque' (kg cm); it keeps working and keeps reporting kg cm, but declare 'effort' instead, which reports N m`
- `unable to ping motor id '<N>'; joint '<j>' will be skipped on the bus`
- `<n> of <m> servos did not answer: <list>; refusing to configure because 'allow_missing_servos' is false`
- `continuing without <n> of <m> servos because 'allow_missing_servos' is true: <list>; their joints mirror their commands into their states until the servos answer`
- `motor id '<N>' mode changed from <old> to <new>`
- `read failed for motor id '<N>' (<k> in a row)`
- `motor id '<N>' stopped answering after <n> attempts; dropping it from the read cycle until the hardware is re-activated`
- `motor id '<N>' answered on activation; adding it back`
- `motor id '<N>' reports status 0x<hh> (<bits>)`
- `motor id '<N>' cleared its fault; re-enabling torque`
- `bus totals: transactions <n>, failed <n> (<x> per million), worst consecutive <n>, dropped <n> [<per-servo counts>]`
- `port '<port>' is already held exclusively by another process<holder>; refusing to share the bus. Two programs on one servo bus produce garbage reads and a joint that drifts.`
- `another process holds the lock on port '<port>'<holder>; refusing to share the bus. Two programs on one servo bus produce garbage reads and a joint that drifts.`
- `not allowed to open port '<port>': <reason>. The port usually belongs to group 'dialout'; 'sudo usermod -a -G dialout $USER' and a new login session fixes that.`
- `port '<port>' does not exist: <reason>. 'ls /dev/ttyACM* /dev/ttyUSB*' lists what is plugged in; the <param name="port"> of the <hardware> block chooses between them.`
- `running as root: the kernel lets a root process open '<port>' even though it is marked exclusive, ... This package's scan, set_id and calibrate_midpoint take it; screen and minicom do not.`
- `port '<port>' is held by another process<holders>; refusing to share the bus -- stop that process first (a running controller manager holds the port for as long as its hardware component is configured). Nothing was sent to the servos.`
- `check servo power (USB does not power the servos), the wiring, and the baud rate: a servo set to another rate answers only at that rate`
- `process(es) <list> had '<port>' open before this tool took it; they hold no lock and can still write to the bus`
- `... the hardware interface accepts ids 1..253; give this servo another id with set_id before putting it in a URDF`
- `servo <id> is still moving (<a> -> <b>); hold it still at the intended midpoint and run again. Its torque is now OFF; no EEPROM byte was written.`

<!-- reference:log-lines:end -->


## Testing

### Without motors

`colcon test` runs the bench test instead of skipping it whenever `WAVESHARE_HIL` is `1` in the environment. The bench test drives the servos and writes their EEPROM on whatever is at `/dev/ttyACM0` (see [On the reference bench](#on-the-reference-bench)), so make sure `WAVESHARE_HIL` is not set in your shell before running this, from the workspace root:

```bash
colcon build --packages-select waveshare_servos
colcon test --packages-select waveshare_servos
colcon test-result --all --verbose
```

- `colcon test` exits 0 even when a test fails (unless it is given `--return-code-on-test-failure`); `colcon test-result --all --verbose` is the verdict, and it exits non-zero on any failure.
- The suite needs no motors, but `test_tools_cli` takes `/dev/ttyACM0` for its whole run whenever that path exists (it never sends a byte), so stop anything that uses the port first; otherwise the test waits 30 s and fails. `test_hil_eeprom` opens the port for one case.
- It holds gtest/gmock cases in which the driver and the tools talk to a simulated servo bus on a pseudo-terminal, xacro render tests, a launch test of the example on mock hardware, the self-test of the bench test's gates, the release-document tests below, and ament lint (with the vendored library excluded).
- Two of the tests cover the shipped example without touching a servo: `test_urdf_xacro` renders `description/urdf/example.urdf.xacro` with and without `use_mock_hardware` and reads the XML back, and `test_example_launch` brings `example.launch.py` up on `mock_components/GenericSystem` and asserts the three active controllers, the deliberately inactive `diff_drive_controller`, and `/joint_states`.
- Three tests keep the release documents honest. `test_readme` checks that this README's relative links and anchors resolve (web links are not checked), and checks against the code the names in every reference table, the hardware-parameter and launch-argument defaults, the status-bit names, the exit codes, the tools' parameters and id ranges, and the lines under [Messages quoted in this README](#messages-quoted-in-this-readme); other values (joint defaults, legal-value ranges, units and scales, messages quoted elsewhere in the text) are not checked by a test. `test_release_metadata` checks `package.xml`, `CHANGELOG.rst` and the license files, and that no release document points at a file outside the repository. `test_vendored_files` checks the vendored library against the checksums in [THIRD_PARTY.md](THIRD_PARTY.md), and that THIRD_PARTY.md reproduces the upstream MIT notices.
- On Ubuntu 24.04, `ament_cppcheck` skips every file, because it flags cppcheck 2.13 as slow, unless the environment variable `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` is set.
- The bench test is registered too and skips itself while `WAVESHARE_HIL` is not `1` (`SKIP: WAVESHARE_HIL is not 1`).

### On the reference bench

> **Warning: the bench test is for the reference bench only**, exactly four ST3025 servos at ids 1-4. Never run it on an assembled robot.
>
> - It drives ids 1 and 2 as position servos within ±π/2 of the bench description's joint zero (servo tick 1024), and spins ids 3 and 4 as wheels. A position servo at id 3 or 4 is switched to wheel mode first, which is an EEPROM write.
> - It deliberately writes EEPROM on ids 2 and 4 (H15: `set_id` 4 -> 253 -> 4; H16: `calibrate_midpoint` on id 2), under a journal at `~/.local/state/waveshare_servos/hil_eeprom_journal.snap`, and restores them.
> - It does not check the bus's identity before its first scenarios ([Known issues and limitations](#known-issues-and-limitations), item 13).

```bash
WAVESHARE_HIL=1 colcon test --packages-select waveshare_servos --ctest-args -R hil_check
colcon test-result --all --verbose
```

- Set `WAVESHARE_HIL=1` on the command line, as above. Never `export` it or put it in a shell profile: every later `colcon test` in that shell, including the motorless one, would then run the bench test.
- It skips, with the reason printed, unless `WAVESHARE_HIL=1`, the port exists and is readable and writable, the user is not root, and `ros2` is on `PATH`.
- Nothing else may hold the port while it runs. It refuses to run as root (the kernel lets a root process ignore `TIOCEXCL`, so the exclusivity check would silently lie), and it stops the wheels at the end of every scenario and again at exit.
- Environment variables:
  - `WAVESHARE_HIL_SCENARIOS`: a space-separated subset of the scenarios, when you want one row back rather than the whole run (default: all twenty, in the order below);
  - `WAVESHARE_HIL_PORT`: the port (default `/dev/ttyACM0`);
  - `WAVESHARE_HIL_OUT`: where the report and the recordings go (default `hil_check.d` in the test's working directory, `build/waveshare_servos/`);
  - `WAVESHARE_HIL_SOAK_S`: the soak length of H11 in seconds (default 600, at least 40). The whole run must end inside ctest's 40-minute (2400 s) time limit for this test, and the harness's time budget allows a soak of up to 750 s (a full run with the default soak took 1883 s on the reference bench). With a much longer soak, that limit kills the run in the middle of H11 while both wheels are commanded at 1 rad/s. It kills every process the run started, so none of the clean-up runs and the wheels keep turning until you switch off the servo supply. For a soak above 750 s, raise the test's `TIMEOUT` in `CMakeLists.txt` by the same amount;
  - `WAVESHARE_HIL_EEPROM_BASELINE`: an EEPROM snapshot of the bench for `H17.matches_baseline` to compare with; without one, that row is a SKIP.
- `--ctest-args -L hil` runs only this test and `-LE hil` excludes it.
- One run writes a stable-keyed report, `build/waveshare_servos/hil_check.d/hil_check.txt` and `hil_check.json`, plus every recording it took. The script is `test/hil_check.sh`; the recorder and the checkers are in `test/hil/`.
- A full run takes about 31 minutes. The scenarios run in the order below; the report lists its rows in its own order.

| run order | scenario | what it does |
|---|---|---|
| 1 | H1 | the shipped example launch, idle: the controllers come up, every servo answers, the bus cost and loop rate are measured |
| 2 | H1B | the shipped example, commanded: `joint1` to 0 and 0.6 rad, both wheels at 2 rad/s, then Ctrl-C, with the wheels checked stopped and the port free |
| 3 | H2 | position moves of `joint1` (0 and 0.6 rad) on the bench test's own description, with the goal registers read back |
| 4 | H3 | both wheels at 2.0 rad/s for 6 s, then a stop |
| 5 | H4 | the wheels at +2 and -2 rad/s for 12 s: the unwrapped position against the velocity over several revolutions |
| 6 | H5A | a declared servo that does not exist (id 9) with `allow_missing_servos` `false`: the configure is refused |
| 7 | H5B | the same with `allow_missing_servos` `true`: the stack comes up and the wheels turn |
| 8 | H5C | `allow_missing_servos` `false` with every servo present: the stack comes up and `joint1` moves |
| 9 | H6 | the exclusive port lock, both ways: a second program is refused while the driver holds the port, and the driver refuses a port another program holds |
| 10 | H7 | `inverted` on `joint2` and `joint4`: the joint-frame readings, the mirrored servo registers, the `load` sign and the unsigned `effort` |
| 11 | H8 | per-joint `max_speed` and `max_accel`: the goal-speed and acceleration registers and the speeds they give |
| 12 | H9 | every state interface on `/dynamic_joint_states`, a phantom joint, and a deactivate/activate cycle of the hardware component |
| 13 | H10 | shutdown on SIGINT and on SIGTERM: exit 0, deactivate before shutdown, the wheels stopped, the port free |
| 14 | H12 | `enforce_command_limits: true` with URDF limits tighter than the driver's: the controller manager's limiter clamps the commands |
| 15 | H13 | `scan`, read-only, cross-checked against a separate read of the same registers |
| 16 | H14 | `scan`, `set_id` and `calibrate_midpoint` refuse a port held by the controller manager or by a flock-only holder; `set_id` and `calibrate_midpoint` refuse bad arguments, a taken id and an id no servo answers; the EEPROM is unchanged (`factory_reset` has no scenario: [Known issues and limitations](#known-issues-and-limitations), item 12) |
| 17 | H15 | **EEPROM writer:** `set_id` 4 -> 253 -> 4, journaled |
| 18 | H16 | **EEPROM writer:** `calibrate_midpoint` on id 2, and its refusal of the wheel id 3, journaled |
| 19 | H11 | **the ten-minute soak:** the full cycle at 100 Hz, with the failed-transaction rate from the `bus totals:` line |
| 20 | H17 | the EEPROM as the run found it, and as `WAVESHARE_HIL_EEPROM_BASELINE` recorded it |

The gates themselves can be checked with no hardware at all: from the package directory, `python3 test/hil/hil_gates.py --self-test` runs them over synthetic data with injected defects. How the bench test decides a row is in the [appendix](#how-the-bench-test-decides).

**An interrupted run.** A run killed during H15 or H16 (a SIGKILL, or ctest's time limit, which skips the script's own cleanup) can leave servo 4 at id 253 or servo 2 with a changed offset, with the journal in place.
Run the bench test again on the same port: its pre-flight restores, from the journal, only the registers the scenarios write (id, offset, mode, torque and lock, registers 5, 31, 32, 33, 40 and 55), removes the journal and continues.
It stops instead, and names the journal, when the journal was taken on another port, when the port is held, or when the bench differs from the journal elsewhere.
In the [devcontainer](#devcontainer) the journal, like the scenario's own snapshot under `build/`, lives inside the container, and a rebuild deletes both. Do not rebuild the container until the journal has been restored and removed; otherwise servo 2's former offset is lost.
Then inspect and restore by hand, from the workspace root (the helper is a build-tree binary, not installed).
Run these lines one at a time, not as one pasted block. The first line prints the port the journal was taken on. The journal applies only to the bench it came from, because the restore cannot tell two buses of the same servo model apart. If that port is not `/dev/ttyACM0`, use it in the `scan` line and in the restore line; if that adapter was re-plugged and came back under another name, use that name, but only once `scan` shows that bench's servos there. Run the restore only when `scan` shows the bench: ids 1, 2, 3 and 4, or 253 in place of 4. If you are not sure it is the same bench, stop. The restore writes the servos' EEPROM (id, offset and mode), then puts torque back as the journal recorded it (on, on the reference bench), holding each servo where it is. Run the last line only when the restore exited with status 0; on success it prints either `restored; a fresh snapshot matches the source` or `the bench already matches the source; nothing written`. That line deletes the journal, the only copy of the bench's state before the interrupted scenario that this procedure keeps. If the restore refuses, keep the journal. The scenario's own copy is `build/waveshare_servos/hil_check.d/H15/pre_eeprom.snap` or `H16/pre_eeprom.snap`, until the next run reaches that scenario.

```bash
J=~/.local/state/waveshare_servos/hil_eeprom_journal.snap
cat "$J.port"                                   # the port the journal was taken on
ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0
build/waveshare_servos/hil_eeprom --port /dev/ttyACM0 restore --from "$J" --allow-regs 5,31,32,33,40,55
rm "$J" "$J.port"                               # only after the restore printed "restored"
```

The restore moves back at most one stray id, and only one of the missing servo's model; it refuses anything ambiguous.
This paragraph is from the bench test's code: no interrupted run was provoked to check it.


## Known issues and limitations

1. **Torque comes on before the first goal.** Activation enables torque before the first `write()` sends a goal. After `calibrate_midpoint`, or on a cold start with the goal register at 0, a joint can start toward a stale goal for about one control period. The same holds for the deactivate/activate that recovers a servo that lost power (item 8); there the stale goal is normally the last goal the driver sent, not 0, and the joint's controller then drives it to its last command ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost)). Whether the firmware latches the goal to the present position when torque comes on is unmeasured.
2. **`read()` and `write()` always return OK**, so a dead bus never reaches `on_error`. Silent servos are dropped after `max_read_fails` cycles, and the component stays active.
3. **`io_timeout_ms` is sized for 1 Mbaud.** Its default and floor do not scale with `baudrate` (worked out from wire time, not measured; [Hardware parameters](#hardware-parameters)).
4. **With `port` set to a `/dev/serial/by-id` link, the driver's busy-port error cannot name the holder** (from the code, not run). The tools can.
5. **The FATAL for a `pos` joint without a position command** says a velocity command interface "only paces the move"; it does not ([Command interfaces and limits](#command-interfaces-and-limits)).
6. **Scales and bits are approximate or unverified.** `effort` and `current` are unsigned and approximate, the voltage scale is unverified, status bits 4, 6 and 7 are unverified, and the SDK and a memory table disagree on bits 1 and 4 ([State interfaces](#state-interfaces)).
7. **The mode write is read back, but the EEPROM unlock and lock results are not checked.**
8. **A servo that loses power mid-run** returns with torque off (the measured power-up state), and answering again does not re-enable torque (only activation and a clearing status fault do), so unless it reported a fault just before or after the outage it stays limp until the hardware is deactivated and activated. That activation switches torque on before it writes a goal (item 1), so a position joint can start toward a stale goal (the last goal the driver sent) for about one control period, and then runs to its controller's last command at up to `max_speed`: keep clear of the joint before you activate ([Recovery after a servo is lost](#recovery-after-a-servo-is-lost)). The wheel acceleration register (SRAM) is rewritten at four edges, but whether they catch every brown-out is untested. Inferred from the measured power-up state and the code; a mid-run brown-out was not measured.
9. **Above four servos the sizing is a model**, and above 30 position servos the goals split across two packets ([Sizing the bus](#sizing-the-bus)).
10. **`is_async` tearing is unproven**, one way or the other: one 120 s recording found nothing torn.
11. **JointTrajectoryController 4.42.1 crashes on velocity-only joints** (upstream). Use a velocity controller for wheels, as the example does; the example's position-and-velocity trajectory controller was run on the reference bench and does not crash.
12. **`factory_reset` was verified on one servo** and has no bench-test scenario. Its refusal of an id that no servo answers was run on a real bus (exit status 3, nothing written).
13. **The bench test assumes the reference bench** and has no identity check before its first scenarios ([Testing](#on-the-reference-bench)).
14. **A lost USB adapter is not detected as a failure** (every servo is dropped, the component stays active), and deactivate/activate does not reopen the port. [Recovery after a servo is lost](#recovery-after-a-servo-is-lost) gives the two ways out. The second one's port release and re-activation were measured with the adapter left plugged in; the unplug itself was not measured.
15. **The WARN printed when the driver runs as root** names `scan`, `set_id` and `calibrate_midpoint` as the programs that take the advisory lock; `factory_reset` takes it too.


## Versioning and changes

The version is in `package.xml`, and [CHANGELOG.rst](CHANGELOG.rst) lists the changes of every version. `ros2 pkg xml waveshare_servos -t version` prints the installed version.

**What the version number covers.** Versions follow semantic versioning for the package's user-facing interface: the plugin name `waveshare_servos/WaveshareServos`; the hardware and joint parameters (names, types, defaults, ranges); the command and state interface names and units; the tools' names, parameters and exit codes; and the example launch file's arguments. A change that breaks one of these bumps the major version.

**Not covered:** the installed C++ headers and the include layout (they put generic names such as `units.hpp`, `servo_bus.hpp`, `SCS.h` and `INST.h` on a consumer's include path), `ServoBus` and linking the plugin library directly, the vendored library, log text (including the format of the `bus totals:` line), the example's values, and the test harness.
A downstream package can include `waveshare_servos.hpp` and link the plugin library, and such a package builds, but that use is not covered by the version number.

### Upgrading from 0.1.0 (Humble)

- **Jazzy only.** The Jazzy versions of this package do not build on Humble; 0.1.0, on the `humble` branch, is the Humble version.
- The example's wheel controller `joint_trajectory_velocity_controller` (a JointTrajectoryController) is replaced by `joint_velocity_controller` (`velocity_controllers/JointGroupVelocityController`), which takes `std_msgs/msg/Float64MultiArray` on `/joint_velocity_controller/commands` ([step 4](#4-move-a-joint-and-the-wheels)).
- The tools' `device_port` and `baud_rate` are `port` and `baudrate`; the old names are refused with exit code 64, not ignored.
- `set_id` requires `start_id` and `new_id`, and `calibrate_midpoint` requires `id`; 0.1.0 defaulted them to 1.
- `calibrate_midpoint` refuses a wheel (a servo not in mode 0) instead of rewriting its mode, and leaves the torque off.
- The `torque` state interface (kg cm) is deprecated in favour of `effort` (N m).
- State interfaces are free-form; 0.1.0 required exactly `position`, `velocity`, `torque` and `temperature`, in that order.
- `allow_missing_servos` defaults to `false`: a servo that does not answer fails the configure, and with the controller manager's defaults `ros2_control_node` exits. 0.1.0 logged a warning and carried on.
- The old example's `example_param_hw_*` parameters are ignored with a WARN.
- The port and the baud rate are hardware parameters; 0.1.0 hard-coded them.
- `scan` and `factory_reset` are new.
- The example's `prefix` xacro argument and macro parameter are gone: a macro call with `prefix=` fails, and `prefix:=` on the command line is ignored.
- `on_init` refuses descriptions that 0.1.0 accepted, including a single-turn joint whose limits leave the encoder range: a 0.1.0-style description with `offset` 0 and ±1.57 rad limits is refused ([A minimal description](#a-minimal-description)).
- The port is opened exclusively: the driver fails to configure while another process holds the port exclusively or holds its lock, and while the driver has the port, other programs cannot open it (unless they run as root). A serial monitor that already had the port open is not stopped; the driver configures and names it in a warning.
- Torque is enabled at activation.
- The driver clamps commands: positions to the position command interface's `min`/`max`, wheel speeds to `max_speed`.
- The tools exit non-zero on failure.

This is a summary; the complete list is CHANGELOG.rst, 1.0.0, 'Breaking changes (upgrading from 0.1.0)' and 'Behaviour changes with an unchanged description'.
Users of a pre-release `jazzy` snapshot: see CHANGELOG.rst for the change to `io_timeout_ms`.


## License and third-party code

Licensed under GPL-3.0-or-later; see [LICENSE](LICENSE).

The servo packet layer is the SCServo library by Feetech, vendored in the copy Waveshare distributes for its Bus Servo Adapter, with two additions taken from [adityakamath/SCServo_Linux](https://github.com/adityakamath/SCServo_Linux) and one local fix.
[THIRD_PARTY.md](THIRD_PARTY.md) records where each vendored file comes from, what was changed, their checksums, their licensing, and how to refresh them.
`include/visibility_controls.h` and `bringup/launch/example.launch.py` keep their upstream Apache-2.0 headers; the license text is [LICENSES/Apache-2.0.txt](LICENSES/Apache-2.0.txt).


## Contributing

Issues are welcome at <https://github.com/htchr/waveshare_servos/issues>, and pull requests are welcome on the same repository.
A status bit confirmed on hardware is especially welcome: open an issue with what you did and what you saw.
Changes are developed and released as [Developing and releasing](#developing-and-releasing) describes.


## Appendix: engineering notes

How the numbers above were measured, how the bench test decides, and how the package is developed and released.

### How the cost table was measured

Each column of [What a cycle costs](#what-a-cycle-costs) is the mean of three runs per arm of the same three bench scenarios, the two arms alternating run by run in a single session: the left column is one `FeedBack()` round trip per joint plus an acknowledged per-wheel acceleration write, the right column is one `INST_SYNC_READ` covering all four servos plus two unacknowledged broadcast writes.
The run-to-run spread inside each arm was 0.008 ms or less, so the difference is not noise.

The write saving is larger than it looks because the per-servo path was not spending its time on the wire: the vendored `SyncWriteSpe` writes each wheel's acceleration register with a call that blocks waiting for that servo's status packet, and two of those were most of the 1.447 ms.

The same numbers hold over a long window: three ten-minute soaks at 100 Hz, about 60 000 cycles each, averaged 1.647 ms read and 0.013 ms write, one part in 1200 away from the nine-second measurement.

The published values did not change. Both read paths funnel through the same decoder, and a side-by-side capture found `position`, `velocity`, `load` and `current` bit-identical in 800 of 800 samples with the servos at rest.
`voltage` and `temperature` are the exception, and they are **not** bit-identical between two reads, of either path. They dither by up to one ADC count, at the same rate between two reads of the *same* path as between the two paths, so it is the servo's converter and not the transport.
The bench test does not compare the two transports at all. The pty tests in `test/test_lifecycle_over_pty.cpp` do: they compare all nine interfaces, `voltage` and `temperature` included, and require exact equality. That is correct there because the fake bus is a fixed register file that does not dither.

### `rw_rate`, `is_async` and the bench test

- `is_async="true"`: a 120 s recording of a wheel at 2 rad/s (12 251 samples) contained no repeated sample and no doubled position step, so nothing torn was observed; but that is a negative result over one window, not a proof that the async wrapper copies the state arrays under a lock. An async component stops the wheels correctly on `SIGINT`, tested at 2 rad/s against a synchronous control run; `rw_rate` was not separately SIGINT-tested with a wheel turning, since it does not touch the shutdown path.
- The bench test's gates assume a 100 Hz component. At `rw_rate="50"` with a wheel near 2 rad/s, its rate-scaled step gate (`g1b.<joint>`: each position step bounded by the reported velocity times the publish interval) charges a 20 ms position step against a 10 ms allowance and starts failing (the slope gate `g2` is unaffected); with `is_async="true"` set as well it fails on nearly every step, because the timestamp jitter that was providing the margin disappears. Once the controller manager decimates the loop by three or more (with `update_rate: 100`, an `rw_rate` below about 40; it picks the nearest whole number of cycles), each value repeats at least three times on a turning wheel, and the stale-run rule, which excludes runs of three identical samples while the joint moves, starts excluding most of the recording instead (worked out from that rule and the controller manager's rate selection, not measured).

### How the bench test decides

A row is `PASS`, `FAIL`, `SKIP`, `ABORTED` or, for exactly two frozen keys the bench cannot supply a stimulus for, `INCONCLUSIVE`; a `NOTE` row records a fact and is never a gate.
**There is no mechanism anywhere in it that turns a `FAIL` into anything else.**

Three invariants carry the run: the reported position never steps more than half a revolution; its slope over a half-second window tracks the reported `velocity`; and over a multi-revolution run the travel matches the integral of the velocity.
`python3 test/hil/hil_gates.py --self-test` checks those gates against synthetic data with an injected 2π jump, a frozen run, a doubled velocity and a missed wrap, and needs no hardware at all.

### What the bench test does not do, and why

**No step of it asks a person to be at the adapter**: no connector pull, no wheel held by hand, no ammeter.
That is a deliberate scope decision, not an oversight: no automated test pulls a connector mid-run.
What the bench test does instead is gate the recoverable half of a lost servo (the hardware component set `inactive` and then `active`, which `H9` cycles and gates), and the pty tests that run in every `colcon test` cover the unplugging half deterministically:

- a servo that stops answering mid-run, the drop after `max_read_fails`, the re-ping on activate and the post-gap warning: `test/test_lifecycle_over_pty.cpp`, a fake servo told to stop answering between two `read()` calls;
- the status byte: synthetic status values fed through the same fake bus, over all 256 of them.

That proves the **decode**. It does not prove what any bit means on the ST3025's firmware, and neither would holding a wheel.
So **no status bit is named in this package without a primary source**: bits 0, 1, 2, 3 and 5 are named after FEETECH's own SDK, and bits 4, 6 and 7 are published as `bitN (meaning unverified)`.
The current scale (`current_per_count_a`) cannot be validated without an ammeter either, so `effort` stays documented as approximate.

### Developing and releasing

The development rules:

- Work from the workspace root with the commands of [Without motors](#without-motors), adding `--symlink-install` to the build while you develop.
- Write the failing test first, and watch it fail for the expected reason before writing the code.
- Tests live in `test/` and are registered inside `if(BUILD_TESTING)` in `CMakeLists.txt`.
- Every test passes with no motors attached; anything that needs the hardware is gated behind `WAVESHARE_HIL=1` and skips otherwise, so never leave `WAVESHARE_HIL` set in a shell you develop in.
- Drive `ServoBus` through a pseudo-terminal instead of mocking the vendored library, so the vendored packet code runs unchanged; never change the vendored files to make a test pass ([THIRD_PARTY.md](THIRD_PARTY.md)).
- Lint is part of the suite: a lint failure is a failing test.
- Declare every test dependency in `package.xml`, as a `<test_depend>` unless it is already an `<exec_depend>` (`rosdep install` installs both types). An undeclared one builds on a developer machine and fails on a clean checkout. `controller_manager` stays an `<exec_depend>` only, although `test/test_example_launch.py` imports it: `ament_lint_auto_find_test_dependencies()` loads the CMake config of every `<test_depend>`, and controller_manager's pulls in `tl_expected`, whose deprecation warning would make the build's stderr non-empty (release check 1).
- `colcon test` exits 0 even when a test fails (unless it is given `--return-code-on-test-failure`); `colcon test-result --all --verbose` is the verdict.
- There is no CI, by decision, so the checks below are run by hand.

**Before a release**, in this order:

1. A build whose `log/latest_build/waveshare_servos/stderr.log` is empty (0 bytes).
2. The full motorless suite, as in [Without motors](#without-motors), with nothing on the port and `WAVESHARE_HIL` not set.
3. rosdep in a clean environment, from the workspace root:

```bash
env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep keys --from-paths src --ignore-src
env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep check --from-paths src --ignore-src --rosdistro jazzy
env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep install --from-paths src --ignore-src --rosdistro jazzy --simulate --reinstall -y
```

4. A copy install in a fresh workspace: a plain `colcon build`, its motorless tests, and one launch of the example on mock hardware. Run it from the package directory of a clean clone, not of a working tree with build products; `test_tools_cli` holds the port, so nothing else may use it.

```bash
mkdir -p /tmp/release_ws/src && cp -r . /tmp/release_ws/src/waveshare_servos
cd /tmp/release_ws && env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c '
  source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash &&
  colcon test && colcon test-result --all --verbose &&
  timeout -s INT 20 ros2 launch waveshare_servos example.launch.py use_mock_hardware:=true gui:=false'
```

5. The clean-machine build in Docker, the only check that finds an undeclared dependency (every other check runs on a machine that already has everything installed). Run it from the package directory of a clean clone of the release commit. It passes when `docker run` exits 0 and `colcon test-result` reports a total greater than zero with 0 errors and 0 failures. Read the output as well: `set -e` does not stop the recipe when a command followed by `&&` fails, so a failed `colcon build` does not end it by itself, and the test result is then empty or incomplete:

```bash
docker run --rm -v "$PWD":/ws/src/waveshare_servos:ro ros:jazzy-ros-core bash -c '
  set -e; apt-get update && apt-get install -y python3-rosdep python3-colcon-common-extensions build-essential
  rosdep init && rosdep update
  cd /ws && . /opt/ros/jazzy/setup.sh
  rosdep install --from-paths src --ignore-src --rosdistro jazzy -y          # no -r
  colcon build && . install/setup.sh
  colcon test --ctest-args -E "^(test_tools_cli|test_hil_eeprom|hil_check)$"
  colcon test-result --all --verbose'
```

6. The full bench test on the reference bench ([On the reference bench](#on-the-reference-bench)).
7. The version bumped in `package.xml`, with a new top section in `CHANGELOG.rst`; `test_release_metadata` checks that the two agree.
8. The vendored checksums: the `sha256sum -c` hand check in [THIRD_PARTY.md](THIRD_PARTY.md).

### Maintenance notes

Comments in `src/`, `include/` and `test/` cite section numbers and finding ids of the development plan and of its per-stage specifications and review notes.
Those documents are not in the repository; each comment carries its own reasoning, so the citation can be read as a label.

Code follow-ups known and documented rather than fixed, none of them scheduled:

- torque is enabled in `on_activate` before the first goal write (not scheduled);
- `read()` and `write()` never return `ERROR`, so a dead bus never reaches `on_error` (not scheduled);
- the `io_timeout_ms` default and floor are not baud-aware (not scheduled);
- the two FATALs `a joint does not have a command interfaces` and `a joint is using a command interface that isn't position or velocity` name neither the joint nor the interface (not scheduled);
- the driver's busy-port holder lookup does not resolve a `/dev/serial/by-id` path (not scheduled);
- the FATAL text "a velocity command interface only paces the move" (not scheduled);
- the WARN printed when running as root omits `factory_reset` from the programs that take the advisory lock (not scheduled);
- `set_mode` does not check the EEPROM unlock and lock results, and its comment says a write with the lock closed is "silently dropped", which contradicts the memory-table note that such a write is applied and lost at power-off (not scheduled);
- `test/hil_check.sh` has no bench-identity check before its first scenarios (not scheduled);
- `test/hil_check.sh` sends each wheel command as one `ros2 topic pub --once`, which can be lost with exit status 0 ([step 4](#4-move-a-joint-and-the-wheels)): a lost command in H8, or a lost start of the H11 soak, fails a row (`H8.speed_register.<stack>.id3`, `H11.wheels_turning`), but no row needs the wheel commands of H9 and H10 or the soak's stop to have taken effect, so a lost one there fails nothing, and the scenario runs without the wheel motion it was meant to cause, or with the wheels still turning after a lost stop (worked out from the bench test's code; not scheduled);
- torque is not re-enabled when a servo answers again after missed reads, so a servo that power-cycles inside the drop budget stays limp (not scheduled);
- a USB adapter that disconnects leaves the component active with every servo dropped, and deactivate/activate keeps the dead descriptor (not scheduled);
- a `vel` joint with `unwrap` false that is dropped after `max_read_fails` reports the position it read at activation instead of its last reading, because the drop keeps the last reading only for a joint that unwraps (not scheduled).
