# ros2_control Waveshare servo hardware interface

The `ros2_control` implementation for Waveshare ST series servo motors.

Specifically designed for [Waveshare ST3025 servo motors](https://www.waveshare.com/product/st3025-servo.htm) and their [Bus Servo Adapter](https://www.waveshare.com/product/bus-servo-adapter-a.htm), but should work with all of their ST series motors and controllers.


## Set Up

This hardware interface is developed for ros2 Jazzy.  
Previous work for ros2 Humble is saved as a checkpoint on the ["humble" branch](https://github.com/htchr/waveshare_servos/tree/humble).

Testing has been done with the bus servo adapter connected via USB to a Jetson Orin Nano or x86 Ubuntu desktop.  
The Jetson ran ros2 Jazzy and Isaac ros inside a Docker container.  
The desktop ran ros2 Jazzy using the devcontainer in this repo.  

It should work with any system using [ros2_control](https://github.com/ros-controls/ros2_control).

### Included Devcontainer

1. Install the "Remote Development" extension pack for VS Code
2. Open the directory in VS Code
3. Select "Reopen in container"

### Direct Install

1. Clone the package into your `src` directory:
    ```bash
    git clone https://github.com/htchr/waveshare_servos.git
    ```
2. Install the dependencies
    ```bash
    rosdep install --from-paths ./src --ignore-src -r -y
    ```
3. Build your workspace.
4. Source your workspace.


## Usage

If you use USB, make sure your user is in the `dialout` group and the port has the correct permissions

```bash
sudo chmod 666 /dev/ttyACM0
sudo usermod -a -G dialout $USER
```

Reference the `example.launch.py`, `example_controllers.yaml`, and `example.ros2_control.xacro` files to reference how to use this hardware interface in another robot system.
The packaged example declares four joints, matching the four servos of the reference bench: `joint1` and `joint2` are position servos (ids 1 and 2), `joint3` and `joint4` are wheels (ids 3 and 4).
Delete the joints you do not have; the driver only talks to the ids its description declares.

To verify your installation works, launch the example launch file:

```bash
ros2 launch waveshare_servos example.launch.py
```

Move a position-controlled servo with:

```bash
ros2 topic pub --once /joint_trajectory_position_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, joint_names: ['<joint_name>'], points: [{positions: [<position>], time_from_start: {sec: 1, nanosec: 0}}]}"
```

Move the velocity-controlled servos with:

```bash
ros2 topic pub --once /joint_velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [<velocities>]}"
```

The message carries no joint names.
`data` holds one velocity (rad/s) per joint, in the order of the `joints` list of `joint_velocity_controller` in `example_controllers.yaml` -- not servo id order or URDF order.
For example, with

```yaml
joint_velocity_controller:
  ros__parameters:
    joints:
      - joint4
      - joint3
```

`{data: [1.0, -0.5]}` turns `joint4` at 1.0 rad/s and `joint3` at -0.5 rad/s.
Check the order on a running system with `ros2 param get /joint_velocity_controller joints`.

Each servo keeps turning at its commanded velocity until a new command arrives; send a zero for every joint (e.g. `{data: [0.0, 0.0]}`) to stop them.
A command whose length does not match the `joints` list stops all the velocity-controlled servos and deactivates the controller; reactivate it with `ros2 control switch_controllers --activate joint_velocity_controller`.

## State interfaces

A joint may declare any subset of the state interfaces below, in any order, and a joint that declares none is legal too.

The driver exports what it was asked for, in **description order**: joint by joint as the `<ros2_control>` block lists them, and within each joint in the order its `<state_interface>` elements appear.
That is the order `ros2 control list_hardware_components -v` prints.

It is **not** a promise about `/joint_states` or `/dynamic_joint_states`.
The ordering on those topics is `joint_state_broadcaster`'s, produced downstream of the driver's export, and it need not match the description -- on the reference bench it reliably does not.
Every declared interface is published; only its position in the list is the broadcaster's business.
**Read interfaces by name, never by index.**

| name | unit | notes |
| --- | --- | --- |
| `position` | rad | joint frame; unbounded once unwrapped |
| `velocity` | rad/s | joint frame |
| `effort` | N m | a magnitude, see below |
| `current` | A | a magnitude, see below |
| `voltage` | V | |
| `temperature` | deg C | |
| `load` | fraction of full PWM | about -1.023 .. +1.023, not clamped; signed |
| `status` | bitmask | see below |
| `torque` | kg cm | **deprecated** alias of `effort`, kept bit-for-bit at its old value |

`torque` still works and still reports kg cm; declare `effort` instead, which reports N m. A joint that declares `torque` logs one deprecation warning at load time.

### What `inverted` flips

`inverted="true"` flips **three** state interfaces -- `position`, `velocity` and `load` -- and both command interfaces. It does **not** flip `effort`, `current` or `torque`.

The servo's protocol allows a signed current (`ReadCurrent` decodes a sign bit exactly as `ReadSpeed` does), but this firmware never sets it: a recording with the wheels driven in both directions contains no negative current sample at all. `current` is therefore a **magnitude**, and so are the `effort` and `torque` derived from it. Negating a magnitude would make every mirrored joint read uniformly negative, which tells you less than leaving it unsigned.

**So, on an inverted joint, `effort` and `current` are unsigned magnitudes and do not agree in sign with `velocity`. If you need to know which way a joint is working, read `load`: it is the direction-bearing signal** -- its sign is real (the servo reports it in bit 10 of the load word) and the driver flips it with the joint.

`effort == current * torque_constant_nm_per_a` holds for every joint, inverted or not.

### The `status` interface and its bits

`status` is the servo's status byte, published as a plain bitmask (`0.0` .. `255.0`), or **NaN** while the driver has no reply from that servo -- never pinged, or dropped after `max_read_fails`. `0.0` means "the servo answered and reported no fault". Guard it with `if (std::isfinite(v)) { bits = static_cast<uint8_t>(v); }`.

| bit | mask | name the driver prints | source |
| --- | --- | --- | --- |
| 0 | 0x01 | `voltage` | FEETECH SDK `ERRBIT_VOLTAGE` |
| 1 | 0x02 | `angle` | FEETECH SDK `ERRBIT_ANGLE` |
| 2 | 0x04 | `overheat` | FEETECH SDK `ERRBIT_OVERHEAT` |
| 3 | 0x08 | `overcurrent` | FEETECH SDK `ERRBIT_OVERELE` |
| 4 | 0x10 | `bit4 (meaning unverified)` | none |
| 5 | 0x20 | `overload` | FEETECH SDK `ERRBIT_OVERLOAD` |
| 6 | 0x40 | `bit6 (meaning unverified)` | none |
| 7 | 0x80 | `bit7 (meaning unverified)` | none |

The names come from FEETECH's own Python SDK ([FTServo_Python](https://github.com/ftservo/FTServo_Python), `scservo_sdk/protocol_packet_handler.py`, retrieved 2026-09-16), which decodes the same packet byte this driver captures. **No vendor memory table could be read** (the Waveshare wiki returns HTTP 403 and both vendor PDFs are scanned images), and **nothing here establishes which of these bits this servo's firmware actually raises.** That is why the raw byte is published, why three bits are printed as unverified, and why this table says only what the driver prints. If you confirm a bit on real hardware, please open an issue with what you did and what you saw.


## Missing servos, and how to make the node start anyway

`allow_missing_servos` defaults to **`false`**.
If a servo does not answer its ping, `on_configure` logs one FATAL naming every missing id and joint, releases the port and returns `FAILURE`.

At controller-manager startup the component's target state is `active`, so a refused configure makes `set_initial_hardware_components_state` throw, and **`ros2_control_node` does not come up** -- `hardware_components_initial_state.shutdown_on_initial_state_failure` defaults to `true` (`controller_manager_parameters.yaml:45-50`; the generated header on this machine, `/opt/ros/jazzy/include/controller_manager/controller_manager_parameters.hpp:79`, shows the same default).

If you would rather the node came up with the component left unconfigured, set that **controller-manager** parameter -- it is not a driver parameter:

```yaml
controller_manager:
  ros__parameters:
    hardware_components_initial_state:
      shutdown_on_initial_state_failure: false
```

With it, the refusal is logged as an ERROR, the node runs, the controllers stay unclaimed, and you can bring the component up by hand once the bus is fixed: `ros2 control set_hardware_component_state <name> inactive`, then `active`.

The other way round is `<param name="allow_missing_servos">true</param>` in the `<hardware>` block: the driver configures anyway, logs one WARN per missing servo, and those joints mirror their commands into their states until the servos answer.
Use the controller-manager parameter when you want the failure to be loud but survivable; use `allow_missing_servos` when you are deliberately running with part of the robot absent.

### Recovery after a servo is lost

A servo that stops answering mid-run is dropped from the read cycle after `max_read_fails` consecutive failed reads, with one ERROR.
`allow_missing_servos` has no say in that, and none in whether a dropped joint is re-armed: it is a configure-time policy only.

| route | `allow_missing_servos` | servo answers again? | outcome |
|---|---|---|---|
| `active -> inactive -> active` (deactivate/activate) | either | yes | re-pinged in `on_activate`, added back, INFO `motor id '%d' answered on activation; adding it back`; port never released |
| `active -> inactive -> active` | either | no | activates anyway; the joint stays a phantom, `read()` mirrors its command; the drop ERROR already in the log is the record |
| `unconfigured -> inactive` (a fresh configure) | `true` | no | configures, one WARN per missing servo |
| `unconfigured -> inactive` | `false` | no | FATAL, `close_port()`, `FAILURE`; the component stays `unconfigured` and the port is free |
| `unconfigured -> inactive` | either | yes | configures normally |

So: a **re-configure** after a drop is a fresh configure and is subject to the gate; a **deactivate/activate** is not.


## Bench verification

Everything in `test/` runs with no hardware except one test, `hil_check`, which drives the real servos.
It is registered by `colcon test` like any other test and **skips itself** unless it is asked for:

```bash
colcon test --packages-select waveshare_servos                       # hil_check: SKIPPED
WAVESHARE_HIL=1 colcon test --packages-select waveshare_servos \
  --ctest-args -R hil_check                                          # about 30 minutes of bus time
```

It skips, with the reason printed, unless `WAVESHARE_HIL=1`, the port exists and is readable and writable, the user is not root, and `ros2` is on `PATH`.
`WAVESHARE_HIL_PORT` selects the port (default `/dev/ttyACM0`); `--ctest-args -L hil` runs only this test and `-LE hil` excludes it.
The script is `test/hil_check.sh`, the recorder and the checkers are in `test/hil/`, and one run writes a stable-keyed report (`hil_check.txt`, `hil_check.json`) plus every recording it took.

Nothing else may hold the port while it runs, it refuses to run as root (the kernel lets a root process ignore `TIOCEXCL`, so the exclusivity check would silently lie), and it stops the wheels at the end of every scenario and again at exit.
A row is `PASS`, `FAIL`, `SKIP`, `ABORTED` or -- for exactly two frozen keys the bench cannot supply a stimulus for -- `INCONCLUSIVE`. **There is no mechanism anywhere in it that turns a `FAIL` into anything else.**

Three invariants carry the run: the reported position never steps more than half a revolution; its slope over a half-second window tracks the reported `velocity`; and over a multi-revolution run the travel matches the integral of the velocity.
`python3 test/hil/hil_gates.py --self-test` checks those gates against synthetic data with an injected 2 pi jump, a frozen run, a doubled velocity and a missed wrap, and needs no hardware at all.

### What the bench does not do, and why

**No step of it asks a person to be at the adapter** -- no connector pull, no wheel held by hand, no ammeter.
The two checks that used to need hands are covered deterministically instead, by the pty tests that run in CI on every build:

- a servo that stops answering mid-run, the drop after `max_read_fails`, the re-ping on activate and the post-gap warning: `test/test_lifecycle_over_pty.cpp`, a fake servo told to stop answering between two `read()` calls;
- the status byte: synthetic status values fed through the same fake bus, over all 256 of them.

That proves the **decode**. It does not prove what any bit means on this firmware, and neither would holding a wheel.
So **no status bit is named here without a primary source**: bits 0, 1, 2, 3 and 5 are named after FEETECH's own SDK, and bits 4, 6 and 7 are published as `bitN (meaning unverified)`.
The current scale (`current_per_count_a`) cannot be validated without an ammeter either, so `effort` stays documented as approximate.


## Additional Tools

Also included are some helper functions wrapped in ros2 nodes for ease-of-use.

### Change Motor ID

To control multiple motors, they will need different IDs.

To set a new ID, plug in 1 motor at a time (make sure to turn off power in between), and run:

```bash
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
```

### Set Midpoint

The following command will set the middle position (tick 2048, pi radians, 180 degrees) of a given motor:

```bash
ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<id>
```


## TODO

- software tests


## License

Most of the servo code is from the SCServo_Linux package available on their website.
Waveshare does not include a license in the example files.
When asked, they said to use the GPLv3 license. 

Some of the servo code is from [adityakamath on github](https://github.com/adityakamath/SCServo_Linux).
