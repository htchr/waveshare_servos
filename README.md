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

The launch file takes four arguments:

| argument | default | effect |
| --- | --- | --- |
| `port` | `/dev/ttyACM0` | serial port of the bus servo adapter |
| `baudrate` | `1000000` | bus baud rate |
| `use_mock_hardware` | `false` | `true` swaps in `mock_components/GenericSystem`, which opens no serial port. Accepts `true`, `false`, `1` or `0` in any case; any other value aborts the render on purpose |
| `gui` | `true` | `false` starts no RViz |

For example: `ros2 launch waveshare_servos example.launch.py port:=/dev/ttyUSB0 gui:=false`.

Under `use_mock_hardware:=true` every command is mirrored straight back into its state, so this is
how to try the example with no hardware at all.
One mock-only quirk is worth knowing before reporting it as a bug: a velocity-commanded joint's
`position` never advances, so the wheels stand still in RViz and the odometry below stays at zero.
That is `mock_components/GenericSystem`, not the driver.
`effort` and `temperature` likewise read a constant `0.0`.

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

### Driving the wheels as a differential base

`example_controllers.yaml` also configures a `diff_drive_controller` over the two wheels.
It ships in its own apt package -- `sudo apt install ros-jazzy-diff-drive-controller` -- and is
declared as an `<exec_depend>`, so `rosdep install --from-paths src --ignore-src -r -y` picks it up
too.
Without it the launch still brings up the other three controllers, but the `diff_drive_controller`
spawner exits non-zero and says why.

The launch file loads it but leaves it **inactive**, because it commands the same `joint3/velocity`
and `joint4/velocity` interfaces as `joint_velocity_controller`, and ros2_control hands each command
interface to one controller only.
Swap them atomically -- activating first, without the deactivate, is refused:

```bash
ros2 control switch_controllers --strict \
  --deactivate joint_velocity_controller --activate diff_drive_controller
```

The controller subscribes to `geometry_msgs/msg/TwistStamped` (4.42.1 has no unstamped option) and
brakes 0.5 s after the last message, so publish continuously rather than with `--once`:

```bash
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: base_link}, twist: {linear: {x: 0.1}}}'
```

`wheel_radius` (0.05 m) and `wheel_separation` (0.20 m) are **example values for a bench with no
chassis** -- measure them on a real robot.
They are picked so the arithmetic checks by eye: `linear.x: 0.1` is 2.0 rad/s on both wheels, and
`angular.z: 1.0` is -2.0 rad/s on the left wheel (`joint3`, `left_wheel_names`) and +2.0 rad/s on the
right (`joint4`).
Same sign on both wheels for `linear.x`, opposite signs for `angular.z`; a positive `angular.z`
drives the right wheel forward.

`/diff_drive_controller/odom` integrates the wheels' **unwrapped** multi-turn `position` state
(`position_feedback: true`), so it does not jump once per revolution.
Two expected artefacts: the pose takes one step at activation, because the controller seeds its
previous-wheel-position memory at zero and 4.42.1 has no reset service -- cycle the controller
through `unconfigured` and back to `active` to zero the pose -- and while nothing is publishing
`cmd_vel` the controller logs `Velocity command timed out. Braking.` about once a second.

Go back with the reverse switch:

```bash
ros2 control switch_controllers --strict \
  --deactivate diff_drive_controller --activate joint_velocity_controller
```

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
  --ctest-args -R hil_check                                          # about 27 minutes of bus time: ~17 of scenarios plus the ten-minute soak
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

## Sizing the bus

Everything in this section was measured on the reference bench: four ST3025 servos, ids 1-4, on `/dev/ttyACM0` through the supplied Bus Servo Adapter at 1 000 000 baud, `update_rate: 100`, on a non-RT kernel with no FIFO scheduling privileges.
These are that bench's numbers, not a promise about your robot. The cost model below is the part that is meant to travel; the totals are not.

### What a cycle costs

| per 100 Hz control cycle, four servos | before | after |
|---|---|---|
| `read()` -- position, velocity, load, voltage, temperature, current, status | 3.064 ms | **1.645 ms** |
| `write()` -- two position goals and two wheel speeds | 1.447 ms | **0.014 ms** |
| both, out of the 10 ms period | 4.51 ms (45 %) | **1.66 ms (16.6 %)** |

Each column is the mean of three alternating runs of the same three bench scenarios, one arm after the other on the same afternoon: the left column is one `FeedBack()` round trip per joint plus an acknowledged per-wheel acceleration write, the right column is one `INST_SYNC_READ` covering all four servos plus two unacknowledged broadcast writes.
The run-to-run spread inside each arm was 0.008 ms or less, so the difference is not noise.

The read saving is a round trip removed per extra servo.
The write saving is larger than it looks because the old path was not spending its time on the wire: the vendored `SyncWriteSpe` writes each wheel's acceleration register with a call that blocks waiting for that servo's status packet, and two of those were most of the 1.447 ms. The acceleration register is now written at four edges instead of every cycle, and what is left is two broadcast packets the kernel accepts in microseconds.

The same numbers hold over a long window: three ten-minute soaks at 100 Hz, about 60 000 cycles each, averaged 1.647 ms read and 0.013 ms write -- one part in 1200 away from the nine-second measurement above.

The published values did not change. Both read paths funnel through the same decoder, and a side-by-side capture found `position`, `velocity`, `load` and `current` bit-identical in 800 of 800 samples. `voltage` and `temperature` are the exception, and they are **not** bit-identical between two reads -- of either path. They dither by up to one ADC count, at the same rate between two reads of the *same* path as between the two paths, so it is the servo's converter and not the transport. Nothing here compares the two transports on those two interfaces, and no test in this package demands equality on them.

### How the cost scales with servo count

Measured per transaction, by least squares over one to four servos:

```
sync read of n servos        t(n) = 0.476 + 0.290 n   ms
one FeedBack() per servo     t(n) = 0.750 n           ms
```

The crossover is n = 1.03, so the sync read wins from two servos up, and every servo after the first costs **0.29 ms** instead of 0.75 ms.
Above four servos this is a model, not a measurement: it is a straight line through four points on one bench with one adapter, and nothing here has been run with more.

Broadcast sync writes are unacknowledged, so they cost the caller almost nothing regardless of servo count -- 0.004 ms measured at a 100 Hz cadence -- but they do occupy the bus for `(8 + n * (nLen + 1)) * 10 us`, which is exactly wire time at 1 Mbaud, with `nLen` 7 bytes for a position record and 2 for a wheel speed.

**How many joints fit at 100 Hz.** Allowing half the 10 ms period for bus work and scaling the model by the measured 1.19 tail factor:

```
1.19 * (0.476 + 0.290 n) + 0.02 <= 5   ->   n <= 12.8
```

So **twelve joints at 100 Hz with 50 % headroom**, on a p99 basis. On a mean basis the same budget allows fifteen or sixteen, and that is a mean-based figure -- do not size a robot with it.
Two cross-checks agree that twelve is the honest number: pure wire time would allow sixteen, and the adapter's USB framing (below) puts a 4 ms floor under a twelve-servo cycle whatever the baud rate.
For comparison, the per-servo path this replaces holds **five** joints in the same budget, so the change slightly more than doubles the joint count a 100 Hz loop can carry.

**What is actually binding** is that 0.29 ms of marginal cost per servo. It is not the baud rate: the adapter is USB CDC and the host polls it once per 1 ms USB frame, so a back-to-back read loop bottoms out at 2.000 ms per cycle -- 499.7 Hz with four servos -- no matter what the line rate is. Raising the baud above 1 Mbaud would not move any number in this section, and this package does not recommend it.
It is also not the packet size limits: a sync write chunks at 30 position records or 82 wheel-speed records per packet, and a bus that large is far past the timing ceiling above.

### `io_timeout_ms`, and how often a transaction fails

`io_timeout_ms` is the per-transaction budget the driver hands to the vendored serial layer (`SCSerial::IOTimeOut`). Legal range **2..1000**, default **5**.

This is the first hardware parameter this README documents; the rest are listed with their defaults in the `<hardware>` block of `description/ros2_control/example.ros2_control.xacro`.

**Breaking change in this release.** The lower bound moved from 1 to 2 and the default from 20 to 5. A description that sets `io_timeout_ms` to 1 no longer configures -- `on_init` fails with a message naming the new range. That is deliberate: at 1 ms a sync read of four servos does not merely time out, it occasionally returns *the previous cycle's* reply frames, with correct headers, correct ids in the correct slots, correct length bytes and correct checksums. Eight of 3000 reads did exactly that on this bench. No flush can prevent it, so the value is refused instead.

**Size it for the burst, not for one transaction.** A single `FeedBack()` survives a 1 ms timeout comfortably; a sync read of four ids fails 98.55 % of the time at 1 ms and 0.00 % at 2 ms and above. The floor the driver checks against is roughly `0.48 + 0.29 * servos` milliseconds, scaled by about 1.2 for the tail, plus a millisecond:

| servos | 1 | 4 | 8 | 9 | 10 | 12 | 16 | 30 |
|---|---|---|---|---|---|---|---|---|
| advisory floor (ms) | 2 | 3 | 5 | 5 | 6 | 6 | 8 | 12 |

Below that floor the driver says so. A value you did not set is raised to the floor with one INFO; a value you did set is either warned about and demoted to the one-read-per-servo path, or refused outright if you pinned `feedback_mode` to `sync_read`. The 5 ms default is silent up to nine servos.

**What a silent servo costs.** Exactly one full timeout per cycle -- once, not once per servo, and it does not matter where the dead id sits in the list. Against a 1.60 ms clean read:

| `io_timeout_ms` | 2 | 3 | 5 | 10 | 20 |
|---|---|---|---|---|---|
| cost of one absent servo | +0.46 ms | +1.47 ms | +3.56 ms | +8.57 ms | +18.60 ms |
| read cycle, relative | 1.29x | 1.92x | 3.23x | 6.31x | 12.44x |

At 100 Hz that is the whole argument for a small value: with one servo dead, 5 ms keeps the loop running at about half rate, 10 ms stalls it, and the vendored library's own 100 ms default would stall it for ten cycles. A servo that is dead *at configure time* never enters the sync-read list at all and costs nothing; this is the price of one that goes quiet mid-run, for the `max_read_fails` cycles before it is dropped.

**It costs nothing when the bus is healthy.** Mean read latency is flat at 1.60-1.63 ms for every timeout from 2 ms to 20 ms, so there is no reason to keep a large value "for safety". Running the shipped driver with the timeout put back to 20 ms moved the measured read average by 0.003 ms, against a run-to-run spread of 0.008 ms. A healthy sync read never comes near its deadline.

**Sub-millisecond values are meaningless** on this adapter. It is USB CDC and the host polls it once per 1 ms frame: a 1 ms read window collects, on average, 0.0 of the 84 reply bytes, and all 84 are waiting 2 ms later. Design your timing constants on a 1 ms granularity.

**How often a transaction fails, on this bench.**

> Over three ten-minute soaks at 100 Hz -- four servos at 1 Mbaud, both wheels turning at 1 rad/s and the arm holding position, `io_timeout_ms: 5` -- the driver recorded **0 failed transactions in 182 994**, with a worst consecutive failing run of 0, no servo dropped, and zero failures on each of the four ids individually.
>
> Zero events is not a failure rate of zero. With nothing observed, the honest statement is an upper bound: by the rule of three, the failed-transaction rate on this bench is **below 17 per million at 95 % confidence** (3/182 994 = 16.4) -- under about six failures per hour of continuous operation at 100 Hz. The true rate may be far lower, and this measurement cannot tell you.

A fourth clean ten-minute soak inside the full suite brings the pool to 0 failures in 244 158 transactions, which tightens the same bound to 13 per million (3/244 158 = 12.3). Measure it on your own bus with

```bash
WAVESHARE_HIL=1 WAVESHARE_HIL_SCENARIOS=H11 colcon test --packages-select waveshare_servos \
  --ctest-args -R hil_check                                   # ten minutes, plus setup
```

and read the `H11.fail_rate` row; the driver also prints one `bus totals:` line per deactivation with the same counters and a per-servo breakdown.

**What a failure actually does.** The driver keeps the last good sample for that joint rather than publishing the `-1` a timed-out read returns, counts the failure against that servo, and after `max_read_fails` consecutive failures drops the servo from the read cycle with one ERROR (`src/waveshare_servos.cpp:1732-1748`). See [Recovery after a servo is lost](#recovery-after-a-servo-is-lost) for what happens next and how to get the joint back.

### Running the bus slower, or off the control thread

`ros2_control` 4.48 can read and write the hardware at a lower rate than the controller manager's `update_rate`, and can run it on its own thread. Both are **attributes of the `<ros2_control>` element**, not `<param>` children of `<hardware>`, which is the mistake that costs an afternoon:

```xml
<!-- optional, ros2_control 4.48: read/write the hardware at a different rate than the
     controller manager's update_rate, and/or on its own thread
<ros2_control name="${name}" type="system" rw_rate="50" is_async="true">
-->
```

This package sets neither, for the reasons below. `ros2 control list_hardware_components` prints `read/write rate:` and `is_async:` unconditionally; that is how you check an attribute took effect.

Measured here at `update_rate: 100` with four servos:

- `rw_rate="50"` does exactly what it says. Bus occupancy halves -- 49.98 against 99.96 transactions per second, so 8.2 % of wall clock instead of 16.5 % -- with the per-read cost unchanged at about 1.65 ms and no failed transactions. The unwrapper, the command pacing and the drop policy are unaffected.
- `is_async="true"` changed no measurable number at four servos: read 1.648 ms against the synchronous 1.653 ms, `/joint_states` unchanged at 100 Hz. A 120 s recording of a wheel at 2 rad/s (12 251 samples) contained no repeated sample and no doubled position step, so nothing torn was observed -- but that is a negative result over one window, not a proof that the async wrapper copies the state arrays under a lock. An async component stops the wheels correctly on `SIGINT`, tested at 2 rad/s against a synchronous control run; `rw_rate` was not separately SIGINT-tested with a wheel turning, since it does not touch the shutdown path.

The caveats are worth more than the results:

- **`/joint_states` keeps publishing at the controller manager's `update_rate`.** A lower `rw_rate` does not slow the topic down; it makes each value repeat. Anything differentiating `position` numerically will read zero apparent motion on alternate samples.
- **Pick an `rw_rate` that divides the `update_rate`.** `rw_rate="60"` against `update_rate: 100` is reported by `list_hardware_components` as `60 Hz` and actually runs at 50 Hz -- the controller manager decimates the loop by an integer, and nothing logs the difference. The CLI shows the rate you asked for, not the rate you got.
- **A value above the `update_rate`, and `rw_rate="0"`, are silently treated as the `update_rate`.** No warning, no error, no mention in the log, so a typo is invisible except through `list_hardware_components`.
- **This package's bench gates assume a 100 Hz component.** At `rw_rate="50"` with a wheel near 2 rad/s, `hil_check`'s velocity-consistency gate charges a 20 ms position step against a 10 ms allowance and starts failing; with `is_async="true"` set as well it fails on nearly every step, because the timestamp jitter that was providing the margin disappears. Below `update_rate/2` the stale-run rule starts excluding most of the recording instead. `rw_rate` is outside what this bench checks, which is why the package does not set it.
- **An async component asks for a real-time thread.** Without RT privileges the controller manager warns once per component (`Could not enable FIFO RT scheduling policy`) and runs it at normal priority -- which is the configuration in which async jitter is least predictable. Set up RT scheduling if you use it.
- `thread_priority` is deprecated in 4.48 in favour of `async_params`; do not copy it from older examples.

On this driver `is_async` buys nothing, because `read()` is the only expensive thing in the control thread and no controller here was missing a deadline. Use it if you have something else in that thread; use `rw_rate` if the bus, not the CPU, is your bottleneck.

### What this section does not cover

Four things a reader could reasonably expect from the numbers above, and which nobody has measured:

- **More than four servos.** The cost model, the twelve-joint ceiling and the timeout floor table are all fitted on one to four servos, on one bench, through one adapter. Nothing above four has been run. Treat the model as a model.
- **A servo that loses power mid-run.** The wheel acceleration register is SRAM and a power cycle clears it. The driver writes it at four edges -- when the groups are built, on activation, when a servo starts answering again, and when a fault clears -- but whether those edges catch every real brown-out is untested, and so is whether this firmware comes back with torque enabled. A wheel that reboots and is not caught by one of those edges runs unramped, and nothing logs it.
- **More than 30 position servos.** Their goal positions split across two packets about 2.5 ms apart. Whether that skew is visible in motion cannot be answered on a four-servo bench.
- **`is_async` and torn samples.** The 120 s recording above found nothing torn, which is a negative result over one window and not a proof that the framework copies the driver's state arrays under a lock.


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


## License

Most of the servo code is from the SCServo_Linux package available on their website.
Waveshare does not include a license in the example files.
When asked, they said to use the GPLv3 license. 

Some of the servo code is from [adityakamath on github](https://github.com/adityakamath/SCServo_Linux).
