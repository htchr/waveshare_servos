# Operation

This page tells what the driver does at start-up, at a fault, after the loss of a servo or the
adapter, and at shutdown. Then it gives troubleshooting, the messages and the known issues.

## Safety

- WARNING: Keep the servo supply switch within reach whenever the wheels can move. Only this switch
  always stops the servos.
- WARNING: Stop the driver only with a clean shutdown (Ctrl-C) over a bus that works. Nothing on
  the servo stops a wheel when the driver stops writing. Thus after SIGKILL, a crash, an
  out-of-memory kill or a [lost adapter](#lost-adapter), the wheels keep their last speed (measured
  after SIGKILL at 2 rad/s). The position servos hold their goals with torque on.
- WARNING: Keep clear of the joints at activation. Torque comes on before the first goal, so a
  joint can start toward an old goal ([Known issues](#known-issues)).
- CAUTION: Torque stays on after [shutdown](#shutdown). The servos hold their pose and draw current
  until you switch off their supply.

## Start-up and missing servos

At configure, the driver pings each servo up to `ping_attempts` times. A servo that answers gets
one ping. A missing servo costs `ping_attempts` × `io_timeout_ms` at configure and nothing in later
cycles. It gets the WARN `unable to ping motor id '<N>'; joint '<j>' will be skipped on the bus`.
Then `allow_missing_servos` decides what occurs:

- `false` (the default): the FATAL `<n> of <m> servos did not answer: <list>; ...` names each
  missing id and joint. The driver releases the port and the configure fails. At start-up, this
  makes `ros2_control_node` exit, because the controller-manager parameter below is `true` by
  default and read-only at run time (ros2_control 4.48.0).
- `true`: the driver configures and logs the WARN `continuing without <n> of <m> servos ...`. It
  says that the joints mirror their commands, but only the `position` state of a `pos` joint does
  this ([State interfaces](configuration.md#state-interfaces)).

For a loud failure that the node survives, set that parameter to `false` in the controller-manager
YAML:

```yaml
controller_manager:
  ros__parameters:
    hardware_components_initial_state:
      shutdown_on_initial_state_failure: false
```

This command shows its value on a running node:

```bash
ros2 param get /controller_manager hardware_components_initial_state.shutdown_on_initial_state_failure
```

With the parameter `false`, the refusal is an ERROR, the node continues to run, and the component
stays `unconfigured` with the port free. No controller that uses the component can activate, so
the example spawner stops at `joint_state_broadcaster` (from the code). When the bus is correct,
read the warning in [Recovery](#recovery-after-a-servo-is-lost). Then do its `inactive` and
`active` steps, and start the controllers again. While a servo is still missing, the `inactive`
step fails with exit status 1.

## Servo faults

When the status byte of a servo changes to a value that is not 0, the driver logs
`motor id '<N>' reports status 0x<hh> (<bits>)` ([Status bits](configuration.md#status-bits)). When
the byte goes back to 0, it logs `motor id '<N>' cleared its fault; re-enabling torque` and
switches torque on. The joint then moves to its command again, and it can trip again if the cause
is still there. Not measured: if the ST3025 clears a bit while its cause is still there.

WARNING: Keep clear of a joint that tripped. When the fault clears, and at the next activation, the
joint moves to its command at up to `max_speed`.

1. Deactivate the hardware component ([Recovery](#recovery-after-a-servo-is-lost) gives the
   commands). This parks the position joints where they are and stops the wheels.
2. Remove the cause.
3. Activate the component.

## Recovery after a servo is lost

A servo can stop answering after a brown-out, a protection trip or a pulled connector. A silent
servo costs a full `io_timeout_ms` in each cycle ([cost](bus-timing.md#cost-of-a-silent-servo)).
Thus the driver drops it after `max_read_fails` consecutive failed reads. The default of 50 is 0.5 s
at 100 Hz. The budget counts cycles, so the drop takes longer at a lower `rw_rate`.

Before the drop, a failed read keeps the last good sample. The first failed read in a row, and each
200th, logs `read failed for motor id '<N>' (<k> in a row)`. The drop logs the ERROR
`motor id '<N>' stopped answering after <n> attempts; ...`, and the joint then reads as a missing
servo. A servo that lost power and answers before the drop comes back with torque off and stays
limp.

WARNING: Keep hands and objects out of the range of each joint before you activate. The drop ERROR
does not tell why the servo stopped, so do this at each recovery.

- Torque comes on before the first goal, so a position joint can start toward an old goal
  ([Known issues](#known-issues)). This goal is 0 for a servo that got power after the `inactive`
  step, while the component was `unconfigured`, or behind a lost adapter. Tick 0 is -π/2 rad for
  `joint1` and `joint2` of the example.
- The controllers stay active (measured). From the first cycle, each joint moves to its last
  command at up to `max_speed`, and the wheels turn at their last speed. The out-of-limit hold does
  not stop this move.

To recover a dropped or limp servo, deactivate and activate the hardware component:

```bash
ros2 control list_hardware_components    # shows the component names
ros2 control set_hardware_component_state example_ws_ros2_control inactive
ros2 control set_hardware_component_state example_ws_ros2_control active
```

On activation, the driver pings each missing or dropped servo. For each servo that answers, it logs
`motor id '<N>' answered on activation; adding it back` and switches torque on. A servo that does
not answer stays absent. A deactivate/activate does not check `allow_missing_servos`, so a robot
that got back two servos of three can activate again. A new configure checks it, as at
[start-up](#start-up-and-missing-servos).

## Lost adapter

If the USB adapter disconnects, the driver drops each servo, and the component stays active. No
message names the lost adapter. The wheels keep their last speed, because no stop reaches them over
a lost adapter. Until you recover, only the servo supply switch stops them.

A deactivate/activate does not help, because it does not close the port. An adapter that you
reconnect while the driver holds the old port usually gets a new `/dev/ttyACMn` name. Thus set
`port` to its `/dev/serial/by-id/` name ([Serial port access](setup.md#serial-port-access)).

WARNING: Keep clear of the joints before the activation in each procedure below. A servo that lost
power can start toward goal 0.

To recover with a restart (this always works):

1. Stop the launch with Ctrl-C.
2. Connect the adapter again.
3. Start the launch again. Activation sets each velocity command to 0, and the new wheel controller
   has no command to replace it.

To recover without a restart (tested on the bench without an unplug):

WARNING: Publish zero wheel speeds before you set the component `active`. The wheel controller
stays active and sends its last command again.

1. Set the component `unconfigured`. This closes the port.
2. Connect the adapter again.
3. Publish zero wheel speeds.
4. Set the component `active`.

```bash
ros2 control set_hardware_component_state example_ws_ros2_control unconfigured
# re-plug the adapter here
ros2 topic pub -t 3 -r 10 /joint_velocity_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0]}"
ros2 control set_hardware_component_state example_ws_ros2_control active
```

## Shutdown

On SIGINT or SIGTERM (for example Ctrl-C), the controller manager deactivates and then shuts down
the hardware. The driver stops the wheels, parks each position joint where it is, and closes the
port. Each deactivation logs the [bus totals line](bus-timing.md#bus-totals-line).

Torque stays on after shutdown (measured). Of the tools, only `calibrate_midpoint` and
`factory_reset` switch it off.

At Ctrl-C, `controller_manager.pal_statistics` can log two ERRORs,
`Exception in publisher thread: context cannot be slept with ...`. An upstream shutdown race after
the deactivation causes them. They do no harm.

## Troubleshooting

Each entry starts with the text that you see. `<...>` marks text that the program fills in.

### Port and bus access

- `port '<port>' is already held exclusively by another process<holder>; ...` or
  `another process holds the lock on port '<port>'<holder>; ...`: another program, usually a
  controller manager or a tool, holds the port or its lock. Stop that program. `<holder>` gives its
  pid if the driver can see it. A tool in the same condition exits with status 1.
- To find a holder, run `find /proc/[0-9]*/fd -lname /dev/ttyACM0 2>/dev/null`. The number after
  `/proc/` is the pid. The command sees only the processes of your user, so run it as root to see
  all. It prints `Permission denied` for each process that it cannot read, and then exits with
  status 1. `2>/dev/null` hides only these lines.
- `not allowed to open port '<port>': ...`: see [Serial port access](setup.md#serial-port-access).
- `port '<port>' does not exist: ...`: the adapter is not connected or has another name. Run
  `ls /dev/ttyACM* /dev/ttyUSB* /dev/serial/by-id/`.
- `<n> of <m> servos did not answer: ...`: check the supply, the wiring, the ids and the baud rate
  ([Find your servos](setup.md#find-your-servos)).
- `scan` finds no servo: also make sure that the jumper cap of the adapter is on B
  ([Set up](../README.md#set-up)).

### Configuration messages

- `motor id '<N>' mode changed from <old> to <new>`: expected once after a change of `type`. It is
  an EEPROM write.
- `Could not enable FIFO RT scheduling policy ...` or `Unable to lock the memory ...`: see
  [Real-time scheduling](setup.md#real-time-scheduling). The driver continues to run.
- `hardware parameter '<name>' is not used by this driver; ignoring it` or
  `joint '<j>' parameter '<name>' is not used by this driver; ignoring it`: a typo. The parameter
  that you meant is at its default. Read the `bus configuration:` line.
- `a joint does not have a command interfaces` or
  `a joint is using a command interface that isn't position or velocity`: these FATALs name no
  joint. Each `<joint>` needs at least one command interface, and the driver accepts only `position`
  and `velocity` ([Command interfaces and limits](configuration.md#command-interfaces-and-limits)).

### The launch does not stop

After a FATAL, the launch does not stop by itself. Press Ctrl-C.

- A FATAL at configure (port missing or busy, servos missing) makes `ros2_control_node` exit, but
  `robot_state_publisher`, RViz and one spawner continue to run. That spawner waits with no timeout.
  The other one stops after about two minutes: `Failed to acquire lock after multiple attempts.`
- A FATAL at load (a bad parameter in `on_init`) does not stop the node. After
  `Failed to initialize hardware '<name>'` and `Could not load and initialize hardware. ...`, it
  logs `Waiting for data on 'robot_description' topic to finish initialization` once a second. Its
  controller services never appear (controller_manager 4.48.0 code), so the spawners act as above.

### Run-time warnings

- `read failed for motor id '<N>' (<k> in a row)` or
  `motor id '<N>' reports status 0x<hh> (<bits>)`: see [Recovery](#recovery-after-a-servo-is-lost)
  or [Servo faults](#servo-faults).
- `Overrun might occur, Total time : ... (Expected < ...)`: the loop missed its period. It can occur
  without [real-time scheduling](setup.md#real-time-scheduling).
- `Velocity command timed out. Braking.` (`diff_drive_controller`, while nothing publishes
  `cmd_vel`) and the `pal_statistics` ERRORs at Ctrl-C ([Shutdown](#shutdown)): expected.

### A joint ignores commands

- A joint does not move after start-up: look for
  `joint '<j>' starts at <x> rad, outside its limits [<min>, <max>]; ...`. Command a position inside
  the limits.
- A `ros2 topic pub` command printed `publishing #1` and had no effect: a new `ros2 topic pub`
  process can lose its first message. Send three copies (`-t 3 -r 10`) and send a lost command
  again. If it still has no effect, run `ros2 control list_controllers`.
- An inactive `joint_trajectory_position_controller` or `joint_velocity_controller` ignores
  messages and logs nothing. `joint_velocity_controller` is inactive after a command of the wrong
  length and while `diff_drive_controller` is active. Its activation does not apply a command sent
  while it was inactive, so send it again (ros2_controllers 4.42.1 code, seen on the bench).

## Log messages

This list holds the main driver and tool messages, as printed. `<...>` marks a value that the
program fills in, and `...` marks text left out. Messages of ROS itself are not in it.

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

## Known issues

This is the one list of known issues and limits. Other pages name an issue by its title.

- **Torque before the first goal.** Activation switches torque on before the first `write()` sends
  a goal, so a joint can start toward an old goal for about one control period. After a cold start
  that goal is 0. After [calibrate_midpoint](tools.md#calibrate_midpoint), it is the old goal in the
  new frame. After a [recovery](#recovery-after-a-servo-is-lost), it is normally the last goal sent.
  Not measured: if the firmware sets the goal to the present position when torque comes on.
- **A dead bus does not reach `on_error`.** `read()` and `write()` always return OK. The driver
  drops each silent servo, and the component stays active.
- **Timeouts sized for 1 Mbaud.** The default and the floor of `io_timeout_ms` do not change with
  `baudrate` ([Other baud rates](bus-timing.md#other-baud-rates)).
- **Busy-port holder with a by-id path.** With a `/dev/serial/by-id/` link as `port`, the busy-port
  FATAL cannot name the holder (from the code). The tools can. For `find`, use the device name that
  `readlink -f <port>` shows.
- **Wrong text in the pos-joint FATAL.** It says that a velocity command interface "only paces the
  move". This is not correct: the driver sets the goal speed itself.
- **The command-interface FATALs name no joint.** They also do not name the interface. See
  [Configuration messages](#configuration-messages).
- **Approximate scales and unverified status bits.** `effort` and `current` are unsigned and
  approximate. Nobody verified the voltage scale or status bits 4, 6 and 7. The SDK and a memory
  table disagree on bits 1 and 4 ([Status bits](configuration.md#status-bits)).
- **EEPROM lock results not checked for the mode write.** The driver reads the mode back, but it
  does not check the results of the EEPROM unlock and lock. To check a new mode, switch the servo
  off and on, then run `scan`.
- **Servo power loss during a run.** A servo that loses power comes back with torque off. Unless a
  fault clears, it stays limp until a [recovery](#recovery-after-a-servo-is-lost). Nobody tested if
  the four rewrites of the wheel acceleration register catch each brown-out
  ([Wheel acceleration](design.md#wheel-acceleration)). Both points come from the code and the
  measured power-up state.
- **A dropped wheel without unwrap reports its activation position.** Such a `vel` joint does not
  keep its last measured position (from the code). A dropped joint with `unwrap` true keeps it.
- **More than four servos.** The cost model and the timeout floors come from one to four servos on
  one bench ([Cost per servo](bus-timing.md#cost-per-servo)). Above 30 position servos, the goals go
  in two packets about 2.5 ms apart. Nobody measured if this shows in motion.
- **Torn samples with `is_async`.** One 120 s recording found no torn sample, but this is not a
  proof ([rw_rate and is_async](bus-timing.md#rw_rate-and-is_async)).
- **Trajectory controller on wheels.** A velocity-only trajectory on a continuous joint, or with a
  first point at `time_from_start` 0, makes `joint_trajectory_controller` 4.42.1 crash
  `ros2_control_node` (seen on mock hardware). The crash comes before the first command, so the
  wheels keep their previous speed. On velocity-only joints, this controller is also a PID position
  tracker, not a passthrough. Use `JointGroupVelocityController` or `diff_drive_controller`, as the
  example does. The example's position-and-velocity trajectory controller ran on the bench with no
  crash.
- **`factory_reset` has no bench scenario.** Tests on one servo verified it. Its refusal of a silent
  id ran on a real bus (exit status 3, nothing written).
- **The bench check has no identity check.** It does not check the bus before its first scenarios.
  Run it only on the [reference bench](setup.md#the-reference-bench).
- **Bench wheel commands can be lost.** `test/hil_check.sh` sends each wheel command with one
  `ros2 topic pub --once`. A lost command fails a row only in H8 and at the start of the H11 soak.
  After a run, make sure that the wheels stopped.
- **Lost adapter not detected.** A deactivate/activate does not open the port again
  ([Lost adapter](#lost-adapter)).
- **The root warning omits `factory_reset`.** The root WARN names `scan`, `set_id` and
  `calibrate_midpoint` as the programs that take the advisory lock. `factory_reset` also takes it.
