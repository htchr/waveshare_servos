# Configure a robot

The plugin is `waveshare_servos/WaveshareServos`. Start from the example or the minimal
description, then use the reference tables. [Bus timing](bus-timing.md) explains `io_timeout_ms`
and `feedback_mode`, and [Operation](operation.md) explains faults, recovery and messages.

## Adapt the example

1. Copy `description/`, `bringup/config/example_controllers.yaml` and
   `bringup/launch/example.launch.py` into your package. Edit the copies, not the installed files.
2. Point the copied launch file at your package.
3. In the copied `example.urdf.xacro`, change `$(find waveshare_servos)` in `xacro:include` to
   your package. Else the installed block (ids 1-4, 3 and 4 as wheels) stays in use. Your edits
   then have no effect, and no warning occurs.
4. Set the ids and types from `scan` ([Find your servos](setup.md#find-your-servos)). Set the
   offsets ([Joint frame](#joint-frame)).
5. If you remove a joint, also remove it from the controllers YAML. Else the spawner of its
   controller fails.

For a mirrored differential base, add `<param name="inverted">true</param>` to the right wheel, so
that one positive velocity drives both wheels forward. The example does not, because its URDF is a
serial chain ([Joint frame](#joint-frame)).

## A minimal description

One position servo and one wheel on one bus. Neither joint sets `type`. The command interfaces
make `pan` a `pos` joint (mode 0) and `wheel` a `vel` joint (mode 1).

**WARNING:** Change the ids to agree with your bus before you load this block. Else the driver can
rewrite the mode register (EEPROM) of servos 1 and 3 ([Joint parameters](#joint-parameters)). The
`type` column of `scan` shows the present mode of each servo.

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

`port` and `baudrate` show their defaults. `offset` 3.141593 makes tick 2048 joint zero, so the
limits map to ticks 1024 and 3072. With `offset` 0 the driver refuses the block at load: the
limit -1.5708 rad maps to tick -1024 ([Joint frame](#joint-frame)). The URDF needs joints with the
same names: `pan` revolute with the same `<limit>` values, and `wheel` continuous.

## Hardware parameters

All eleven are optional `<param>` children of `<hardware>`. The example sets `port` and `baudrate`.

<!-- reference:hardware-parameters:begin -->

| parameter | type | default | legal values | what it does |
|---|---|---|---|---|
| `port` | string | `/dev/ttyACM0` | any non-empty path | Serial device of the adapter. The driver opens and locks it at configure, not at load ([Port lock](design.md#port-lock)). |
| `baudrate` | integer | `1000000` | `9600`, `19200`, `38400`, `57600`, `115200`, `500000`, `1000000` | Must be the rate stored in the servos (the `baud` column of `scan`). See [Other baud rates](bus-timing.md#other-baud-rates). |
| `io_timeout_ms` | integer (ms) | `5` | `2`..`1000` | Time limit for one bus transaction. See [Transaction timeout](bus-timing.md#transaction-timeout) and [Timeout floor](bus-timing.md#timeout-floor). |
| `ping_attempts` | integer | `3` | `1`..`10` | Pings per servo at configure, and per absent servo at activation. An absent servo costs up to `ping_attempts` x `io_timeout_ms` each time. |
| `max_read_fails` | integer | `50` | `1`..`1000000` | Number of **consecutive** failed reads that drop a servo: 0.5 s at 100 Hz, 1 s at 50 Hz. See [Recovery](operation.md#recovery-after-a-servo-is-lost). |
| `allow_missing_servos` | bool | `false` | `true` / `false` (any case) | `false`: configure fails if a servo does not answer its ping. `true`: configure continues without it. See [Start-up](operation.md#start-up-and-missing-servos). |
| `protocol` | string | `sms_sts` | `sms_sts` (any case) | Servo protocol family. The driver knows `scscl` but refuses it, because it is not implemented. |
| `feedback_mode` | string | `auto` | `auto`, `sync_read`, `per_servo` (any case) | One sync read per cycle, or one read per servo. See [Feedback mode](bus-timing.md#feedback-mode). |
| `encoder_steps` | integer | `4096` | even, `2`..`32768` | Steps per revolution, used in every angle and rate conversion. The **register** defaults of `max_speed` and `max_accel` do not scale with it. |
| `current_per_count_a` | double (A/count) | `0.006` | `> 0` and `<= 1` | Scale of the current register (69-70). Not verified with an ammeter. |
| `torque_constant_nm_per_a` | double (N m/A) | `0.8825985` | `> 0` and `<= 100` | `effort = current x torque_constant_nm_per_a`. The default is 9.0 kgf cm/A. |

<!-- reference:hardware-parameters:end -->

One INFO per load shows the values in use, after the [timeout floor](bus-timing.md#timeout-floor).
Read it after each change to the description. The stock example on the bench gives:

```text
bus configuration: port '/dev/ttyACM0', 1000000 baud, protocol 'sms_sts', io timeout 5 ms, 3 ping attempt(s), drop a servo after 50 consecutive read failures, allow_missing_servos false, feedback_mode 'auto', 4096 encoder steps per revolution, 0.006 A per current count, 0.8825985 N m/A
```

## Parameter values

These rules apply to every hardware and joint parameter:

- The driver removes whitespace at both ends. An absent parameter keeps its default, but an empty
  one (`<param name="x"></param>`) is an error.
- `hardware_interface::stod` and `stoi_generic` parse numbers without the locale. Trailing
  characters (`20ms`, `1.0f`, `1,5`), `nan`, `inf` and overflow are errors, so no NaN can pass a
  range check.
- Integers accept `+` and leading zeros (`+001` is `1`), but not hex, `1e6` or `1000000.0`.
- Booleans are `true` or `false` in any case, not `1`, `0`, `yes`, `no`, `on` or `off`.
- A bad value never changes a setting in part. An exclusive minimum is exactly `value > min`.
- An unknown name gives one WARN, and the driver continues to load. Thus a typo leaves a
  parameter at its default. Read the `bus configuration:` line.
- If a `<param>` occurs two times, the URDF parser keeps the LAST value with no message.

The driver checks `port`, `baudrate`, `protocol`, `feedback_mode`, `io_timeout_ms`,
`ping_attempts`, `max_read_fails`, `allow_missing_servos`, `encoder_steps`, `current_per_count_a`
and `torque_constant_nm_per_a` in this order, then the timeout floor, then the joints. The FIRST
bad value gives one FATAL, and `on_init` returns `ERROR`. The driver parses nothing after it and
does not open the port ([The launch does not stop](operation.md#the-launch-does-not-stop)).

[Log messages](operation.md#log-messages) gives most rejection messages. A well-formed `baudrate`
that is not one of the seven rates and `protocol` `scscl` have their own messages. An odd
`encoder_steps` gets the out-of-range message. The `max_speed` and `max_accel` caps are WARNs, and
the driver continues to load. Each successful load gives one INFO:
`parsed <n> joints: <p> position (mode 0), <v> velocity (mode 1)`. The FATAL text "only paces the
move" for a `pos` joint is not correct ("Wrong text in the pos-joint FATAL" in
[Known issues](operation.md#known-issues)).

## Joint parameters

Each is a `<param>` child of `<joint>`. Only `id` is necessary.

<!-- reference:joint-parameters:begin -->

| parameter | type | default | legal values | what it does |
|---|---|---|---|---|
| `id` | integer | none, necessary | `1`..`253`, unique in the `<ros2_control>` block | Bus id (the `id` column of `scan`). [set_id](tools.md#set_id) moves a servo off id 0. 254 is the broadcast id and 255 the packet header. |
| `type` | `pos` or `vel` | inferred | exactly `pos` or `vel` (**case-sensitive**). `pos` needs a `position` command interface, and `vel` must not have one. | `pos`: mode 0, goal position. `vel`: mode 1, closed-loop wheel. If absent, `vel` for a joint with only a velocity command interface, else `pos`. |
| `offset` | double (rad) | `0.0` | any finite number. On a `pos` joint, the limits must map into one turn. | The servo angle that is joint zero ([Joint frame](#joint-frame)). It also moves the position of a `vel` joint. |
| `inverted` | bool | `false` | `true` / `false` (any case) | Flips the `position`, `velocity` and `load` states and both commands ([Joint frame](#joint-frame)). |
| `max_speed` | double (rad/s), a magnitude | 6000 steps/s = **9.2039 rad/s** at 4096 | `> 0`, and 0.000767 rad/s or more (1 step/s when rounded). Above 32767 steps/s (**50.2639 rad/s**): capped, with a WARN. | `pos` joint: the highest goal speed that the driver sends. `vel` joint: the limit of the velocity command. |
| `max_accel` | double (rad/s^2), a magnitude | register value 150 = **23.0097 rad/s^2** at 4096 | `0` (no ramp), or 0.0767 rad/s^2 or more (1 count when rounded). Above 255 counts (**39.1165 rad/s^2**): capped, with a WARN. | Acceleration register 41. A `pos` joint sends it with each goal. A `vel` joint gets it at four events ([Wheel acceleration](design.md#wheel-acceleration)). |
| `unwrap` | bool | `true` for `vel`, `false` for `pos` | `true` / `false` (any case). The driver refuses `true` on a `pos` joint. | Makes `position` a multi-turn count ([Multi-turn wheel position](#multi-turn-wheel-position)). One INFO per joint that unwraps. |

<!-- reference:joint-parameters:end -->

**WARNING:** Make sure that each `type` agrees with the servo at that id. If they differ, the
driver rewrites mode register 33 (EEPROM). It does this at configure, and when an activation adds
back a missing or dropped servo. It reads the mode back ("EEPROM lock results not checked for the
mode write" in [Known issues](operation.md#known-issues)). Expect this INFO once after a `type`
change: `motor id '<N>' mode changed from <old> to <new>`.

`max_speed` and `max_accel` are SI magnitudes in the joint frame, so `inverted` does not apply.
The driver converts them to counts as value x `encoder_steps` / 2π. The acceleration register
takes a further division by 100 (the Feetech table scale, not verified on the ST3025). An absent
value writes the raw register value (6000 or 150), so only an explicit `max_accel` uses this
scale. `max_accel` 0 means no ramp: the driver writes 0 to the register, and does not skip it.

`inverted` is a `<param>`, not an attribute of `<joint>`. An unknown joint parameter gives only a
WARN, so `invert` instead of `inverted` gives a servo that turns the wrong way (measured).

## Joint frame

The driver converts between servo ticks and the joint frame as follows, with `sign` = -1 if
`inverted` is true, else +1:

```text
servo angle      theta     = tick x 2 pi / encoder_steps
joint position   q         = sign x (theta - offset)
goal tick        goal tick = (sign x q + offset) x encoder_steps / 2 pi
```

Thus `offset` names the tick that is joint zero. At 4096 steps, the example value 1.570796 is tick
1024: the bench servos at tick 1026 read 0.00307 rad (measured). The value 3.141593 is tick 2048,
where `calibrate_midpoint` puts the present position. After a calibration, use 3.141593. With
1.570796, a calibrated servo reads +π/2.

The single-turn check applies to `pos` joints only. The driver maps each limit with
`lround((sign x q + offset) x encoder_steps / 2 pi)`, and the result must be in
`[0, encoder_steps - 1]`. `inverted` swaps the two ends. With one finite limit or none, the driver
gives a WARN and checks that limit, or joint zero.

`inverted` flips exactly the `position`, `velocity` and `load` states and both commands. The
ST3025 never sets the current sign bit (bit 15): 0 negative samples in 3343 messages, wheels
driven both ways. A negated magnitude makes every mirrored joint read negative. Thus `current`,
`effort` and `torque` stay magnitudes, and on an inverted joint they do not agree in sign with
`velocity`. Read `load` (sign on bit 10) for the direction of work.

## Command interfaces and limits

| command interface | allowed on | unit | driver reads its `min`/`max` | driver bound | framework bound (only with `enforce_command_limits: true`) |
|---|---|---|---|---|---|
| `position` | `pos` joints (necessary there) | rad, joint frame | **yes**, both optional. A value that is not a number, or `min` above `max`, is a FATAL. | Clamps every goal to `[min, max]`. INFO `joint '<j>' position commands clamped to [<min>, <max>] rad`. | `[max(min, URDF lower), min(max, URDF upper)]` for `revolute` and `prismatic`. A `continuous` joint has no URDF position limits. |
| `velocity` | both types, the only command interface of a `vel` joint | rad/s, joint frame | **no** | `vel` joint: clamped to ±`max_speed`, then rounded to whole steps/s (1 step/s = 0.00153 rad/s). `pos` joint: used only in a cycle with no measurement. | symmetric: `min(abs(min), max, URDF <limit velocity>)` |
| any other (`effort`, `acceleration`, ...) | refused | | | FATAL `a joint is using a command interface that isn't position or velocity` | |
| none | refused | | | FATAL `a joint does not have a command interfaces` | |

- **Position joints.** The driver sets the goal speed itself ([Goal speed](design.md#goal-speed)).
  Thus a raw step through a controller that does not interpolate runs at up to `max_speed`, and
  the servo ramp shapes the move. On the bench, a 0.5 rad step in 10 ms peaked at 3.14 rad/s and
  settled in 0.30 s. For slow moves, use a trajectory controller or a lower `max_speed`.
- **Wheels.** Four bench commands agree with this rule: the ST3025 rounds the goal speed toward
  zero to a multiple of 50 steps/s (0.0767 rad/s). By this rule, a command below 0.0767 rad/s does
  not turn the wheel (worked out, not measured).
- **Out-of-limit hold.** The driver can find a `pos` joint more than one encoder step outside its
  limits at activation, or at a shutdown or error park. It then holds the joint there until a
  command inside the limits comes. At activation it logs:
  `joint '<j>' starts at <x> rad, outside its limits [<min>, <max>]; holding it there until it is commanded to a position inside them`

ros2_control merges the URDF `<limit>` and the command interface `min`/`max`, and the more
restrictive value wins. Keep them equal. The example `<limit>` values are caps, not measurements:
`velocity` 9.2 rad/s (below the default `max_speed`) and `effort` 1.0 N m. The framework ignores
`lower` and `upper` on a continuous joint. But keep a `<limit>` on each wheel, because
`joint_trajectory_controller` from ros2_controllers before 4.42.0 crashes at load without it.

`enforce_command_limits` is a controller-manager parameter, `false` by default in ros2_control
4.48.0 and in the example. The controller manager reads it when the hardware loads, so a change at
run time has no effect. Off, the driver clamp and `max_speed` are the only bounds. On, the
framework clamps first with the merged limits. `<limits enable="false"/>` turns off only the
framework limits, not the driver clamp.

**CAUTION:** Turn `enforce_command_limits` on only if your servos start inside their range. With
it on, a `pos` joint measured more than 0.0087 rad outside its limits makes
`JointSaturationLimiter` throw when a controller claims the joint. Nothing catches the exception,
so expect `ros2_control_node` to abort (from the ros2_control 4.48.0 source, not measured).

A command interface declared two times gives `already existing key` from the resource manager.
Commands start as NaN or as `initial_value`. `on_activate` sets each position command to the
measured position (0.0 for an absent servo) and each velocity command to 0.0.

## State interfaces

A joint can declare any subset of the nine state interfaces, in any order, or none. The driver
exports them in description order (`ros2 control list_hardware_components -v`), but the topic
order comes from `joint_state_broadcaster` and is different on the bench. Read interfaces by name,
never by index. `/joint_states` carries only `position`, `velocity` and `effort`. The others go
only to `/dynamic_joint_states`.

<!-- reference:state-interfaces:begin -->

| name | unit | frame / sign | source and scale | notes |
|---|---|---|---|---|
| `position` | rad | joint frame, flipped by `inverted` | reg 56, `sign x (ticks x 2 pi / encoder_steps - offset)` | unbounded when `unwrap` is on |
| `velocity` | rad/s | joint frame, flipped | reg 58, `sign x speed_steps x 2 pi / encoder_steps` | on the bench every reading was a multiple of 50 steps/s (0.0767 rad/s). A stopped servo can read 0.0767 rad/s in one message. |
| `effort` | N m | a **magnitude**, not flipped | `current x torque_constant_nm_per_a` | **approximate, scale unverified** (the current scale is not verified) |
| `current` | A | a magnitude, not flipped | reg 69, `counts x current_per_count_a` | **approximate, scale unverified**. The firmware never sets the sign bit. |
| `voltage` | V | none | reg 62, `raw x 0.1` | **approximate, scale unverified**: the 0.1 V/count scale is from a third-party table. The bench reads 12.3 V on a 12 V supply. |
| `temperature` | deg C | none | reg 63, raw | |
| `load` | fraction of full PWM | joint frame, flipped | reg 60, `raw / 1000`, sign on bit 10 | about -1.023..+1.023, not clamped. The only member of the effort family with a direction. |
| `status` | bitmask 0..255 | none | status byte of the reply frame | NaN while there is no reply ([Status bits](#status-bits)) |
| `torque` | kgf cm (the log text says kg cm) | a magnitude | `current x torque_constant_nm_per_a / 0.0980665` | **deprecated** alias of `effort`, **approximate, scale unverified**. At the default constant this is `current x 9.0`. |

<!-- reference:state-interfaces:end -->

- `torque` still reports kgf cm, and gives one WARN per joint at load. Declare `effort` instead.
- The driver does not serve `moving` (register 66).
- At load, the driver refuses an unknown name, a `data_type` other than `double` and a repeated
  name. The resource manager refuses a `<gpio>` or `<sensor>` in the same block, because the
  driver exports joint interfaces only.
- All nine start as NaN. For `status`, NaN means no reply yet, and 0.0 means a reply with no
  fault. An `initial_value` shows only until the driver first publishes the joint.
- For an absent or dropped servo, the `position` of a `pos` joint mirrors its command. A `vel`
  joint holds its last measured position if it unwraps, else its activation value, and 0.0 if the
  servo never answered. `status` reads NaN, and the other states read 0.0. A failed read keeps the
  last good sample.

## Status bits

`status` is the status byte as a bitmask from 0.0 to 255.0, or NaN with no reply. Make sure that
it is finite before a cast: `if (std::isfinite(v)) { bits = static_cast<uint8_t>(v); }`.
[Servo faults](operation.md#servo-faults) tells what the driver does on a fault.

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

The names come from the FEETECH Python SDK
([FTServo_Python](https://github.com/ftservo/FTServo_Python),
`scservo_sdk/protocol_packet_handler.py`, retrieved 2026-09-16), which decodes the same byte. An
STS3215 memory table gives other names for bits 1 and 4. No test on the ST3025 confirms either
source, or that the byte is register 65, and the motorless tests prove only the decode path. Thus
the driver publishes the raw byte and uses the names only in log text. If you confirm a bit, open
an issue ([Contributing](../README.md#contributing)). Then change `kStatusBitNames` in
`src/units.cpp`, this table and the source citation together.

## Multi-turn wheel position

`unwrap` is on by default for `vel` joints and refused on `pos` joints. It makes `position` a
continuous multi-turn count, so odometry does not jump once per revolution. A register wrap from
4095 to 12 moves the count forward 13 ticks. The first sample of a new count is the register value.

The driver reduces each change between two accepted samples into `[-steps/2, +steps/2)` and adds
it to the count. Thus the wheel must turn less than half a revolution between samples. For
commanded speed, the margin is 34 times at the default `max_speed` (10 ms cycle), and 6.25 times
at the 50.3 rad/s cap. A longer gap aliases silently by whole revolutions, for example +3000
ticks read as -1096. The driver logs a WARN ("may be off by whole revolutions") when `max_speed` x
gap is π or more.

The driver KEEPS the count across a hardware deactivate and activate, because the controllers stay
active and nothing tells them. A restart of the count moves the `diff_drive_controller` odometry
0.314 m for each lost turn (example radius). ros2_controllers 4.42.1 has no reset service for it.
A wheel turned by hand more than half a turn while inactive aliases in the same way, with no WARN.

The count restarts only when `read()` drops the servo and when the port opens or closes. A dropped
servo that answers again reports its register value
([Position unwrapper](design.md#position-unwrapper)).

## Other ros2_control settings

| element / attribute | read by | effect on this driver |
|---|---|---|
| `<ros2_control name="..." type="system">` and `<plugin>` | framework | `name` is the component and logger name (`<cm logger>.hardware_component.system.<name>`). The plugin is a `SystemInterface`. |
| `rw_rate="N"` and `is_async="true"` on `<ros2_control>` | framework | Rate and thread of `read()` and `write()`. The driver does not read them ([rw_rate and is_async](bus-timing.md#rw_rate-and-is_async)). |
| an attribute that the framework does not know | nobody | The parser drops it silently, so a typo has no visible effect. |
| `min`, `max` and `<limits enable="false"/>` of a command interface | driver (position `min`/`max`), framework | [Command interfaces and limits](#command-interfaces-and-limits). |
| `<param name="initial_value">` in an interface | framework | [State interfaces](#state-interfaces). |
| `data_type` of `<state_interface>` | driver | Must be `double` (the default). |
| URDF `<joint><limit>` | framework only | Merged with the command interface `min`/`max` when `enforce_command_limits` is on. |
| xacro arguments `port`, `baudrate`, `use_mock_hardware`, launch argument `gui` | the example only | [Launch arguments](setup.md#launch-arguments). |

## Several buses

Use one `<ros2_control>` block with its own `port` for each adapter. Ids must be unique in a block,
but blocks on different ports can use the same id. The [port lock](design.md#port-lock), not an id
check, refuses a second block on the same port at configure.
