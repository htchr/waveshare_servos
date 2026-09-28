# Driver design

`WaveshareServos`, the ros2_control plugin, calls `ServoBus`, the only code that uses the bus.
`ServoBus` calls the vendored SMS_STS packet layer, which the package does not edit
([The rule](../THIRD_PARTY.md#the-rule-these-files-are-not-edited)). The numbers come from the
[reference bench](setup.md#the-reference-bench), unless the text says otherwise.

## Code layout

| Part | Files | Notes |
| --- | --- | --- |
| `waveshare_servos` (shared) | `src/waveshare_servos.cpp`, `src/units.cpp` | The plugin. Links `servo_bus` PRIVATE. `units.hpp` and `units.cpp` have no ROS. |
| `servo_bus` (static) | `src/servo_bus.cpp` | Vendored layer plus port lock. Linted, all package warnings. Not installed. |
| `scservo` (static) | the 14 vendored files | Built with `-Wno-vla`. The linters skip it. Not installed. |
| `servo_tools`, `tool_cli` (static) | `src/servo_tools.cpp`, `src/tool_params.cpp`, `src/tool_main.cpp` | ROS-free tool logic (links `servo_bus` PUBLIC), parameter layer, `run_tool()`. Not installed. |
| header-only | `include/position_unwrapper.hpp`, `src/param_parsing.hpp`, `src/driver_defaults.hpp` | The unwrapper has no ROS. |
| installed headers | `include/` | Not a supported API ([Versioning](development.md#versioning)). |
| `hil_*` programs | `test/hil/` | The [bench helpers](bench-check.md#bench-helpers). Test builds only. |

## Port lock

`ServoBus` adds exclusive access to the tty, the real `errno` when it cannot get the port, and a
`close()` that releases the device. `TIOCEXCL` makes every later unprivileged `open()` fail with
`EBUSY` (`LOCK_OPEN_FAILED`). Root gets through it, but then `flock(LOCK_EX | LOCK_NB)` fails with
`EWOULDBLOCK` (`LOCK_FAILED`). Neither stops a process that opened the port first, so
`on_configure` finds it with `port_holder_pids()` and logs a WARN.

`open()` locks its own descriptor before `begin()`, because `perror()` in `begin()` loses `errno`.
Also, a process that loses the lock never sets `TIOCEXCL`. `close()` clears `TIOCEXCL` (`TIOCNXCL`)
before `end()`, because the flag belongs to the tty and a later `open()` otherwise gets `EBUSY`.

## Servo registers

"Table" is the Feetech STS3215 memory table, and "measured" means measured on the bench ST3025.

| Register | Name | Memory | Notes |
| --- | --- | --- | --- |
| 0-4 | Version and model | EEPROM, read-only | 0-1 firmware, 3-4 model. Bench: firmware 3.20, model 6410 (measured). |
| 5 | Id | EEPROM | 0-253. Id 254 (`0xFE`) is the broadcast id. |
| 6 | Baud rate | EEPROM | 0-7 = 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400. Factory value 0. |
| 31-32 | Position offset | EEPROM | Sign-magnitude, sign on bit 11 (measured). |
| 33 | Mode | EEPROM | 0 position, 1 wheel. A write clears register 40 (measured). |
| 40 | Torque on or off | SRAM | 0 off, 1 on, 0 after power-up. A write of 128 calibrates the midpoint, then 40 reads 0 (measured). |
| 41 | Acceleration | SRAM | No unlock needed. A power cycle clears it. Writes to 40 and 46 and 5 s of idle time do not (measured). |
| 42-47 | Goal position, time, speed | SRAM | Goal 0 after power-up (measured). The package always writes goal time 0. |
| 55 | EEPROM write lock | SRAM | 0 open, 1 closed, 1 after power-up (measured). |

An EEPROM WRITE with the lock closed takes effect at once, and a power cycle discards it (table).
Thus only a read-back of register 55 proves an unlock or a lock, and the tools read it back after
each. `set_mode()` writes register 33 (unlock, write, lock, read back) only for a different mode,
because EEPROM wears out. It must run before `EnableTorque`, and it does not examine the lock
results ("EEPROM lock results not checked for the mode write" in
[Known issues](operation.md#known-issues)). For a RESET, see
[factory_reset](tools.md#factory_reset).

## Signed values

Signed words are sign-magnitude, not two's complement: bit 15 is the direction and bits 0-14 the
magnitude. This applies to the goal position, wheel speed, present position, speed and current. Thus
the goal -100 goes out as `64 80`, not `9c ff`. `send_commands()` clamps goals to [-32767, 32767]
ticks before the cast to `s16`. At -32768, `sign_magnitude_encode()` gives `ff ff`, but
`SMS_STS::SyncWritePosEx` overflows in `s16`. The clamp keeps the frames of the driver the same as
the frames of the library.

The load has its sign on bit 10. The decoder keeps bits 11-15 in the magnitude, as
`SMS_STS::ReadLoad` does, so `0xffff` gives -64511, not -1023, and `load` agrees with the library. A
real servo reports a magnitude of 0-1000 and never sets bits 11-15 (at rest: `0x0410`, that is -16).

## Sync write

A sync-write frame is `FF FF FE len 83 start-register record-length`, then the id and record of each
servo, then a checksum. It goes to the broadcast id, so it costs no round trip, and a wheel gets no
addressed write in a normal cycle. The position record (register 41, 7 bytes) holds the
acceleration, goal position, goal time 0 and goal speed: 30, 1000 and 500 give
`1e e8 03 00 00 f4 01`. The wheel record (register 46, 2 bytes) holds the goal speed.

`writeSCS` has no bounds check on `txBuf[255]`. Thus `ServoBus` sends at most
(255 - 8) / (record + 1) records per frame: 30 position or 82 speed records. `static_assert`s check
that each limit fits and one more record does not. Frames need no gap (0 mismatches in 1500 pairs).
An empty group sends nothing. A refusal (`NOT_OPEN`, or `INVALID_ID` for the whole group) also sends
nothing, so a wheel keeps its speed, and the driver logs `sync write refused: ...` (ERROR).

## Sync read

The request is `FF FF FE (n+4) 82 38 0F <ids> ~sum` (15 bytes from register 56). Each servo answers
with a 21-byte frame, `FF FF id 11 status data[15] ~sum`, back to back in request order (measured).

`parse_sync_read_burst()` walks forward only through the id list. An id that it passes did not
answer. A frame whose id is not at or after the cursor is stale, foreign or a duplicate. The
vendored receiver has no such check: at 1 ms it put 71 frames in the wrong slot in 3000 reads. A
frame must also have the header, the length byte `0x11` and a checksum that `ServoBus` calculates
without the library (0 bad in 5000 reads). After a rejected frame, the walk moves one byte, because
the rejected bytes can hide the real header, and then continues.

A short burst is normal with a silent servo (the others decoded correctly in 1200 of 1200 reads).
It fails one slot per missing frame, and one transaction in the
[bus totals](bus-timing.md#bus-totals-line). A corrupt frame counts in `bad_frames` and
`missing_frames`. The driver does not log these counters. Only the tests read them through
`sync_read_stats()`.

## Receive buffer

`ServoBus` owns the receive buffer and never calls `syncReadBegin()` or `syncReadEnd()`, so the
real-time path allocates nothing. The buffer holds 30 frames of 21 bytes plus 18 bytes of slack,
because `syncReadPacketRx` can read 18 bytes past its data. The walker checks `pos + 21 <= length`
before it reads a frame, and it reads only the `length` that `readSCS` returned. The bytes after
`length` hold the previous burst. `syncReadRxBuffMax` is exactly 21 bytes for each id. More makes
each healthy cycle wait the full timeout (+18.6 ms at 20 ms), and less cuts the last frame.

## Late frames

`rFlushSCS` is only a `tcflush`, and a late frame of the last cycle has not arrived yet. Thus a late
frame can pass every check, so the driver refuses `io_timeout_ms` 1
([Transaction timeout](bus-timing.md#transaction-timeout)). Only after a short burst,
`drain_input()` discards input until a deadline of `sync_read_drain_ms` (2 ms). With the drain,
all 2000 reads at 1 ms failed and none gave old data. A full burst needs no drain (0 stale frames
in 500 at 5 ms).

## Feedback block

The feedback block is registers 56-70 (15 bytes, words little endian). Its offsets are 0-1
position, 2-3 speed, 4-5 load, 6 voltage, 7 temperature, 10 moving (not published) and 13-14
current. `FeedbackBlock` holds the raw block in the units of the `ReadX(-1)` accessors. `decode()`
in `units.hpp` does all scales, signs and `inverted`, in the operator order ticks × 2 × π / steps
of the 0.1.0 driver. `read_feedback_one()` sends `Read(id, 56, 15)`, the same bytes as
`FeedBack(id)`. Both transports use one decoder and `apply_feedback()`, so the nine values agree.

The package does not call `ReadPos`, `ReadSpeed`, `ReadLoad` or `ReadCurrent` with id -1, because
they skip the sign decode when `SMS_STS::Err` holds an earlier failure. A bus test sets `Err` to 1
to catch this. `SCS::Error` is one shared byte that `Read`, `Ping` and `Ack` all write, and the
vendored receiver does not change it for a silent id. Thus the sync path takes the status from
byte 4 of each frame, and the per-servo path reads `SCS::Error` immediately after `Read()`. A
silent slot keeps status 0, like a healthy servo, so callers must test `FeedbackBlock::valid`.

## Read cycle

On the sync path, `read()` sends one sync read for all present servos before the work on each
joint. Thus all samples come from one window of about 1.6 ms, and a fault-clear `EnableTorque`
cannot come between two reads. The `valid` flag of each slot is the only gate, because a burst that
lost one frame still carries the others. The read group (`r_ids_`, `r_js_`, `r_blocks_`) holds
every present joint in URDF order, and `r_slot_` maps a joint to its slot.

A dead id costs a sync read a full timeout in each cycle, but a sync write only its record bytes.
Thus an absent servo never goes into the read group, and a dropped servo leaves it but stays in the
write groups. Do not change this, because it changes the packets and thus the goal registers that
the bench check reads back. The activation probe ([Feedback mode](bus-timing.md#feedback-mode))
judges only the servos whose seeding read answered. It reads the full group, because a firmware can
answer one id and fail four. The seeding and park reads use `read_feedback_one()`, because a one-id
sync read gains nothing (0.755 ms against 0.750 ms).

## Wheel acceleration

A position servo gets its acceleration in each goal record, so it gets it back one cycle after a
restart. A wheel record has no acceleration. Thus `write_wheel_acceleration()` writes register 41
of a present `vel` joint at four edges only. Do not use this for positions.

1. `build_groups()`, after `set_mode()`: at configure, and when activation finds a servo again.
2. `on_activate`, after `EnableTorque`: a wheel with a power cycle while inactive lost register 41.
3. `read()`, when a joint answers after a missed read: a power cycle is longer than one period.
4. `read()`, when a status fault clears: a protection trip can reset the SRAM (not measured).

One write and its ack take 0.594 ms. The vendored `SyncWriteSpe` writes register 41 in each cycle,
so the driver saves about 0.57 ms per wheel per cycle. After a recovery, activation writes the
register at edges 1 and 2, so tests count the change, not the writes. A failed write logs one WARN
(`could not write the acceleration register of motor id ...`), and the next edge is the retry.
`SCS::Ack` returns 1 for any status byte and overwrites `SCS::Error`, so read the status first.

## Goal speed

The servo runs its own trapezoidal profile to each goal and stops there. Thus the driver sets the
goal speed so that the servo arrives when the next setpoint comes: `|goal - measured| / period`.
This term covers the new chord and the lag. The trajectory velocity is larger than the mean slope
on a segment that accelerates, so with it the servo stutters. Use the `period` of `write()`, never
1 / `update_rate`. With `rw_rate`, ros2_control 4.48.0 gives the time since the last write.

A zero period (only from the park) uses the last non-zero period, 0.01 s before the first write.
With no measured position, the driver uses the commanded velocity. Position goal speed 0 means full
speed, and made the servo jump at each trajectory end. Thus the minimum is 1. The maximum is
`max_speed`, and a speed that is too high does no harm.

## Position unwrapper

`include/position_unwrapper.hpp` is header-only integer arithmetic with no ROS
([rules for users](configuration.md#multi-turn-wheel-position)). One rule serves a register that
wraps and a firmware that counts turns. The test `a_gap_longer_than_half_a_revolution_aliases`
keeps the aliasing failure. Do not re-seed after a gap, because the error then becomes the full
travel. Do not use the speed register either. A failed read never calls `unwrap_ticks()`, so the
next change starts from the last real sample.

At a drop, `read()` sets `pos_cmds_` to `pos_states_` for a joint that unwraps. An absent joint
publishes `pos_cmds_` as its position, so the position does not jump back to the activation
position.

`note_unwrap_gap()` takes the period as the gap. After failed reads, the gap is
max(period, cycle × (failed reads + 1)), with cycle = max(last period, `io_timeout_ms`). A WARN
comes when `max_speed` × gap is π or more, for the first gap and every 200th. It says "may be",
because the speed of a silent servo is not known. A late cycle with no failed read also causes it
(the bench recorded overruns of 12-30 ms).

## Lifecycle

| Callback | What it does |
| --- | --- |
| `on_init` | Reads and checks the parameters. It does not open the port. |
| `on_configure` | Opens the port. The missing-servo gate comes before `build_groups()`, so a refusal writes no EEPROM. A failure closes the port here, because no `on_error` follows. |
| `on_activate` | Clears the bus totals and pings the missing servos again. For each present joint: `EnableTorque`, edge 2, the seeding read. Last, the probe. |
| `on_deactivate` | Calls `stop_and_park(false)`, logs the bus totals line and clears the probe result. |
| `on_error` | Logs the bus totals line, calls `park_and_close()` and returns SUCCESS. |
| `on_shutdown`, `on_cleanup` | `on_shutdown` calls `park_and_close()`. `on_cleanup` closes the port. |

`on_shutdown` and `on_error` can run on a component that was not active. Thus
`stop_and_park(true)` parks only at measured positions, also outside the limits. It sends no
position goal to a servo with no measured position or that `read()` dropped. It removes that servo
from the position group, so the port must close next. It does not prune the wheel group, so each
wheel, also a dropped one, gets the goal speed 0.

## Vendored library traps

Two traps apply when `ServoBus` calls the vendored layer. For all other defects, see
[Known defects](../THIRD_PARTY.md#known-defects-and-where-the-package-works-around-them).

- A closed bus must never get to a vendored call that reads. With `fd` -1, `FD_SET` in `readSCS`
  stops the process under `_FORTIFY_SOURCE`. Thus the calls of `ServoBus` that read examine the
  port first.
- `SCS::Ack` returns 1 for success and 0 for every failure, never -1, but `readByte` returns -1.
  Thus `write_acc()` tests `!= 0`. With `!= -1`, the WARN about an unramped wheel never comes.

## Tool internals

The four tools share `servo_tools`. [Command-line tools](tools.md) gives the behaviour users see.

### Tool structure

`servo_tools` has no ROS dependency, so `test_servo_tools` runs it on the
[fake servo bus](development.md#fake-servo-bus). Each tool executable is one line that calls
`run_tool()`. Before rclcpp starts, `run_tool()` installs handlers for SIGINT, SIGTERM, SIGHUP and
SIGQUIT that only set a flag, and ignores SIGPIPE. It reads the parameter overrides, refuses each
one that it cannot apply, and stops rclcpp before it opens the bus. Exit 64 comes before the port
opens, 70 from an exception and 130 from a signal ([Exit codes](tools.md#exit-codes)).

The tools never call `declare_parameter`. With it, a wrong type throws, an old `device_port` has
no effect, and an id cut to 8 bits can become the broadcast id 254. `tool_params` accepts `/**`, the
exact node name, and `/*` only for a node with no namespace. It adds the leading slash that rcl
omits for `-p setid:port:=...`. The `ToolParamsRclcpp` tests keep this measured rclcpp behaviour.

Each run function returns `kCannotOpen` on a closed bus before any vendored call. It reads back
every write, because an ack proves nothing. Also, a servo with register 8 (status return level) at 0
applies a WRITE but sends no ack (table). Results go to stdout and diagnostics to stderr, and the
executable adds the `<tool>: ` prefix.

### Checked transactions

Only the tools use `checked_ping`, `checked_read`, `checked_write` and `checked_reset`. `SCS::Read`
checks neither the id nor the length byte, and `SCS::Ack` refuses an ack from another id. No
vendored call sees a second reply from two servos on one id. A checked call refuses a closed bus,
id `0xFE` or `0xFF`, or a byte count outside 1-64. Otherwise it does one `readSCS` of the exact
reply length. If a byte arrived, it drains for 2 ms to count a second reply.

The call then checks the header, length byte, checksum and id, and gives a `ReplyKind`. `WRONG_ID`
applies to every instruction except WRITE: `checked_write` accepts an ack from any id (`from_id`). A
silent id costs one window, and a full scan takes about 3.8 s. Keep each window below 1000 ms,
because `readSCS` puts it all in `tv_usec`.

### Late acks

[Tool parameters](tools.md#tool-parameters) gives the ack windows (`kEepromAckMs`). A commit that
is longer than its window sends its ack into the next transaction. The ST3025 acknowledges an id
write from the old id (measured). A ping then sees a 6-byte status frame from the wrong id, and a
read sees a status frame with no data.

The write that commits is the id write, the write of 128 to register 40, or the RESET. After it, a
run accepts one late ack from the old or new id. It records `late_ack_from` and `late_ack_ms`, logs
`late ack from id N, ... it was repeated`, and repeats the transaction. A second late ack, a reply
from another id, or silence after the 500 ms check window normally gives exit 6. The bench showed
no late ack (the acks came 3-25 ms after the id write).

### Signals during an EEPROM write

A tool never stops between an EEPROM unlock and its lock, or between a RESET and its check. Before
that point, a stop gives exit 130 and a message that tells what the tool did, for example
`interrupted; nothing was written` or `interrupted; its torque is now OFF; the reset was not sent`.

The tool prints the pre-write notice (the ids to scan if the run fails), the only help after a
SIGKILL. Then it blocks the four stop signals on its thread (`pthread_sigmask`) and buffers its
diagnostics. The handlers stay, so a signal to another thread only sets the flag. The last stop
check, before the unlock (for `factory_reset`, the RESET), reads the flag and `sigpending()`. A
later signal waits until the sequence and its check are complete, and then sets `signal_deferred`.
