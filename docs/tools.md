# Command-line tools

The package has four tools: [`scan`](#scan), [`set_id`](#set_id),
[`calibrate_midpoint`](#calibrate_midpoint) and [`factory_reset`](#factory_reset). Each tool takes
its parameters after `--ros-args` as `-p name:=value` ([Tool internals](design.md#tool-internals)).

WARNING: Stop the controller manager before you use a tool. Also close each other program that
uses the port, for example `screen` or `minicom`, because it can write to the bus during a change.

Each tool takes the same exclusive port lock as the driver. If another process holds the port or
its lock, the tool sends nothing and exits with code 1. The message names each holder as
`pid <N> <name>` (the name cut to 15 characters), or says `no holder visible in /proc` for a holder
of another user ([Log messages](operation.md#log-messages)). A `/dev/serial/by-id/` path works. A
program that opened the port without the lock does not stop a tool: the tool warns and continues.

Results go to stdout. Other lines go to stderr with the tool name first, for example `scan: `.
The usage line, rcl argument errors and the vendored `serial speed <N>` and `perror` lines have
no prefix. Before its first write, a tool prints an `about to ...` line. It tells where `scan` can
find the servo if the run stops or fails.

## Tool parameters

<!-- reference:tool-parameters:begin -->

| tool | parameters | necessary | ranges |
|---|---|---|---|
| `scan` | `port`, `baudrate` | none | `port`: a device path. `baudrate`: one of the seven rates |
| `set_id` | `start_id`, `new_id`, `port`, `baudrate` | `start_id` and `new_id` | `start_id` 0..253. `new_id` 1..253, different from `start_id`, and free on the bus |
| `calibrate_midpoint` | `id`, `port`, `baudrate` | `id` | `id` 0..253 |
| `factory_reset` | `id`, `port`, `baudrate` | `id` | `id` 0..253 |

<!-- reference:tool-parameters:end -->

`port` is a string that is not empty, with the default `/dev/ttyACM0`. `baudrate` is an integer
with the default `1000000`. It must be the current rate of the servos, and one of the seven rates in
[Hardware parameters](configuration.md#hardware-parameters). The ids are integers with no default.

These errors give exit code 64 before the port opens. The tool prints all of them, the overrides for
another node first and then by parameter name. Then it prints the usage line.

- A wrong type (`-p port:=0` is an integer, `-p baudrate:=1e6` a double), or a value out of range.
  The tool checks an id as a 64-bit integer, so 300 does not become 44.
- An unknown name, or the 0.1.0 names `device_port` and `baud_rate`, also next to the new names.
- An override for another node, which rclcpp drops with no message. The tools accept no prefix,
  `/**`, their node name, and `/*` only with no namespace (rclcpp matches `/*` to one path
  segment). The same rules apply to `--params-file` keys.
- `start_id` equal to `new_id`.

The tool reports a positional argument, or an argument that rcl cannot parse (for example
`-p port`), alone.

The tools send up to 3 pings for each id. The read timeout, also the ack window of SRAM writes, is
`max(5, ceil(510000 / baudrate) + 2)` ms. This gives 5 ms at 1000000 and 500000 baud, 16 ms at
38400 and 56 ms at 9600. EEPROM writes and the RESET wait 100 ms for the ack, then up to 500 ms for
the check.

## scan

```bash
ros2 run waveshare_servos scan --ros-args -p port:=/dev/ttyACM0
```

`scan` pings each id from 0 to 253 and reads the registers of each servo that answers. It writes
nothing and takes about 4 s at 1000000 baud ([Find your servos](setup.md#find-your-servos)). Use
its `id` and `type` columns in [Joint parameters](configuration.md#joint-parameters).

The bench check and the tests parse stdout, so nothing else goes there. It holds a header, one row
of 11 fields for each id that answered, in id order, and the footer
`found <N> servo(s) on <port> at <baud> baud: ids ...` or `found no servo ...`. `type` is `pos` for
mode 0, `vel` for mode 1 and `-` otherwise, and `offset` is registers 31-32 with the sign on bit 11.
`?` replaces a value that `scan` cannot read and each value from it, but never the id.

On stderr, `scan` writes an `id <n>: ...` line for each anomaly and each note, then a hint. An
anomaly gives exit code 7, for example [two servos on one id](#two-servos-on-one-id), a reply from
another id or an unreadable register. A note does not change the exit code. Notes are for a status
byte other than 0, for mode 2 or more (no driver support), and for id 0:
`scan: id 0: the hardware interface accepts ids 1..253; give this servo another id with set_id before putting it in a URDF`.
If no servo answers, `scan` exits with code 3 after
`check servo power (USB does not power the servos), the wiring, and the baud rate: ...`.

## Two servos on one id

CAUTION: Connect only the servo that you change. Switch off the servo supply before you connect it.
Factory-new servos all have id 1, and two servos on one id answer on top of each other.

Before their first write, `set_id`, `calibrate_midpoint` and `factory_reset` ping the id up to 3
times. They need one clean reply, and no reply to each earlier ping. A doubled reply (more bytes
after a valid reply), a garbled reply or a reply from another id shows two servos. This applies
also before a clean reply and in the register reads that follow. The tool then refuses at once with
exit code 4 and writes nothing.

The message is `more than one servo may answer at id <id> ... Nothing was written.` A RESET to a
shared id resets both servos. `scan` records the same evidence as an anomaly (exit code 7), for
example `two servos may share this id`, and reads no register from an id with no clean reply.

## set_id

`set_id` writes a new id into the EEPROM of one servo. Connect only that servo.

```bash
ros2 run waveshare_servos set_id --ros-args -p start_id:=<old> -p new_id:=<new>
```

If a servo answers at `new_id`, `set_id` refuses with exit code 4 and writes nothing. After the
write, it checks that the servo answers at `new_id` and not at `start_id`, and that no other
register in 3-39 changed. To make sure that the servo keeps the id, power-cycle it and run `scan`.

## calibrate_midpoint

WARNING: Support the arm before you run `calibrate_midpoint`. The tool switches the torque off and
leaves it off, so a loaded joint can fall. The driver switches the torque on at activation.

`calibrate_midpoint` makes the present position of a position servo (mode 0) read tick 2048
(π rad). It stores the change as the EEPROM offset and does not move the servo. It refuses a servo
in another mode with exit code 4.

1. Move the joint to the angle that you want as its centre, for example with a command.
2. Stop the launch with Ctrl-C. The driver leaves the torque on.
3. Hold the joint at that angle and run
   `ros2 run waveshare_servos calibrate_midpoint --ros-args -p id:=<id>`.
4. Before you command the joint again, set its `offset` to 3.141593
   ([Joint frame](configuration.md#joint-frame)). Then the midpoint is joint zero.

The tool switches the torque off first. With torque on, the servo moves to its old goal in the new
frame when the offset changes. The tool waits for two reads 100 ms apart that differ by 1 tick or
less. After 2 s, it refuses with exit code 4 (`servo <id> is still moving (<a> -> <b>); ...`). Then
it opens the EEPROM lock, writes 128 to register 40 to start the calibration, and closes the lock.
The position must read 2048 (within 3 ticks), with no other change in registers 3-39.

The offset (registers 31-32, sign on bit 11) changes by the settled position minus 2048. Thus
offset 0 at position 1026 becomes -1022 (raw `0x0bfe`). The bench check measured this sign on the
ST3025 (`offset_sign` in the `detail` line). The goal register keeps its old value, so "Torque
before the first goal" ([Known issues](operation.md#known-issues)) applies at the next activation.

## factory_reset

`factory_reset` sets the EEPROM values of one servo back to the factory values, except the id. It
sends the protocol RESET instruction 0x06 (frame `FF FF <id> 02 06 <checksum>`, not in
`include/INST.h`) to that one id. It cannot find a servo that answers at no id, so find the servo
with `scan` first. Before you run it:

- WARNING: Support a loaded arm. The torque goes off first and stays off, so the arm can fall.
- Save the output. Its list of changed registers, with old and new values, is the only record of
  the old offset, angle limits, protection limits and gains.
- Calibrate the joint again before you command it ([`calibrate_midpoint`](#calibrate_midpoint),
  `offset` 3.141593), because its `offset` and limits point to other angles. Each limit that you
  tightened is back at its factory value.

```bash
ros2 run waveshare_servos factory_reset --ros-args -p id:=<id>
```

On one ST3025 (model 6410, firmware 3.20), a RESET set registers 6-39 to the factory values and
kept the id. The test changed registers 7, 9, 11, 13, 21, 37 and 39 first, and all came back. The
baud register went to 0 (1000000 baud), also for a RESET at another rate. The offset and the mode
went to 0, and the driver sets mode 1 again for a `vel` joint at configure. The values stayed after
a power cycle, also with the lock closed, because RESET is not a WRITE. The torque and goal read 0
and the lock 1, so "Torque before the first goal" ([Known issues](operation.md#known-issues))
applies at the next activation.

The tool switches the torque off before the RESET, because a servo that holds a position while its
offset changes can jump. If the torque stays on, it sends no RESET (exit code 5). The ack comes at
the old rate, about 25 ms later. From another rate, the tool then changes the line to 1000000 baud
and keeps the port and the lock (exit code 6 if this fails). For up to 500 ms, it reads registers
3-39, because a read can identify a late ack. A firmware without RESET stays at the old rate, so
the tool also reads there if the servo is silent.

Exit code 0 needs the baud register, the offset and the mode at 0, the same id and model, torque
off and the lock closed. The tool writes torque 0 and lock 1 if they read otherwise. If one of the
three values is not 0 and registers 3-39 read as before, the exit code is 5. Other results after
the RESET give exit code 6. After a reset, `scan` and the driver need `baudrate` 1000000.

## Exit codes

All four tools use these exit codes:

<!-- reference:exit-codes:begin -->

- `0`: done and verified.
- `1`: another process holds the port. The tool sent nothing.
- `2`: the tool cannot open the port, or the port disappeared before the tool wrote to a servo.
- `3`: the servo did not answer (`scan`: no servo answered).
- `4`: the tool refused before it wrote to EEPROM. If `calibrate_midpoint` had switched the torque
  off, it says so.
- `5`: a write, or the reset of `factory_reset`, had no effect. The EEPROM of the servo is
  unchanged.
- `6`: the state of the servo changed or is unknown. Read the message and run `scan`.
- `7` (scan only): a servo gave an unusual reply, for example two servos on one id, or registers
  that `scan` cannot read.
- `64`: incorrect parameters or arguments. The tool did not open the port.
- `70`: an internal error. Report it with the message ([Contributing](../README.md#contributing)).
- `130`: a signal stopped the tool. `scan` stops between two ids and prints what it found.
  `set_id` stops only before its first write. `calibrate_midpoint` stops only before it opens the
  EEPROM lock, and `factory_reset` only before it sends the reset. These two say so if they had
  switched the torque off. A later signal waits until the tool has made and checked the change and
  closed the EEPROM lock.

<!-- reference:exit-codes:end -->
