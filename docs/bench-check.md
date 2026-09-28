# Bench check

> **WARNING:** Use the bench check only on the [reference bench](setup.md#the-reference-bench):
> exactly four ST3025 servos at ids 1-4. Never run it on an assembled robot.

The check moves ids 1 and 2 within ±π/2 of tick 1024 and turns ids 3 and 4 as wheels. The driver
first puts a position servo at id 3 or 4 in wheel mode (an EEPROM write). The check writes EEPROM on
ids 2 and 4 on purpose (`set_id` 4 -> 253 -> 4, `calibrate_midpoint` on id 2) under a
[journal](#eeprom-journal) and restores them. It does not check the bus identity first.
`test/hil_check.sh` runs the scenarios, and `test/hil/` holds the recorder, gates and helpers.

## Run the bench check

> **WARNING:** Set `WAVESHARE_HIL=1` on the command line only. Do not `export` it or put it in a
> shell profile. If you do, every later `colcon test` in that shell moves the servos.

> **WARNING:** Keep `WAVESHARE_HIL_SOAK_S` at 750 or less, or first raise `TIMEOUT` in
> `CMakeLists.txt`. If ctest stops the run during the soak, no clean-up runs. Both wheels then
> turn at 1 rad/s until you switch off the servo supply.

```bash
WAVESHARE_HIL=1 colcon test --packages-select waveshare_servos --ctest-args -R hil_check
colcon test-result --all --verbose
```

The ctest test `hil_check` has `RUN_SERIAL`, `TIMEOUT` 2400 s and the label `hil` (`-L hil` runs
only it, `-LE hil` excludes it). It skips (exit 77) unless `WAVESHARE_HIL=1`, the port is readable
and writable, and `ros2` is on `PATH`. It also skips for root, which can ignore `TIOCEXCL`. It exits
0 if all rows are PASS, SKIP or allowed INCONCLUSIVE, 1 on a FAIL, and 130 on SIGINT or SIGTERM.
Exit 2 is an abort: an ABORTED row, a held port at exit or a failed pre-flight.

The script restarts itself once in a clean environment (`env -i`) with a unique `HIL_TAG`,
`ROS_DOMAIN_ID=77` and localhost-only discovery. It signals only tagged processes, never with
`pkill -f ros2`, `fuser -k` or a bare `kill` on `pgrep` output. It names a port holder without the
tag and stops. Each command has a timeout. Wheels 3 and 4 stop after each scenario and at exit.

| Variable | Default | Use |
|---|---|---|
| `WAVESHARE_HIL_PORT` | `/dev/ttyACM0` | The port, resolved with `readlink -f`. |
| `WAVESHARE_HIL_OUT` | `build/waveshare_servos/hil_check.d` | The [run directory](#report-and-verdicts): the report `hil_check.txt` and `hil_check.json`, and every recording. |
| `WAVESHARE_HIL_WS` | the workspace of this source tree | The package must resolve inside its `install/`, or the run stops with exit 2. |
| `WAVESHARE_HIL_SCENARIOS` | all twenty, in run order | A space-separated subset of the [scenarios](#scenarios). |
| `WAVESHARE_HIL_SOAK_S` | 600 | The H11 soak in seconds, 40 to 750 with `TIMEOUT` 2400. |
| `WAVESHARE_HIL_JOURNAL` | `~/.local/state/waveshare_servos/hil_eeprom_journal.snap` | The [EEPROM journal](#eeprom-journal). |
| `WAVESHARE_HIL_EEPROM_BASELINE` | none | A golden EEPROM snapshot. Without it, `H17.matches_baseline` is SKIP. |
| `WAVESHARE_HIL_TOOLS` | `<install prefix>/lib/waveshare_servos` | Only for older tool binaries, which ignore `port`. Refused on other ports than `/dev/ttyACM0`. |

## Scenarios

| Order | Id | What it checks | Time (s) |
|---|---|---|---|
| 1 | H1 | the shipped example launch, idle: controllers up, every servo answers, bus cost, loop rate | 36.7 |
| 2 | H1B | the shipped example, commanded: `joint1` to 0 and 0.6 rad, wheels at 2 rad/s, Ctrl-C, wheels stopped, port free | 45.7 |
| 3 | H2 | `joint1` to 0 and 0.6 rad on the bench description, goal registers read back | 19.3 |
| 4 | H3 | both wheels at 2.0 rad/s for 6 s, then a stop | 23.9 |
| 5 | H4 | wheels at +2 and -2 rad/s for 12 s: unwrapped position against velocity | 30.8 |
| 6 | H5A | an absent declared servo (id 9), `allow_missing_servos` false: the driver refuses configure | 195.1 |
| 7 | H5B | the same with `allow_missing_servos` true: the stack comes up and the wheels turn | 35.9 |
| 8 | H5C | `allow_missing_servos` false, every servo present: the stack comes up and `joint1` moves | 18.9 |
| 9 | H6 | the exclusive port lock in both directions | 402.5 |
| 10 | H7 | `inverted` on `joint2` and `joint4`: joint-frame values, mirrored registers, `load` sign, unsigned `effort` | 45.8 |
| 11 | H8 | per-joint `max_speed` and `max_accel`: registers and the speeds they give | 84.0 |
| 12 | H9 | every state interface, a phantom joint (id 9), an inactive-active cycle of the component | 85.8 |
| 13 | H10 | shutdown on SIGINT and SIGTERM: exit 0, deactivate before shutdown, wheels stopped, port free | 45.0 |
| 14 | H12 | `enforce_command_limits: true`: the limiter, not the driver, clamps at the URDF limits | 65.5 |
| 15 | H13 | `scan`, read-only, against a separate read of the same registers | 15.9 |
| 16 | H14 | `scan`, `set_id` and `calibrate_midpoint` refuse a held port. The writers refuse bad arguments and a silent id, and `set_id` a taken id. They write nothing. | 57.1 |
| 17 | H15 | EEPROM writer: `set_id` 4 -> 253 -> 4 | 22.9 |
| 18 | H16 | EEPROM writer: `calibrate_midpoint` on id 2 (ABORTED if it rests within 100 ticks of 2048), and its refusal of wheel id 3 (exit 4) | 16.2 |
| 19 | H11 | the soak: the 100 Hz cycle with wheels at 1.0 rad/s, and the failure rate from the `bus totals:` line (last 20 s recorded) | 617.8 |
| 20 | H17 | the EEPROM against the start of the run and against the baseline | 4.4 |

A full run with the default soak takes 1873-1883 s of the 2400 s limit. Each soak second above 600
adds one second, so a soak of 750 s leaves a margin of more than 350 s. H13-H16 run before H11, so
their journal closes long before a ctest timeout, which skips the `EXIT` trap. H17 is last: it
compares the EEPROM with the start of the run. The report lists rows in numeric order.

### Example stack (H1 and H1B)

H1 and H1B run `example.launch.py` with `port:=$PORT gui:=false`. The others use `bench.urdf.xacro`.
H1 stays idle, because its cost bounds come from runs without motion. H1B moves the arm first, and
then the wheels only if the component is still active (else `H1B.arm_survived` fails).
`H1B.diff_drive` fails if `diff_drive_controller` is active or not loaded. `H1B.sequence`, not
`H1B.exit`, is the driver row.

### Component cycle (H9)

After the inactive-active cycle, H9 moves the arm, because `active` alone does not prove the write
path. Only `H9.g1c.joint3` and `.joint4` catch a lost multi-turn count, and only after the cycle, so
`cycle_recorded` needs 100 later samples. `status_not_ping` catches a Ping result as status. The
`H9.cost` rate term uses recorder stamps, so one gap can fail it. A gate on `periodicity.average`
from `/diagnostics` does not have this problem.

### Soak (H11)

H11 runs for `WAVESHARE_HIL_SOAK_S` seconds. SIGTERM, never SIGKILL, stops the stack, so the driver
logs its [bus totals line](bus-timing.md#bus-totals-line). `H11.fail_rate` accepts 50 per million
(rule of three on 0 in 60 000) and a `worst consecutive` of 2. Below 30 000 transactions it is
SKIP. `H11.wheels_turning` needs both wheels at |v| ≥ 0.5 rad/s, as the start command can have no
effect.

### Command limits (H12)

H12 proves that the `JointSaturationLimiter`, not the driver, clamps at the URDF `<limit>`. The
driver never reads `<limit>`, so H12 makes only `<limit>` tighter, and `bench_limits.yaml` turns on
the limiters. The arm gets ±0.8 rad (ceiling ±1.570796 rad, command 1.2 rad), and the wheels 2.0
rad/s (ceiling 9.2038847 rad/s, command 8.0 rad/s). `H12.stimulus` and an import check guard them.

An arm more than 0.0087 rad outside `<limit>` makes the limiter throw, and `ros2_control_node`
aborts (from the ros2_control 4.48.0 source, not measured). So H12 first parks the arm at 0.0 with
no limiter, and gates a rest within 0.05 rad. If the park stack does not come up, H12 records
ABORTED and returns 0. The limited stack gets SIGKILL (exit 137, not gated). The self-test checks
that `bench_limits.yaml` differs from `bench.yaml` in one line. H12 does not cover the `joint2`
clamp or a legal command.

### Tool scenarios (H13 to H17)

- H13: `scan` agrees field by field with [hil_eeprom](#hil_eeprom). `device_port` or a positional
  port exits 64. A time outside 3.375-12 s shows lost retries or a long timeout.
- H14: `scan`, `set_id` and `calibrate_midpoint` refuse a port that the controller manager
  (exit 1) or a flock-only probe holds. `via_servobus` catches a raw `begin()` (`serial speed`).
  Bad arguments exit 64, a taken id 4, a silent id 3. H14 tells the tools to write only to ids 200
  and 300 (= 44), which no servo answers.
- H15 and H16: `hil_eeprom` reads the bench after the tool. H16 needs position within 3 ticks of
  2048, the offset (31-32) moved by position_before - 2048, no other change, lock 1 and torque 0.
- H17: the EEPROM equals the pre-flight snapshot, and no journal remains. SRAM 40 and 55 are a NOTE.

## Report and verdicts

FAIL means that the code did the wrong thing, and ABORTED that the bench cannot ask the question. A
NOTE is never a gate. INCONCLUSIVE is valid only for `H7.load_sign` and `H8.accel_effect`, where the
bench cannot always supply the stimulus. Elsewhere it becomes FAIL. A new valid key is a reviewed
design change. Nothing turns a FAIL into another verdict.

The motion gates check three invariants. No position step is over half a revolution. The 0.5 s slope
follows the velocity, and the travel equals its integral. The gates compare the command times of
the recorder (`time.time()`) with the header stamps of the messages. Thus the stack must use the
real clock, not simulated time.

`hil_gates.py --self-test` runs all gates and checkers on synthetic data with injected defects.
`hil_gates.py --run-dir DIR` writes `hil_check.json` (exit 0, 1 on a FAIL, 2 on an abort). The run
directory also holds `run.log`, `aborted.txt`, `roslog/` and, per scenario, `facts.json`,
recordings and `log.txt`.

## Gate thresholds

| Gate | Bound | Basis |
|---|---|---|
| `g1a`, `g1b`, `g1c` | step ≤ π (and max(\|v\|) × dt < π), excess ≤ 12 ticks, no 2π reduction | worst excess 5.5 ticks, `g1c` exact |
| `g2` | 0.50 s slope within 0.060 rad/s + 2 % of \|v\| | ≥ 25 samples, velocity spread ≤ 0.30 rad/s |
| `g3` | travel within 2 % of the velocity integral + 0.05 rad | ≥ 2 revolutions |
| stale run | 3 identical samples at \|v\| > 0.05 rad/s | left out of `g1b` and `g2` |
| resting drift | wheel 8 ticks, position servo 2 ticks | worst wheel rest 3 ticks, the smallest command moved 311 ticks |
| `H8.accel_effect` | t90 at acc 150 ≤ 0.70 s, at acc 10 ≥ 1.2 s, ratio ≥ 2.5 | worst 0.4996 s, ratios 3.92-4.33. A ratio near 1.0 is INCONCLUSIVE. t90 is coarse: to tighten, use a velocity step or slope metric. |
| cycle time | read mean < 2.00 ms, max < 3.60 ms, write mean < 0.35 ms, max < 1.50 ms | means 1.645 and 0.0136 ms, maxima gated in H1 and H9 only |
| H12 clamps | ±0.010 rad, ±0.060 rad/s | settle error + limiter slack |

The bounds are for four servos at 100 Hz and 1 Mbaud, not a promise for another robot.

## EEPROM journal

> **WARNING:** Do not rebuild the [devcontainer](setup.md#devcontainer) while a journal exists. A
> rebuild deletes it and the snapshots under `build/`. Restore and remove it first.

H13-H16 run under the journal, the only protection against SIGKILL or a ctest timeout (the `EXIT`
trap restores after other signals). `guard_begin` aborts the run if a journal exists. Else it
snapshots ids 1-4, writes `$JOURNAL.port` and the journal, and reads it back. `guard_end` compares a
new snapshot and restores a difference. If one remains, it keeps the journal and aborts the run. The
port file limits a journal to its own bench, also after a re-plug renumbers the adapter.

### Recover an interrupted run

A run killed in H15 or H16 can leave servo 4 at id 253 or servo 2 with a changed offset. A new run
on the same port restores registers 5, 31, 32, 33, 40 and 55 and removes the journal. It stops with
exit 2 if the port differs or another process holds it, or other registers differ.

To recover manually, do these steps from the workspace root, only on the bench of the journal:

1. Set `J` to the journal and `P` to its port. If the adapter came back under a different name,
   set `P` to that name.

   ```bash
   J=~/.local/state/waveshare_servos/hil_eeprom_journal.snap
   P=$(cat "$J.port")
   ```

2. Do a scan on `P`. If the scan does not show ids 1, 2, 3 and 4 (or 253 in place of 4), stop.

   ```bash
   ros2 run waveshare_servos scan --ros-args -p port:="$P"
   ```

3. Restore the journal. The restore writes the EEPROM. Then it switches torque on, as the journal
   recorded it, and holds each servo where it is.

   ```bash
   build/waveshare_servos/hil_eeprom --port "$P" restore --from "$J" --allow-regs 5,31,32,33,40,55
   ```

4. If the restore exits with status 0, remove the journal. If not, keep the journal.

   ```bash
   rm "$J" "$J.port"
   ```

A good restore prints `restored; a fresh snapshot matches the source` or
`the bench already matches the source; nothing written`. Until the next run gets to the scenario,
its snapshot from before the change stays in
`build/waveshare_servos/hil_check.d/H15/pre_eeprom.snap` (or `H16/`). This procedure comes from
the code. Nobody tested it on an interrupted run.

## Bench helpers

CMake builds these test fixtures only with `BUILD_TESTING` and never installs them.

### hil_eeprom

`hil_eeprom` is the EEPROM oracle and repair tool. It uses only vendored reads and writes, not the
[checked transactions](design.md#checked-transactions), and takes the
[port lock](design.md#port-lock). `test_hil_eeprom` tests it on the fake bus. Commands: `snapshot`
(`--census`: ids 0-253), `compare` (two `.snap` files, no port), `restore`, `blockcheck` (up to 64
bytes), `drift`, `read`.

A snapshot reads EEPROM 0, 1, 3-39 and SRAM 40, 55 singly, two tries each. It keeps 42, 56, 62, 63,
65 as evidence. An unreadable byte is `x`, never 0. `compare` counts an `x` as a difference, also
against another `x`. A `.snap` file holds `hil_eeprom snapshot 1`, `ids`, `ok`, `census`, and per
servo `servo <id> eeprom`, `sram` and `volatile`. Restore exits 3 with no write on an `x`, a model
change, or an ambiguous id change.

Restore also refuses a change outside `--allow-regs`, in register 6, or in register 5 except to move
back one stray id. Per servo, it does torque off, unlock, writes and lock, each read back. Then it
sets the goal (the position read after the offset write, or wheel speed 0) and the source torque. A
signal after the first write waits (`a signal arrived during the restore; it was completed first`).
Exit: 0 ok, 1 port held, not equal or block mismatch, 2 cannot open, 3 refused or unreadable. Also
64 usage, 70 internal, 130 signal before a write.

### port_probe

`hil_port_probe` is the second opener in H6 and H14. It tells which layer refused the tty, and never
reads or writes it. Options: `--port`, `--hold SECONDS` (0-60, prints `HOLDING {json}`) and
`--no-exclusive` (flock only). It clears `TIOCEXCL` before close, because close does not clear it
while another descriptor holds the tty. Exit: 0 `acquired`, 10 `refused_by_tiocexcl`, 11
`refused_by_flock`, 20 `open_failed`, 21 `flock_failed`, 22 `tiocexcl_failed`, 64 usage.

### stop_wheels

`hil_stop_wheels` sends `WriteSpe(id, 0, acc)` to ids 3 and 4, waits up to 4 s for three reads of
speed 0, then samples. `--read-only` only samples. `--registers` also reads 33, 40, 41, 42 and 46.
Options: `--port`, `--baud` (1000000), `--ids` (3,4), `--acc` (150), `--samples` (5),
`--interval-ms` (100), `--force`. A wheel moves above 100 raw steps/s or 8 ticks. Exit: 0 ok, 1 port
busy (without `--force`), 2 cannot open, 3 no answer, 4 still moving, 64 usage.

## What the bench check does not do

No step needs a person, an ammeter or a pulled connector. H9 gates the recoverable half of a lost
servo. [Motorless tests](development.md#what-the-tests-cover) cover the other half and 256 status
values. That proves the decode, not what a [status bit](configuration.md#status-bits) means.

## Add a bench scenario

1. In `test/hil_check.sh`, write `h_<name>`: `scenario_begin <name> || return $?`, then
   `scenario_end`. Return 0 to continue, or 2 to abort all later scenarios.
2. Give each tool call `-p port:="$PORT"`. Without it, a tool opens `/dev/ttyACM0`, which can be a
   different bench.
3. Add the name to `SCENARIOS`. Keep H13-H16 before H11, and H17 last.
4. Add a checker to `CHECKERS` in `test/hil/hil_gates.py`.
5. Add its data to `test/hil/hil_fixture.py`.
6. Give each non-NOTE row of a tool scenario a `TOOL_INJECTIONS` entry: a defect that fails it.
7. Run `python3 test/hil/hil_gates.py --self-test`.

The self-test checks that `SCENARIOS` and `CHECKERS` have the same members, and the order of
step 3. A directory that no checker claims gives the FAIL row `unchecked_scenarios`, and a listed
scenario with no directory gives `<name>.did_not_run`. `start_stack` takes the xacro arguments, the
wheels for `@WHEELS@`, a watchdog and the controller YAML (default `bench.yaml`). The watchdog
(default 420 s) must outlast the scenario. H11 passes soak + 180. Copy `SPAWNER_RC`,
`CONTROLLERS_ACTIVE` and `CM_EXIT_CODE` to facts before the next stack.
