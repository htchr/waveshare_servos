# Bus timing

These numbers come from the [reference bench](setup.md#the-reference-bench) at `update_rate: 100`.
The model in [Cost per servo](#cost-per-servo) applies to other robots, but the measured totals do
not. Above four servos, all values come from the model ("More than four servos" in
[Known issues](operation.md#known-issues)).

## Cycle cost

| per 100 Hz control cycle, four servos | one read per servo (pre-release path) | one sync read (current driver) |
|---|---|---|
| `read()`: position, velocity, load, voltage, temperature, current, status | 3.064 ms | **1.645 ms** |
| `write()`: two position goals and two wheel speeds | 1.447 ms | **0.014 ms** |
| both, out of the 10 ms period | 4.51 ms (45 %) | **1.66 ms (16.6 %)** |

One `INST_SYNC_READ` reads all four servos in one window of about 1.6 ms. The pre-release path also
wrote the wheel acceleration register in each cycle and waited for the ack
([Wheel acceleration](design.md#wheel-acceleration)). With `feedback_mode: per_servo`, the driver
reads as in the left column and writes as in the right column (worked out). Each value is the mean
of three runs.

At rest, the two paths gave bit-identical `position`, `velocity`, `load` and `current` in 800 of
800 samples. `voltage` and `temperature` change by up to one ADC count between two reads on either
path, because of the converter in the servo.

## Measure your own bus

Run `ros2 topic echo --once /diagnostics`. It shows `<component>.read_cycle.execution_time` and
`<component>.write_cycle.execution_time` in us, and `periodicity.average` in Hz. The sum of the two
averages, compared with the period, is your bus budget (the bench values are in
[Cycle cost](#cycle-cost)). With more than four servos, measure here first.

## Cost per servo

```text
measured per transaction (least squares over one to four servos):
  sync read of n servos          t(n) = 0.476 + 0.290 n   ms
  one feedback read per servo    t(n) = 0.750 n           ms
worked out (tail factor 1.19, 0.02 ms for the write, half of the 10 ms period):
  joints at 100 Hz               1.19 * (0.476 + 0.290 n) + 0.02 <= 5   ->   n <= 12.8
```

The sync read is faster from two servos (n = 1.03). Twelve joints fit at 100 Hz, and five on the
per-servo path. Do not size a robot on the mean (15.5). A faster baud rate cannot help. 1 Mbaud is
the fastest rate of the servos, and the host polls the USB CDC adapter once per 1 ms frame. Thus
a loop of back-to-back reads of four servos has a period of at least 2.000 ms, not 1.6 ms.

A sync read takes at most 30 ids (about 9.2 ms by the model), and the driver splits a longer list
into chunks. A [sync write](design.md#sync-write) gets no ack. It costs the caller about 0.004 ms
and the bus `(8 + n * (nLen + 1)) * 10 us` at 1 Mbaud (`nLen` is 7 for a position, 2 for a speed).

## Feedback mode

`feedback_mode` (stored in lower case) selects how `read()` gets the feedback. At each activation,
`auto` and `sync_read` send one sync read as a probe, with one retry. Each servo that answered its
activation feedback read must answer the probe. If each one answers, the driver logs the INFO
`feedback for <n> servos travels in one sync read per cycle (INST_SYNC_READ)`.

| value | what `read()` does | if the probe fails |
|---|---|---|
| `auto` (default) | one `INST_SYNC_READ` per cycle | one feedback read per servo, and the WARN `sync read went unanswered by motor id(s) <ids>; falling back to one feedback read per servo for this activation` |
| `sync_read` | one `INST_SYNC_READ` per cycle. Use it to keep a measurement on the fast path. | a FATAL that lists the ids. `on_activate` returns `ERROR`, `on_error` parks the servos and closes the port, and the component goes back to `unconfigured`. |
| `per_servo` | one feedback read per servo, and no probe. The fallback, and the baseline of a comparison. | not applicable |

The default is `auto`, because a servo with other firmware must make the driver fall back, not fail.
The bench servos answered 5000 of 5000 probes. The [timeout floor](#timeout-floor) can change `auto`
to `per_servo` at load. For the probe internals, see [Read cycle](design.md#read-cycle).

## Transaction timeout

`io_timeout_ms` sets `SCSerial::IOTimeOut`, one budget per transaction (a sync read of one chunk),
not per servo. The vendored `readSCS` decreases one `timeval` across its `select()` loop. A
transaction that never completes costs 1.00 to 1.03 times the setting (measured from 2 to 50 ms).
Thus one silent servo costs one timeout per cycle
([Cost of a silent servo](#cost-of-a-silent-servo)). The vendored default of 100 ms costs ten
periods of a 100 Hz loop. The range is 2 to 1000 ms, and 1000 ms is a sanity limit.

| value | measured | result |
|---|---|---|
| 1 ms | A four-servo sync read fails 98.55 % of the time. 8 of 3000 reads passed every check with the frames of the previous cycle. | Refused: no flush can discard bytes that did not arrive yet, so the data is wrong ([Late frames](design.md#late-frames)). |
| 2 to 20 ms | No sync read failed, and the mean read stayed at 1.60 to 1.63 ms. | A larger value gives no benefit on a healthy bus. |
| 5 ms (default) | 1.73 times the slowest clean read (2.892 ms) | 100 Hz survives one silent servo. |

## Timeout floor

A sync read carries a burst of replies, so its timeout must grow with the servo count K. K is the
number of declared `<joint>` elements (absent servos too), at most 30. The floor is
`ServoBus::min_io_timeout_ms(K)` = `ceil(1.19 * (0.476 + 0.290 K)) + 1` ms, in integer arithmetic.
1.19 is the measured ratio of the 99th percentile to the mean (2.058 / 1.734 ms). The slack is 1 ms,
because 3 ms at four servos gave 0 failures in 2000 sync reads. The floor is an upper estimate,
because after a drop the driver reads fewer servos.

| K (joints declared) | 1 | 2-4 | 5-7 | 8-9 | 10-12 | 13-15 | 16-18 | 19-21 | 22-24 | 25-27 | 28-30 (and above) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| floor (ms) | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |

At load, the driver compares `io_timeout_ms` with the floor. It never raises a value that you set,
because that value is also the cost of one silent servo. The 5 ms default is silent up to nine
declared joints.

| `feedback_mode` | `io_timeout_ms` | below the floor | above `max(8, floor)`: 8 ms is the period minus the drain |
|---|---|---|---|
| `per_servo` | any | nothing: one 21-byte reply passed 2000 of 2000 reads down to 1 ms | nothing |
| `auto` or `sync_read` | **not set** (default 5) | raised to the floor, one INFO: `io_timeout_ms raised from 5 to <F> ms for a sync read of <K> servos` (from K = 10) | cannot occur |
| `auto` | set | one WARN: `io_timeout_ms <T> is below the <F> ms a sync read of <K> servos needs here; using one feedback read per servo`. The mode changes to `per_servo`: no burst, not even the probe. | one WARN: `io_timeout_ms is <T> ms and a failed read pays the 2 ms drain on top of it; ...`. Expensive, not wrong. |
| `sync_read` | set | FATAL, `on_init` ERROR: `hardware parameter 'io_timeout_ms' is '<T>', below the <F> ms a sync read of <K> servos needs; feedback_mode is 'sync_read', which rules out the per-servo path that would survive it, so raise io_timeout_ms to at least <F> or use 'auto'` | same WARN as above |

## Cost of a silent servo

On the sync path, a silent servo costs one timeout plus a 2 ms drain per cycle, at any place in the
list. Above 30 servos, this is per chunk. The drain is a deadline that a quiet line does not end,
because the late frame did not arrive yet ([Late frames](design.md#late-frames)). The `per_servo`
path pays the timeout but no drain.

| `io_timeout_ms` | 2 | 3 | 5 | 10 | 20 |
|---|---|---|---|---|---|
| cost of one absent servo, library-level sync read (measured) | +0.46 ms | +1.47 ms | +3.56 ms | +8.57 ms | +18.60 ms |
| the driver's read with one absent servo (worked out: 1.6 ms clean read + measured cost + 2 ms drain) | about 4.1 ms | about 5.1 ms | about 7.2 ms | about 12.2 ms | about 22.2 ms |

At 100 Hz (worked out), 5 ms uses about 7 of the 10 ms. At 10 ms, each cycle overruns and the loop
drops to about 50 Hz. A servo that is absent at configure costs nothing per cycle. A servo that goes
silent costs this for `max_read_fails` cycles, until the driver
[drops it](operation.md#recovery-after-a-servo-is-lost).

## Other baud rates

These values come from wire time at 10 bits per byte, not from measurements. All other numbers on
this page are for 1 Mbaud. The driver does not scale `io_timeout_ms` with `baudrate`, but the
[tools](tools.md#tool-parameters) do (last column). Below 1 Mbaud, set
`io_timeout_ms` to the tools value plus the sync-read burst, or use `feedback_mode: per_servo`.

| baud rate | at the 5 ms default | timeout of the tools |
|---|---|---|
| 115200 | A four-servo sync read (12 + 84 bytes, about 8.3 ms) cannot complete. `auto` falls back with a WARN, and `sync_read` fails activation. | 7 ms |
| 57600 | One feedback read (8 + 21 bytes, about 5.04 ms) does not fit. | 11 ms |
| 38400 | One feedback read (7.6 ms) does not fit. | 16 ms |
| 19200 | A ping (6 + 6 bytes, 6.25 ms) does not fit. With `allow_missing_servos: false`, configure fails with `<n> of <n> servos did not answer: ...`. | 29 ms |
| 9600 | A ping (12.5 ms) does not fit. | 56 ms |

## Failure rate

Three ten-minute soaks at 100 Hz gave 0 failed transactions in 182 994. The bench had four servos
at 1 Mbaud, wheels at 1 rad/s, the arm at rest and `io_timeout_ms: 5`. Zero events is not a zero
rate. By the rule of three, the rate is below 17 per million at 95 % confidence, about six per hour
at 100 Hz. With a fourth soak in the [bench check](bench-check.md#soak-h11), the total is 0 in
244 158, below 13 per million.

## Bus totals line

Each activation ends with one INFO line, at deactivation after the park or from `on_error`. A
shutdown after a deactivation does not log it again. With no read cycle (for example, `on_error`
from `inactive`), there is no line. To read it, stop the launch with Ctrl-C and run
`grep 'bus totals:' <launch log>`. The format, and an example from the motorless tests:

```text
bus totals: transactions N, failed F (R per million), worst consecutive W, dropped D [id1 c1, id2 c2, ...]
bus totals: transactions 13, failed 8 (615384.6 per million), worst consecutive 8, dropped 1 [id1 0, id2 0, id3 8, id4 0]
```

| field | counts | note |
|---|---|---|
| `transactions` | bus reads of `read()`: one per cycle on the sync path, one per polled joint on the `per_servo` path | Thus you can compare the rates of the two paths. The line does not count the reads at activation and at park. |
| `failed` | failed transactions. A burst counts once for any number of lost frames, so the rate is at most 1 000 000 per million. | On the `per_servo` path, a silent servo fails only its own reads (one silent servo of four: 250 000 per million). |
| `worst consecutive` | the longest failure run of any joint in this activation | A servo that is still silent at the next activation starts a new run. |
| `dropped` | the servos that the driver dropped in this activation | |
| `[id<N> <count>, ...]` | the failures of each declared servo, in id order, but not its poll count (`read_attempts_`) | The sum can be more than `failed` when one burst lost two frames. |

The bench check parses the line ([Soak (H11)](bench-check.md#soak-h11)) and expects one transaction
per cycle. Do not change the format. [Versioning](development.md#versioning) does not cover it.

## rw_rate and is_async

ros2_control 4.48.0 can read and write a component at its own rate (`rw_rate`) and on its own
thread (`is_async`). Both are attributes of `<ros2_control>`, not `<param>` elements, for example
`<ros2_control name="${name}" type="system" rw_rate="50" is_async="true">`. The parser ignores an
unknown attribute without a message. To check, read `read/write rate:` (the requested rate, not the
actual rate) and `is_async:` in the output of `ros2 control list_hardware_components`.

| setting | effect | note |
|---|---|---|
| `rw_rate="50"` (measured) | half the bus occupancy (8.2 % of the wall clock against 16.5 %), with the same cost per read and no failure | Command pacing uses the real period. A drop takes twice as long ([Recovery](operation.md#recovery-after-a-servo-is-lost)). |
| `is_async="true"` (measured) | no change (read 1.648 ms against 1.653 ms). The wheels stop correctly on SIGINT. | A 120 s recording showed no torn sample, but one window is not a proof. |
| lower `rw_rate` | `/joint_states` still publishes at `update_rate`, so the values repeat. | Use a divisor of `update_rate`: at 100 Hz, `rw_rate="60"` runs at 50 Hz. `rw_rate="0"` or a value above `update_rate` gives `update_rate`. |
| async priority | `<properties><async thread_priority="N"/></properties>` | An async component asks for a [real-time thread](setup.md#real-time-scheduling). The older `thread_priority` attribute still works. |

The package sets neither attribute, because the bench check assumes a 100 Hz component. At
`rw_rate="50"`, its step gate `g1b` fails for a wheel near 2 rad/s. With `is_async="true"` also set,
it fails on almost every step. `is_async` gives no benefit when `read()` is the only expensive work.
Use `rw_rate` when the bus is the bottleneck, and `is_async` when other work loads the control
thread.
