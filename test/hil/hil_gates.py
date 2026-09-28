"""
Invariant gates, scenario predicates and the report writer for the bench check.

Modes: --self-test (synthetic series with injected defects, no hardware) or --run-dir DIR.
"""

import argparse
import contextlib
import io
import json
import math
import os
import re
import sys

# Driver defaults, copied exactly (never rounded) from src/driver_defaults.hpp and
# include/units.hpp; change them only together with the driver.
ENCODER_STEPS = 4096                             # driver_defaults.hpp kEncoderSteps
STEPS_PER_RAD = ENCODER_STEPS / (2.0 * math.pi)  # 651.8986
TICK = 2.0 * math.pi / ENCODER_STEPS             # 0.00153398 rad, the encoder quantum
MAX_SPEED_COUNTS = 6000
MAX_ACCEL_COUNTS = 150
V_CAP = MAX_SPEED_COUNTS * TICK                  # 9.20388 rad/s, the servo speed ceiling
VQ = 50 * TICK                                   # 0.07670 rad/s, reported-velocity quantum
CURRENT_PER_COUNT_A = 0.006                      # driver_defaults.hpp kCurrentPerCountA
TORQUE_CONSTANT_NM_PER_A = 0.8825985             # driver_defaults.hpp kTorqueConstantNmPerA
NM_PER_KGFCM = 0.0980665                         # units.hpp kNmPerKgfCm
IO_TIMEOUT_MS = 5                                # driver_defaults.hpp kIoTimeoutMs
PING_ATTEMPTS = 3
MAX_READ_FAILS = 50
ALLOW_MISSING_SERVOS = False

# `inverted` flips exactly these three quantities.
# See docs/configuration.md, "Joint frame".
INVERTED_FLIPS = ('position', 'velocity', 'load')

# Import-time guard: effort, current and torque are unsigned magnitudes and never flip.
for _name in ('effort', 'current', 'torque'):
    if _name in INVERTED_FLIPS:
        raise SystemExit(
            "hil_gates: INVERTED_FLIPS contains '%s'; in the driver, "
            'inverted flips position, velocity and load only' % _name)

# Gate tolerances. See docs/bench-check.md, "Gate thresholds".
G1B_SLACK = 12 * TICK      # 0.018408 rad; the worst measured excess is 5.5 ticks
G1C_EPS = 1e-9             # G1c is exact; this is float noise, not a tolerance
G2_WINDOW = 0.50           # s, centred on the sample, taken by time and not by count
G2_MIN_SAMPLES = 25
G2_SPAN_TOL = 0.02         # s the end samples may fall short of the full window
G2_STEADY = 0.30           # rad/s of velocity spread inside an eligible window
G2_A = 0.060               # rad/s, 1.6 half-quanta of the reported velocity
G2_B = 0.020               # 4x the measured 0.5 % systematic bias
G3_REL = 0.02
G3_ABS = 0.05              # rad
STALE_MIN = 3              # bit-identical samples that make a stale run
STALE_MOVING = 0.05        # rad/s; a parked joint repeating itself is not stale

# Resting drift allowed between two readbacks (registers_unchanged): worst measured wheel rest 3
# ticks, smallest real command 311. See docs/bench-check.md, "Gate thresholds".
REST_TICKS = 2             # `pos` joints: a servoed goal does not relax
WHEEL_REST_TICKS = 8       # `vel` joints: 0.70 deg, 2.7x the worst measured rest
WHEEL_MODE = 1             # the servo's mode register: 0 = position, 1 = wheel

# t90 at acc=150 = latency + ramp + settling; eight runs gave 0.457-0.500 s, and 0.70 s still
# holds if the feedback skips two velocity levels.
T90_FAST_MAX = 0.70        # s, t90 at acc=150

# Deliberately loose: t90 fires on a 50-count velocity lattice, so the measured ratio
# (3.92-4.33) moves ~10 % run to run, and ignored ACC gives ~1.0. Tighten the metric, not this.
T90_RATIO_MIN = 2.5        # t90(acc=10)/t90(acc=150)

# Bench bound on the mean read cycle (4 servos, 100 Hz, 1 Mbaud): measured 1.645 ms + 0.35,
# rounded up.
READ_MS_AVG_MAX = 2.00
# Max read cycle: cumulative since activation and set by kernel jitter (mean + 1.92, rounded
# up). Valid only for the short captures of H1 and H9, not for the soak.
READ_MS_MAX_MAX = 3.60
# Mean write cycle: measured 0.0136 ms + 0.30, rounded up.
WRITE_MS_AVG_MAX = 0.35
# Max write cycle: not fitted to the measured mean, because the write max varies 14x between
# windows. Valid only for the short captures of H1 and H9.
WRITE_MS_MAX_MAX = 1.50

# 0 failures in 60 000 reads: the rule of three gives 50 per million (95 %). A rate, so a
# shortened soak uses it too. See docs/bench-check.md, "Soak (H11)".
SOAK_FAIL_PER_MILLION_MAX = 50.0
# Below this the rule of three cannot resolve 50 per million at all (3/30000 = 100), so the row
# SKIPs rather than passing vacuously. 30 000 transactions is five minutes at 100 Hz.
SOAK_MIN_TRANSACTIONS = 30000
# The longest failing run measured in 60 000 cycles was 0. Two allows an isolated hiccup and
# still fires 25x before max_read_fails (50) would drop a servo.
SOAK_WORST_CONSECUTIVE_MAX = 2
# The soak wheels must turn at >= half the commanded 1.0 rad/s: no other H11 row can tell a
# turning bus from a stopped one.
SOAK_WHEEL_MIN_RAD_S = 0.5

NAN = float('nan')
PI = math.pi


def wrap(x):
    """Reduce an angle into (-pi, +pi]."""
    y = math.fmod(x + PI, 2 * PI)
    return (y + 2 * PI if y <= 0.0 else y) - PI


def stale_runs(t, p, v, min_len=STALE_MIN, moving=STALE_MOVING):
    """Return [first, last] index pairs of bit-identical runs taken while the joint moves."""
    runs, start = [], 0
    for k in range(1, len(p) + 1):
        if k < len(p) and p[k] == p[start] and v[k] == v[start]:
            continue
        if k - start >= min_len and abs(v[start]) > moving:
            runs.append((start, k - 1))
        start = k
    return runs


def _excluded(t, p, v):
    """Return every stale index plus the first sample after each run."""
    out = set()
    for first, last in stale_runs(t, p, v):
        out.update(range(first, min(last + 2, len(p))))
    return out


def _unwrapped(p):
    out = [0.0] * len(p)
    for k in range(1, len(p)):
        out[k] = out[k - 1] + wrap(p[k] - p[k - 1])
    return out


def _slope(xs, ys):
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    den = sum((x - mx) ** 2 for x in xs)
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den > 0.0 else 0.0


def g1a(t, p):
    """Gate (i), headline: no step greater than half a revolution, no exclusions."""
    worst = max((abs(wrap(b - a)) for a, b in zip(p, p[1:])), default=0.0)
    return {'ok': worst <= PI, 'worst': worst, 'pairs': max(0, len(p) - 1)}


def aliasing_safe(t, v):
    """Soundness precondition of G1a: real motion per sample stays under half a revolution."""
    worst = max((max(abs(v[k]), abs(v[k + 1])) * (t[k + 1] - t[k]) for k in range(len(v) - 1)),
                default=0.0)
    return {'ok': worst < PI, 'worst': worst, 'pairs': max(0, len(v) - 1)}


def g1b(t, p, v):
    """Gate (i), rate-scaled: the step is bounded by the motion the velocity allows."""
    skip = _excluded(t, p, v)
    worst, pairs, bad = -math.inf, 0, 0
    for k in range(len(p) - 1):
        if k in skip or k + 1 in skip:
            continue
        pairs += 1
        excess = abs(wrap(p[k + 1] - p[k])) - max(abs(v[k]), abs(v[k + 1])) * (t[k + 1] - t[k])
        worst = max(worst, excess)
        bad += excess > G1B_SLACK
    return {'ok': bad == 0, 'worst': worst if pairs else 0.0, 'pairs': pairs,
            'excluded': len(skip), 'bad': bad}


def g1c(t, p):
    """Gate (i), absolute: no step ever needed reducing. Only asked of unwrapping joints."""
    worst = max((abs((b - a) - wrap(b - a)) for a, b in zip(p, p[1:])), default=0.0)
    return {'ok': worst <= G1C_EPS, 'worst': worst, 'pairs': max(0, len(p) - 1)}


def g2(t, p, v):
    """Gate (ii): the position slope tracks the reported velocity over a centred 0.5 s window."""
    skip = _excluded(t, p, v)
    n, half = len(t), G2_WINDOW / 2.0
    out = {'ok': True, 'windows': 0, 'bad': 0, 'worst': 0.0, 'mean': 0.0, 'tol': 0.0,
           'edge': 0, 'stale': 0, 'unsteady': 0}
    lo = hi = 0
    for k in range(n):
        first, last = t[k] - half, t[k] + half
        while lo < n and t[lo] < first:
            lo += 1
        while hi < n and t[hi] <= last:
            hi += 1
        if (hi - lo < G2_MIN_SAMPLES or t[lo] > first + G2_SPAN_TOL or
                t[hi - 1] < last - G2_SPAN_TOL):
            out['edge'] += 1
        elif skip & set(range(lo, hi)):
            out['stale'] += 1
        elif max(v[lo:hi]) - min(v[lo:hi]) > G2_STEADY:
            out['unsteady'] += 1
        else:
            mean_v = sum(v[lo:hi]) / (hi - lo)
            resid = abs(_slope(t[lo:hi], _unwrapped(p[lo:hi])) - mean_v)
            out['windows'] += 1
            out['bad'] += resid > G2_A + G2_B * abs(mean_v)
            if resid > out['worst']:
                out['worst'], out['mean'] = resid, mean_v
    out['tol'] = G2_A + G2_B * abs(out['mean'])
    out['ok'] = out['bad'] == 0
    return out


def g3(t, p, v):
    """Gate (iii): travel as reported -- no unwrapping here -- against the velocity integral."""
    travel = p[-1] - p[0] if p else 0.0
    integral = sum(0.5 * (v[k] + v[k + 1]) * (t[k + 1] - t[k]) for k in range(len(t) - 1))
    tol = G3_REL * abs(integral) + G3_ABS
    return {'ok': abs(travel - integral) <= tol, 'T': travel, 'I': integral,
            'd': abs(travel - integral), 'tol': tol, 'wraps': int(abs(integral) / (2 * PI))}


# A run directory holds one sub-directory per scenario: facts.json, recordings, log.txt.
# See docs/bench-check.md, "Report and verdicts".


def _load(path):
    try:
        with open(path) as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def _text(path):
    try:
        with open(path, errors='replace') as handle:
            return handle.read()
    except OSError:
        return ''


class Scenario:
    """Read one scenario directory."""

    def __init__(self, run_dir, name):
        self.name = name
        self.path = os.path.join(run_dir, name)
        self.facts = _load(os.path.join(self.path, 'facts.json')) or {}
        self.log = _text(os.path.join(self.path, 'log.txt'))

    def present(self):
        return os.path.isdir(self.path)

    def fact(self, key, default=''):
        return str(self.facts.get(key, default))

    def flag(self, key):
        return self.fact(key) == 'true'

    def number(self, key, default=NAN):
        try:
            return float(self.fact(key))
        except ValueError:
            return default

    def rec(self, label):
        return _load(os.path.join(self.path, label + '.json')) or {}

    def logged(self, *needles):
        return sum(1 for line in self.log.splitlines() if all(n in line for n in needles))

    def regs(self, label, servo):
        return (self.rec(label).get('ids', {}) or {}).get(str(servo), {}) or {}

    def reg(self, label, servo, name, default=-1):
        return self.regs(label, servo).get('registers', {}).get(name, default)

    def txt(self, name):
        return _text(os.path.join(self.path, name))


def series(rec, joint):
    """Return the times (header stamps), positions and velocities of one joint."""
    rows = []
    for s in rec.get('joint_states', []):
        if joint in s[2]:
            k = s[2].index(joint)
            if k < len(s[3]) and k < len(s[4]):
                rows.append((s[1], s[3][k], s[4][k]))
    rows.sort()
    return [r[0] for r in rows], [r[1] for r in rows], [r[2] for r in rows]


def joints_of(rec):
    for s in rec.get('joint_states', []):
        return list(s[2])
    return []


def iface(rec, joint, name):
    """Return the times and values of one /dynamic_joint_states interface."""
    t, y = [], []
    for m in rec.get('dynamic_joint_states', []):
        names = m.get('joint_names', [])
        if joint in names:
            block = m['interfaces'][names.index(joint)]
            if name in block['names']:
                t.append(m['stamp'])
                y.append(block['values'][block['names'].index(name)])
    return t, y


def ifaces_of(rec, joint):
    for m in rec.get('dynamic_joint_states', []):
        names = m.get('joint_names', [])
        if joint in names:
            return list(m['interfaces'][names.index(joint)]['names'])
    return []


def rate_hz(t):
    return (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else NAN


def max_gap(t):
    return max((b - a for a, b in zip(t, t[1:])), default=NAN)


def between(t, low, high):
    return [k for k in range(len(t)) if low <= t[k] <= high]


def median(values):
    ordered = sorted(values)
    n = len(ordered)
    if not n:
        return NAN
    return ordered[n // 2] if n % 2 else 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])


# Not a median: the reported velocity is quantised at VQ, so a median can be VQ/2 off.
def mean(values):
    """Return the arithmetic mean of the values, or NaN when there are none."""
    return sum(values) / len(values) if values else NAN


def flips(quantity):
    """Whether `inverted` flips this quantity; the H7 rows read it."""
    return quantity in INVERTED_FLIPS


def cycle_ms(text, which):
    """Return the average and maximum read/write cycle time in ms, from /diagnostics."""
    found = re.findall(r'%s_cycle\.execution_time[^\n]*\n\s*value:[^\n]*'
                       r'Avg:\s*([-\d.]+)\s*\[[-\d.]+\s*-\s*([-\d.]+)\]' % which, text)
    if not found:
        return NAN, NAN
    return median([float(a) for a, _ in found]) / 1e3, max(float(b) for _, b in found) / 1e3


BUS_TOTALS = re.compile(
    r'bus totals: transactions (\d+), failed (\d+) \([\d.]+ per million\), '
    r'worst consecutive (\d+), dropped (\d+)')
# The per-servo tail of the driver's bus totals line: it names the ids that failed, and
# H11.per_servo requires it. See docs/bus-timing.md, "Bus totals line".
BUS_TOTALS_TAIL = re.compile(r'\bid(\d+) (\d+)')


def bus_totals(text):
    """Return the LAST (transactions, failures, worst_consecutive, dropped) the driver printed."""
    found = BUS_TOTALS.findall(text)
    if not found:
        return None
    return tuple(int(x) for x in found[-1])


def bus_totals_tail(text):
    """Return the LAST totals line's [(id, failures), ...] pairs, or [] when it carries none."""
    lines = [line for line in text.splitlines() if BUS_TOTALS.search(line)]
    if not lines:
        return []
    # Only what follows the last '[' on that line: the aggregate ahead of it carries bare numbers
    # too, and a tail-shaped match found there would be a count nobody printed.
    _, bracket, tail = lines[-1].rpartition('[')
    return [(int(i), int(n)) for i, n in BUS_TOTALS_TAIL.findall(tail)] if bracket else []


def fail_per_million(transactions, failures):
    """Failures per million transactions, or NaN when nothing was attempted."""
    return NAN if not transactions else 1e6 * failures / transactions


def kv(**facts):
    """Render row facts as stable key=value text."""
    return ' '.join('%s=%s' % (k, '%.4f' % x if isinstance(x, float) else x)
                    for k, x in facts.items())


class Report:
    """Hold the report's stable-keyed rows."""

    def __init__(self, allowed):
        self.rows = []
        self.allowed = set(allowed)
        self.prefix = ''

    def row(self, key, verdict, text):
        """Append one row. An INCONCLUSIVE row whose key is not frozen becomes a FAIL."""
        key = '%s.%s' % (self.prefix, key) if self.prefix else key
        if verdict == 'INCONCLUSIVE' and key not in self.allowed:
            verdict = 'FAIL'
            text += '  (INCONCLUSIVE is not an allowed verdict for this key)'
        self.rows.append({'key': key, 'verdict': verdict, 'detail': text})

    def ck(self, key, ok, text='', **facts):
        """Append one gated row. Nothing rewrites a FAIL."""
        self.row(key, 'PASS' if ok else 'FAIL', (kv(**facts) + ' ' + text).strip())

    def count(self, verdict):
        return sum(1 for row in self.rows if row['verdict'] == verdict)


def gate_rows(R, joint, t, p, v, unwrap=False):
    """Emit G1a, aliasing_safe, G1b, G2 and, for an unwrapping joint, G1c."""
    if len(t) < 3:
        R.row('g1a.%s' % joint, 'SKIP', 'fewer than three samples')
        return
    one, alias, tight, two = g1a(t, p), aliasing_safe(t, v), g1b(t, p, v), g2(t, p, v)
    R.ck('g1a.%s' % joint, one['ok'], pairs=one['pairs'], worst=one['worst'], bound=PI)
    R.ck('aliasing_safe.%s' % joint, alias['ok'], worst=alias['worst'], bound=PI)
    R.ck('g1b.%s' % joint, tight['ok'], pairs=tight['pairs'], excluded=tight['excluded'],
         worst=tight['worst'], tol=G1B_SLACK, violations=tight['bad'])
    if not two['windows']:
        R.row('g2.%s' % joint, 'SKIP', kv(windows=0, edge=two['edge'], stale=two['stale'],
                                          unsteady=two['unsteady']))
    else:
        R.ck('g2.%s' % joint, two['ok'], windows=two['windows'], worst=two['worst'],
             tol=two['tol'], violations=two['bad'])
    if unwrap:
        three = g1c(t, p)
        R.ck('g1c.%s' % joint, three['ok'], pairs=three['pairs'], reduced=three['worst'])


def port_rows(R, S):
    """Emit the two port-holder rows every scenario carries."""
    R.ck('port_free_before', S.flag('port_free_before'), '[%s]' % S.fact('port_holders_before'))
    R.ck('port_free_after', S.flag('port_free_after'), '[%s]' % S.fact('port_holders_after'))


# One second at 100 Hz after the cycle: proves the recorder still ran. g1c needs only one
# step to see a multi-turn reset.
H9_CYCLE_MIN_SAMPLES = 100

# H2's arm bounds, reused by H1B and H9: 0.005 rad target error and a 0.40 rad/s velocity
# step at either end of the recording.
ARM_TARGET_TOL = 0.005
ARM_LURCH_MAX = 0.40


# See docs/bench-check.md, "Component cycle (H9)".
def cycle_rows(R, S, rec):
    """
    Emit the recovery rows for H9's inactive/active component cycle.

    g1c already gates wheel continuity; these add state, post-cycle samples and an arm move.
    """
    R.ck('cycle_state',
         S.fact('hw_state_after_cycle') == 'active' and S.fact('cycle_inactive_rc') == '0' and
         S.fact('cycle_active_rc') == '0',
         'the component came back from the inactive/active cycle',
         state=S.fact('hw_state_after_cycle'), expected='active',
         inactive_rc=S.fact('cycle_inactive_rc'), active_rc=S.fact('cycle_active_rc'))
    done = S.number('t_cycle_done', NAN)
    seen = {j: len(between(series(rec, j)[0], done, math.inf)) for j in ('joint3', 'joint4')}
    fewest = min(seen.values()) if seen else 0
    R.ck('cycle_recorded', not math.isnan(done) and fewest >= H9_CYCLE_MIN_SAMPLES,
         'samples after the component came back, which is the only window g1c can see a '
         'multi-turn reset in: %s' % kv(**seen),
         fewest=fewest, bound=H9_CYCLE_MIN_SAMPLES, t_cycle_done=done)
    for label, target in (('arm_after_cycle', 0.4), ('arm_after_park', 0.0)):
        for joint in ('joint1', 'joint2'):
            t, p, v = series(S.rec(label), joint)
            final = p[-1] if p else NAN
            R.ck('cycle_arm.%s.%s' % (label, joint), abs(final - target) <= ARM_TARGET_TOL,
                 final=final, target=target, err=abs(final - target), tol=ARM_TARGET_TOL)
            lurch = max(abs(v[1] - v[0]), abs(v[-1] - v[-2])) if len(v) >= 2 else NAN
            R.ck('cycle_no_lurch.%s.%s' % (label, joint), lurch <= ARM_LURCH_MAX, dv=lurch,
                 bound=ARM_LURCH_MAX)


# Scenario checkers. Each takes the report (prefixed with the scenario name), its scenario
# directory, and a context shared between scenarios.

IFACES = ('position', 'velocity', 'effort', 'current', 'voltage', 'temperature', 'load',
          'status', 'torque')
OFFSET = 1.570796          # the offset every pos joint of the bench descriptions carries
ESCAPE = ('controller_manager hardware_components_initial_state.'
          'shutdown_on_initial_state_failure: false keeps the node alive with the component '
          'unconfigured')
PING_DIAGNOSIS = ('a status of 1, 2, 3, 4 or 9 means the driver latched SCS::Error from a Ping '
                  'or an Ack (src/SCS.cpp:261, Error = bBuf[2]) instead of the feedback Read')
# Servo ids the shipped example declares (example.ros2_control.xacro, joint1..joint4); H1 and
# H1B run that example. Add the id of a new example joint here too.
EXAMPLE_IDS = (1, 2, 3, 4)

# H12 stimulus (h_H12 renders it; H12.stimulus checks it) and the driver's own ceilings.
# POS_CMD_LIMIT is the arm's command-interface max; it equals OFFSET only by coincidence.
H12_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
H12_ARM_LIMIT = 0.8            # rad, the <limit lower/upper> H12 renders (arm_pos_limit)
H12_ARM_COMMAND = 1.2          # rad, past that limit and inside the driver's POS_CMD_LIMIT
H12_WHEEL_LIMIT = 2.0          # rad/s, the <limit velocity> H12 renders (wheel_vel_limit)
H12_WHEEL_COMMAND = 8.0        # rad/s, past that limit and inside the driver's V_CAP
POS_CMD_LIMIT = 1.570796       # rad, the arm's <command_interface> max: the driver's own ceiling

# Import-time guard: each H12 axis needs limit < command <= driver ceiling, or a clamp at the
# limit would not prove that the limiter made it.
for _axis, _limit, _command, _driver in (
        ('position', H12_ARM_LIMIT, H12_ARM_COMMAND, POS_CMD_LIMIT),
        ('velocity', H12_WHEEL_LIMIT, H12_WHEEL_COMMAND, V_CAP)):
    if not _limit < _command <= _driver:
        raise SystemExit(
            "hil_gates: H12's %s clamp is no longer attributable: the rendered <limit> %.6f, the "
            'command %.6f and the driver-side ceiling %.6f must satisfy limit < command <= '
            'driver, or a clamp landing on the limit is not evidence that the controller '
            "manager's JointSaturationLimiter produced it"
            % (_axis, _limit, _command, _driver))

# The arm must rest within 0.05 rad of 0 before the limited stack starts: the limiter throws,
# not clamps, 0.0087 rad outside <limit>. See docs/bench-check.md, "Command limits (H12)".
H12_PARK_MARGIN = 0.05

# 0.010 rad: the arm's settle error (0.0033) plus the limiter's 0.002 bound tolerance,
# rounded up.
H12_POS_TOL = 0.010

# The same +-0.060 rad/s as H8.speed_wheel, measured under the same 8.0 -> 2.0 rad/s clamp.
H12_VEL_TOL = 0.060

# Logged once per joint by hardware_interface when enforce_command_limits is on; parsed for
# the joint names that H12.limiter_lines checks.
LIMITER_LINE = re.compile(r"Creating JointSaturationLimiter for joint '([^']+)' "
                          r"in hardware '([^']+)'")


def mean_vel(rec, joint, lead, trail):
    """Return the mean velocity between t_cmd + lead and t_stop - trail, and the count."""
    t, _, v = series(rec, joint)
    inside = between(t, rec.get('t_cmd', 0.0) + lead, rec.get('t_stop', 0.0) - trail)
    return mean([v[k] for k in inside]), len(inside)


def settle(rec, joint, target, band=0.01):
    """Return the seconds from t_cmd after which the joint never leaves band of target."""
    t, p, _ = series(rec, joint)
    t0 = rec.get('t_cmd', 0.0)
    for k in range(len(t) - 1, -1, -1):
        if abs(p[k] - target) > band:
            return t[k + 1] - t0 if k + 1 < len(t) else NAN
    return 0.0 if t else NAN


def t90(rec, joint, threshold):
    """Return the seconds from t_cmd to the first sample whose speed reaches the threshold."""
    t, _, v = series(rec, joint)
    t0 = rec.get('t_cmd', 0.0)
    for k in range(len(t)):
        if t[k] >= t0 and abs(v[k]) >= threshold:
            return t[k] - t0
    return NAN


def final_of(rec, joint):
    p = series(rec, joint)[1]
    return p[-1] if p else NAN


def h1(R, S, C):
    port_rows(R, S)
    pings = sum(S.logged("unable to ping motor id '%d'" % i) for i in EXAMPLE_IDS)
    # Pings for ids not in EXAMPLE_IDS, counted as a group: a servo added to the example but not
    # to EXAMPLE_IDS still fails this row.
    unlisted = S.logged('unable to ping motor id') - pings
    live = S.logged("Successful 'activate'")
    R.ck('activate', S.flag('controllers_active') and live and not pings and not unlisted,
         active=S.fact('controllers_active'), activate_lines=live, unable_to_ping=pings,
         unlisted_id_pings=unlisted)
    rec = S.rec('steady')
    names = joints_of(rec)
    t = series(rec, names[0])[0] if names else []
    R.ck('rate', abs(rate_hz(t) - 100.0) <= 2.0, '[no-regression]', hz=rate_hz(t),
         tol='100+-2', n=len(t))
    R.ck('gap', max_gap(t) <= 0.050, hz_gap_s=max_gap(t), bound=0.050)
    diag = S.txt('diagnostics.txt')
    for which, avg_bound, max_bound in (('read', READ_MS_AVG_MAX, READ_MS_MAX_MAX),
                                        ('write', WRITE_MS_AVG_MAX, WRITE_MS_MAX_MAX)):
        avg, worst = cycle_ms(diag, which)
        C['h1_%s_ms' % which] = avg
        if math.isnan(avg):
            R.row('%s_ms' % which, 'SKIP', 'no %s_cycle.execution_time in /diagnostics' % which)
        else:
            R.ck('%s_ms' % which, avg < avg_bound and worst < max_bound, '[no-regression]',
                 avg_ms=avg, avg_bound=avg_bound, max_ms=worst, max_bound=max_bound)
    R.ck('port_line', S.logged("bus on '") == 1, open_info_lines=S.logged("bus on '"), expected=1)
    for joint in names:
        gate_rows(R, joint, *series(rec, joint))


# See docs/bench-check.md, "Example stack (H1 and H1B)".
def h1b(R, S, C):
    """
    Gate the shipped example while it is commanded.

    Uses the measurements and bounds of H2 (arm), H3 (wheels) and H10 (shutdown).
    """
    port_rows(R, S)
    R.ck('activate', S.flag('controllers_active') and S.fact('hw_state') == 'active',
         'the three example controllers reached active and the component is up',
         active=S.fact('controllers_active'), hw_state=S.fact('hw_state'), expected='active')
    # diff_drive_controller must be loaded but inactive: it claims the same wheel command
    # interfaces as joint_velocity_controller. '' means it did not load (package missing).
    state = S.fact('diff_drive_state')
    R.ck('diff_drive', state == 'inactive', 'loaded and inactive, as example.launch.py spawns it '
         "(--inactive); '' means the controller never loaded",
         state=state or "''", expected='inactive')
    # The arm, with H2's bounds: 0.005 rad target error, a 0.40 rad/s lurch at either end, and
    # 0.02 rad of overshoot on the move that has somewhere to go.
    for label, target in (('ex_move_to_0', 0.0), ('ex_move_to_06', 0.6)):
        t, p, v = series(S.rec(label), 'joint1')
        final = p[-1] if p else NAN
        R.ck('target.%s' % label, abs(final - target) <= ARM_TARGET_TOL, final=final,
             target=target, err=abs(final - target), tol=ARM_TARGET_TOL)
        lurch = max(abs(v[1] - v[0]), abs(v[-1] - v[-2])) if len(v) >= 2 else NAN
        R.ck('no_lurch.%s' % label, lurch <= ARM_LURCH_MAX, dv=lurch, bound=ARM_LURCH_MAX)
    p = series(S.rec('ex_move_to_06'), 'joint1')[1]
    R.ck('overshoot', (max(p) - 0.6 if p else NAN) <= 0.02, bound=0.02,
         overshoot=max(p) - 0.6 if p else NAN)
    # h_H1B skips the wheels when the component is not active after the arm move; this row makes
    # that skip a FAIL that names the states.
    R.ck('arm_survived', S.fact('hw_state_after_arm') == 'active' and
         S.fact('arm_state_after') == 'active' and S.flag('vel_attempted'),
         'the example JTC claims both position and velocity command interfaces, a setup '
         'suspected of crashing the control node; this row is where that crash would land',
         hw_state=S.fact('hw_state_after_arm'), controller=S.fact('arm_state_after') or "''",
         wheels_commanded=S.fact('vel_attempted'))
    # The wheels, with H3's bounds: mean speed within 0.020 rad/s of 2.0, and after the stop a
    # mean |v| under 0.05 rad/s with under 0.01 rad of drift.
    rec = S.rec('ex_vel_2')
    for joint in ('joint3', 'joint4'):
        t, p, v = series(rec, joint)
        mean_v, n = mean_vel(rec, joint, 1.0, 0.5)
        R.ck('mean_vel.%s' % joint, abs(mean_v - 2.0) <= 0.020, mean=mean_v, target=2.0,
             tol=0.020, n=n)
        # No sample may go backwards by more than one tick (position is quantised at TICK): a
        # wheel reported against its command is what H1B exists to catch.
        inside = between(t, rec.get('t_cmd', 0.0) + 1.0, rec.get('t_stop', 0.0) - 0.5)
        steps = [p[b] - p[a] for a, b in zip(inside, inside[1:])]
        back = sum(1 for d in steps if d < -TICK)
        R.ck('monotonic.%s' % joint, bool(steps) and back == 0,
             'commanded +2.0 rad/s, so no sample may go backwards by more than one encoder tick',
             backward_steps=back, of=len(steps), tick=TICK)
        after = between(t, rec.get('t_stop', 0.0) + 1.0, math.inf)
        rest = mean([abs(v[k]) for k in after])
        drift = abs(p[after[-1]] - p[after[0]]) if after else NAN
        R.ck('stopped.%s' % joint, rest < 0.05 and drift < 0.01, mean_abs_v=rest, bound=0.05,
             dp=drift, dp_bound=0.01)
        gate_rows(R, joint, t, p, v)
    # SIGINT teardown. `exit` is ros2 launch's aggregate status (any process), weaker than
    # H10.exit; `sequence` is the check for the driver's own teardown.
    R.ck('exit', S.fact('launch_exit_code') == '0',
         "ros2 launch's aggregate exit status: non-zero if the CM, either spawner or the "
         'robot_state_publisher exited badly, so read it with `sequence` below',
         exit_code=S.fact('launch_exit_code'), expected=0)
    off, down = S.log.find('deactivate'), S.log.find('shutdown')
    R.ck('sequence', 0 <= off < down, 'deactivate before shutdown, both present',
         deactivate_at=off, shutdown_at=down)
    R.ck('port_free', S.flag('port_free_within_10s'),
         'holders 10 s after the launch exited=[%s]' % S.fact('port_holders_after_exit'))
    R.ck('retakeable', S.rec('probe_after_exit').get('verdict') == 'acquired',
         'released, not merely closed', verdict=S.rec('probe_after_exit').get('verdict'))
    # Read back off the bus after the stack is gone, so a wheel left turning on a latched goal
    # speed is reported rather than stopped-and-forgotten by scenario_end's final_stop_wheels.
    stopped = S.rec('after_exit')
    R.ck('wheels_after_exit', stopped.get('any_moving') is False,
         any_moving=stopped.get('any_moving'))


def h2(R, S, C):
    port_rows(R, S)
    for label, target in (('move_to_0', 0.0), ('move_to_06', 0.6)):
        t, p, v = series(S.rec(label), 'joint1')
        final = p[-1] if p else NAN
        R.ck('target.%s' % label, abs(final - target) <= 0.005, final=final, target=target,
             err=abs(final - target), tol=0.005)
        lurch = max(abs(v[1] - v[0]), abs(v[-1] - v[-2])) if len(v) >= 2 else NAN
        R.ck('no_lurch.%s' % label, lurch <= 0.40, dv=lurch, bound=0.40)
    rec = S.rec('move_to_06')
    p = series(rec, 'joint1')[1]
    R.ck('overshoot', (max(p) - 0.6 if p else NAN) <= 0.02, bound=0.02,
         overshoot=max(p) - 0.6 if p else NAN)
    want = round((0.6 + OFFSET) * STEPS_PER_RAD)
    got = S.reg('post_readback', 1, 'goal_position_raw')
    R.ck('register', abs(got - want) <= 1, goal_position_raw=got, expected=want)
    for joint in joints_of(rec):
        gate_rows(R, joint, *series(rec, joint))


def h3(R, S, C):
    port_rows(R, S)
    rec = S.rec('vel_2')
    for joint in ('joint3', 'joint4'):
        t, p, v = series(rec, joint)
        mean_v, n = mean_vel(rec, joint, 1.0, 0.5)
        R.ck('mean_vel.%s' % joint, abs(mean_v - 2.0) <= 0.020, mean=mean_v, target=2.0, tol=0.020,
             n=n)
        after = between(t, rec.get('t_stop', 0.0) + 1.0, math.inf)
        rest = mean([abs(v[k]) for k in after])
        drift = abs(p[after[-1]] - p[after[0]]) if after else NAN
        R.ck('stopped.%s' % joint, rest < 0.05 and drift < 0.01, mean_abs_v=rest, bound=0.05,
             dp=drift, dp_bound=0.01)
        gate_rows(R, joint, t, p, v)
    R.row('g3', 'SKIP', '6 s at 2 rad/s is 12.0 rad = 1.9 revolutions, under the two-wrap '
                        'precondition of gate (iii)')


def h4(R, S, C):
    port_rows(R, S)
    rec = S.rec('spin')
    for joint, servo in (('joint3', 3), ('joint4', 4)):
        t, p, v = series(rec, joint)
        if not t:
            R.ck('g3.%s' % joint, False, 'no samples')
            continue
        inside = between(t, rec.get('t_cmd', 0.0) + 0.5, rec.get('t_stop', 0.0) - 0.2)
        travel = g3(*[[s[k] for k in inside] for s in (t, p, v)])
        R.ck('g3.%s' % joint, travel['ok'], T=travel['T'], I=travel['I'], d=travel['d'],
             tol=travel['tol'], wraps=travel['wraps'])
        R.ck('wraps.%s' % joint, travel['wraps'] >= 2, wraps=travel['wraps'], bound=2)
        R.ck('range.%s' % joint, max(p) - min(p) > 2 * PI, span=max(p) - min(p), bound=2 * PI)
        gate_rows(R, joint, t, p, v, unwrap=True)
        r0 = S.regs('pre_readback', servo).get('pos_last_raw', -1)
        r1 = S.regs('post_readback', servo).get('pos_last_raw', -1)
        resid = ((p[-1] - p[0]) - (r1 - r0) * TICK) % (2 * PI)
        resid = min(resid, 2 * PI - resid)
        R.ck('raw_crosscheck.%s' % joint, resid <= 3 * TICK, r0=r0, r1=r1, travel=p[-1] - p[0],
             residual_mod_2pi=resid, bound=3 * TICK)
        R.ck('seed.%s' % joint, abs(wrap(p[0] - r0 * TICK)) <= 3 * TICK,
             seed=abs(wrap(p[0] - r0 * TICK)), bound=3 * TICK)


def registers_unchanged(S, before, after, ids):
    """Say whether the registers and the present position match in two readbacks."""
    # The registers must be equal.  The present position may differ by the resting allowance of
    # that servo's mode: REST_TICKS for `pos`, WHEEL_REST_TICKS for a wheel that relaxes.
    same, detail, worst = True, [], (-1, 0, 0, '')
    for servo in ids:
        was, now = S.regs(before, servo), S.regs(after, servo)
        if not was or not now:
            return False, 'readback %s or %s has no id %d' % (before, after, servo)
        if not was.get('n') or not now.get('n'):
            # a readback where the servo never answered would make the comparison vacuous
            return False, 'id %d answered %s/%s samples in %s/%s' % (
                servo, was.get('n'), now.get('n'), before, after)
        for field in ('torque_enable', 'acc', 'goal_position_raw', 'goal_speed_raw'):
            old, new = S.reg(before, servo, field), S.reg(after, servo, field)
            if old != new:
                same = False
                detail.append('id%d %s %s->%s' % (servo, field, old, new))
        if was.get('mode') != now.get('mode'):
            same = False
            detail.append('id%d mode %s->%s' % (servo, was.get('mode'), now.get('mode')))
        # A wheel-mode servo holds no position and relaxes after being driven; the allowance is
        # per mode, and an unrecorded mode falls back to the stricter `pos` bound.
        wheel = now.get('mode') == WHEEL_MODE and was.get('mode') == WHEEL_MODE
        bound = WHEEL_REST_TICKS if wheel else REST_TICKS
        moved = abs(now.get('pos_last_raw', 0) - was.get('pos_last_raw', 0))
        if moved > bound:
            same = False
            detail.append('id%d moved %d ticks (%s bound %d)' % (
                servo, moved, 'wheel' if wheel else 'pos', bound))
        if moved > worst[0]:
            worst = (moved, servo, bound, 'wheel' if wheel else 'pos')
    if detail:
        return same, ', '.join(detail)
    # A passing row still reports the drift it saw: a resting wheel that settles a tick or two is
    # the expected behaviour, and hiding it would leave a later reader nothing to compare against.
    return same, 'identical, worst drift id%d %d of %d ticks (%s)' % (
        worst[1], worst[0], worst[2], worst[3])


def h5a(R, S, C):
    port_rows(R, S)
    R.ck('refused', not S.flag('controllers_active') and S.fact('spawner_rc') != '0',
         active=S.fact('controllers_active'), spawner_rc=S.fact('spawner_rc'),
         hw_state=S.fact('hw_state'))
    R.ck('fatal', S.logged('id 9', 'missing') == 1, lines_naming_id9=S.logged('id 9', 'missing'),
         expected=1)
    R.ck('port_released', S.fact('port_holders_running') == '',
         'while the CM still ran: [%s]' % S.fact('port_holders_running'))
    unchanged, detail = registers_unchanged(S, 'pre_readback', 'post_readback', (1, 2, 3, 4))
    R.ck('registers', unchanged, detail)
    R.row('cm_exit_code', 'NOTE', 'a fact, not a gate: %s' % S.fact('cm_exit_code', '?'))
    R.row('escape_hatch', 'NOTE', ESCAPE)


def h5b(R, S, C):
    port_rows(R, S)
    R.ck('active', S.flag('controllers_active'), active=S.fact('controllers_active'),
         spawner_rc=S.fact('spawner_rc'))
    warns = S.logged("unable to ping motor id '9'")
    R.ck('ping_warn', warns == 1, ping_warns=warns, expected=1)
    rec = S.rec('wheels')
    t, p, v = series(rec, 'joint5')
    worst = max([abs(x) for x in p + v], default=NAN)
    R.ck('phantom', bool(p) and worst <= 1e-9, max_abs=worst, bound=1e-9, n=len(p))
    names = joints_of(rec)
    stamps = series(rec, names[0])[0] if names else []
    R.ck('rate', abs(rate_hz(stamps) - 100.0) <= 2.0, '[no-regression]', hz=rate_hz(stamps),
         tol='100+-2')
    avg, reference = cycle_ms(S.txt('diagnostics.txt'), 'read')[0], C.get('h1_read_ms', NAN)
    if math.isnan(avg) or math.isnan(reference):
        R.row('read_ms', 'SKIP', 'no read_cycle.execution_time in H1 or here')
    else:
        R.ck('read_ms', avg <= reference + 0.15, '[no-regression]', read_ms=avg,
             h1_read_ms=reference, allowance=0.15)


def h5c(R, S, C):
    port_rows(R, S)
    R.ck('active', S.flag('controllers_active') and S.fact('hw_state') == 'active' and
         S.logged('unable to ping') == 0, active=S.fact('controllers_active'),
         hw_state=S.fact('hw_state'), unable_to_ping=S.logged('unable to ping'))
    final = final_of(S.rec('move_to_03'), 'joint1')
    R.ck('move', abs(final - 0.3) <= 0.005, final=final, target=0.3, tol=0.005)


def h6(R, S, C):
    port_rows(R, S)
    probe = S.rec('probe_while_active')
    R.ck('tiocexcl', probe.get('verdict') in ('refused_by_tiocexcl', 'refused_by_flock'),
         'acquired is a FAIL', verdict=probe.get('verdict', 'no_result'),
         open_errno=probe.get('open_errno'), flock_errno=probe.get('flock_errno'))
    rec = S.rec('probe_window')
    names = joints_of(rec)
    stamps = series(rec, names[0])[0] if names else []
    drift = max((max(series(rec, j)[1] or [0.0]) - min(series(rec, j)[1] or [0.0])
                 for j in ('joint1', 'joint2')), default=NAN)
    new_logs = S.number('driver_warns_after', -1) - S.number('driver_warns_before', -2)
    R.ck('undisturbed', abs(rate_hz(stamps) - 100.0) <= 2.0 and max_gap(stamps) <= 0.050 and
         drift <= 2 * TICK and new_logs == 0, hz=rate_hz(stamps), gap_s=max_gap(stamps),
         arm_drift=drift, drift_bound=2 * TICK, new_warn_error_lines=new_logs)
    for key, label, kind, excl in (('driver_refused_flock', 'flock_holder', 'flock-only', 'true'),
                                   ('driver_refused_excl', 'excl_holder', 'TIOCEXCL', 'false')):
        # The probe's own pid comes from its HOLDING line, printed the moment it owns the port;
        # the probe's RESULT line only exists once the hold is over.
        pid = S.fact('%s_probe_pid' % label)
        held = S.rec(label)
        took = held.get('verdict') == 'acquired' and str(held.get('no_exclusive')).lower() == excl
        # The refusal counts only if the holder still had the port; an expired hold is a run
        # fault, not a driver defect.
        within = S.fact('%s_refusal_within_hold' % label)
        R.ck(key.replace('driver_refused', 'refusal_within_hold'), within == 'true',
             'false means the hold expired before the driver reached on_configure; rerun it, the '
             'row below is then not evidence about the driver', within=within or 'unrecorded',
             probe_pid=pid or 'none')
        # Baseline: this holder's own pre readback, not the scenario's pre_readback, which H6's
        # own stack legitimately overwrote.
        unchanged, detail = registers_unchanged(S, '%s_pre' % label, '%s_post' % label,
                                                (1, 2, 3, 4))
        ok = (took and within == 'true' and
              S.fact('%s_controllers_active' % label) == 'false' and
              S.fact('%s_spawner_rc' % label) not in ('0', '') and
              S.number('%s_driver_refusals' % label, 0) == 1 and
              pid != '' and S.fact('%s_port_holders' % label) == pid and unchanged)
        if label == 'excl_holder':
            ok = ok and S.number('%s_serial_speed_lines' % label, -1) == 0
        R.ck(key, ok, 'registers: %s' % detail, holder=kind, probe_pid=pid or 'none',
             holder_verdict=held.get('verdict', 'no_result'),
             holder_no_exclusive=str(held.get('no_exclusive', 'unknown')).lower(),
             expected_no_exclusive=excl, refusal_within_hold=within or 'unrecorded',
             active=S.fact('%s_controllers_active' % label),
             spawner_rc=S.fact('%s_spawner_rc' % label),
             refusals=S.fact('%s_driver_refusals' % label),
             holders=S.fact('%s_port_holders' % label),
             serial_speed_lines=S.fact('%s_serial_speed_lines' % label, 'n/a'))
    released = S.rec('probe_after_release')
    R.ck('released', released.get('verdict') == 'acquired',
         'a refusal here is a leaked exclusive flag',
         verdict=released.get('verdict'), rc=released.get('rc'))


def h7(R, S, C):
    port_rows(R, S)
    moves = S.rec('pos_move')
    for joint in ('joint1', 'joint2'):
        final = final_of(moves, joint)
        R.ck('pos_reported.%s' % joint, abs(final - 0.6) <= 0.005,
             'inversion must be invisible above the driver', final=final, target=0.6, tol=0.005)
    centre = round(OFFSET * STEPS_PER_RAD)
    # joint2 and joint4 are the inverted half of each mirrored pair: a register is mirrored
    # exactly when `inverted` flips that quantity.
    mirror = {'position': -1 if flips('position') else 1,
              'velocity': -1 if flips('velocity') else 1}
    goal = {i: S.reg('pos_readback', i, 'goal_position_raw') for i in (1, 2)}
    want = {i: round((sign * 0.6 + OFFSET) * STEPS_PER_RAD)
            for i, sign in ((1, 1), (2, mirror['position']))}
    R.ck('pos_register', all(abs(goal[i] - want[i]) <= 1 for i in (1, 2)), id1=goal[1],
         expected1=want[1], id2=goal[2], expected2=want[2])
    R.ck('pos_symmetry', abs(goal[1] + goal[2] - 2 * centre) <= 2 and goal[1] - centre >= 300,
         centre=centre, d1=goal[1] - centre, d2=goal[2] - centre,
         sum=goal[1] + goal[2] - 2 * centre)
    spin = S.rec('vel_spin')
    for joint in ('joint3', 'joint4'):
        mean_v, n = mean_vel(spin, joint, 1.0, 0.5)
        R.ck('vel_reported.%s' % joint, abs(mean_v - 2.0) <= 0.020, mean=mean_v, expected='+2.0',
             tol=0.020, n=n)
        gate_rows(R, joint, *series(spin, joint), unwrap=True)
    ticks = round(2.0 * STEPS_PER_RAD)
    want4 = mirror['velocity'] * ticks
    speed = {i: S.reg('vel_readback', i, 'goal_speed_raw', 0) for i in (3, 4)}
    R.ck('vel_register', abs(speed[3] - ticks) <= 1 and abs(speed[4] - want4) <= 1, id3=speed[3],
         expected3=ticks, id4=speed[4], expected4=want4)
    slope = {i: S.regs('vel_readback', i).get('pos_slope_ls_rad_s', NAN) for i in (3, 4)}
    low, high = sorted((1.94 * mirror['velocity'], 2.06 * mirror['velocity']))
    R.ck('vel_physical', 1.94 <= slope[3] <= 2.06 and low <= slope[4] <= high,
         'the raw encoder walks in opposite directions for the same ROS command',
         raw_slope_id3=slope[3], raw_slope_id4=slope[4], id4_low=low, id4_high=high)
    load = S.rec('load_step')
    t_cmd = load.get('t_cmd', 0.0)
    loads = {}
    for joint in ('joint3', 'joint4'):
        t, y = iface(load, joint, 'load')
        loads[joint] = median([y[k] for k in between(t, t_cmd, t_cmd + 0.3)])
    third, fourth = loads['joint3'], loads['joint4']
    same_sign = flips('load')     # the whole construction of the row, taken from the constant
    if min(abs(third), abs(fourth)) >= 0.05:
        R.ck('load_sign', ((third > 0) == (fourth > 0)) == same_sign,
             'a mirrored pair commanded alike reports the same sign iff inverted flips load',
             L3=third, L4=fourth, expect_same_sign=same_sign)
    else:
        R.row('load_sign', 'INCONCLUSIVE', kv(L3=third, L4=fourth, floor=0.05) +
              ' load is not observable on a free-running bench; the flip is covered '
              'deterministically by the pty test')
    every, idle, amps, torques = [], [], [], []
    for joint in ('joint1', 'joint2', 'joint3', 'joint4'):
        every += iface(load, joint, 'load')[1]
        amps += iface(load, joint, 'current')[1]
        torques += iface(load, joint, 'effort')[1]
        if joint in ('joint1', 'joint2'):
            idle += [abs(x) for x in iface(load, joint, 'load')[1]]
    R.ck('load_range', bool(every) and max(abs(x) for x in every) <= 1.0 + 1e-9 and
         median(idle) <= 0.30, max_abs_load=max((abs(x) for x in every), default=NAN),
         bound=1.0, idle_median=median(idle), idle_bound=0.30)
    if not amps or max(amps) <= 0.0:
        R.row('effort_unsigned', 'SKIP', kv(max_current=max(amps) if amps else 'no samples') +
              ' no joint drew current, so the sign claim is not exercised')
    else:
        unsigned = not (flips('effort') or flips('current'))
        R.ck('effort_unsigned', unsigned and min(amps) >= 0.0 and min(torques) >= 0.0,
             'both must be >= 0 on every joint, the inverted ones included',
             min_current=min(amps), min_effort=min(torques), unsigned_family=unsigned)
    signs = []
    for joint in ('joint3', 'joint4'):
        v = median(series(load, joint)[2] or [NAN])
        current = median(iface(load, joint, 'current')[1] or [NAN])
        signs.append('%s sign(v)=%+d sign(I)=%+d' % (joint, 1 if v >= 0 else -1,
                                                     1 if current >= 0 else -1))
    R.row('effort_vs_velocity_sign', 'NOTE', 'never a gate; on an inverted joint the two '
          'deliberately disagree: ' + ', '.join(signs))


def h8(R, S, C):
    port_rows(R, S)
    for stack, speeds, acc in (('slow', {1: 652, 3: 1304}, 10), ('fast', {1: 2608, 3: 6000}, 150)):
        for servo, want in speeds.items():
            raw = S.reg('%s_readback' % stack, servo, 'goal_speed_raw')
            R.ck('speed_register.%s.id%d' % (stack, servo), abs(raw - want) <= 1,
                 goal_speed_raw=raw, expected=want)
        written = S.reg('%s_readback' % stack, 4, 'acc')
        R.ck('accel_register.%s.id4' % stack, written == acc, acc=written, expected=acc)
    mean_v, n = mean_vel(S.rec('slow_wheel'), 'joint3', 1.0, 0.5)
    R.ck('speed_wheel', abs(mean_v - 2.0) <= 0.060, commanded=8.0, max_speed=2.0, mean=mean_v,
         tol=0.060, n=n)
    slow = settle(S.rec('settle_slow'), 'joint1', 1.0)
    fast = settle(S.rec('settle_fast'), 'joint1', 1.0)
    ratio = slow / fast if fast else NAN
    R.ck('speed_arm', 0.85 <= slow <= 1.60 and fast <= 0.65 and ratio >= 2.0,
         'bounds 0.85..1.60, <=0.65, ratio>=2.0', t_slow=slow, t_fast=fast, ratio=ratio)
    # 2.7 rad/s is 90 % of the commanded 3.0, but it is not a reportable speed: t90 fires on the
    # next lattice value up, 1800 raw = 2.7612 rad/s = 92.0 % of command.  See T90_RATIO_MIN.
    slow = t90(S.rec('t90_slow'), 'joint4', 2.7)
    fast = t90(S.rec('t90_fast'), 'joint4', 2.7)
    ratio = slow / fast if fast else NAN
    facts = kv(t90_acc10=slow, t90_acc150=fast, ratio=ratio, fast_bound=T90_FAST_MAX,
               ratio_floor=T90_RATIO_MIN)
    if slow >= 1.2 and fast <= T90_FAST_MAX and ratio >= T90_RATIO_MIN:
        R.row('accel_effect', 'PASS', facts + ' ACC is obeyed in wheel mode')
    elif fast <= T90_FAST_MAX and slow <= T90_FAST_MAX and abs(ratio - 1.0) <= 0.15:
        R.row('accel_effect', 'INCONCLUSIVE', facts + ' the firmware ignores ACC in wheel mode; '
              'the register row still proves the value was written')
    else:
        R.row('accel_effect', 'FAIL', facts + ' read accel_register.slow.id4 and '
              'accel_register.fast.id4 first: those are exact equality rows and are the '
              'authoritative evidence that the ACC write arrived, so if they are green the '
              'driver is not implicated and this is a measurement failure. t90 is quantised - '
              'present_speed moves in 50 raw counts, 2.7 rad/s = 1760.1 raw is not reportable, '
              'so t90 fires on 1800 raw = 92 % of command, inside the velocity loop ring, and '
              'the ratio moves ~10 % run to run (see T90_RATIO_MIN). Suspect the driver only if '
              'a register row is red or the ratio is far outside 2.5..4.4')


def h9(R, S, C):
    port_rows(R, S)
    rec = S.rec('interfaces')
    real, ghost = ('joint1', 'joint2', 'joint3', 'joint4'), 'joint5'
    order = {j: ifaces_of(rec, j) for j in real}
    # Presence is gated; order is only a NOTE: the driver exports in description order and the
    # broadcaster reorders. See docs/configuration.md, "State interfaces".
    absent = {j: [i for i in IFACES if i not in order[j]] for j in real}
    R.ck('present', not any(absent.values()),
         '; '.join('%s=%s' % (j, ','.join(v) or 'none') for j, v in absent.items()),
         missing=sum(len(v) for v in absent.values()), of=len(real) * len(IFACES))
    R.row('order', 'NOTE', 'never a gate - the driver exports in description order '
          '(waveshare_servos.cpp on_export_state_interfaces), and /dynamic_joint_states ordering '
          "is the broadcaster's, not the driver's. declared=%s. seen %s"
          % (','.join(IFACES),
             '; '.join('%s=%s' % (j, ','.join(order[j])) for j in real)))
    listed = S.txt('hardware_interfaces.txt')
    missing = ['%s/%s' % (j, i) for j in real + (ghost,) for i in IFACES
               if '%s/%s' % (j, i) not in listed]
    R.ck('listed', not missing, ','.join(missing[:6]), missing=len(missing), of=45)
    values = {(j, i): iface(rec, j, i)[1] for j in real for i in IFACES}
    every = [x for column in values.values() for x in column]
    R.ck('finite', bool(every) and all(math.isfinite(x) for x in every), n=len(every),
         non_finite=sum(1 for x in every if not math.isfinite(x)))

    def column(name):
        return [x for j in real for x in values[(j, name)]]

    def bounded(key, name, low, high, spread=None):
        data = column(name)
        ok = bool(data) and min(data) >= low and max(data) <= high
        facts = kv(min=min(data) if data else NAN, max=max(data) if data else NAN, low=low,
                   high=high)
        if spread is not None:
            worst = max((max(values[(j, name)]) - min(values[(j, name)])
                         for j in real if values[(j, name)]), default=NAN)
            ok = ok and worst <= spread
            facts += ' ' + kv(worst_spread=worst, spread_bound=spread)
        R.row(key, 'PASS' if ok else 'FAIL', facts)

    bounded('position', 'position', -200.0, 200.0)
    bounded('velocity', 'velocity', -10.2, 10.2)
    bounded('temperature', 'temperature', 10.0, 70.0, spread=8.0)
    bounded('voltage', 'voltage', 5.0, 14.0, spread=1.0)
    bounded('effort_range', 'effort', -3.0, 3.0)
    idle = [abs(x) for j in real for x in iface(rec, j, 'current')[1][:500]]
    R.ck('current', bool(column('current')) and max(abs(x) for x in column('current')) <= 6.0 and
         median(idle) <= 0.35, max_abs_a=max((abs(x) for x in column('current')), default=NAN),
         bound=6.0, idle_median_a=median(idle), idle_bound=0.35)
    peak = max((abs(x) for x in column('effort')), default=NAN)
    R.ck('effort_nonzero', peak >= 0.02, max_abs_nm=peak, bound=0.02)
    pair = alias = 0.0
    for j in real:
        for torque, current in zip(values[(j, 'effort')], values[(j, 'current')]):
            pair = max(pair, abs(torque - current * TORQUE_CONSTANT_NM_PER_A))
        for torque, kgfcm in zip(values[(j, 'effort')], values[(j, 'torque')]):
            if kgfcm != 0.0:
                alias = max(alias, abs(torque / kgfcm - NM_PER_KGFCM))
    R.ck('effort_vs_current', pair <= 1e-9, 'a magnitude identity: neither side flips',
         worst=pair, bound=1e-9, k_t=TORQUE_CONSTANT_NM_PER_A)
    R.ck('torque_alias', alias <= 1e-6, worst=alias, bound=1e-6, nm_per_kgfcm=NM_PER_KGFCM)
    warned = [S.logged("joint '%s' declares the deprecated" % j) for j in real + (ghost,)]
    R.ck('deprecation_warn', all(n == 1 for n in warned),
         'warns per joint=%s, expected one each' % ','.join(str(n) for n in warned))
    status = column('status')
    R.ck('status_present', bool(status) and all(math.isfinite(x) for x in status), n=len(status),
         non_finite=sum(1 for x in status if not math.isfinite(x)))
    R.ck('status_zero', bool(status) and all(x == 0.0 for x in status),
         non_zero=sum(1 for x in status if x != 0.0), of=len(status))
    # The window starts at t_cycle_done (after both transitions); status_zero covers every
    # sample. A missing t_cycle_done fails the row instead of passing an empty window.
    done = S.number('t_cycle_done', NAN)
    after = []
    for j in real:
        t, y = iface(rec, j, 'status')
        after += [y[k] for k in between(t, done, math.inf)]
    leaked = sorted({x for x in after if x in (1.0, 2.0, 3.0, 4.0, 9.0)})
    R.ck('status_not_ping', not math.isnan(done) and not leaked,
         'values equal to a servo id after the inactive/active cycle came back: %s. %s'
         % (leaked, PING_DIAGNOSIS), t_cycle_done=done, n=len(after))
    cycle_rows(R, S, rec)
    wrong = []
    for name in IFACES:
        data = iface(rec, ghost, name)[1]
        if not data:
            wrong.append('%s missing' % name)
        elif name != 'status':
            if not all(math.isfinite(x) for x in data):
                wrong.append('%s not finite' % name)
            if name in ('position', 'velocity', 'effort') and any(x != 0.0 for x in data):
                wrong.append('%s not 0.0' % name)
    R.ck('phantom', not wrong, ', '.join(wrong) or 'finite, and zero where it must be (status '
         'alone may be NaN)')
    diag = S.txt('diagnostics.txt')
    read_avg, read_max = cycle_ms(diag, 'read')
    write_avg, write_max = cycle_ms(diag, 'write')
    stamps = series(rec, real[0])[0]
    if math.isnan(read_avg):
        R.row('cost', 'SKIP', 'no read_cycle.execution_time in /diagnostics')
    else:
        # Maxima gated (short capture). The rate term uses recorder stamps, so a recorder gap
        # can fail it. See docs/bench-check.md, "Component cycle (H9)".
        R.ck('cost', read_avg < READ_MS_AVG_MAX and write_avg < WRITE_MS_AVG_MAX and
             read_max < READ_MS_MAX_MAX and write_max < WRITE_MS_MAX_MAX and
             abs(rate_hz(stamps) - 100.0) <= 2.0,
             '[no-regression]', read_ms=read_avg, write_ms=write_avg, read_max=read_max,
             write_max=write_max, hz=rate_hz(stamps), interfaces_per_joint=9)
    plus, minus = [], []
    for j in ('joint3', 'joint4'):
        t, y = iface(rec, j, 'current')
        plus += [y[k] for k in between(t, S.number('t_plus0', 0), S.number('t_plus1', 0))]
        minus += [y[k] for k in between(t, S.number('t_minus0', 0), S.number('t_minus1', 0))]
        gate_rows(R, j, *series(rec, j), unwrap=True)
    for j in ('joint1', 'joint2'):
        gate_rows(R, j, *series(rec, j))
    counts = (median(plus) / CURRENT_PER_COUNT_A, median(minus) / CURRENT_PER_COUNT_A)
    if counts[0] >= 5 and counts[1] <= -5:
        verdict = 'signed_by_direction'
    elif counts[0] >= 5 and counts[1] >= 5:
        verdict = 'magnitude_only'
    else:
        verdict = 'inconclusive'
    R.row('current_sign_verdict', 'NOTE', kv(verdict=verdict, plus_counts=counts[0],
          plus_a=median(plus), minus_counts=counts[1], minus_a=median(minus)) +
          ' evidence, never a gate: signed_by_direction calls for a follow-up, it does not '
          'change the rule that current is an unsigned magnitude')


def h10(R, S, C):
    port_rows(R, S)
    R.ck('exit', S.fact('exit_code') == '0', exit_code=S.fact('exit_code'), expected=0)
    off, down = S.log.find('deactivate'), S.log.find('shutdown')
    R.ck('sequence', 0 <= off < down, deactivate_at=off, shutdown_at=down)
    stopped = S.rec('after_exit')
    R.ck('wheels', stopped.get('any_moving') is False, any_moving=stopped.get('any_moving'))
    R.ck('port_free', S.flag('port_free_within_10s'),
         'holders 10 s after exit=[%s]' % S.fact('port_holders_after_exit'))
    R.ck('retakeable', S.rec('probe_after_exit').get('verdict') == 'acquired',
         'released, not merely closed', verdict=S.rec('probe_after_exit').get('verdict'))
    # The second stimulus: SIGTERM to a bare ros2_control_node with a wheel turning.
    R.ck('term_exit', S.fact('term_exit_code') == '0', exit_code=S.fact('term_exit_code'),
         expected=0)
    R.ck('term_wheels', S.rec('term_after_exit').get('any_moving') is False and
         S.flag('term_port_free'), any_moving=S.rec('term_after_exit').get('any_moving'),
         port_free=S.fact('term_port_free'))
    rec = S.rec('shutdown')
    marks = [float(x) for x in S.fact('transitions').split() if x]
    runs, detail, late = [], [], 0
    for joint in joints_of(rec):
        t, p, v = series(rec, joint)
        for first, last in stale_runs(t, p, v):
            near = min((abs(t[first] - mark) for mark in marks), default=math.inf)
            runs.append(t[last] - t[first])
            late += near > 1.0
            detail.append('%s %.3fs %.2fs from a transition' % (joint, runs[-1], near))
    R.ck('freeze', len(runs) <= 2 and all(x <= 0.40 for x in runs) and not late,
         '; '.join(detail), runs=len(runs), bound=2, longest=max(runs, default=0.0),
         longest_bound=0.40, not_near_a_transition=late)


def h11(R, S, C):
    port_rows(R, S)
    R.ck('active', S.flag('controllers_active') and S.fact('hw_state') == 'active' and
         S.logged('unable to ping') == 0, active=S.fact('controllers_active'),
         hw_state=S.fact('hw_state'), unable_to_ping=S.logged('unable to ping'))
    totals = bus_totals(S.log)
    if totals is None:
        # All four rows, not three: a row that is simply absent is invisible in the report.
        R.row('transactions', 'SKIP', 'no "bus totals:" line; the stack was killed, not stopped')
        R.row('fail_rate', 'SKIP', 'no "bus totals:" line')
        R.row('no_drop', 'SKIP', 'no "bus totals:" line')
        R.row('per_servo', 'SKIP', 'no "bus totals:" line')
    else:
        n, bad, worst, dropped = totals
        pairs = bus_totals_tail(S.log)
        named = ('[%s]' % ', '.join('id%d %d' % pair for pair in pairs) if pairs
                 else '[no per-servo tail]')
        soak = S.number('soak_s', NAN)
        expected = 100.0 * soak                       # the bench runs every scenario at 100 Hz
        R.ck('transactions', n >= 0.95 * expected, seen=n, expected=expected, soak_s=soak)
        if n < SOAK_MIN_TRANSACTIONS:
            R.row('fail_rate', 'SKIP', kv(transactions=n, need=SOAK_MIN_TRANSACTIONS))
        else:
            rate = fail_per_million(n, bad)
            # The per-servo tail rides in the detail, so a FAIL names the failing ids.
            R.ck('fail_rate', rate <= SOAK_FAIL_PER_MILLION_MAX and
                 worst <= SOAK_WORST_CONSECUTIVE_MAX, named, transactions=n, failed=bad,
                 per_million=rate, bound=SOAK_FAIL_PER_MILLION_MAX, worst_consecutive=worst,
                 worst_bound=SOAK_WORST_CONSECUTIVE_MAX)
        R.ck('no_drop', dropped == 0 and S.logged('stopped answering') == 0, dropped=dropped,
             drop_lines=S.logged('stopped answering'))
        # The per-servo tail is required. Presence only: nothing here says how many servos the
        # run declared.
        R.ck('per_servo', bool(pairs), named, servos=len(pairs))
    rec = S.rec('steady_soak')
    names = joints_of(rec)
    stamps = series(rec, names[0])[0] if names else []
    diag = S.txt('diagnostics.txt')
    read_avg, read_max = cycle_ms(diag, 'read')
    write_avg, write_max = cycle_ms(diag, 'write')
    if math.isnan(read_avg):
        R.row('cost', 'SKIP', 'no read_cycle.execution_time in /diagnostics')
    else:
        # Averages and rate only: the maxima span ~60 000 cycles here, far beyond the short
        # windows the *_MAX_MAX bounds came from.
        R.ck('cost', read_avg < READ_MS_AVG_MAX and write_avg < WRITE_MS_AVG_MAX and
             abs(rate_hz(stamps) - 100.0) <= 2.0, '[no-regression]', read_ms=read_avg,
             write_ms=write_avg, hz=rate_hz(stamps), read_max_ungated=read_max,
             write_max_ungated=write_max)
    R.ck('gap', max_gap(stamps) <= 0.050, hz_gap_s=max_gap(stamps), bound=0.050)
    # The soak's load: both wheels must turn (one lost `topic pub` leaves them still). A NaN
    # mean (no samples) fails.
    turning = [mean(series(rec, joint)[2]) for joint in ('joint3', 'joint4')]
    R.ck('wheels_turning', all(abs(w) >= SOAK_WHEEL_MIN_RAD_S for w in turning),
         joint3=turning[0], joint4=turning[1], bound=SOAK_WHEEL_MIN_RAD_S)
    R.row('cost_max', 'NOTE', kv(read_max=read_max, write_max=write_max) +
          ' cumulative since activation over the whole soak; UNMEASURED as a bound -- three '
          'clean soaks are needed to characterise a 60000-cycle extreme')
    # One dict comprehension, not a comprehension over (name, value) pairs and not dict() over a
    # generator: flake8-comprehensions rejects both of those (C416, C402) and lint is a test here.
    hot = {j: max(iface(rec, j, 'temperature')[1], default=NAN) for j in names}
    R.row('temperature', 'NOTE', kv(**hot) +
          ' evidence, never a gate: no thermal baseline for a ten-minute spin has been measured')
    for joint in ('joint3', 'joint4'):
        gate_rows(R, joint, *series(rec, joint), unwrap=True)
    for joint in ('joint1', 'joint2'):
        gate_rows(R, joint, *series(rec, joint))


# See docs/bench-check.md, "Command limits (H12)".
def h12(R, S, C):
    """
    Gate H12: the controller manager's JointSaturationLimiter clamps at the rendered <limit>.

    Each command lies between <limit> and the driver's own ceiling, so only the limiter clamps.
    """
    port_rows(R, S)
    # Exact equality with the constants (same decimal text), so a mismatch is real drift; a
    # missing fact is NaN and fails.
    stimulus = (('arm_pos_limit', H12_ARM_LIMIT), ('arm_command', H12_ARM_COMMAND),
                ('wheel_vel_limit', H12_WHEEL_LIMIT), ('wheel_command', H12_WHEEL_COMMAND))
    drifted = ['%s=[%s] wanted %s' % (key, S.fact(key), want)
               for key, want in stimulus if S.number(key) != want]
    R.ck('stimulus', not drifted,
         'the four numbers h_H12 rendered and published: %s'
         % (', '.join(drifted) or 'all four agree with the constants in this file'),
         **{key: S.number(key) for key, _ in stimulus})
    # The park stack came up and stopped cleanly (TERM -> exit 0; 137 = killed). Repeats h_H12's
    # guard on purpose, and is the only gate on park_cm_exit_code.
    R.ck('park', S.fact('park_spawner_rc') == '0' and S.flag('park_controllers_active') and
         S.fact('park_cm_exit_code') == '0',
         'the limiters-off stack that parks the arm before the clamp test',
         spawner_rc=S.fact('park_spawner_rc') or "''",
         controllers_active=S.fact('park_controllers_active') or "''",
         cm_exit_code=S.fact('park_cm_exit_code') or "''")
    # Where the arm really rests, read off the servos with the port free: raw * TICK - OFFSET
    # (both arm joints are non-inverted). Must be within H12_PARK_MARGIN of 0.
    for servo in (1, 2):
        raw = S.regs('arm_pre', servo).get('pos_last_raw', -1)
        rad = raw * TICK - OFFSET
        R.ck('arm_inside_limits.id%d' % servo, raw >= 0 and abs(rad) <= H12_PARK_MARGIN,
             'the trap-(c) precondition: the limiter throws 0.0087 rad outside the +-%g <limit> '
             'and the controller manager then deactivates the arm instead of clamping'
             % H12_ARM_LIMIT,
             pos_last_raw=raw, rad=rad, margin=H12_PARK_MARGIN)
    # Own keys: h_H12 runs three stacks and the shared facts hold the last one's. Tells "no
    # clamp" apart from "no stack".
    R.ck('limited_stack', S.fact('limited_spawner_rc') == '0' and
         S.flag('limited_controllers_active'),
         'the enforce_command_limits stack (bench_limits.yaml) came up',
         spawner_rc=S.fact('limited_spawner_rc') or "''",
         controllers_active=S.fact('limited_controllers_active') or "''",
         cm_exit_code=S.fact('limited_cm_exit_code') or "''")
    # From the limited stack's own log (log.txt mixes all three stacks). Compared sorted: the
    # lines come in hash-map order, but exactly one per joint is required.
    limited_log = S.fact('limited_cm_log')
    found = LIMITER_LINE.findall(S.txt(limited_log))
    joints = sorted(name for name, _ in found)
    R.ck('limiter_lines', joints == sorted(H12_JOINTS),
         'one JointSaturationLimiter per joint, matched as a set because these come out in '
         'hash-map order; without it every clamp row below could be met by a limiters-off stack',
         log=limited_log or "''", joints=','.join(joints) or 'none',
         expected=','.join(sorted(H12_JOINTS)),
         hardware=','.join(sorted({hw for _, hw in found})) or 'none')
    # Final sample after 2.5 s of post-roll: 0.8 (limiter), 1.2 (no clamp) or 1.5708 (driver).
    # 0.8 is 0.4 rad (40x the tolerance) from the nearest other; `max` is reported, not gated.
    arm = S.rec('pos_clamp')
    final, samples = final_of(arm, 'joint1'), series(arm, 'joint1')[1]
    R.ck('pos_clamp', abs(final - H12_ARM_LIMIT) <= H12_POS_TOL,
         'clamped at the rendered <limit>, not at the command and not at the driver ceiling',
         final=final, clamped_at=H12_ARM_LIMIT, tol=H12_POS_TOL, commanded=H12_ARM_COMMAND,
         driver_ceiling=POS_CMD_LIMIT, max=max(samples) if samples else NAN)
    # Measured as H8.speed_wheel (mean over t_cmd + 1.0 .. t_stop - 0.5, same tolerance). No
    # stop is published here; wheels_stopped proves the stop.
    wheels = S.rec('vel_clamp')
    for joint in ('joint3', 'joint4'):
        mean_v, n = mean_vel(wheels, joint, 1.0, 0.5)
        R.ck('vel_clamp.%s' % joint, abs(mean_v - H12_WHEEL_LIMIT) <= H12_VEL_TOL,
             'clamped at the rendered <limit velocity>, not at the command',
             mean=mean_v, clamped_at=H12_WHEEL_LIMIT, tol=H12_VEL_TOL,
             commanded=H12_WHEEL_COMMAND, driver_ceiling=V_CAP, n=n)
    # The driver's write path, read off the servo after SIGKILL: 2.0 rad/s = 1303.797 counts
    # (+-1 rounding); an unclamped 8.0 rad/s would be 5215.
    want = round(H12_WHEEL_LIMIT * STEPS_PER_RAD)
    for servo in (3, 4):
        raw = S.reg('vel_readback', servo, 'goal_speed_raw')
        R.ck('vel_register.id%d' % servo, abs(raw - want) <= 1,
             "the driver's own last write of the clamped command, independent of the read path",
             goal_speed_raw=raw, expected=want,
             unclamped=round(H12_WHEEL_COMMAND * STEPS_PER_RAD))
    # A limiter throw comes at the first enforcement, not at activation, and leaves pos_clamp
    # meaningless; this row makes the throw visible.
    state = S.fact('arm_state_after')
    R.ck('arm_state_after', state == 'active',
         'the arm controller survived the clamp; a trap-(c) throw deactivates it and makes every '
         'other H12 row unreadable',
         state=state or "''", expected='active')
    # Read off the bus after the SIGKILL left a goal speed standing: a latched goal speed can
    # outlive its process. See docs/operation.md, "Safety".
    stopped = S.rec('vel_stop')
    R.ck('wheels_stopped', stopped.get('any_moving') is False,
         'the SIGKILL left a goal speed standing; this is the read that says it was cleared',
         any_moving=stopped.get('any_moving'))
    # The arm off its limit, parked by a limiters-off stack (park_final). Its three stack facts
    # ride in the detail as the explanation if the park failed.
    for servo in (1, 2):
        raw = S.regs('arm_post', servo).get('pos_last_raw', -1)
        rad = raw * TICK - OFFSET
        R.ck('arm_parked_after.id%d' % servo, raw >= 0 and abs(rad) <= H12_PARK_MARGIN,
             'the bench was handed back parked rather than resting on the tightened limit, '
             'which is 5.7 ticks from the trap-(c) throw',
             pos_last_raw=raw, rad=rad, margin=H12_PARK_MARGIN,
             park_final_spawner_rc=S.fact('park_final_spawner_rc') or "''",
             park_final_controllers_active=S.fact('park_final_controllers_active') or "''",
             park_final_cm_exit_code=S.fact('park_final_cm_exit_code') or "''")


# Tool scenarios H13-H17: scan, refusals, set_id, calibrate_midpoint, and the EEPROM as found.
# See docs/bench-check.md, "Tool scenarios (H13 to H17)".

# Ids that answer on this bench (the tool scenarios gate on these); equal to EXAMPLE_IDS only
# because the example was written for this bench.
BENCH_IDS = EXAMPLE_IDS
BENCH_MODES = {1: 0, 2: 0, 3: 1, 4: 1}         # register 33: two arms (0), two wheels (1)
SAFE_SILENT_IDS = (200, 201, 44)               # H14's write targets; 44 is 300 narrowed to 8 bits
TEMP_ID = 253                                  # H15's temporary id: the top of scan's range
CALIB_ID = 2                                   # H16 calibrates the arm that rests near 1026
WHEEL_ID = 3                                   # ... and is refused on this wheel
MIDPOINT_TOL = 3                               # ticks, servo_tools.hpp kMidpointTolTicks
# Scan time: ~250 silent ids x 3 attempts x 5 ms. The floor catches lost retries, the ceiling
# a long timeout; a narrowed id range is H15.scan_sees_move's job.
SCAN_FLOOR_S = 0.9 * 250 * 3 * 0.005
SCAN_CEIL_S = 12.0
EEPROM_ADDRS = (0, 1) + tuple(range(3, 40))    # every EEPROM byte hil_eeprom reads; 2 is undefined


# scan's stdout: a header of these 11 columns, one 11-token row per servo, then the footer.
# `?` is legal in any column but id. See docs/tools.md, "scan".
SCAN_COLUMNS = ('id', 'type', 'mode', 'model', 'baud_reg', 'baud', 'position', 'voltage_V',
                'temp_C', 'status', 'offset')
SCAN_FOOTER = re.compile(r'found (no servo|\d+ servo\(s\)) on \S+ at \d+ baud(: ids( \d+)+)? '
                         r'\(pinged (ids \d+\.\.\d+|no id), \d+ attempts each, [\d.]+ s\)'
                         r'(, interrupted (after|before) id \d+)?$')
DETAIL_LINE = re.compile(r'^[a-z_]+: detail (.*)$')


def scan_table(text):
    """
    Parse scan's stdout into (header_ok, rows_by_id, bad_lines, footer).

    Rows are parsed even when the header is wrong, so a drifted header fails only H13.table.
    """
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    header_ok = bool(lines) and tuple(lines[0].split()) == SCAN_COLUMNS
    footer = lines[-1] if lines and SCAN_FOOTER.match(lines[-1]) else None
    rows, bad = {}, []
    for line in lines[1 if header_ok else 0:-1 if footer else None]:
        tokens = line.split()
        if len(tokens) == len(SCAN_COLUMNS) and tokens[0].isdigit() and int(tokens[0]) not in rows:
            rows[int(tokens[0])] = dict(zip(SCAN_COLUMNS, tokens))
        else:
            bad.append(line)
    return header_ok, rows, bad, footer


def tool_detail(err_text):
    """Return the key=value fields of a tool's last `<tool>: detail` line, or None."""
    found = None
    for line in err_text.splitlines():
        match = DETAIL_LINE.match(line.strip())
        if match:
            found = dict(field.split('=', 1) for field in match.group(1).split() if '=' in field)
    return found


def _int(text):
    """Return the text as an int, or None when it is not one (`?`, `none`, `busy`, '')."""
    text = str(text).strip()
    return int(text) if re.fullmatch(r'-?\d+', text) else None


def int_fact(S, key):
    """Return the fact as an int, or None when it is missing or not a number (`none`, `busy`)."""
    return _int(S.fact(key))


def decode11(raw):
    """Sign-magnitude on bit 11, the offset register's convention (servo_tools offset_from_raw)."""
    return -(raw & ~0x800) if raw & 0x800 else raw


def decode15(raw):
    """Sign-magnitude on bit 15, the present-position register's convention (ReadPos)."""
    return -(raw & 0x7fff) if raw & 0x8000 else raw


def tick_distance(a, b):
    """Return the distance between two encoder readings on the 4096-tick circle."""
    d = abs(a - b) % ENCODER_STEPS
    return min(d, ENCODER_STEPS - d)


def _ids(values):
    return ' '.join(str(i) for i in values) or 'none'


def snap_file(path):
    """
    Parse a hil_eeprom .snap file into the shape of its RESULT json.

    An unreadable value stays 'x'; a missing or foreign file parses to {} (a difference).
    """
    lines = _text(path).splitlines()
    if not lines or lines[0].strip() != 'hil_eeprom snapshot 1':
        return {}
    snap = {'ok': False, 'census': None, 'ids': {}}
    for line in lines[1:]:
        tokens = line.split()
        if tokens[:1] == ['ok']:
            snap['ok'] = tokens[1:] == ['true']
        elif tokens[:1] == ['census']:
            census_ids = [_int(token) for token in tokens[1:]]
            snap['census'] = census_ids if None not in census_ids else None
        elif tokens[:1] == ['servo'] and len(tokens) > 2 and tokens[1].isdigit():
            block = snap['ids'].setdefault(tokens[1], {'eeprom': {}, 'sram': {}, 'volatile': {}})
            if tokens[2] == 'eeprom':
                block['eeprom'] = {str(reg): 'x' if _int(value) is None else _int(value)
                                   for reg, value in enumerate(tokens[3:]) if reg != 2}
                block['n'] = sum(1 for value in block['eeprom'].values() if value != 'x')
            elif tokens[2] in ('sram', 'volatile'):
                pairs = (token.split('=', 1) for token in tokens[3:] if '=' in token)
                block[tokens[2]] = {reg: 'x' if _int(value) is None else _int(value)
                                    for reg, value in pairs}
    return snap


def _block(snap, servo):
    return (snap.get('ids') or {}).get(str(servo)) or {}


def _reg(block, reg):
    """One register of a snapshot block: EEPROM below 40, else SRAM; None when absent."""
    return (block.get('eeprom' if reg < 40 else 'sram') or {}).get(str(reg))


def _word(block, first):
    low, high = _reg(block, first), _reg(block, first + 1)
    return low | high << 8 if isinstance(low, int) and isinstance(high, int) else None


def _snapshot_problems(snap):
    """Say why a snapshot as a whole is not evidence: none at all, an error, ok not true."""
    if not snap:
        return ['no snapshot']
    why = ['error %s' % snap['error']] if snap.get('error') else []
    return why + ([] if snap.get('ok') is True else ['ok is %s' % snap.get('ok')])


def _unreadable_values(snap):
    """Name every value of every servo in the snapshot that is not a number (hil_eeprom's 'x')."""
    return ['id %s %s %s unreadable' % (servo, part, reg)
            for servo, block in sorted((snap.get('ids') or {}).items(), key=lambda s: int(s[0]))
            for part in ('eeprom', 'sram', 'volatile')
            for reg, value in sorted((block.get(part) or {}).items(), key=lambda r: int(r[0]))
            if not isinstance(value, int)]


def _diffs(a, b, ids, regs, with_census=False):
    """
    Compare two loaded snapshots register by register; return [{'at': (id, reg) or None, 'text'}].

    Never vacuous: a bad snapshot, a missing id or an unreadable value is a difference, at=None.
    """
    out = [{'at': None, 'text': '%s: %s' % (side, why)}
           for side, snap in (('A', a), ('B', b)) for why in _snapshot_problems(snap)]
    for servo in ids:
        x, y = _block(a, servo), _block(b, servo)
        if not x or not y:
            out.append({'at': None,
                        'text': 'id %s: missing in %s' % (servo, 'A' if not x else 'B')})
            continue
        for reg in regs:
            p, q = _reg(x, reg), _reg(y, reg)
            if not isinstance(p, int) or not isinstance(q, int):
                out.append({'at': None, 'text': 'id %s reg %d: unreadable (%s -> %s)'
                            % (servo, reg, p, q)})
            elif p != q:
                out.append({'at': (servo, reg), 'text': 'id %s reg %d: %s -> %s'
                            % (servo, reg, p, q)})
    if with_census and (not isinstance(a.get('census'), list) or
                        a.get('census') != b.get('census')):
        out.append({'at': None, 'text': 'census: %s -> %s' % (a.get('census'), b.get('census'))})
    return out


def eeprom_diff(S, a, b, ids, eeprom_only, census=False):
    """
    Return the differences between two snapshots over EEPROM, plus 40 and 55 unless eeprom_only.

    An empty list means both were read and are equal (the censuses too, with census=True).
    """
    loaded = [S.rec(x) if isinstance(x, str) else x for x in (a, b)]
    regs = EEPROM_ADDRS + (() if eeprom_only else (40, 55))
    return [d['text'] for d in _diffs(loaded[0], loaded[1], ids, regs, census)]


def census(S, label):
    """Return a snapshot's census as a list of ids, or None when it was not taken or not read."""
    got = S.rec(label).get('census')
    return got if isinstance(got, list) and all(isinstance(i, int) for i in got) else None


def holders_left(S, allowed):
    """
    Name each tool run whose `<label>_holders_after` holds a pid not allowed for that label.

    A holder from before the tool is allowed; any other pid, or a missing fact, is a leak.
    """
    left = []
    for label, pids in allowed.items():
        key = '%s_holders_after' % label
        if key not in S.facts:
            left.append('%s unrecorded' % key)
        else:
            left += ['%s=%s' % (key, pid) for pid in S.fact(key).split() if pid not in pids]
    return left


def journal_row(R, S):
    """Emit the journal row of H13-H16: guard_end found the bench unchanged, or restored it."""
    R.ck('journal_cleared', S.fact('journal_left') == 'false',
         "true: guard_end could not restore the bench and kept the journal; '': guard_end never "
         'ran', journal_left=S.fact('journal_left') or "''")


def scan_fields(row, block):
    """Say where one scan row disagrees with the servo's snapshot block (H13.fields); [] if not."""
    if row is None:
        return ['no row in the table']
    if not block:
        return ['no servo in the snapshot']
    mode, volatile = _reg(block, 33), block.get('volatile') or {}
    offset = _word(block, 31)
    wrong = []
    for column, got, want in (
            ('model', _int(row['model']), _word(block, 3)),
            ('mode', _int(row['mode']), mode),
            ('type', row['type'], {0: 'pos', 1: 'vel'}.get(mode, '-')),
            ('baud_reg', _int(row['baud_reg']), _reg(block, 6)),
            ('offset', _int(row['offset']), None if offset is None else decode11(offset))):
        if got is None or want is None or got != want:
            wrong.append('%s %s, snapshot %s' % (column, row[column], want))
    rest = REST_TICKS if mode == 0 else WHEEL_REST_TICKS
    position, at = _int(row['position']), volatile.get('56')
    if position is None or not isinstance(at, int) or tick_distance(position, decode15(at)) > rest:
        wrong.append('position %s, snapshot %s (bound %d)' % (row['position'], at, rest))
    try:
        volts = float(row['voltage_V'])
    except ValueError:
        volts = NAN
    if not isinstance(volatile.get('62'), int) or not abs(volts - 0.1 * volatile['62']) <= 0.2001:
        wrong.append('voltage_V %s, snapshot %s (bound 0.2 V)' % (row['voltage_V'],
                                                                  volatile.get('62')))
    temp = _int(row['temp_C'])
    if temp is None or not isinstance(volatile.get('63'), int) or abs(temp - volatile['63']) > 2:
        wrong.append('temp_C %s, snapshot %s (bound 2)' % (row['temp_C'], volatile.get('63')))
    return wrong


def h13(R, S, C):
    """
    Gate scan on the bench: one scan, cross-checked against hil_eeprom's independent snapshot.

    The two share nothing above ServoBus; read_only proves that no scan wrote anything.
    """
    port_rows(R, S)
    journal_row(R, S)
    R.ck('rc', int_fact(S, 'scan_rc') == 0, rc=S.fact('scan_rc') or "''", expected=0)
    out = S.txt('scan.out.txt')
    header_ok, rows, bad, footer = scan_table(out)
    R.ck('table', header_ok and footer is not None and not bad,
         'the header, rows of 11 tokens, the footer, and nothing else on stdout',
         header=header_ok, rows=len(rows), footer=footer is not None, bad_lines=len(bad),
         first_bad=repr(bad[0]) if bad else 'none')
    ids = sorted(rows)
    R.ck('ids', ids == list(BENCH_IDS), ids=_ids(ids), expected=_ids(BENCH_IDS),
         missing=_ids(i for i in BENCH_IDS if i not in rows),
         unlisted=_ids(i for i in ids if i not in BENCH_IDS))
    pre = S.rec('pre_eeprom')
    unsound = _snapshot_problems(pre) + _unreadable_values(pre)
    R.ck('census_agrees', not unsound and census(S, 'pre_eeprom') == ids,
         '; '.join(unsound) or 'the table and hil_eeprom census agree',
         table=_ids(ids), census=_ids(census(S, 'pre_eeprom') or []))
    for servo in BENCH_IDS:
        wrong = unsound + scan_fields(rows.get(servo), _block(pre, servo))
        R.ck('fields.%d' % servo, not wrong, '; '.join(wrong) or
             'model, mode and type, baud_reg, offset, position, voltage and temperature agree '
             'with pre_eeprom')
    table_modes = {i: _int(rows[i]['mode']) for i in BENCH_IDS if i in rows}
    snap_modes = {i: _reg(_block(pre, i), 33) for i in BENCH_IDS}
    R.ck('modes', bool(table_modes) and snap_modes == BENCH_MODES and
         all(mode == BENCH_MODES[i] for i, mode in table_modes.items()),
         'table=%s snapshot=%s expected=%s' % (table_modes, snap_modes, BENCH_MODES))
    seconds = S.number('scan_seconds')
    R.ck('timing', SCAN_FLOOR_S <= seconds <= SCAN_CEIL_S,
         'catches a lost retry or a long timeout, NOT a narrowed range (H15.scan_sees_move does)',
         seconds=seconds, floor=SCAN_FLOOR_S, ceiling=SCAN_CEIL_S)
    lines, on_stdout = int_fact(S, 'scan_serial_speed_lines'), out.count('serial speed')
    R.ck('serial_speed', lines == 1 and on_stdout == 0,
         'opened once, through open_bus, which sends the vendored line to stderr',
         lines=S.fact('scan_serial_speed_lines') or "''", expected=1, on_stdout=on_stdout)
    if S.fact('scan_default_skipped'):
        R.row('default_params', 'SKIP', S.fact('scan_default_skipped'))
    else:
        _, default_rows, _, _ = scan_table(S.txt('scan_default.out.txt'))
        same = bool(rows) and sorted(default_rows) == ids and all(
            default_rows[i]['mode'] == rows[i]['mode'] and
            default_rows[i]['model'] == rows[i]['model'] for i in ids)
        R.ck('default_params', int_fact(S, 'scan_default_rc') == 0 and same,
             "no parameter at all: the defaults are the hardware parameters' defaults",
             rc=S.fact('scan_default_rc') or "''", ids=_ids(sorted(default_rows)),
             explicit_ids=_ids(ids))
    for key, label in (('usage.stale', 'stale_scan'), ('usage.positional', 'positional_scan')):
        R.ck(key, int_fact(S, label + '_rc') == 64 and
             int_fact(S, label + '_serial_speed_lines') == 0,
             'refused before the port was opened', rc=S.fact(label + '_rc') or "''",
             serial_speed_lines=S.fact(label + '_serial_speed_lines') or "''")
    diffs = eeprom_diff(S, 'pre_eeprom', 'post_eeprom', BENCH_IDS, False, census=True)
    R.ck('read_only', not diffs, '; '.join(diffs[:6]) or
         'EEPROM, 40, 55 and census identical before and after the four scans', diffs=len(diffs))
    runs = ['scan', 'stale_scan', 'positional_scan']
    runs += [] if S.fact('scan_default_skipped') else ['scan_default']
    left = holders_left(S, {label: () for label in runs})
    probe = S.rec('released')
    R.ck('released', probe.get('verdict') == 'acquired' and not left, ', '.join(left),
         verdict=probe.get('verdict', 'no_result'))
    R.row('model', 'NOTE', 'registers 3-4 as scan prints them, raw: %s' % ' '.join(
        'id%d=%s' % (i, rows[i]['model']) for i in ids))
    R.row('firmware', 'NOTE', 'registers 0-1: %s' % ' '.join(
        'id%d=%s.%s' % (i, _reg(_block(pre, i), 0), _reg(_block(pre, i), 1)) for i in BENCH_IDS))
    R.row('lock', 'NOTE', 'register 55 before the scans: %s' % ' '.join(
        'id%d=%s' % (i, _reg(_block(pre, i), 55)) for i in BENCH_IDS))


H14_TOOLS = (('scan', 'scan'), ('set_id', 'set_id'), ('calibrate', 'calibrate_midpoint'))
H14_USAGE = ('stale', 'bad_type', 'range', 'cal_range', 'missing', 'positional', 'foreign_node')


def _names_pid(text, pid):
    return bool(pid) and re.search(r'\bpid %s\b' % re.escape(pid), text) is not None


def h14(R, S, C):
    """
    Gate every tool's refusals, which must all write nothing.

    Stage A: the CM holds the port; B: a flock-only probe holds it; C: port free, bad arguments.
    """
    port_rows(R, S)
    journal_row(R, S)
    R.ck('cm_up', S.flag('controllers_active'),
         'without the stack every cm_* row below would be vacuous',
         controllers_active=S.fact('controllers_active') or "''")
    cm_pids = S.fact('cm_pids').split()
    for short, tool in H14_TOOLS:
        label = 'cm_' + short
        err = S.txt(label + '.err.txt')
        named = [pid for pid in cm_pids if _names_pid(err, pid)]
        R.ck('cm_refused.' + tool, int_fact(S, label + '_rc') == 1 and
             int_fact(S, label + '_serial_speed_lines') == 0 and
             'held by another process' in err and bool(named),
             rc=S.fact(label + '_rc') or "''",
             serial_speed_lines=S.fact(label + '_serial_speed_lines') or "''",
             cm_pids=_ids(cm_pids), named=_ids(named))
    rec = S.rec('tools_window')
    names = joints_of(rec)
    stamps = series(rec, names[0])[0] if names else []
    drift = max((max(series(rec, j)[1] or [0.0]) - min(series(rec, j)[1] or [0.0])
                 for j in ('joint1', 'joint2')), default=NAN)
    new_logs = S.number('driver_warns_after', -1) - S.number('driver_warns_before', -2)
    R.ck('cm_undisturbed', bool(rec) and abs(rate_hz(stamps) - 100.0) <= 2.0 and
         max_gap(stamps) <= 0.050 and drift <= 2 * TICK and new_logs == 0,
         'the H6 terms over the 12 s the tools ran against the live stack',
         hz=rate_hz(stamps), gap_s=max_gap(stamps), arm_drift=drift, drift_bound=2 * TICK,
         new_warn_error_lines=new_logs)
    probe_pid = S.fact('probe_pid').strip()
    for short, tool in H14_TOOLS:
        label = 'fl_' + short
        rc, lines = int_fact(S, label + '_rc'), int_fact(S, label + '_serial_speed_lines')
        named = _names_pid(S.txt(label + '.err.txt'), probe_pid)
        R.ck('flock_refused.' + tool, rc == 1 and named, rc=S.fact(label + '_rc') or "''",
             probe_pid=probe_pid or "''", names_it=named)
        R.ck('via_servobus.' + tool, rc == 1 and named and lines == 0,
             'refused by the lock before begin(): no `serial speed` line on either stream',
             rc=S.fact(label + '_rc') or "''", names_probe=named,
             serial_speed_lines=S.fact(label + '_serial_speed_lines', 'missing') or "''")
    alive = {short: S.fact('fl_%s_holder_alive' % short) for short, _ in H14_TOOLS}
    held = bool(probe_pid) and all(value == 'true' for value in alive.values())
    # ABORTED, never PASS or FAIL: an expired holder makes the flock rows no evidence at all.
    R.row('holder_alive_throughout', 'PASS' if held else 'ABORTED',
          kv(probe_pid=probe_pid or "''", **{k: v or "''" for k, v in alive.items()}) +
          ('' if held else ' the flock-only stimulus did not last through stage B, so the '
                           'flock_refused and via_servobus rows are not evidence; rerun H14'))
    for label in H14_USAGE:
        R.ck('usage.' + label, int_fact(S, label + '_rc') == 64 and
             int_fact(S, label + '_serial_speed_lines') == 0,
             'refused before the port was opened', rc=S.fact(label + '_rc') or "''",
             serial_speed_lines=S.fact(label + '_serial_speed_lines') or "''")
    R.ck('refuse_taken', int_fact(S, 'taken_rc') == 4 and
         'id 3 already answers' in S.txt('taken.err.txt'),
         'new id 3 answers, so the taken check refuses before anything reaches the silent 200',
         rc=S.fact('taken_rc') or "''", expected=4)
    R.ck('refuse_silent_start', int_fact(S, 'silent_start_rc') == 3,
         rc=S.fact('silent_start_rc') or "''", expected=3)
    R.ck('refuse_silent_calibrate', int_fact(S, 'cal_silent_rc') == 3,
         rc=S.fact('cal_silent_rc') or "''", expected=3)
    diffs = eeprom_diff(S, 'pre_eeprom', 'post_eeprom', BENCH_IDS, True, census=True)
    unsafe = [i for i in SAFE_SILENT_IDS if i in (census(S, 'pre_eeprom') or ())]
    R.ck('nothing_written', not diffs and not unsafe, '; '.join(diffs[:6]) or
         'EEPROM and census identical before and after every refusal',
         diffs=len(diffs), answering_write_targets=_ids(unsafe))
    pre, post = S.rec('pre_eeprom'), S.rec('post_eeprom')
    R.row('sram', 'NOTE', 'the stack legitimately writes 40/55; before -> after: %s' % ' '.join(
        'id%d:40=%s->%s,55=%s->%s' % (i, _reg(_block(pre, i), 40), _reg(_block(post, i), 40),
                                      _reg(_block(pre, i), 55), _reg(_block(post, i), 55))
        for i in BENCH_IDS))
    allowed = {'cm_' + short: cm_pids for short, _ in H14_TOOLS}
    allowed.update({'fl_' + short: [probe_pid] for short, _ in H14_TOOLS})
    allowed.update({label: () for label in H14_USAGE + ('taken', 'silent_start', 'cal_silent')})
    left = holders_left(S, allowed)
    probe = S.rec('released')
    R.ck('released', probe.get('verdict') == 'acquired' and not left,
         'no holder but the controller manager (stage A) or the probe (stage B) after any tool '
         + ', '.join(left), verdict=probe.get('verdict', 'no_result'))


def h15(R, S, C):
    """
    Gate set_id's round trip 4 -> 253 -> 4, the first of the two EEPROM writers.

    Reads hil_eeprom's own snapshots; back_eeprom is taken before the helper's restore.
    """
    start = 4
    port_rows(R, S)
    journal_row(R, S)
    for label in ('move', 'back'):
        R.ck(label + '_rc', int_fact(S, label + '_rc') == 0,
             rc=S.fact(label + '_rc') or "''", expected=0)
    moved_ids = sorted(set(BENCH_IDS) - {start} | {TEMP_ID})
    R.ck('moved', census(S, 'moved_eeprom') == moved_ids,
         census=_ids(census(S, 'moved_eeprom') or []), expected=_ids(moved_ids))
    pre, moved, back = S.rec('pre_eeprom'), S.rec('moved_eeprom'), S.rec('back_eeprom')
    wrong = eeprom_diff(S, pre, moved, [i for i in BENCH_IDS if i != start], True)
    was, now = _block(pre, start), _block(moved, TEMP_ID)
    if not was or not now:
        wrong.append('no servo %d in pre or no servo %d in moved' % (start, TEMP_ID))
    for reg in EEPROM_ADDRS if was and now else ():
        want = TEMP_ID if reg == 5 else _reg(was, reg)
        if not isinstance(_reg(now, reg), int) or not isinstance(want, int) or \
                _reg(now, reg) != want:
            wrong.append('%d reg %d: %s, expected %s' % (TEMP_ID, reg, _reg(now, reg), want))
    R.ck('only_id_changed', not wrong, '; '.join(wrong[:6]) or
         'servo %d reads as it did at %d but for register 5, ids 1-3 untouched' % (TEMP_ID, start),
         diffs=len(wrong))
    R.ck('lock_closed', _reg(now, 55) == 1 and _reg(_block(back, start), 55) == 1,
         'the tools leave register 55 at 1 (the verified lock)',
         moved=_reg(now, 55), back=_reg(_block(back, start), 55))
    _, scanned, _, _ = scan_table(S.txt('moved_scan.out.txt'))
    R.ck('scan_sees_move', int_fact(S, 'moved_scan_rc') == 0 and sorted(scanned) == moved_ids,
         'the hardware proof that scan reaches the top of its range',
         rc=S.fact('moved_scan_rc') or "''", ids=_ids(sorted(scanned)), expected=_ids(moved_ids))
    diffs = eeprom_diff(S, pre, back, BENCH_IDS, True)
    R.ck('restored_by_tool', not diffs and census(S, 'back_eeprom') == list(BENCH_IDS),
         '; '.join(diffs[:6]) or 'the round trip alone left EEPROM and census as found',
         census=_ids(census(S, 'back_eeprom') or []))
    # A refused restore (exit 3) also prints `writes: []`, so the empty list counts only when the
    # restore ran to the end: rc 0 and RESULT ok.
    restore = S.rec('restore')
    writes = restore.get('writes')
    eeprom_writes = [w for w in writes if w.get('kind') == 'eeprom'] \
        if isinstance(writes, list) else None
    R.ck('restore_writes', restore.get('ok') is True and int_fact(S, 'restore_rc') == 0 and
         eeprom_writes == [],
         'hil_eeprom restore after the round trip ran and had no EEPROM byte to write',
         rc=S.fact('restore_rc') or "''", result_ok=restore.get('ok'),
         eeprom_writes='unrecorded' if eeprom_writes is None else len(eeprom_writes))
    diffs = eeprom_diff(S, 'pre_eeprom', 'post_eeprom', BENCH_IDS, False, census=True)
    R.ck('restored', not diffs, '; '.join(diffs[:6]) or
         'EEPROM, 40, 55 and census as the scenario found them', diffs=len(diffs))
    details = [tool_detail(S.txt(label + '.err.txt')) or {} for label in ('move', 'back')]
    for key, field in (('ack_from', 'id_write_ack'), ('ack_ms', 'ack_ms'),
                       ('verify_ms', 'verify_ms'), ('new_id_pings', 'new_id_pings'),
                       ('late_ack_from', 'late_ack_from'), ('lock_before', 'lock_before')):
        R.row(key, 'NOTE', 'move=%s back=%s' % tuple(
            d.get(field, 'no detail line') for d in details))
    R.row('torque_after_id_write', 'NOTE', 'register 40 at %d after the move %s, before it %s'
          % (TEMP_ID, _reg(now, 40), _reg(was, 40)))


def h16(R, S, C):
    """
    Gate calibrate_midpoint on id 2, and its refusal on the wheel id 3.

    offset_delta: the offset must move by as much as position_before was off 2048.
    """
    port_rows(R, S)
    journal_row(R, S)
    R.ck('rc', int_fact(S, 'cal_rc') == 0, rc=S.fact('cal_rc') or "''", expected=0)
    read = S.rec('cal_pos')
    value = read.get('value')
    R.ck('position_2048', read.get('ok') is True and isinstance(value, int) and
         abs(value - 2048) <= MIDPOINT_TOL, 'hil_eeprom read --addr 56 right after the tool',
         read_ok=read.get('ok'), value=value, tol=MIDPOINT_TOL)
    detail = tool_detail(S.txt('cal.err.txt'))
    pre, cal, wheel = S.rec('pre_eeprom'), S.rec('cal_eeprom'), S.rec('wheel_eeprom')
    before, after = _word(_block(pre, CALIB_ID), 31), _word(_block(cal, CALIB_ID), 31)
    position = _int((detail or {}).get('position_before', ''))
    if detail is None or None in (before, after, position):
        R.ck('offset_delta', False, 'no detail line' if detail is None else
             'unreadable: offset before %s, after %s, position_before %s'
             % (before, after, (detail or {}).get('position_before')))
    else:
        moved, wanted = abs(decode11(after) - decode11(before)), abs(position - 2048)
        R.ck('offset_delta', abs(moved - wanted) <= MIDPOINT_TOL,
             '|offset change| against |position_before - 2048|, bit-11 decoded',
             offset_before=decode11(before), offset_after=decode11(after), moved=moved,
             position_before=position, wanted=wanted, tol=MIDPOINT_TOL)
    diffs = _diffs(pre, cal, BENCH_IDS, EEPROM_ADDRS)
    stray = [d['text'] for d in diffs if d['at'] not in ((CALIB_ID, 31), (CALIB_ID, 32))]
    R.ck('only_offset_changed', bool(diffs) and not stray, '; '.join(stray[:6]) or
         ('no EEPROM byte changed at all' if not diffs else
          'only id %d registers 31-32' % CALIB_ID),
         changed=len(diffs), stray=len(stray))
    R.ck('lock_closed', _reg(_block(cal, CALIB_ID), 55) == 1, lock=_reg(_block(cal, CALIB_ID), 55))
    R.ck('torque_off', _reg(_block(cal, CALIB_ID), 40) == 0,
         'left off: with torque on, the servo moves to its old goal in the new frame',
         torque=_reg(_block(cal, CALIB_ID), 40))
    diffs = eeprom_diff(S, cal, wheel, BENCH_IDS, False)
    names_mode = re.search(r'\bis in mode %d\b' % BENCH_MODES[WHEEL_ID],
                           S.txt('wheel.err.txt')) is not None
    R.ck('wheel_refused', int_fact(S, 'wheel_rc') == 4 and names_mode and not diffs,
         '; '.join(diffs[:6]) or 'refused with nothing written', rc=S.fact('wheel_rc') or "''",
         expected=4, names_mode=names_mode, diffs=len(diffs))
    diffs = eeprom_diff(S, 'pre_eeprom', 'post_eeprom', BENCH_IDS, False, census=True)
    R.ck('restored', not diffs, '; '.join(diffs[:6]) or
         'EEPROM, 40, 55 and census as the scenario found them', diffs=len(diffs))
    detail = detail or {}
    for key in ('offset_sign', 'register40_after', 'torque_before', 'settle_ms', 'ack_ms',
                'late_ack_from'):
        R.row(key, 'NOTE', 'from the detail line: %s' % detail.get(key, 'no detail line'))
    at_rest = _block(pre, CALIB_ID).get('volatile', {}).get('56')
    after_tool = _int(detail.get('position_after', ''))
    R.row('sag_on_torque_off', 'NOTE', 'position_before %s - pre 56 %s = %s' % (
        position, at_rest, position - at_rest
        if isinstance(position, int) and isinstance(at_rest, int) else '?'))
    R.row('creep_after_tool', 'NOTE', 'cal_pos %s - position_after %s = %s' % (
        value, after_tool, value - after_tool
        if isinstance(value, int) and isinstance(after_tool, int) else '?'))
    R.row('goal_register', 'NOTE', 'register 42 after the calibration %s, before it %s'
          % (_block(cal, CALIB_ID).get('volatile', {}).get('42'),
             _block(pre, CALIB_ID).get('volatile', {}).get('42')))


def h17(R, S, C):
    """
    Gate the bench's EEPROM at the end of the run against the run's start and a golden baseline.

    matches_baseline needs WAVESHARE_HIL_EEPROM_BASELINE.
    """
    port_rows(R, S)
    initial = snap_file(os.path.join(S.path, 'initial_eeprom.snap'))
    diffs = eeprom_diff(S, initial, 'final_eeprom', BENCH_IDS, True, census=True)
    final_census = census(S, 'final_eeprom')
    R.ck('eeprom_as_found', not diffs and final_census == list(BENCH_IDS),
         '; '.join(diffs[:6]) or 'EEPROM and census as the pre-flight read them',
         census=_ids(final_census or []), expected=_ids(BENCH_IDS), diffs=len(diffs))
    R.ck('journal_clear', S.fact('journal_present') == 'false',
         'a journal left behind means a restore failed or a guard path skipped guard_end',
         journal_present=S.fact('journal_present') or "''")
    path = os.path.join(S.path, 'baseline.snap')
    if os.path.exists(path):
        diffs = eeprom_diff(S, snap_file(path), 'final_eeprom', BENCH_IDS, True, census=True)
        R.ck('matches_baseline', not diffs, '; '.join(diffs[:6]) or
             'EEPROM and census as the golden baseline', diffs=len(diffs))
    elif S.fact('baseline_source'):
        R.ck('matches_baseline', False, 'a baseline was given (%s) but never reached the '
             'scenario directory' % S.fact('baseline_source'))
    else:
        R.row('matches_baseline', 'SKIP', 'no baseline given (WAVESHARE_HIL_EEPROM_BASELINE '
              'unset); set it to compare the bench with a golden snapshot')
    final = S.rec('final_eeprom')
    R.row('sram', 'NOTE', 'the driver scenarios legitimately touch torque; initial -> final: %s'
          % ' '.join('id%d:40=%s->%s,55=%s->%s' % (
              i, _reg(_block(initial, i), 40), _reg(_block(final, i), 40),
              _reg(_block(initial, i), 55), _reg(_block(final, i), 55)) for i in BENCH_IDS))


CHECKERS = (('H1', h1), ('H1B', h1b), ('H2', h2), ('H3', h3), ('H4', h4), ('H5A', h5a),
            ('H5B', h5b), ('H5C', h5c), ('H6', h6), ('H7', h7), ('H8', h8), ('H9', h9),
            ('H10', h10), ('H11', h11), ('H12', h12), ('H13', h13), ('H14', h14), ('H15', h15),
            ('H16', h16), ('H17', h17))
TOOL_SCENARIOS = ('H13', 'H14', 'H15', 'H16', 'H17')


# Not scenarios: `roslog` (the per-scenario ROS_LOG_DIR). Scratch directories must start with
# '_' or '.'.
NOT_A_SCENARIO = ('roslog',)


def unchecked_rows(R, run_dir):
    """
    FAIL for a scenario directory that ran on the bench but that no checker in CHECKERS reads.

    run() walks CHECKERS, so such a scenario would drive the motors and gate nothing.
    """
    try:
        present = sorted(name for name in os.listdir(run_dir)
                         if os.path.isdir(os.path.join(run_dir, name)) and
                         name not in NOT_A_SCENARIO and not name.startswith(('_', '.')))
    except OSError:
        return
    orphans = [name for name in present if name not in {n for n, _ in CHECKERS}]
    if orphans:
        R.row('unchecked_scenarios', 'FAIL',
              'ran on the bench and no checker in CHECKERS reads them, so nothing they '
              'recorded was gated: %s' % ','.join(orphans))


def run(run_dir, allowed, seconds, port_free, expected=None):
    """Check every scenario in the run directory, write the report, return the exit code."""
    report = Report(allowed)
    aborted = {}
    for line in _text(os.path.join(run_dir, 'aborted.txt')).splitlines():
        key, _, detail = line.partition('\t')
        if key.strip():
            aborted[key.strip()] = detail.strip()
    # The scenario list hil_check.sh ran: an expected scenario with no directory and no abort is
    # FAIL did_not_run, not the SKIP of a scenario nobody asked for.
    expected = expected.split() if isinstance(expected, str) else list(expected or ())
    missing = '%s was in the scenario list of this run, but left no directory and no abort'
    context = {}
    for name, checker in CHECKERS:
        scenario = Scenario(run_dir, name)
        report.prefix = ''
        if name in aborted:
            report.row(name, 'ABORTED', aborted[name])
        elif not scenario.present() and name in expected:
            report.row('%s.did_not_run' % name, 'FAIL', missing % name)
        elif not scenario.present():
            report.row(name, 'SKIP', 'scenario was not run')
        else:
            report.prefix = name
            checker(report, scenario, context)
    report.prefix = ''
    # Names no checker knows: an abort is reported (a SCENARIOS entry with no h_ function aborts
    # with rc 127); one that left nothing is did_not_run.
    known = [name for name, _ in CHECKERS]
    for name in sorted(set(aborted) - set(known)):
        report.row(name, 'ABORTED', aborted[name])
    for name in expected:
        if name not in known and name not in aborted and \
                not os.path.isdir(os.path.join(run_dir, name)):
            report.row('%s.did_not_run' % name, 'FAIL', missing % name)
    unchecked_rows(report, run_dir)
    for row in report.rows:
        print('%-12s %-30s %s' % (row['verdict'], row['key'], row['detail']))
    counts = {verdict: report.count(verdict) for verdict in
              ('PASS', 'FAIL', 'SKIP', 'INCONCLUSIVE', 'ABORTED', 'NOTE')}
    summary = ('hil_check: %d PASS, %d FAIL, %d SKIP, %d INCONCLUSIVE, %d ABORTED  '
               '(run %s, port free at exit: %s)' %
               (counts['PASS'], counts['FAIL'], counts['SKIP'], counts['INCONCLUSIVE'],
                counts['ABORTED'], seconds, 'yes' if port_free else 'no'))
    print(summary)
    with open(os.path.join(run_dir, 'hil_check.json'), 'w') as handle:
        json.dump({'rows': report.rows, 'counts': counts, 'summary': summary,
                   'allowed_inconclusive': sorted(report.allowed),
                   'port_free_at_exit': bool(port_free), 'duration': seconds}, handle, indent=1)
    if counts['ABORTED'] or not port_free:
        return 2
    return 1 if counts['FAIL'] else 0


def _synthetic(duration=12.0, hz=100.0, speed=2.0):
    """Build a clean wheel series: constant speed, position quantised to one tick."""
    quantised = round(speed / VQ) * VQ
    t = [k / hz for k in range(int(duration * hz))]
    return t, [round(quantised * x / TICK) * TICK for x in t], [quantised] * len(t)


def _self_test():
    """Run every gate over synthetic series with injected defects; return an exit code."""
    failures = []

    def expect(condition, what):
        print('%-6s %s' % ('ok' if condition else 'NOT OK', what))
        if not condition:
            failures.append(what)

    t, p, v = _synthetic()

    # A gate that fires on clean data is useless, so the clean series is checked first.
    expect(g1a(t, p)['ok'], 'clean series passes G1a')
    expect(aliasing_safe(t, v)['ok'], 'clean series passes aliasing_safe')
    expect(g1b(t, p, v)['ok'], 'clean series passes G1b')
    expect(g1c(t, p)['ok'], 'clean series passes G1c')
    clean = g2(t, p, v)
    expect(clean['ok'] and clean['windows'] > 100, 'clean series passes G2')
    travel = g3(t, p, v)
    expect(travel['ok'] and travel['wraps'] >= 2, 'clean series passes G3 over 3 wraps')
    expect(stale_runs(t, p, v) == [], 'clean series has no stale run')

    # Defect 1: one 2 pi step. G1a cannot see it -- wrap(2 pi) is 0, which is what G1c is for.
    jumped = [x + (2 * PI if k >= len(p) // 2 else 0.0) for k, x in enumerate(p)]
    expect(not g1c(t, jumped)['ok'], 'injected 2 pi jump caught by G1c')
    expect(g1a(t, jumped)['ok'], '... and G1a passes it, as an aliased step must')

    # Defect 2: a frozen run and its catch-up step (gated on the bench as H10.freeze).
    start = len(p) // 3
    frozen = [p[start] if start <= k < start + 6 else x for k, x in enumerate(p)]
    runs = stale_runs(t, frozen, v)
    expect(len(runs) == 1 and runs[0][1] - runs[0][0] + 1 == 6,
           'injected frozen run caught by the stale-run rule')

    # Defect 3: the reported velocity is twice the position slope.
    expect(not g2(t, p, [2.0 * x for x in v])['ok'], 'injected doubled velocity caught by G2')

    # Defect 4: one missed unwrap -- the position drops a whole revolution and stays there.
    missed = [x - (2 * PI if k >= 2 * len(p) // 3 else 0.0) for k, x in enumerate(p)]
    expect(not g3(t, missed, v)['ok'], 'injected missed wrap caught by G3')

    # aliasing_safe is itself a gate: a sampler too slow for the motion invalidates G1a.
    expect(not aliasing_safe([0.4 * k for k in range(20)], [V_CAP] * 20)['ok'],
           'too-slow sampling caught by aliasing_safe')

    # Velocity rows need a mean: a median of a VQ-quantised series can be VQ/2 off, nearly twice
    # the +-0.020 rad/s the rows allow.
    lattice = [2.0 - 0.5 * VQ] * 5 + [2.0 + 0.5 * VQ] * 4
    expect(abs(mean(lattice) - 2.0) < 0.020 <= abs(median(lattice) - 2.0),
           'a quantised velocity needs its mean, not its median')
    window = [lattice[k % len(lattice)] for k in range(200)]
    fake = {'t_cmd': 0.0, 't_stop': 10.0,
            'joint_states': [[0.0, 0.01 * k, ['w'], [0.0], [window[k]], [0.0]]
                             for k in range(200)]}
    got, n = mean_vel(fake, 'w', 0.0, 0.0)
    expect(n == 200 and abs(got - mean(window)) < 1e-12, 'mean_vel is the arithmetic mean')

    # The H7 rows are built from INVERTED_FLIPS, so a wrong constant changes a gate.
    expect(flips('position') and flips('velocity') and flips('load') and
           not flips('effort') and not flips('current') and not flips('torque'),
           'INVERTED_FLIPS drives the H7 rows')

    # The soak parser, clean and defective: a gate that cannot fire is not a gate.
    clean = ('[INFO] bus totals: transactions 60000, failed 0 (0.0 per million), '
             'worst consecutive 0, dropped 0 [id1 0]\n')
    expect(bus_totals(clean) == (60000, 0, 0, 0), 'bus_totals reads the driver totals line')
    expect(bus_totals('nothing here') is None, 'bus_totals returns None with no totals line')
    two = clean + clean.replace('transactions 60000', 'transactions 7')
    expect(bus_totals(two) == (7, 0, 0, 0), 'bus_totals takes the last window, not the first')
    expect(fail_per_million(60000, 0) == 0.0 <= SOAK_FAIL_PER_MILLION_MAX,
           'a clean soak passes the failed-transaction bound')
    expect(fail_per_million(60000, 6) > SOAK_FAIL_PER_MILLION_MAX,
           'injected 100-per-million failure rate caught by the soak bound')
    expect(math.isnan(fail_per_million(0, 0)), 'an empty soak is NaN, not a division by zero')
    # The per-servo tail is required, so H11.per_servo's parser is proved with and without it.
    expect(bus_totals_tail(two) == [(1, 0)], 'bus_totals_tail reads the last line per-servo tail')
    expect(bus_totals_tail(clean.replace(' [id1 0]', '')) == [],
           'a totals line that dropped its per-servo tail is caught')

    _self_test_limits_yaml(expect)
    _self_test_checkers(expect)
    _self_test_unchecked_rows(expect)
    _self_test_expected(expect)
    _self_test_scenario_membership(expect)

    print('hil_gates --self-test: %d check(s) failed' % len(failures) if failures
          else 'hil_gates --self-test: all checks passed')
    return 1 if failures else 0


def _self_test_limits_yaml(expect):
    """
    Prove controllers/bench_limits.yaml is bench.yaml with only enforce_command_limits flipped.

    H12's attribution needs the same bench with the limiters on; comments are ignored.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    bodies = []
    for name in ('bench.yaml', 'bench_limits.yaml'):
        text = _text(os.path.join(here, 'controllers', name))
        expect(bool(text), 'the self-test reads controllers/%s (%d bytes)' % (name, len(text)))
        bodies.append([(n, line.rstrip()) for n, line in enumerate(text.splitlines(), 1)
                       if line.strip() and not line.lstrip().startswith('#')])
    plain, limited = bodies
    expect(len(plain) == len(limited),
           'bench.yaml and bench_limits.yaml have the same significant-line count (%d vs %d)'
           % (len(plain), len(limited)))
    diff = [(a, b) for a, b in zip(plain, limited) if a[1] != b[1]]
    # zip() stops at the shorter list, so lines that only one file has are named as surplus.
    surplus = plain[len(limited):] + limited[len(plain):]
    expect(len(diff) == 1 and not surplus,
           'they differ in exactly one significant line (%s%s)'
           % ('; '.join('bench.yaml:%d [%s] vs bench_limits.yaml:%d [%s]'
                        % (a[0], a[1].strip(), b[0], b[1].strip()) for a, b in diff) or 'none',
              '; unmatched: %s' % ','.join(line.strip() for _, line in surplus)
              if surplus else ''))
    changed = [(a[1].strip(), b[1].strip()) for a, b in diff]
    expect(changed == [('enforce_command_limits: false', 'enforce_command_limits: true')],
           'and the one difference is enforce_command_limits, false -> true (%s)' % changed)


def _self_test_checkers(expect):
    """
    Drive every checker and run() over a synthetic run tree (hil_fixture.py).

    Catches a checker that cannot run at all; verdicts are left to the dedicated self-tests.
    """
    import shutil
    import tempfile

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import hil_fixture

    root = tempfile.mkdtemp(prefix='hil_gates_self_test_')
    try:
        hil_fixture.build(root)
        context = {}
        for name, checker in CHECKERS:
            report = Report(())
            report.prefix = name
            try:
                checker(report, Scenario(root, name), context)
            except Exception as exc:                                    # noqa: BLE001
                expect(False, '%s runs over the fixture (raised %s: %s)'
                       % (name, type(exc).__name__, exc))
                continue
            expect(bool(report.rows), '%s runs over the fixture and emits %d row(s)'
                   % (name, len(report.rows)))
        _self_test_soak_load(expect, root)
        _self_test_h1_ping(expect, root)
        _self_test_h12_clamps(expect, root)
        _self_test_tools(expect, root)
        _self_test_row_coverage(expect, root)
        _self_test_unreadable_snapshots(expect, root)
        quiet, code = io.StringIO(), None
        try:
            with contextlib.redirect_stdout(quiet):
                code = run(root, (), '0s', True)
        except Exception as exc:                                        # noqa: BLE001
            expect(False, 'run() completes over the fixture (raised %s: %s)'
                   % (type(exc).__name__, exc))
        expect(code in (0, 1), 'run() completes over the fixture (exit %s)' % code)
        written = _load(os.path.join(root, 'hil_check.json')) or {}
        expect(bool(written.get('rows')), 'run() writes hil_check.json with %d row(s)'
               % len(written.get('rows', [])))
        seen = {row['key'].split('.')[0] for row in written.get('rows', [])}
        missing = [name for name, _ in CHECKERS if name not in seen]
        expect(not missing, 'every scenario reaches the report (missing: %s)'
               % (','.join(missing) or 'none'))
    finally:
        shutil.rmtree(root, ignore_errors=True)


def _self_test_soak_load(expect, root):
    """
    Prove H11.wheels_turning fires: re-run h11 on a copy of the fixture with the wheels still.

    The copy's name starts with '_', so run() keeps the original and unchecked_rows() skips it.
    """
    import shutil

    stopped = os.path.join(root, '_H11_wheels_stopped')
    shutil.rmtree(stopped, ignore_errors=True)
    shutil.copytree(os.path.join(root, 'H11'), stopped)
    path = os.path.join(stopped, 'steady_soak.json')
    recording = _load(path) or {}
    for sample in recording.get('joint_states', []):
        for joint in ('joint3', 'joint4'):
            if joint in sample[2]:
                sample[3][sample[2].index(joint)] = 0.0     # position: parked, not advancing
                sample[4][sample[2].index(joint)] = 0.0     # velocity: reported as stopped
    with open(path, 'w') as handle:
        json.dump(recording, handle)

    def verdicts(name):
        report = Report(())
        report.prefix = 'H11'
        h11(report, Scenario(root, name), {})
        return {row['key']: row['verdict'] for row in report.rows}

    moving, still = verdicts('H11'), verdicts('_H11_wheels_stopped')
    expect(moving.get('H11.wheels_turning') == 'PASS',
           'a soak with the wheels turning passes wheels_turning')
    expect(still.get('H11.wheels_turning') == 'FAIL',
           'a stationary soak caught by wheels_turning')
    # The reason the row had to exist, asserted rather than claimed in a comment. Naming the other
    # failures keeps this readable when it is a *different* row that broke.
    others = sorted(key for key, verdict in still.items()
                    if verdict == 'FAIL' and key != 'H11.wheels_turning')
    expect(not others, '... and it is the only H11 row that can see a stationary soak '
                       '(other FAILs: %s)' % (','.join(others) or 'none'))


def _self_test_h1_ping(expect, root):
    """
    Prove H1.activate fails for each silent id in EXAMPLE_IDS, and for an unlisted id 5.

    Each case is a '_'-prefixed copy of H1 with one driver `unable to ping` warning appended.
    """
    import shutil

    def verdicts(name):
        report = Report(())
        report.prefix = 'H1'
        h1(report, Scenario(root, name), {})
        return {row['key']: row['verdict'] for row in report.rows}

    expect(verdicts('H1').get('H1.activate') == 'PASS',
           'a launch where every example servo answered passes activate')
    # id 5 is not in EXAMPLE_IDS and the example does not declare it. It stands in for the drift
    # this row cannot otherwise see: a servo added to the description whose id nobody added here.
    for dead in tuple(EXAMPLE_IDS) + (5,):
        silent = os.path.join(root, '_H1_dead_id%d' % dead)
        shutil.rmtree(silent, ignore_errors=True)
        shutil.copytree(os.path.join(root, 'H1'), silent)
        with open(os.path.join(silent, 'log.txt'), 'a') as handle:
            handle.write("[WARN] unable to ping motor id '%d'; joint 'joint%d' will be skipped "
                         'on the bus\n' % (dead, dead))
        rows = verdicts('_H1_dead_id%d' % dead)
        expect(rows.get('H1.activate') == 'FAIL',
               'a silent servo id %d caught by H1.activate' % dead)
        # The same argument _self_test_soak_load makes: if some other row also turns red the
        # evidence is muddled, because then it is not this row that is carrying the check.
        others = sorted(key for key, verdict in rows.items()
                        if verdict == 'FAIL' and key != 'H1.activate')
        expect(not others, '... and it is the only H1 row that can see a silent id %d '
                           '(other FAILs: %s)' % (dead, ','.join(others) or 'none'))


def _self_test_h12_clamps(expect, root):
    """
    Prove H12's clamp rows, and only they, fail when the command arrives unclamped.

    One recording column gets the command; h12 emits no consistency gates, so that is safe.
    """
    import shutil

    def verdicts(name):
        report = Report(())
        report.prefix = 'H12'
        h12(report, Scenario(root, name), {})
        return {row['key']: row['verdict'] for row in report.rows}

    base = verdicts('H12')
    unhappy = sorted(key for key, verdict in base.items() if verdict != 'PASS')
    expect(not unhappy, 'a clamped H12 fixture passes every h12 row (not PASS: %s)'
           % (','.join(unhappy) or 'none'))
    # (what, recording, joint_states column, unclamped command, the only rows that must fail).
    # Sample layout: [rx, stamp, names, positions, velocities, efforts] (hil_fixture.rec).
    for what, label, column, value, keys in (
            ('arm position', 'pos_clamp', 3, H12_ARM_COMMAND, ('H12.pos_clamp',)),
            ('wheel velocity', 'vel_clamp', 4, H12_WHEEL_COMMAND,
             ('H12.vel_clamp.joint3', 'H12.vel_clamp.joint4'))):
        name = '_H12_unclamped_%s' % label
        copy = os.path.join(root, name)
        shutil.rmtree(copy, ignore_errors=True)
        shutil.copytree(os.path.join(root, 'H12'), copy)
        path = os.path.join(copy, '%s.json' % label)
        recording = _load(path) or {}
        for sample in recording.get('joint_states', []):
            sample[column] = [value] * len(sample[column])
        with open(path, 'w') as handle:
            json.dump(recording, handle)
        rows = verdicts(name)
        red = sorted(key for key, verdict in rows.items() if verdict == 'FAIL')
        expect(red == sorted(keys),
               'an unclamped %s (%.1f, straight through the <limit>) caught by %s and by nothing '
               'else (red: %s)' % (what, value, ','.join(keys), ','.join(red) or 'none'))


# Injected defects for H13-H17: (description, mutation, rows it must turn, FAIL unless named).
# Each makes ONE defect in a fixture copy; every non-NOTE row must appear in some entry.


def _save(path, data):
    with open(path, 'w') as handle:
        json.dump(data, handle)


def _set_facts(changes):
    """Mutation: set facts in facts.json; a value of None deletes the fact."""
    def mutate(path):
        name = os.path.join(path, 'facts.json')
        facts = _load(name) or {}
        for key, value in changes.items():
            if value is None:
                facts.pop(key, None)
            else:
                facts[key] = value
        _save(name, facts)
    return mutate


def _edit_json(label, change):
    """Mutation: `change` edits <label>.json in place."""
    def mutate(path):
        name = os.path.join(path, label + '.json')
        data = _load(name)
        change(data)
        _save(name, data)
    return mutate


def _edit_text(filename, change):
    """Mutation: `change(text, facts)` returns the file's new text."""
    def mutate(path):
        name = os.path.join(path, filename)
        text = change(_text(name), _load(os.path.join(path, 'facts.json')) or {})
        with open(name, 'w') as handle:
            handle.write(text)
    return mutate


def _delete(filename):
    """Mutation: the file is gone."""
    def mutate(path):
        os.remove(os.path.join(path, filename))
    return mutate


def _each(*mutations):
    """Mutation: several edits that together are ONE defect (a register in two snapshots)."""
    def mutate(path):
        for one in mutations:
            one(path)
    return mutate


def _register(label, servo, reg, value):
    """Mutation: one register of one servo in a snapshot's RESULT json; `value` may map the old."""
    def change(data):
        part = data['ids'][str(servo)]['eeprom' if reg < 40 else 'sram']
        part[str(reg)] = value(part[str(reg)]) if callable(value) else value
    return _edit_json(label, change)


def _unreadable(label, value=True, flag=True):
    """
    Mutation: a byte hil_eeprom could not read ('x' in the value, n one short, ok false).

    `value` and `flag` inject the two halves separately, so a row must check both.
    """
    def change(data):
        if value:
            block = data['ids'][min(data['ids'], key=int)]
            block['eeprom']['13'] = 'x'
            block['n'] -= 1
        if flag:
            data['ok'] = False
    return _edit_json(label, change)


def _snap_file(filename, value=True, flag=True, servo=None, reg=13, to='x'):
    """Mutation: the same in a .snap file, whose eeprom line holds register r at token r + 3."""
    def change(text, facts):
        lines, pending = text.splitlines(), value
        for k, line in enumerate(lines):
            tokens = line.split()
            if flag and tokens == ['ok', 'true']:
                lines[k] = 'ok false'
            elif (pending and tokens[:1] == ['servo'] and tokens[2:3] == ['eeprom'] and
                  servo in (None, int(tokens[1]))):
                tokens[3 + reg] = to
                lines[k], pending = ' '.join(tokens), False
        return '\n'.join(lines) + '\n'
    return _edit_text(filename, change)


def _scan_rows(filenames, change):
    """Mutation: `change` maps the data rows of scan tables (lists of 11 tokens) to new rows."""
    def edit(text, facts):
        lines = text.splitlines()
        rows = change([line.split() for line in lines[1:-1]])
        return '\n'.join(lines[:1] + ['%3s  %-4s%6s%7s%10s%9s%10s%11s%8s%8s%8s' % tuple(row)
                                      for row in rows] + lines[-1:]) + '\n'
    return _each(*(_edit_text(name, edit) for name in filenames))


def _scan_cell(filenames, servo, column, value):
    """Mutation: one cell of one servo's row, `value` mapping the old token to the new."""
    def change(rows):
        return [row[:column] + [value(row[column])] + row[column + 1:] if row[0] == str(servo)
                else row for row in rows]
    return _scan_rows(filenames, change)


def _drop_samples(first, last):
    """Change for _edit_json: a recording loses samples [first, last), i.e. a gap."""
    def change(data):
        del data['joint_states'][first:last]
    return change


def _red(*keys):
    return {key: 'FAIL' for key in keys}


def _tool_injections():
    """Build TOOL_INJECTIONS (see the comment above _save)."""
    scans = ('scan.out.txt', 'scan_default.out.txt')    # a bench fault shows in BOTH scans
    out = []
    # Rows every tool scenario carries: the two port rows, and the journal for H13-H16.
    for name in TOOL_SCENARIOS:
        out += [('port_free_before=false', _set_facts({'port_free_before': 'false'}),
                 _red(name + '.port_free_before')),
                ('port_free_after=false', _set_facts({'port_free_after': 'false'}),
                 _red(name + '.port_free_after'))]
        if name != 'H17':
            out += [('journal_left=true', _set_facts({'journal_left': 'true'}),
                     _red(name + '.journal_cleared')),
                    ('the fact journal_left deleted', _set_facts({'journal_left': None}),
                     _red(name + '.journal_cleared'))]

    # H13, scan.
    out += [
        ('scan_rc=7', _set_facts({'scan_rc': '7'}), _red('H13.rc')),
        ("header column 'offset' renamed",
         _edit_text('scan.out.txt', lambda t, f: t.replace(' offset\n', ' ofs\n', 1)),
         _red('H13.table')),
        ('an extra row for 253 in both scans',
         _scan_rows(scans, lambda rows: rows + [['253'] + rows[-1][1:]]),
         _red('H13.ids', 'H13.census_agrees')),
        ('row 4 removed from both scans',
         _scan_rows(scans, lambda rows: [row for row in rows if row[0] != '4']),
         _red('H13.ids', 'H13.census_agrees', 'H13.fields.4')),
        ('census [1,2,3,4,200] in both pre and post',
         _each(_edit_json('pre_eeprom', lambda d: d.update(census=[1, 2, 3, 4, 200])),
               _edit_json('post_eeprom', lambda d: d.update(census=[1, 2, 3, 4, 200]))),
         _red('H13.census_agrees')),
        ("id 2's offset negated in the table", _scan_cell(('scan.out.txt',), 2, 10,
                                                          lambda v: str(-int(v))),
         _red('H13.fields.2')),
        ('id 3 at mode 0 / type pos in both tables and in both snapshots',
         _each(_scan_cell(scans, 3, 1, lambda v: 'pos'), _scan_cell(scans, 3, 2, lambda v: '0'),
               _register('pre_eeprom', 3, 33, 0), _register('post_eeprom', 3, 33, 0)),
         _red('H13.modes')),
        ('scan_seconds=0.4', _set_facts({'scan_seconds': '0.4'}), _red('H13.timing')),
        ('scan_seconds=16', _set_facts({'scan_seconds': '16'}), _red('H13.timing')),
        ('scan_serial_speed_lines=2', _set_facts({'scan_serial_speed_lines': '2'}),
         _red('H13.serial_speed')),
        ("the 'serial speed' line moved to stdout",
         _each(_edit_text('scan.err.txt', lambda t, f: t.replace('serial speed 1000000\n', '')),
               _edit_text('scan.out.txt', lambda t, f: 'serial speed 1000000\n' + t)),
         _red('H13.serial_speed', 'H13.table')),
        ('scan_default_rc=1', _set_facts({'scan_default_rc': '1'}), _red('H13.default_params')),
        ('scan_default_skipped set',
         _set_facts({'scan_default_skipped': 'the port under test is not /dev/ttyACM0'}),
         {'H13.default_params': 'SKIP'}),
        ('stale_scan_rc=0', _set_facts({'stale_scan_rc': '0'}), _red('H13.usage.stale')),
        ('positional_scan_serial_speed_lines=1',
         _set_facts({'positional_scan_serial_speed_lines': '1'}), _red('H13.usage.positional')),
        ('post id 1 reg 13 + 1', _register('post_eeprom', 1, 13, lambda v: v + 1),
         _red('H13.read_only')),
        ('an x in post', _unreadable('post_eeprom'), _red('H13.read_only')),
        ('probe verdict refused', _edit_json('released', lambda d: d.update(verdict='refused')),
         _red('H13.released')),
        ('scan_holders_after=999', _set_facts({'scan_holders_after': '999'}),
         _red('H13.released'))]
    for servo in BENCH_IDS:
        out.append(('temp_C + 5 in row %d' % servo,
                    _scan_cell(('scan.out.txt',), servo, 8, lambda v: str(int(v) + 5)),
                    _red('H13.fields.%d' % servo)))

    # H14, the refusals.
    tools = (('scan', 'scan'), ('set_id', 'set_id'), ('calibrate', 'calibrate_midpoint'))
    out += [
        ('controllers_active=false', _set_facts({'controllers_active': 'false'}),
         _red('H14.cm_up')),
        ('a 0.2 s gap in tools_window', _edit_json('tools_window', _drop_samples(500, 520)),
         _red('H14.cm_undisturbed')),
        ('tools_window.json deleted', _delete('tools_window.json'), _red('H14.cm_undisturbed')),
        ('driver_warns_after + 1', _set_facts({'driver_warns_after': '4'}),
         _red('H14.cm_undisturbed')),
        ('fl_scan_rc=127 with serial_speed_lines=none',
         _set_facts({'fl_scan_rc': '127', 'fl_scan_serial_speed_lines': 'none'}),
         _red('H14.via_servobus.scan', 'H14.flock_refused.scan')),
        ('the fact fl_calibrate_serial_speed_lines deleted',
         _set_facts({'fl_calibrate_serial_speed_lines': None}),
         _red('H14.via_servobus.calibrate_midpoint')),
        ('fl_set_id_rc=134', _set_facts({'fl_set_id_rc': '134'}),
         _red('H14.via_servobus.set_id', 'H14.flock_refused.set_id')),
        ('fl_scan_holder_alive=false', _set_facts({'fl_scan_holder_alive': 'false'}),
         {'H14.holder_alive_throughout': 'ABORTED'}),
        # Without the holder's pid no flock row is evidence either: each asks for "pid <probe_pid>"
        # in its stderr, and released for no holder but the probe's -- so all eight move.
        ('probe_pid empty', _set_facts({'probe_pid': ''}),
         dict(_red('H14.released', *['H14.%s.%s' % (row, tool) for row in (
             'flock_refused', 'via_servobus') for _, tool in tools]),
             **{'H14.holder_alive_throughout': 'ABORTED'})),
        ('bad_type_rc=134', _set_facts({'bad_type_rc': '134'}), _red('H14.usage.bad_type')),
        ('taken_rc=3', _set_facts({'taken_rc': '3'}), _red('H14.refuse_taken')),
        ("'id 3 already answers' removed from stderr",
         _edit_text('taken.err.txt', lambda t, f: t.replace('id 3 already answers', '')),
         _red('H14.refuse_taken')),
        ('silent_start_rc=0', _set_facts({'silent_start_rc': '0'}),
         _red('H14.refuse_silent_start')),
        ('cal_silent_rc=0', _set_facts({'cal_silent_rc': '0'}),
         _red('H14.refuse_silent_calibrate')),
        ('post id 3 reg 33 = 0', _register('post_eeprom', 3, 33, 0),
         _red('H14.nothing_written')),
        ('an x in post', _unreadable('post_eeprom'), _red('H14.nothing_written')),
        ('cm_scan_holders_after=999', _set_facts({'cm_scan_holders_after': '999'}),
         _red('H14.released'))]
    for short, tool in tools:
        out += [
            ('cm_%s_rc=0' % short, _set_facts({'cm_%s_rc' % short: '0'}),
             _red('H14.cm_refused.' + tool)),
            ('fl_%s_serial_speed_lines=1' % short,
             _set_facts({'fl_%s_serial_speed_lines' % short: '1'}),
             _red('H14.via_servobus.' + tool)),
            # via_servobus asks for the holder's pid too, so the pid's absence turns it as well
            ('the probe pid removed from fl_%s.err.txt' % short,
             _edit_text('fl_%s.err.txt' % short,
                        lambda t, f: re.sub(r'\bpid %s ?' % re.escape(f['probe_pid']), '', t)),
             _red('H14.flock_refused.' + tool, 'H14.via_servobus.' + tool))]
    for label in ('stale', 'bad_type', 'range', 'cal_range', 'missing', 'positional',
                  'foreign_node'):
        out.append(('%s_rc=0' % label, _set_facts({'%s_rc' % label: '0'}),
                    _red('H14.usage.' + label)))

    # H15, set_id 4 -> 253 -> 4.
    out += [
        ('move_rc=6', _set_facts({'move_rc': '6'}), _red('H15.move_rc')),
        ('back_rc=5', _set_facts({'back_rc': '5'}), _red('H15.back_rc')),
        ('moved census [1,2,3,200,253]',
         _edit_json('moved_eeprom', lambda d: d.update(census=[1, 2, 3, 200, 253])),
         _red('H15.moved')),
        ('the offset of 253 differs from pre[4]',
         _register('moved_eeprom', TEMP_ID, 31, lambda v: v + 1), _red('H15.only_id_changed')),
        ('an x in moved', _unreadable('moved_eeprom'), _red('H15.only_id_changed')),
        ('moved[253].55=0', _register('moved_eeprom', TEMP_ID, 55, 0), _red('H15.lock_closed')),
        ('back[4].55=0', _register('back_eeprom', 4, 55, 0), _red('H15.lock_closed')),
        ('row 253 removed from moved_scan.out.txt',
         _scan_rows(('moved_scan.out.txt',),
                    lambda rows: [row for row in rows if row[0] != str(TEMP_ID)]),
         _red('H15.scan_sees_move')),
        ('back[1] reg 13 + 1', _register('back_eeprom', 1, 13, lambda v: v + 1),
         _red('H15.restored_by_tool')),
        ('an eeprom write in the restore RESULT',
         _edit_json('restore', lambda d: d['writes'].append(
             {'id': 4, 'reg': 5, 'from': 253, 'to': 4, 'bytes': 1, 'kind': 'eeprom'})),
         _red('H15.restore_writes')),
        # A refused restore (exit 3) also prints `writes: []`, so the row reads rc and ok too.
        ('restore_rc=3 (the restore refused; its RESULT still has writes [])',
         _set_facts({'restore_rc': '3'}), _red('H15.restore_writes')),
        ('the restore RESULT ok false, exit 3, writes []',
         _edit_json('restore', lambda d: d.update(ok=False, exit=3, writes=[])),
         _red('H15.restore_writes')),
        ('post id 4 reg 5 = 253', _register('post_eeprom', 4, 5, 253), _red('H15.restored'))]

    # H16. Calibration writes persist, so a defect is injected into both cal and wheel snapshots;
    # otherwise wheel_refused (wheel == cal) would change too.
    def persisted(servo, reg, value):
        return _each(_register('cal_eeprom', servo, reg, value),
                     _register('wheel_eeprom', servo, reg, value))

    def offset_as_before(path):
        before = _load(os.path.join(path, 'pre_eeprom.json'))['ids'][str(CALIB_ID)]['eeprom']
        _each(persisted(CALIB_ID, 31, before['31']), persisted(CALIB_ID, 32, before['32']))(path)

    out += [
        ('cal_rc=6', _set_facts({'cal_rc': '6'}), _red('H16.rc')),
        ('cal_pos.value=2060', _edit_json('cal_pos', lambda d: d.update(value=2060)),
         _red('H16.position_2048')),
        ('cal_pos.json with ok false', _edit_json('cal_pos', lambda d: d.update(ok=False,
                                                                                value=None)),
         _red('H16.position_2048')),
        ('detail position_before=1500',
         _edit_text('cal.err.txt', lambda t, f: re.sub(r'position_before=\d+',
                                                       'position_before=1500', t)),
         _red('H16.offset_delta')),
        ('the detail line removed',
         _edit_text('cal.err.txt', lambda t, f: ''.join(
             line for line in t.splitlines(True) if ': detail ' not in line)),
         _red('H16.offset_delta')),
        ('cal id 1 reg 13 + 1', persisted(1, 13, lambda v: v + 1),
         _red('H16.only_offset_changed')),
        ('cal id 2 reg 33 = 1', persisted(CALIB_ID, 33, 1), _red('H16.only_offset_changed')),
        ('cal offset equal to pre', offset_as_before,
         _red('H16.only_offset_changed', 'H16.offset_delta')),
        ('cal[2].55=0', persisted(CALIB_ID, 55, 0), _red('H16.lock_closed')),
        ('cal[2].40=1', persisted(CALIB_ID, 40, 1), _red('H16.torque_off')),
        ('wheel_rc=0', _set_facts({'wheel_rc': '0'}), _red('H16.wheel_refused')),
        ('wheel id 3 reg 33 = 0', _register('wheel_eeprom', WHEEL_ID, 33, 0),
         _red('H16.wheel_refused')),
        ('post id 2 reg 31 differs', _register('post_eeprom', CALIB_ID, 31, lambda v: v + 1),
         _red('H16.restored'))]

    # H17. final is compared with both initial and baseline, so a changed final turns both rows;
    # the two sources are injected alone.
    out += [
        ('final id 2 reg 31 differs', _register('final_eeprom', 2, 31, lambda v: v + 1),
         _red('H17.eeprom_as_found', 'H17.matches_baseline')),
        ('final census + 253', _edit_json('final_eeprom', lambda d: d['census'].append(253)),
         _red('H17.eeprom_as_found', 'H17.matches_baseline')),
        ('an x in initial', _snap_file('initial_eeprom.snap'), _red('H17.eeprom_as_found')),
        ('journal_present=true', _set_facts({'journal_present': 'true'}),
         _red('H17.journal_clear')),
        ('baseline id 3 reg 33 = 0', _snap_file('baseline.snap', flag=False, servo=3, reg=33,
                                                to='0'), _red('H17.matches_baseline')),
        ('no baseline given (the file absent, baseline_source empty)',
         _each(_delete('baseline.snap'), _set_facts({'baseline_source': ''})),
         {'H17.matches_baseline': 'SKIP'}),
        ('a baseline given whose copy is missing', _delete('baseline.snap'),
         _red('H17.matches_baseline'))]
    return tuple(out)


TOOL_INJECTIONS = _tool_injections()

# The rows that read a snapshot, and the snapshots each reads (_self_test_unreadable_snapshots).
SNAPSHOT_ROWS = (
    ('H13.read_only', ('pre_eeprom', 'post_eeprom')),
    ('H13.census_agrees', ('pre_eeprom',)),
    ('H13.fields.1', ('pre_eeprom',)), ('H13.fields.2', ('pre_eeprom',)),
    ('H13.fields.3', ('pre_eeprom',)), ('H13.fields.4', ('pre_eeprom',)),
    ('H14.nothing_written', ('pre_eeprom', 'post_eeprom')),
    ('H15.only_id_changed', ('pre_eeprom', 'moved_eeprom')),
    ('H15.restored_by_tool', ('pre_eeprom', 'back_eeprom')),
    ('H15.restored', ('pre_eeprom', 'post_eeprom')),
    ('H16.only_offset_changed', ('pre_eeprom', 'cal_eeprom')),
    ('H16.wheel_refused', ('cal_eeprom', 'wheel_eeprom')),
    ('H16.restored', ('pre_eeprom', 'post_eeprom')),
    ('H17.eeprom_as_found', ('initial_eeprom.snap', 'final_eeprom')),
    ('H17.matches_baseline', ('baseline.snap', 'final_eeprom')))


def _verdicts(root, directory, name):
    """Run checker `name` over root/directory and return {row key: verdict}."""
    report = Report(())
    report.prefix = name
    dict(CHECKERS)[name](report, Scenario(root, directory), {})
    return {row['key']: row['verdict'] for row in report.rows}


def _injected(root, name, tag, mutate):
    """Copy root/name to a sibling whose name leads with '_', apply `mutate`, return verdicts."""
    import shutil

    copy = '_%s_%s' % (tag, name)
    shutil.rmtree(os.path.join(root, copy), ignore_errors=True)
    shutil.copytree(os.path.join(root, name), os.path.join(root, copy))
    mutate(os.path.join(root, copy))
    return _verdicts(root, copy, name)


def _self_test_tools(expect, root):
    """
    Prove every row of h13..h17 by differential injection, as for H12.

    The green fixture must be all PASS or NOTE; each injection must change exactly its rows.
    """
    base = {}
    for name in TOOL_SCENARIOS:
        try:
            base[name] = _verdicts(root, name, name)
        except Exception as exc:                                        # noqa: BLE001
            base[name] = {}
            expect(False, '%s runs over the green fixture (raised %s: %s)'
                   % (name, type(exc).__name__, exc))
            continue
        unhappy = sorted(key for key, verdict in base[name].items()
                         if verdict not in ('PASS', 'NOTE'))
        expect(bool(base[name]) and not unhappy,
               'the green %s fixture emits rows and every one is PASS or NOTE (%d rows; not: %s)'
               % (name, len(base[name]), ','.join(unhappy) or 'none'))
    for number, (what, mutate, rows) in enumerate(TOOL_INJECTIONS):
        name = next(iter(rows)).split('.')[0]
        try:
            got = _injected(root, name, 'inj%03d' % number, mutate)
        except Exception as exc:                                        # noqa: BLE001
            expect(False, '%s: %s (raised %s: %s)' % (name, what, type(exc).__name__, exc))
            continue
        changed = {key: got.get(key, 'MISSING') for key in set(base[name]) | set(got)
                   if got.get(key) != base[name].get(key)}
        expect(changed == rows, '%s: %s turns exactly %s (changed: %s)' % (
            name, what, kv(**rows), kv(**changed) or 'nothing'))


def _self_test_row_coverage(expect, root):
    """Prove every non-NOTE row of h13..h17 has an injection, and every injection a row."""
    emitted = set()
    for name in TOOL_SCENARIOS:
        try:
            emitted |= {key for key, verdict in _verdicts(root, name, name).items()
                        if verdict != 'NOTE'}
        except Exception:                                               # noqa: BLE001
            pass                                        # _self_test_tools has already said so
    injected = set()
    for _, _, rows in TOOL_INJECTIONS:
        injected |= set(rows)
    expect(bool(emitted) and emitted == injected,
           'every non-NOTE row of h13..h17 has an injected defect, and every injection a row '
           '(no injection: %s; no such row: %s)' % (
               ','.join(sorted(emitted - injected)) or 'none',
               ','.join(sorted(injected - emitted)) or 'none'))


def _self_test_unreadable_snapshots(expect, root):
    """
    Prove an 'x' or ok false in any snapshot a row reads turns that row red.

    Each half is injected on its own (see _unreadable) into each snapshot in SNAPSHOT_ROWS.
    """
    for key, labels in SNAPSHOT_ROWS:
        name, row = key.split('.', 1)
        for label in labels:
            for kind, value, flag in (('an x', True, False), ('ok false', False, True)):
                if label.endswith('.snap'):
                    mutate = _snap_file(label, value=value, flag=flag)
                else:
                    mutate = _unreadable(label, value=value, flag=flag)
                try:
                    got = _injected(root, name, 'unreadable', mutate).get(key)
                except Exception as exc:                                # noqa: BLE001
                    got = 'raised %s: %s' % (type(exc).__name__, exc)
                expect(got == 'FAIL', '%s in %s turns %s red (got %s)' % (kind, label, key, got))


def _self_test_unchecked_rows(expect):
    """Prove a stray scenario directory no checker claims gives FAIL unchecked_scenarios."""
    import shutil
    import tempfile

    root = tempfile.mkdtemp(prefix='hil_gates_unchecked_')
    try:
        for name, _ in CHECKERS:
            os.makedirs(os.path.join(root, name))
        os.makedirs(os.path.join(root, '_scratch'))
        os.makedirs(os.path.join(root, 'roslog'))
        clean = Report(())
        unchecked_rows(clean, root)
        expect(not clean.rows, 'every CHECKERS directory, roslog and a _ directory are claimed '
               '(rows: %s)' % [row['key'] for row in clean.rows])
        os.makedirs(os.path.join(root, 'H99'))
        stray = Report(())
        unchecked_rows(stray, root)
        found = [row for row in stray.rows if row['key'] == 'unchecked_scenarios']
        expect(len(found) == 1 and found[0]['verdict'] == 'FAIL' and 'H99' in found[0]['detail'],
               'a stray H99 directory gives FAIL unchecked_scenarios naming it (rows: %s)'
               % [(row['key'], row['verdict']) for row in stray.rows])
    finally:
        shutil.rmtree(root, ignore_errors=True)


def _self_test_expected(expect):
    """Prove an expected scenario with no directory and no abort is FAIL did_not_run."""
    import shutil
    import tempfile

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import hil_fixture

    root = tempfile.mkdtemp(prefix='hil_gates_expected_')

    def rows(expected, aborted=''):
        with open(os.path.join(root, 'aborted.txt'), 'w') as handle:
            handle.write(aborted)
        with contextlib.redirect_stdout(io.StringIO()):
            run(root, (), '0s', True, expected=expected)
        return {row['key']: row['verdict']
                for row in (_load(os.path.join(root, 'hil_check.json')) or {}).get('rows', [])}

    try:
        hil_fixture.build(root)
        shutil.rmtree(os.path.join(root, 'H13'))
        got = rows('H13')
        expect(got.get('H13.did_not_run') == 'FAIL',
               'run(expected="H13") with no H13 directory gives FAIL H13.did_not_run '
               '(H13 rows: %s)'
               % {k: v for k, v in got.items() if k.startswith('H13')})
        got = rows('H13', 'H13\tport held\n')
        expect(got.get('H13') == 'ABORTED' and 'H13.did_not_run' not in got,
               'an aborted H13 is ABORTED, not did_not_run (H13 rows: %s)'
               % {k: v for k, v in got.items() if k.startswith('H13')})
        got = rows('H14')
        expect(got.get('H13') == 'SKIP' and 'H13.did_not_run' not in got,
               'an H13 nobody asked for stays a SKIP (H13 rows: %s)'
               % {k: v for k, v in got.items() if k.startswith('H13')})
        got = rows('H13 H99')
        expect(got.get('H99.did_not_run') == 'FAIL',
               'an expected name no checker knows, with no directory, is did_not_run too '
               '(H99 rows: %s)' % {k: v for k, v in got.items() if k.startswith('H99')})
        got = rows('H13 H99', 'H99\tno h_H99 function\n')
        expect(got.get('H99') == 'ABORTED',
               'and an abort of a name no checker knows (a missing h_ function) is reported '
               '(H99 rows: %s)' % {k: v for k, v in got.items() if k.startswith('H99')})
    finally:
        shutil.rmtree(root, ignore_errors=True)


def _self_test_scenario_membership(expect):
    """
    Prove hil_check.sh's default SCENARIOS and CHECKERS name the same set, in a valid order.

    Order: H13-H16 run before the soak H11, and H17 runs last.
    """
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'hil_check.sh')
    found = re.findall(r'^SCENARIOS=\$\{WAVESHARE_HIL_SCENARIOS:-"([^"]*)"\}$', _text(script),
                       re.MULTILINE)
    expect(len(found) == 1, 'hil_check.sh has one default SCENARIOS list (found %d)' % len(found))
    names = found[0].split() if found else []
    known = [name for name, _ in CHECKERS]
    expect(sorted(names) == sorted(known),
           'the default SCENARIOS and CHECKERS are the same set (only in SCENARIOS: %s; only in '
           'CHECKERS: %s)' % (','.join(sorted(set(names) - set(known))) or 'none',
                              ','.join(sorted(set(known) - set(names))) or 'none'))
    order = {name: k for k, name in enumerate(names)}
    expect('H11' in order and names[-1:] == ['H17'] and
           all(order.get(name, len(names)) < order['H11'] for name in TOOL_SCENARIOS[:-1]),
           'H13-H16 run before the soak H11 and H17 runs last: %s' % ' '.join(names))


def main(argv):
    """Run the self-test, or check one run directory and write its report."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--run-dir')
    parser.add_argument('--allow-inconclusive', default='',
                        help='the frozen key list, passed in by hil_check.sh')
    parser.add_argument('--seconds', default='0s')
    parser.add_argument('--port-free', default='false')
    parser.add_argument('--expected', default=None,
                        help='the scenario list hil_check.sh ran; a name in it that left no '
                             'directory and no abort is FAIL <name>.did_not_run')
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
    if not args.run_dir:
        parser.error('one of --self-test and --run-dir is required')
    return run(args.run_dir, args.allow_inconclusive.split(), args.seconds,
               args.port_free == 'true', args.expected)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
