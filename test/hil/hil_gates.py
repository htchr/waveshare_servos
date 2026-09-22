"""
Invariant gates, scenario predicates and the report writer for the bench check.

PHASE2_SPEC 12.2 (the gates), 12.4 (the output contract) and 12.5 (the ten scenarios).

  hil_gates.py --self-test          the gates over synthetic series with injected defects
  hil_gates.py --run-dir DIR ...    check one run directory and write its report

--self-test needs no hardware, no ROS and no run directory; it is the part `colcon test` can
prove. Command times (t_cmd, t_stop) and message header stamps are one clock here: the recorder
takes time.time() and the controller manager stamps with a real, unsimulated clock. There is no
path from FAIL to anything else anywhere in this file (decision D5); INCONCLUSIVE is legal only
for the keys hil_check.sh freezes and passes in with --allow-inconclusive.
"""

import argparse
import contextlib
import io
import json
import math
import os
import re
import sys

# Driver constants, copied from the driver's own defaults and never rounded (PHASE2_SPEC 12.3);
# they are updated with the sections they come from, never independently.
ENCODER_STEPS = 4096                             # param_parsing.hpp kEncoderSteps
STEPS_PER_RAD = ENCODER_STEPS / (2.0 * math.pi)  # 651.8986
TICK = 2.0 * math.pi / ENCODER_STEPS             # 0.00153398 rad, the encoder quantum
MAX_SPEED_COUNTS = 6000
MAX_ACCEL_COUNTS = 150
V_CAP = MAX_SPEED_COUNTS * TICK                  # 9.20388 rad/s, the servo speed ceiling
VQ = 50 * TICK                                   # 0.07670 rad/s, reported-velocity quantum
CURRENT_PER_COUNT_A = 0.006                      # param_parsing.hpp kCurrentPerCountA
TORQUE_CONSTANT_NM_PER_A = 0.8825985             # param_parsing.hpp kTorqueConstantNmPerA
NM_PER_KGFCM = 0.0980665                         # units.hpp kNmPerKgfCm
IO_TIMEOUT_MS = 5                                # param_parsing.hpp kIoTimeoutMs (PHASE3 5.19)
PING_ATTEMPTS = 3
MAX_READ_FAILS = 50
ALLOW_MISSING_SERVOS = False

# Decision D4 (PHASE2_SPEC 3.2): inverted flips exactly these three quantities.
INVERTED_FLIPS = ('position', 'velocity', 'load')

# The guard D4/A9 asks for: effort, current and torque are unsigned magnitudes and never flip.
for _name in ('effort', 'current', 'torque'):
    if _name in INVERTED_FLIPS:
        raise SystemExit(
            "hil_gates: INVERTED_FLIPS contains '%s'; PHASE2_SPEC 3.2 and decision D4 say "
            'inverted flips position, velocity and load only' % _name)

# Gate tolerances, every one derived and justified in PHASE2_SPEC 12.2.
G1B_SLACK = 12 * TICK      # 0.018408 rad; the worst Phase 1 excess is 5.5 ticks
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

# Resting drift of `present_position` between two readbacks of the same servo, used by
# registers_unchanged() for H5A.registers and for H6's two holder rows.  The registers themselves
# (torque_enable, acc, goal_position_raw, goal_speed_raw, mode) are compared for *equality* and
# that is what actually proves the driver did not re-program anything; these two numbers only say
# how far the shaft may be found to have settled in between.
#
# A `pos` joint is servoing a goal position the whole time, so it has no relaxation path and keeps
# the original bound.  A `vel` (wheel-mode) joint is commanded a *speed* of zero and nothing holds
# its shaft: when the drive stops, gearbox backlash and elastic wind-up in the wheel unwind, and
# the encoder reports that as a slow position change.  The magnitude is set by mechanics, not by
# elapsed time, so the allowance is a constant and not a rate.
#
# The number comes from the three bench runs, not from what makes one row pass.  The pairing rule
# is spelled out here because the count depends entirely on it, and an unstated rule got the
# sample misquoted once already (decision W2(i)):
#   1. Corpus.  Every *.json under one run directory that parses and carries a top-level "ids"
#      map.  Which files that is is the one ambiguity that matters, so all three readings are
#      given below: (A) only basenames ending _readback.json; (B) every file whose id blocks
#      carry a "registers" dict, i.e. every full register readback, which adds the H8 *_stop and
#      H10 after_exit/term_after_exit reads; (C) every file whose id blocks carry pos_last_raw,
#      which additionally admits the position-only stop_wheels reads.
#   2. Time key.  The file's mtime - the instrument writes the file when the read completes, and
#      the readback JSON carries no wall clock of its own.  Sort ascending within a run
#      directory; never pair across run directories.
#   3. Pairing.  Per servo id, pair each readback with the next readback in that order that also
#      answers for that id (n >= 1).
#   4. Eligibility.  Keep the pair only if both endpoints report mode == 1 (wheel) and
#      moving == false.
#   5. Drift.  pos_last_raw(later) - pos_last_raw(earlier), absolute, no modulo unwrap - exactly
#      the quantity registers_unchanged() compares, so the sample and the gate measure the same
#      thing.
#   6. Exclusion.  Drop a pair whose |drift| > 50 ticks: the wheel was driven in between (or the
#      raw value wrapped), which is not resting drift.  This is a clean separation and not a
#      convenient one - the 24 pairs it drops are 311, 316, 436, 437, 439, 456, 610, 616, 860,
#      1402, 1497, 1500, 1501 and 2697 ticks, so there is a 260-tick empty gap between the
#      largest kept value and the smallest dropped one.
# Over hil_out, hil_out2 and hil_out3 that rule gives:
#   (A) 48 intervals, 3085 s (51 min) of wheel rest, 10 of them >= 100 s: 46 drifted 0 ticks, one
#       1 tick (over 18.8 s) and one 3 ticks (over 193 s - id4 in H5A of run 3, the 1.5 min after
#       H4 had driven it three revolutions).
#   (B) 64 intervals, 3085 s, 14 of them >= 100 s: 62 zeros, the same 1 tick, the same 3 ticks.
#       This is the corpus behind the earlier "64 intervals / 3 ticks once" quotation, which was
#       right for corpus B and wrong only in not saying which corpus it was.
#   (C) 132 intervals, 3114 s, 14 of them >= 100 s: 129 zeros, the same 1 tick, and 3 ticks
#       twice - the second 1.1 s after H10 had stopped id4 in run 1.
# The 3-tick ceiling also shows up inside single 0.4 s readback windows: of 188 windows on a
# non-moving wheel, 181 read 0 ticks and the largest was 3, with the speed register reading zero.
# So 3 ticks (0.26 deg) is the worst resting drift under every one of the three corpora, and it
# only ever appears shortly after the wheel was driven.
#
# 8 ticks = 0.70 deg is 2.7x that worst observation, and still ~39x below the smallest change a
# real wheel command produced across the three runs (311 raw ticks; most are 436 or more), so the
# row loses no power to catch a driver that really touched a wheel.  The readback is modulo 4096,
# so this row was never what would catch a whole number of revolutions - the register comparison
# and the scenario's motion gates are.  The tail is carried by one or two nonzero samples
# depending on the sampling above, so the choice is deliberately conservative; what would refine
# it is a dozen more >= 190 s rests recorded immediately after a spin, the only condition under
# which a nonzero was ever seen.
#
# For the record, the run-3 H5A red that motivated this was not the driver: ids 1, 2 and 3 moved 0
# ticks, every writable register on all four ids was byte-identical pre vs post, and the driver's
# entire contact with the bus in that scenario was 66 ms of 195 s (port open at 1789665509.710,
# FATAL at 1789665509.777) with nothing else holding the port for the remaining ~194 s.  Run 1
# recorded "identical" for the same row, so the 3 ticks are nondeterministic mechanical
# relaxation.  A driver defect would move a register or move the wheel by hundreds of ticks.
REST_TICKS = 2             # `pos` joints: unchanged, PHASE2_SPEC 12.5's H5A row
WHEEL_REST_TICKS = 8       # `vel` joints: 0.70 deg; see the derivation above
WHEEL_MODE = 1             # the servo's mode register: 0 = position, 1 = wheel

# H8.accel_effect, the acc=150 arm of the t90 gate.  Spec 12.4 set 0.35 s from the ramp term
# alone: acc=150 -> 150 counts x 100 steps/s^2 x TICK = 23.01 rad/s^2 -> 2.7/23.01 = 0.117 s,
# times ~3 for slack.  t90 is measured from t_cmd, so it is a sum of three terms, only the
# middle of which ACC governs:
#   latency   t_cmd -> first non-zero reported |v|           0.050 - 0.080 s (measured)
#   ramp      0 -> 2.0 rad/s (66 % of the commanded 3.0)     0.200 s at the achievable
#             ceiling; the wheel is torque-limited to ~10-15 rad/s^2, not 23.01
#   settling  66 % -> 90 %, the velocity loop's asymptote    0.16 - 0.24 s (measured on five
#             independent recordings: 0.16 / 0.18 / 0.22 / 0.24 / 0.24 s)
# Latency and settling are ACC-independent, so the floor of t90 on this servo is
# 0.050 + 0.160 = 0.21 s even with an instantaneous ramp, and 0.050 + 0.117 + 0.160 = 0.327 s
# with a perfectly obeyed ACC=150 ramp - i.e. flawless firmware lands at the old bound.  The
# bench measured eight independent acc=150 recordings (hil_out, hil_out2, hil_out3 and the five
# H4+H8 repeats) and none came in under 0.4570 s: 0.4570 0.4692 0.4694 0.4796 0.4896 0.4995
# 0.4996 0.4996, mean 0.4829, sd 0.0166.  0.70 s sits 1.40x above the worst of those and ~1.8x
# above the floor, while still being 2.8x under the acc=10 measurement (mean 1.9655 s), so the
# gate keeps its discriminating power: the ratio clause below, not this absolute bound, is what
# proves ACC is honoured.
#
# Does this absolute clause have the same quantisation problem as the ratio (see T90_RATIO_MIN)?
# Yes - it is the same t90 on the same 50-count lattice - but its margin is safe, and that was
# checked rather than assumed.  Two ways of counting:
#   scatter at the level the row actually fires on (1800 raw): (0.70 - 0.4829) / 0.0166 = 13.1
#     sd, and 1.40x the worst of eight.  Nothing there is close.
#   whole-quantum slips, i.e. a run whose feedback skips the 1800 level and first reports higher.
#     Re-evaluating t90 on the same eight recordings, one level at a time:
#       1850 raw  max 0.5496 s   0.70 / 0.5496 = 1.27x   OK
#       1900 raw  max 0.6396 s   0.70 / 0.6396 = 1.09x   OK
#       1950 raw  max 0.8088 s   0.70 / 0.8088 = 0.87x   breach
#     So the absolute clause survives a two-quantum slip and only a three-quantum one breaks it,
#     while the observed failure mode is a single level.  It needs no change.
T90_FAST_MAX = 0.70        # s, t90 at acc=150; see the derivation above

# H8.accel_effect, the ratio clause.  This floor is deliberately loose and the looseness is the
# metric's, not the driver's.  Read this before tightening it.
#
# What the bench measured, t90(acc=10)/t90(acc=150) over eight independent observations
# (hil_out, hil_out2, hil_out3, repeat/r1..r5):
#   3.9220  3.9223  3.9825  4.0279  4.0633  4.0851  4.2552  4.3314
#   mean 4.0737, sd 0.1493 (3.7 % of the mean); min 3.9220, max 4.3314, spread 10 % of the mean.
# The old floor of 4.0 sat at -0.49 sd, essentially on the distribution's median: three of the
# eight observations fall below it and three of the five dedicated repeats FAILED on it.
#
# The driver is not at fault, and that is established rather than assumed:
#   * the acc register reads back exactly 10 in the slow stack and exactly 150 in the fast stack
#     in all eight runs (slow_readback / fast_readback, id4) - see accel_register.*.id4, which is
#     an exact equality row.  THAT is the authoritative evidence that the ACC write arrived;
#   * an independent least-squares fit of the acc=10 ramp (|v| vs t over the 0.3-2.4 rad/s band
#     of the rising ramp) is reproducible to ~1 %: 1.4807-1.5236 rad/s^2, mean 1.4955, sd 0.0148,
#     against the 1.5340 nominal (10 counts x 100 steps/s^2 x TICK).  The same fit on the fast
#     stack gives 7.31-7.94 rad/s^2, a factor of 5.06 - so the physical ACC effect is ~5x and the
#     t90 ratio under-reads it, because latency and settling are ACC-independent.
#
# The flakiness is in the measurement.  present_speed is quantised to exactly 50 raw counts
# (0.076699039 rad/s; every one of the 45 distinct |v| levels seen on joint4 across the sixteen
# recordings is an integer multiple of it, so one raw count is 0.0015339808 rad/s).  The gate's
# 2.7 rad/s threshold is 1760.13 raw, which is NOT a reportable level, so t90 cannot fire at
# 2.7 rad/s: it fires on the next lattice value up, 1800 raw = 2.761165 rad/s (measured, not
# inferred) = 92.04 % of the 3.0 rad/s command - inside the
# velocity loop's ringing rather than on the clean part of the ramp.  Whether a given run's
# feedback happens to land on the 1800 level, or dwells at 1750 first, or skips straight past,
# is what decides that run's ratio.  Level skips demonstrably happen in the recordings.
# Decision W1 was to keep this t90 metric (a direct least-squares ramp-slope measurement was
# considered and rejected) and to set the floor from the measured distribution instead.
#
# The floor: 2.5.  Both constraints, with the arithmetic:
#   (a) real headroom below the observed minimum.  3.9220 / 2.5 = 1.57x, i.e. 1.4220 absolute =
#       9.5 sd of the observed ratio below the worst observation and 10.5 sd below the mean.  It
#       also clears the worst credible quantisation slip, which 3.0 would not.  Re-evaluating the
#       ratio one lattice level at a time in BOTH arms gives minima 3.6356 (1850), 3.1735 (1900)
#       and 2.8434 (1950); the worst mixed case - the fast arm slipping while the slow arm does
#       not - is its 1900-raw worst against the slow arm's 1800-raw best, 1.8898/0.6396 = 2.9549
#       (one level only: 1.8898/0.5496 = 3.4384).  2.5 sits below all of those, by 1.14x on the
#       tightest.  3.0 would clear the 2.9549 case by 1.02x, which is exactly the hair's breadth
#       this decision exists to avoid.
#   (b) it still cannot pass if ACC is ignored.  If the register had no effect both arms would be
#       draws from one distribution - the acc=150 one - and the ratio would tend to 1.0.  The
#       widest ratio inside that single measured distribution is 0.4996/0.4570 = 1.093, and the
#       sd of a ratio of two independent draws from it is sqrt(2) x cv = sqrt(2) x 0.0344 =
#       0.0487, so 1.0 + 5 sd = 1.24 and 1.0 + 10 sd = 1.49.  2.5 is 2.29x the widest observed
#       ignored-case ratio, 2.01x the 5 sd ceiling, and 30.8 sd above 1.0.  An ACC-ignored
#       firmware lands in the INCONCLUSIVE branch below (|ratio - 1| <= 0.15), never in PASS.
# So the floor sits at 1.57x below "obeyed" and 2.29x above "ignored" - loose on purpose, and
# still on the right side of both by a wide margin.  What would justify tightening it is a
# threshold that lands ON the lattice instead of between levels - 1750 raw = 2.6845 rad/s is
# 89.5 % of command, nearer the intended 90 % than the 92 % this gate actually measures, and at
# that level the same eight recordings give ratios 4.1323 .. 4.5749, mean 4.3538, sd 0.1327, all
# eight of them above even the old 4.0 - or the least-squares slope metric above.  Not a tighter
# number on this threshold.
T90_RATIO_MIN = 2.5        # t90(acc=10)/t90(acc=150); see the derivation above

# Cost bounds at four servos, 100 Hz, 1 Mbaud (Phase 3). Before Phase 3 the read cycle was four
# sequential FeedBack() round trips -- 3.072 ms measured at this cadence, probe 3 Q1b -- and the
# driver measured 3.0646 ms (phase2 hil_out4 H1.read_ms), i.e. the driver adds nothing above the
# bus. Phase 3 makes it one sync read of four ids.
#
# MEASURED, not predicted (PHASE3 5.11's binding rule, applied at 5.31 step 11): the mean H1 read
# average of the three candidate A/B runs is 1.64538 ms (phase3_evidence/cand_r1 1.64125,
# cand_r2 1.64664, cand_r3 1.64823), against the 1.734 ms probe 3 Q1 predicted. The rule is
# ceil_to_0.05(mean + 0.35): 1.64538 + 0.35 = 1.99538 -> 2.00. The 0.35 is 15x the 0.023 ms
# HEAD-vs-HEAD spread of 30 identical-code Phase 1 runs -- the only run-to-run variation ever
# measured in this statistic here. NOT a tail factor: this bounds a MEAN. The recomputed value is
# SMALLER than the 2.15 this constant shipped with, so it is tightened, not relaxed. These are
# BENCH bounds for four servos, not a promise about any other robot.
READ_MS_AVG_MAX = 2.00
# The max is cumulative since activation and is dominated by the non-RT kernel, not by the bus:
# 30 runs of IDENTICAL Phase 1 code spanned 3.64..4.98 ms at a 3.06 mean, an excursion of up to
# 1.92 ms over a ~2000-cycle window. Re-derived on the measured mean rather than the predicted
# one: 1.64538 + 1.92 = 3.5654, rounded up to 3.60 (it was 1.73 + 1.92 -> 4.00). Every gated
# candidate window clears it with 0.94 ms to spare -- the worst read max any sync-read H1 or H9
# produced is 2.6571 ms (cand_full_r1 H1), over a 2.17..2.66 ms span across nine such windows.
# SCOPE: a ~2000-cycle window, i.e. the 9 s / 12 s captures of H1 and H9 ONLY. It does not
# transfer to the soak, whose window is 30x longer -- see h11 (PHASE3 5.15), which deliberately
# does not gate on it.
READ_MS_MAX_MAX = 3.60
# Write: 1.4521 ms before (hil_out4), of which SyncWriteSpe for two wheels is 1.152 ms at this
# cadence (probe 2 Q1b). MEASURED (5.11's binding rule): the three candidate A/B runs mean
# 0.013578 ms (cand_r1 0.01402, cand_r2 0.01341, cand_r3 0.01331) -- 22x below the 0.30 ms the
# spec predicted, because what Phase 3 removed was two blocking Ack() round trips inside
# SyncWriteSpe, not airtime, so there is no residual driver-side cost to leave room for.
# ceil_to_0.05(0.013578 + 0.30) = 0.35, again tighter than the 0.60 shipped.
WRITE_MS_AVG_MAX = 0.35
# Worst write excursion above the mean seen in Phase 1/2 is 0.88 ms (2.3240 - 1.4475, recon
# hil_and_docs.md 4.1, hil_cand_r3_b S2 enforce true). 0.30 + 0.88 = 1.18, rounded up to 1.50.
# NOT re-derived on the measured mean, deliberately: 5.11's binding rule covers the two avg
# constants only, and unlike the read max the write max is not stationary enough to bound. Across
# the nine gated sync-read windows it spans 0.0434..0.6287 ms, a 14x spread (worst: cand_t20_r1
# H9), and the three ten-minute soaks span 0.0645..0.4539 ms. 0.013578 + 0.88 would give 0.90,
# which the worst observed value already reaches 70 % of. The same principle 5.11 states for
# raising a bound applies to lowering one: do not fit a threshold to a tail nobody has
# characterised (PHASE3 section 8 Q10). Same ~2000-cycle scope caveat as READ_MS_MAX_MAX.
WRITE_MS_MAX_MAX = 1.50

# The soak of PHASE3 5.13. Probe 3 Q1 saw 0 failures in 60 000 sync reads over ten minutes at
# 100 Hz; with zero events the honest statement is an upper bound, and the rule of three gives
# 3/60000 = 50 per million at 95 % confidence. The gate therefore asserts only that this bench is
# no worse than the bound its own measurement could resolve. It is a RATE, not a count, so a
# shortened soak (WAVESHARE_HIL_SOAK_S) uses the same threshold.
SOAK_FAIL_PER_MILLION_MAX = 50.0
# Below this the rule of three cannot resolve 50 per million at all (3/30000 = 100), so the row
# SKIPs rather than passing vacuously. 30 000 transactions is five minutes at 100 Hz.
SOAK_MIN_TRANSACTIONS = 30000
# Probe 3 Q1 measured a longest consecutive failing run of 0 over 60 000 cycles. Two allows an
# isolated hiccup and still fires 25x before max_read_fails (50) would drop a servo.
SOAK_WORST_CONSECUTIVE_MAX = 2
# PHASE3 5.13 makes a turning wheel part of the soak's definition -- "a stationary bus is not the
# bus a robot runs on" -- but the scenario asks for it with one unchecked `topic pub`, and no other
# H11 row can tell a spinning bus from a stopped one (a constant position with a zero velocity
# satisfies g1a/g1b/g1c/g2 exactly, and the failed-transaction rate of a stopped bus is not the
# number jazzy.md item 3 asks for). Half of the commanded 1.0 rad/s: six reported-velocity quanta
# (VQ = 0.0767 rad/s) above zero, and far enough below 1.0 to survive any plausible load droop.
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
    """Return every stale index plus the first sample after each run (12.2)."""
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


#
# The run directory, as hil_check.sh writes it: one sub-directory per scenario holding
# facts.json (the shell's key=value facts), the recorder's <label>.json, the stop_wheels
# readbacks, the port_probe results and log.txt (the stack's merged stdout).
#


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


# Not a median: every velocity row of 12.5 asks for an average, and the reported velocity is
# quantised at VQ, so a median can only ever be a lattice value -- up to half a quantum away.
def mean(values):
    """Return the arithmetic mean of the values, or NaN when there are none."""
    return sum(values) / len(values) if values else NAN


def flips(quantity):
    """Whether `inverted` flips this quantity (PHASE2_SPEC 3.2, D4). The H7 rows read it."""
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
# The per-servo tail the driver is required to print (PHASE3 5.14, frozen string L5). The rate row
# bounds the aggregate, so this is parsed for two reasons only: so a FAIL can say which ids the
# failures fell on, and so H11.per_servo can assert the tail is there at all (5.30 row 23 -- an
# aggregate alone does not satisfy jazzy.md item 3's "per servo").
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
    """Hold the stable-keyed rows of PHASE2_SPEC 12.4."""

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
        """Append one gated row. Nothing rewrites a FAIL (decision D5)."""
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
    """Emit the two port-holder rows every scenario carries (PHASE2_SPEC 12.4 item 3)."""
    R.ck('port_free_before', S.flag('port_free_before'), '[%s]' % S.fact('port_holders_before'))
    R.ck('port_free_after', S.flag('port_free_after'), '[%s]' % S.fact('port_holders_after'))


# One second of a 100 Hz loop. H9.cycle_recorded needs enough samples AFTER the component came
# back for g1c to be able to see a multi-turn reset in them, and one step is all it takes: a
# reset shows up as a single oversized position difference between two consecutive samples. A
# hundred is therefore not a statistical floor, it is "the recorder was still running and the
# loop was really turning over", set far enough above 2 that a recording which stopped in the
# middle of the transition cannot squeak past it.
H9_CYCLE_MIN_SAMPLES = 100

# H2's two numbers, reused verbatim so H9's post-cycle move is measured the same way H2 measures
# the arm it is being compared against: 0.005 rad of target error (h2's `target` rows) and
# 0.40 rad/s of step between the first or last pair of velocity samples (h2's `no_lurch`).
ARM_TARGET_TOL = 0.005
ARM_LURCH_MAX = 0.40


def cycle_rows(R, S, rec):
    """
    Emit the recovery rows for the inactive/active component cycle (jazzy.md section 6 step 6).

    WHAT WAS ALREADY GATED, and is deliberately NOT repeated here. H9 has cycled the hardware
    component since Phase 2, but exactly one consequence of the cycle was ever gated, and only
    implicitly: g1c.joint3 and g1c.joint4, emitted by gate_rows() over this same `interfaces`
    recording. g1c (hil_gates.py:359-362) asks whether any consecutive step of the unwrapped
    position ever needed reducing mod 2*pi, and a cycle that reset a wheel's multi-turn count
    shows up there as exactly one oversized step. That is measured, not assumed: Phase 4's commit
    e0293cc replaced on_activate's unconditional reset with a seeded check, and the two archived
    runs bracket it --

      phase5_evidence/pre_phase4_driver_2026-09-21_1445/hil_check.txt
        FAIL  H9.g1c.joint3  pairs=... reduced=37.6991      (and the same row for joint4)
      phase5_evidence/post_phase4_baseline_2026-09-21_1659/hil_check.txt
        PASS  H9.g1c.joint3  pairs=... reduced=0.0000       (and the same row for joint4)

    37.6991 rad is 6 * 2*pi, i.e. the whole accumulated turn count vanishing in one sample. So
    requirement (c) -- the unwrapped wheel position is continuous across the cycle -- is COVERED,
    by a row with a real failure on record. A second row asserting continuity would add a PASS
    and no coverage, and would make this report overstate what it checks.

    WHAT WAS NOT GATED, and is added here. Two things the cycle is supposed to restore, neither
    of which any row could see before:

    (a) the component returns to 'active'. Both set_hardware_component_state calls sent their
        output to /dev/null and their exit status was discarded, and no state was read back
        afterwards, so a cycle that left the component INACTIVE scored nothing at all. Worse, it
        would have made (c) vacuous rather than red: an inactive component publishes no
        /joint_states, g1c would have seen no post-cycle samples, and it would have PASSED.

    (b) the arm still tracks a commanded move. H9's two arm moves (arm_fast, arm_back) both run
        BEFORE t_cycled, so nothing was commanded after the cycle. 'active' is the controller
        manager's opinion of its own state machine; it is not evidence that the write path to
        the bus came back. A move is.

    And one guard that makes (c)'s existing coverage honest rather than incidental:
    cycle_recorded. g1c can only see a reset in samples taken after the component came back, and
    before Phase 5 the 40 s recorder was finishing within a couple of seconds of the cycle. A
    recorder that stops first does not fail g1c -- it PASSES it over a window that cannot contain
    the defect, the same vacuous-gate shape EXAMPLE_IDS had in h1. This row is what stops the
    recorder duration from quietly drifting back under the cycle.
    """
    R.ck('cycle_state',
         S.fact('hw_state_after_cycle') == 'active' and S.fact('cycle_inactive_rc') == '0' and
         S.fact('cycle_active_rc') == '0',
         'the component came back from the inactive/active cycle of section 6 step 6',
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


#
# The ten scenarios of PHASE2_SPEC 12.5. Each checker takes the report (already prefixed with
# the scenario name), its scenario directory, and a context shared between scenarios.
#

IFACES = ('position', 'velocity', 'effort', 'current', 'voltage', 'temperature', 'load',
          'status', 'torque')
OFFSET = 1.570796          # the offset every pos joint of the bench descriptions carries
ESCAPE = ('controller_manager hardware_components_initial_state.'
          'shutdown_on_initial_state_failure: false keeps the node alive with the component '
          'unconfigured (decision D1)')
PING_DIAGNOSIS = ('a status of 1, 2, 3, 4 or 9 means the driver latched SCS::Error from a Ping '
                  'or an Ack (src/SCS.cpp:261, Error = bBuf[2]) instead of the feedback Read')
# Every servo id the shipped example declares: description/ros2_control/example.ros2_control.xacro
# gives joint1..joint4 the ids 1, 2, 3 and 4 (:72-133, one <param name="id"> each). H1 is the only
# scenario that runs that description, so this is the only place the gates learn what "all of them
# answered" means there. It lives here, beside the other facts about the descriptions, because it
# was open coded as (1, 2, 3) inside h1() until now: Phase 4 added joint4 and the tuple there was
# not part of the change, so for a whole phase a dead id 4 passed H1.activate -- the single gate
# over the shipped launch. A joint added to the example has to come past this line to be gated.
EXAMPLE_IDS = (1, 2, 3, 4)

# H12, the limits scenario of jazzy.md section 6 step 8. The first four numbers are the stimulus
# h_H12 renders and publishes (test/hil_check.sh); they are repeated here so the rows can be
# written in terms of them, and H12.stimulus compares every one of them against the fact the
# scenario recorded, so the two files cannot drift apart in silence the way a hard-coded
# expectation can. Beside them are the driver's OWN ceilings, which is what makes a clamp
# attributable at all: each command below sits between the description's ceiling and the driver's,
# so only the controller manager's JointSaturationLimiter can produce the clamped value. Note that
# POS_CMD_LIMIT equals OFFSET on this bench by coincidence of the mechanics -- it is the arm's
# <command_interface name="position"> max, declared once per arm joint and identical in both:
# test/hil/descriptions/bench.urdf.xacro:113-116 for joint1 (the max itself at :115) and :125-128
# for joint2 (at :127). It is NOT the offset (:108 and :123, the same 1.570796) and it is not the
# xacro:if that would add a max_speed param at :110-112, which is what this comment cited until
# Phase 5's review; a render that changed one of the three would leave the other two where they
# were, which is exactly why the number is named by its own line here.
H12_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4')
H12_ARM_LIMIT = 0.8            # rad, the <limit lower/upper> H12 renders (arm_pos_limit)
H12_ARM_COMMAND = 1.2          # rad, past that limit and inside the driver's POS_CMD_LIMIT
H12_WHEEL_LIMIT = 2.0          # rad/s, the <limit velocity> H12 renders (wheel_vel_limit)
H12_WHEEL_COMMAND = 8.0        # rad/s, past that limit and inside the driver's V_CAP
POS_CMD_LIMIT = 1.570796       # rad, the arm's <command_interface> max: the driver's own ceiling

# The attribution argument of h12(), executed rather than asserted in prose. Every clamp row of
# H12 reads "the command landed on the description's ceiling and not on the driver's", and that
# sentence is only evidence if the three numbers per axis really are ordered ceiling < command <=
# driver. Edit one constant -- tighten the driver's command interface, raise the rendered <limit>,
# lower a command -- and each clamp row would keep passing while proving nothing, because the two
# candidate explanations would no longer be distinguishable by the value alone. The same shape as
# the INVERTED_FLIPS guard at the top of this file, and for the same reason: a precondition that
# cannot be measured on the bench is checked where it is written down. It runs at import, so it
# guards a real --run-dir report and not only --self-test.
for _axis, _limit, _command, _driver in (
        ('position', H12_ARM_LIMIT, H12_ARM_COMMAND, POS_CMD_LIMIT),
        ('velocity', H12_WHEEL_LIMIT, H12_WHEEL_COMMAND, V_CAP)):
    if not _limit < _command <= _driver:
        raise SystemExit(
            "hil_gates: H12's %s clamp is no longer attributable: the rendered <limit> %.6f, the "
            'command %.6f and the driver-side ceiling %.6f must satisfy limit < command <= '
            'driver, or a clamp landing on the limit is not evidence that the controller '
            "manager's JointSaturationLimiter produced it (jazzy.md section 6 step 8)"
            % (_axis, _limit, _command, _driver))

# How far inside the tightened limit the arm has to be parked before the limited stack starts.
# This is not a tolerance on anything measured. The throw of trap (c) fires 0.0087 rad outside the
# <limit> (joint_limits/joint_limits_helpers.hpp:32, OUT_OF_BOUNDS_EXCEPTION_TOLERANCE, commented
# there as "0.5 degrees"), and h_H12's park sends the arm to 0.0, so the row only has to separate
# "parked" from "somewhere near the edge": 0.05 rad is 33 encoder ticks, 5.7x the throw tolerance
# and 15x the 0.0033 rad the arm was measured to settle short of a commanded target
# (H2.target.move_to_06, final=0.5967, in the post-Phase-4 baseline run). It cannot fail a park
# that worked, and it cannot pass an arm the limiter is about to throw on.
H12_PARK_MARGIN = 0.05

# The clamped position, taken from the last sample of the recording. 0.010 rad = 6.5 ticks covers
# the two terms between a settled servo and the ceiling it was clamped at: the 0.0033 rad the arm
# settles short of a target (measured above; H2.target gates that same quantity at 0.005), plus
# the 0.002 rad POSITION_BOUNDS_TOLERANCE of joint_limits_helpers.hpp:30, which is the slack the
# limiter itself may carry around a bound. It leaves 0.39 rad of clear air below the 1.2 rad an
# unclamped command would have reached, so nothing is given away by rounding it up to 0.010.
H12_POS_TOL = 0.010

# The clamped wheel speed: the same +-0.060 rad/s H8.speed_wheel uses, and for the same reason.
# That row is this row's stimulus measured against the other clamp -- a wheel commanded 8.0 rad/s
# into a 2.0 rad/s ceiling, there the driver's max_speed and here the description's <limit> -- so
# its spread is the only figure available that was taken under these conditions. The post-Phase-4
# baseline recorded it at mean=1.9942 (1.99418 is 26 reported-velocity quanta, the lattice value
# just under 2.0), i.e. 0.0058 of the 0.060 used.
H12_VEL_TOL = 0.060

# hardware_interface logs this once per joint when enforce_command_limits is on; the format string
# is `Creating JointSaturationLimiter for joint '%s' in hardware '%s'` in libhardware_interface.so.
# Parsed rather than counted because the joint name is the whole point of H12.limiter_lines.
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
    # The same warning about an id EXAMPLE_IDS does not list, counted as a group. A per-id sum can
    # only ever be as current as its list, so this is the other half of the gate: if the example
    # grows a fifth servo and EXAMPLE_IDS is not updated with it, the silent servo still lands on
    # a FAIL row here instead of on nothing, and the row names the drift rather than hiding it.
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


def h1b(R, S, C):
    """
    Gate the shipped example while it is COMMANDED (jazzy.md section 6 steps 1, 3, 4 and 7).

    H1 proves the packaged launch comes up and idles cleanly. Every proof that it MOVES anything,
    and every proof about how it shuts down, was against the bench description and bench.yaml --
    files that ship with the tests, not with the package. These rows are the same measurements
    H2, H3 and H10 make, taken on the stack a user actually runs. Where a bound here matches one
    of theirs it is theirs verbatim, so the two can be read side by side; the comments say which.
    """
    port_rows(R, S)
    R.ck('activate', S.flag('controllers_active') and S.fact('hw_state') == 'active',
         'the three example controllers reached active and the component is up',
         active=S.fact('controllers_active'), hw_state=S.fact('hw_state'), expected='active')
    # Phase 4 ships diff_drive_controller loaded-but-inactive on purpose, because it claims the
    # same joint3/joint4 velocity command interfaces as joint_velocity_controller and
    # ros2_control gives each command interface to exactly one controller. Two regressions land
    # on this row: 'active' (the wheels would be taken away from the controller H1B commands)
    # and '' (the controller is not loaded at all, which on this bench means the separate
    # ros-jazzy-diff-drive-controller apt package is missing). Neither had a row before.
    state = S.fact('diff_drive_state')
    R.ck('diff_drive', state == 'inactive', 'loaded and inactive, as example.launch.py spawns it '
         "(--inactive); '' means the controller never loaded",
         state=state or "''", expected='inactive')
    # Step 3, the arm. H2's numbers verbatim: 0.005 rad of target error, 0.40 rad/s of lurch at
    # either end of the recording, 0.02 rad of overshoot on the move that has somewhere to go.
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
    # The guard that keeps the wheel rows from going quiet. h_H1B skips the wheel command if the
    # component is no longer active after the arm move -- the safe thing to do, because a stack
    # that has just died cannot publish the stop either -- but a skip that reports nothing is how
    # the jazzy.md section 7 crash would hide. It is a FAIL here, and it names the states.
    R.ck('arm_survived', S.fact('hw_state_after_arm') == 'active' and
         S.fact('arm_state_after') == 'active' and S.flag('vel_attempted'),
         'the example JTC claims both position and velocity command interfaces (jazzy.md '
         'section 7); this row is where that crash would land',
         hw_state=S.fact('hw_state_after_arm'), controller=S.fact('arm_state_after') or "''",
         wheels_commanded=S.fact('vel_attempted'))
    # Step 4, the wheels. H3's numbers verbatim: mean speed within 0.020 rad/s of 2.0 (its "1
    # percent"), and after the stop a mean |v| under 0.05 rad/s with under 0.01 rad of drift.
    rec = S.rec('ex_vel_2')
    for joint in ('joint3', 'joint4'):
        t, p, v = series(rec, joint)
        mean_v, n = mean_vel(rec, joint, 1.0, 0.5)
        R.ck('mean_vel.%s' % joint, abs(mean_v - 2.0) <= 0.020, mean=mean_v, target=2.0,
             tol=0.020, n=n)
        # Monotonicity, which H3 leaves to g1b/g2 and which is asked explicitly here because a
        # wheel commanded one way and reported as travelling the other is the failure this
        # scenario exists to catch on the shipped stack. One encoder tick of slack: the position
        # state is quantised at TICK, so two consecutive samples of a barely-moving wheel can
        # legitimately differ by a tick in either direction.
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
    # Step 7, the SIGINT teardown of the packaged launch, asked of the wrapper a user runs and of
    # a stack that has just driven an arm and a wheel.
    #
    # This row gates `ros2 launch`'s AGGREGATE status and nothing narrower, which is weaker than
    # H10's `exit`: that one waits on the controller manager's own process (`wait "$CM_WRAP"`,
    # hil_check.sh's h_H10) and so speaks only for the driver's teardown, while ros2 launch
    # returns non-zero if ANY process it launched did -- either spawner, robot_state_publisher,
    # the controller manager. So a red row here says "something in the packaged launch exited
    # badly under SIGINT" and cannot by itself name the driver. That is still worth gating: it is
    # the status a user's shell sees from the packaged launch, and no other row in this suite
    # looks at it.
    #
    # The separable teardown claim is the `sequence` row below, which finds the controller
    # manager's own 'deactivate' before its own 'shutdown' in the launch output. Reading the CM's
    # exit code out of that output instead was considered and rejected: it would mean parsing
    # launch's own process-exit lines, whose wording is a launch implementation detail, and there
    # is no archived H1B run to pin that wording against (the scenario is new in Phase 5, and
    # phase5_evidence/post_phase4_baseline_2026-09-21_1659/ has no H1B directory). An unverifiable
    # parse asserting a stronger claim is worse than a weaker claim stated honestly.
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
        # The refusal is only evidence about the driver if it happened while the holder still had
        # the port. A hold that expired first lets the driver configure, which is the run's fault
        # and not a driver defect: this row is what tells the two apart.
        within = S.fact('%s_refusal_within_hold' % label)
        R.ck(key.replace('driver_refused', 'refusal_within_hold'), within == 'true',
             'false means the hold expired before the driver reached on_configure; rerun it, the '
             'row below is then not evidence about the driver', within=within or 'unrecorded',
             probe_pid=pid or 'none')
        # 12.5 asks for the readback after the probe exits to be bit-identical to the one before
        # it: the baseline is this holder's own pre readback, taken while the port was free and
        # immediately before it, not the scenario's pre_readback, which H6's own active stack
        # legitimately wrote over.
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
         'a refusal here is a leaked exclusive flag (PHASE2_SPEC 12.4)',
         verdict=released.get('verdict'), rc=released.get('rc'))


def h7(R, S, C):
    port_rows(R, S)
    moves = S.rec('pos_move')
    for joint in ('joint1', 'joint2'):
        final = final_of(moves, joint)
        R.ck('pos_reported.%s' % joint, abs(final - 0.6) <= 0.005,
             'inversion must be invisible above the driver', final=final, target=0.6, tol=0.005)
    centre = round(OFFSET * STEPS_PER_RAD)
    # joint2 and joint4 are the inverted half of each mirrored pair, so every register they
    # carry is mirrored exactly when `inverted` flips that quantity (D4).
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
             'both must be >= 0 on every joint, the inverted ones included (D4)',
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
    # H9's old single `present` row asked two questions at once and gated on both: does every
    # declared name arrive for every joint, and does it arrive in the declared order? Only the
    # first is the driver's to answer, so they are split (decision U2).
    #
    # The gate is presence. A name the description declares and the driver never exports is a
    # real defect -- the resource manager refuses such a description outright.
    #
    # The order is a NOTE and never a gate, because the driver does not own it.
    # WaveshareServos::on_export_state_interfaces (src/waveshare_servos.cpp:817-836) builds its
    # name list by walking info_.joints, and each joint's state_interfaces, in description order,
    # and handles_named (src/waveshare_servos.cpp:797-814) returns the handles in exactly that
    # order; test_load_waveshare_servos.cpp's
    # state_interfaces_are_exported_in_description_order_for_any_subset pins that behaviour in
    # `colcon test`. The reordering seen on /dynamic_joint_states therefore happens downstream of
    # the export -- in the resource manager's handoff or in joint_state_broadcaster's own map --
    # and it is uniform rather than sporadic: every one of the 18,810 joint-blocks of the Phase 2
    # run carried the same permutation. So a surprising order below is not a driver bug; do not
    # "fix" the driver for it. What a consumer may rely on is the name, not its index.
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
    R.ck('effort_vs_current', pair <= 1e-9, 'a magnitude identity: neither side flips (D4)',
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
    # The window is t_cycle_done, not t_cycled, because this row's detail says "after the
    # inactive/active cycle" and t_cycled is recorded BEFORE both transitions (hil_check.sh:755,
    # against the transitions at :769 and :773 and t_cycle_done at :778). The old window was a
    # superset of the claim -- it could not miss a leaked status, but it could report one that
    # arrived before the cycle and blame the cycle for it, and the diagnosis it prints
    # (PING_DIAGNOSIS: a status latched from a Ping or an Ack) is specifically about the ping
    # on_activate runs when the component comes back. Narrowing the window to the marker that
    # really means "after" is what makes the row measure its own sentence.
    #
    # Narrowing costs no coverage, which is why it is the right fix rather than rewording the
    # detail: status_zero above gates EVERY sample of this recording for a non-zero status, and a
    # value equal to a servo id is non-zero, so a leak anywhere still turns a row red. Measured on
    # the fixture with a status of 3.0 injected between t_cycled and t_cycle_done: status_zero
    # FAILs and status_not_ping passes, which is exactly the division of labour intended -- one
    # row says "a status was not zero", this one says "and the cycle's ping is why".
    #
    # NaN-guarded the way cycle_recorded is (hil_gates.py:705-711), and for a sharper reason: an
    # unparseable or missing t_cycle_done makes every `between` comparison false, so the window
    # would be EMPTY and the row would PASS while having looked at nothing. That vacuous pass is
    # the failure mode this file keeps finding (EXAMPLE_IDS in h1, the recorder in
    # cycle_recorded), so the fact has to be there. n is reported so a window that is legitimately
    # present but short is visible; how short it may be is cycle_recorded's row, which fails below
    # H9_CYCLE_MIN_SAMPLES samples after the same marker, and it is the reason this row does not
    # gate a sample count of its own.
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
        # The two max terms are NEW in Phase 3 (5.11): this row asserted averages only before, so
        # H9 has never checked a maximum. H9's diagnostics capture is 6 s, the same ~2000-cycle
        # window the two *_MAX_MAX bounds were derived over, so the scope matches -- unlike h11,
        # which deliberately leaves its maxima ungated.
        #
        # The rate term is the known-weak one, and it predates Phase 3. rate_hz() reads
        # /joint_states header stamps AS RECEIVED BY hil_record.py, so a recorder that misses
        # messages reads as a control loop that missed cycles. Measured on a healthy bench: a
        # single ~0.34 s recorder hole drags this term to 95.6 Hz and FAILs the row while the
        # controller manager's own periodicity.average in the same /diagnostics file reads
        # 100.0008 Hz and its periodicity.min never fell below 91 Hz -- i.e. no cycle ever ran
        # long. It happened on both arms (phase3_evidence/SUITE_DIFF.md, cand_r2 and
        # base_full_r1). The fix, when someone wants it, is to gate on periodicity.average from
        # /diagnostics, which is what the CM publishes for exactly this purpose; it is left alone
        # here because Phase 3 does not re-base a pre-existing term it did not cause.
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
          ' evidence, never a gate: signed_by_direction opens a Phase 3 item, it does not change '
          'a Phase 2 rule (D4)')


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
    # the second stimulus of 12.5: SIGTERM to a bare ros2_control_node with a wheel turning
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
        # All four, not three: a row that is simply absent is invisible in the report, and
        # H11.no_drop and H11.per_servo are both Phase 3 gates (5.30 rows 10 and 23).
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
            # The tail rides in the detail, not in a fact: it is here so a FAIL says which ids the
            # failures fell on, which is the first question a non-zero rate raises (5.18).
            R.ck('fail_rate', rate <= SOAK_FAIL_PER_MILLION_MAX and
                 worst <= SOAK_WORST_CONSECUTIVE_MAX, named, transactions=n, failed=bad,
                 per_million=rate, bound=SOAK_FAIL_PER_MILLION_MAX, worst_consecutive=worst,
                 worst_bound=SOAK_WORST_CONSECUTIVE_MAX)
        R.ck('no_drop', dropped == 0 and S.logged('stopped answering') == 0, dropped=dropped,
             drop_lines=S.logged('stopped answering'))
        # 5.30 row 23: jazzy.md item 3 asks for the count PER SERVO, so the bracketed tail of L5 is
        # required, not a courtesy. Presence only, and deliberately not "one pair per declared
        # servo": nothing this checker can see says how many servos the run declared, and a gate
        # that guesses that would fail a healthy bench. A missing tail is unambiguous.
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
        # Averages and rate only. The maxima ride along as facts because they are cumulative
        # since activation, i.e. the extreme of ~60 000 cycles here against the ~2000 the
        # READ_MS_MAX_MAX / WRITE_MS_MAX_MAX bounds were derived over (5.15). Gating them here
        # would fail a healthy bench and then pressure H1's bound upwards.
        R.ck('cost', read_avg < READ_MS_AVG_MAX and write_avg < WRITE_MS_AVG_MAX and
             abs(rate_hz(stamps) - 100.0) <= 2.0, '[no-regression]', read_ms=read_avg,
             write_ms=write_avg, hz=rate_hz(stamps), read_max_ungated=read_max,
             write_max_ungated=write_max)
    R.ck('gap', max_gap(stamps) <= 0.050, hz_gap_s=max_gap(stamps), bound=0.050)
    # The soak's LOAD, which nothing else here checks. hil_check.sh asks for it with one
    # fire-and-forget `topic pub` whose return value it discards, so a publish that times out
    # waiting for a matching subscription -- a spawner race, a daemon hiccup, a wheels controller
    # that is up but not yet matched -- leaves both wheels at 0 rad/s for the full ten minutes, and
    # every other row still passes. That soak's failed-transaction rate is then measured on a load
    # PHASE3 5.13 forbids and reported by 5.18 under conditions that did not hold. `all`, not a
    # mean of the two: one wheel turning is not the recorded load either. A NaN mean (no samples
    # for that joint) fails the comparison, which is the right direction -- no evidence of motion
    # is not evidence of motion.
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


def h12(R, S, C):
    """
    Gate the JointSaturationLimiter clamp of jazzy.md section 6 step 8 (h_H12, hil_check.sh:968).

    THE ATTRIBUTION ARGUMENT, which is the whole of this scenario and the reason every row below
    is worth anything. The driver clamps too, and it would clamp these same commands: a position
    command is held inside the command interface's own min/max (parsed at
    src/waveshare_servos.cpp:591-621, clamped in write() at :1894-1899) and a wheel's goal speed
    inside max_speed_counts (:1939-1941), 6000 counts = V_CAP = 9.2039 rad/s when no max_speed
    param is given. A command clamped at a number both mechanisms agree on is evidence
    for neither of them -- that is what H8 already measures, on the driver's side. So h_H12
    renders the two <limit> ceilings TIGHTER than the driver's own and leaves the driver's where
    they are:

      joint           rendered <limit>   the driver's own ceiling   commanded here
      joint1/joint2   +-0.8 rad          +-1.570796 rad             1.2 rad
      joint3/joint4   2.0 rad/s          9.2038847 rad/s            8.0 rad/s

    Each command is INSIDE what the driver would pass and OUTSIDE what the description allows, so
    a value landing on 0.8 rad or 2.0 rad/s can only have come from the limiter: nothing else in
    the stack knows those two numbers, and the driver never reads a URDF <limit> at all. That
    ordering is a precondition of reading any row here, so it is not left in this docstring -- the
    import-time guard beside the constants (hil_gates.py:775-786) refuses to run at all if a
    constant is ever edited to break it.

    WHAT IS COVERED.
      stimulus            the four numbers are the shell's, not this file's opinion of them.
      park                the precondition stack came up, so the arm was driven at all.
      arm_inside_limits   where the arm actually came to rest, off the servos, port free. This is
                          trap (c) of the Phase 4 amendment: an arm more than 0.0087 rad outside
                          the tightened <limit> makes compute_position_limits THROW and the
                          controller manager deactivate the arm controller instead of clamping.
      limited_stack       the limiters-on stack came up, so a silent clamp row means "no clamp"
                          rather than "no stack".
      limiter_lines       one JointSaturationLimiter per joint was really built. Without it,
                          every clamp row could be satisfied by a stack with the limiters OFF and
                          a driver that happened to clamp somewhere similar.
      pos_clamp           the arm stopped at the rendered limit, not at the command.
      vel_clamp           the wheels turned at the rendered limit, not at the command.
      vel_register        the driver's own last write of the same clamp, read off the servo.
      arm_state_after     the arm controller is still ACTIVE. A limiter throw takes it down mid
                          scenario, and without this row the throw is silent and every other row
                          here becomes unreadable: pos_clamp would report wherever the arm was
                          abandoned, and a reader would have no way to tell that from a clamp.
      wheels_stopped      the bench was handed back with the wheels stopped.
      arm_parked_after    ... and with the arm off its limit, which is the state the NEXT run
                          needs (5.7 ticks from the trap-(c) throw is not a safe resting place).

    WHAT IS NOT COVERED, and why not.
      - The consistency gates (g1a/g1b/g1c/g2/g3) are deliberately NOT emitted over these
        recordings, unlike h2/h3/h1b. They ask whether the driver's reported position and
        velocity agree with each other, which is a property of the driver's state path and is
        measured over H3, H4 and H9's recordings under the ordinary render. Here they would be
        narrower versions of those same measurements (the arm move is 2 s plus 2.5 s of
        post-roll; the wheel spin publishes no stop, so there is no rest window at all), and a
        red g2 could not distinguish a limiter fault from a driver fault, which is the one
        distinction this scenario exists to make. pos_clamp reports the recording's maximum
        beside its final sample so the approach is visible without being gated.
      - joint2's position clamp. h_H12 commands joint1 alone (`move pos_clamp joint1 1.2`), so
        joint2 has no clamped position to read; joint2 is covered by limiter_lines (its limiter
        was built) and by arm_inside_limits / arm_parked_after (it is parked at both ends).
      - limited_cm_exit_code is reported and never gated: that stack is SIGKILLed on purpose, so
        137 is the correct answer and a 0 there would mean the kill missed.
      - That the limiter passes a LEGAL command through untouched. Every legal arm and wheel move
        in this suite is measured with enforce_command_limits false (bench.yaml:7 for the bench
        scenarios, bringup/config/example_controllers.yaml:23 for H1/H1B), so nothing
        here would catch a limiter that clamped everything to its <limit>. It would take a second
        limited stack and a legal command to add, and jazzy.md section 6 step 8 does not ask for
        it; a limiter that clamped a legal command would show up as H12.pos_clamp passing while
        H2/H7's arm rows still passed on the unlimited render, i.e. as nothing at all. Named here
        so the gap is on the record rather than mistaken for coverage.
    """
    port_rows(R, S)
    # The stimulus, compared fact by fact against the constants above rather than assumed. Both
    # sides are decimal literals of the same text, so the comparison is exact equality and any
    # mismatch is real drift, not float noise: the shell rendered or published a number this
    # file's rows are not written for, and every clamp row below would be measuring the wrong
    # ceiling while still passing. A missing fact reads as NaN, which compares unequal.
    stimulus = (('arm_pos_limit', H12_ARM_LIMIT), ('arm_command', H12_ARM_COMMAND),
                ('wheel_vel_limit', H12_WHEEL_LIMIT), ('wheel_command', H12_WHEEL_COMMAND))
    drifted = ['%s=[%s] wanted %s' % (key, S.fact(key), want)
               for key, want in stimulus if S.number(key) != want]
    R.ck('stimulus', not drifted,
         'the four numbers h_H12 rendered and published: %s'
         % (', '.join(drifted) or 'all four agree with the constants in this file'),
         **{key: S.number(key) for key, _ in stimulus})
    # The first half of the trap-(c) precondition. h_H12 also guards these two and ABORTS the
    # scenario before the limited stack when they are bad, so on the healthy path this row is a
    # second reading of the same numbers -- kept because the guard is in the other file: a guard
    # removed, reordered or made conditional there leaves this row as the only thing that still
    # sees a park stack that never came up. park_cm_exit_code is gated ONLY here (the guard does
    # not read it), and 0 is the right expectation because stop_stack sent TERM: H7, H9 and H11
    # all recorded cm_exit_code 0 for a TERM teardown in the post-Phase-4 baseline run, and 137
    # would mean the driver had to be killed instead of deactivating.
    R.ck('park', S.fact('park_spawner_rc') == '0' and S.flag('park_controllers_active') and
         S.fact('park_cm_exit_code') == '0',
         'the limiters-off stack that parks the arm before the clamp test',
         spawner_rc=S.fact('park_spawner_rc') or "''",
         controllers_active=S.fact('park_controllers_active') or "''",
         cm_exit_code=S.fact('park_cm_exit_code') or "''")
    # The second half, and the one no amount of guarding can force: where the arm actually is.
    # Read off the servos with the port free, so it is the servo's own measured position and not
    # the controller's idea of it -- which is the quantity compute_position_limits throws on.
    # pos_last_raw is a raw encoder tick count; the joint angle is raw * TICK - offset for a pos
    # joint whose `inverted` is false, which is what both arm joints render here (the offset is
    # 1.570796 at bench.urdf.xacro:108 and :123, and inverted2 defaults to false, which h_H12's
    # renders never override). The bound is H12_PARK_MARGIN, far inside the +-0.8 <limit> rather
    # than at the 0.8087 rad where the throw begins: the park commands 0.0, so this row says "the
    # park did what it says" and not merely "we got away with it".
    for servo in (1, 2):
        raw = S.regs('arm_pre', servo).get('pos_last_raw', -1)
        rad = raw * TICK - OFFSET
        R.ck('arm_inside_limits.id%d' % servo, raw >= 0 and abs(rad) <= H12_PARK_MARGIN,
             'the trap-(c) precondition: the limiter throws 0.0087 rad outside the +-%g <limit> '
             'and the controller manager then deactivates the arm instead of clamping'
             % H12_ARM_LIMIT,
             pos_last_raw=raw, rad=rad, margin=H12_PARK_MARGIN)
    # The stack the whole scenario is about. Its own keys, because h_H12 runs three stacks and the
    # shared spawner_rc / controllers_active facts end up holding the LAST one's values (the final
    # park's). Without this row a clamp row that reported nothing would be ambiguous between "the
    # limiter did not clamp" and "there was never a stack to clamp with".
    R.ck('limited_stack', S.fact('limited_spawner_rc') == '0' and
         S.flag('limited_controllers_active'),
         'the enforce_command_limits stack (bench_limits.yaml) came up',
         spawner_rc=S.fact('limited_spawner_rc') or "''",
         controllers_active=S.fact('limited_controllers_active') or "''",
         cm_exit_code=S.fact('limited_cm_exit_code') or "''")
    # The limiters were really built, one per joint. Read out of the ONE controller-manager log
    # h_H12 names in limited_cm_log, not out of S.log: scenario_end concatenates every *.stdout
    # in the directory into log.txt, so the scenario's three stacks all land there and a count
    # taken over the lot could not say which stack produced the lines.
    #
    # Matched as a SET and never by index. jazzy.md's Phase 4 amendment to section 6 step 8
    # records that hardware_interface emits these in hash-map order, so the order carries no
    # information at all; `sorted` on both sides keeps the multiplicity, which does -- the claim
    # is exactly one line per joint, and two lines for one joint or none for another is a real
    # defect in the render.
    limited_log = S.fact('limited_cm_log')
    found = LIMITER_LINE.findall(S.txt(limited_log))
    joints = sorted(name for name, _ in found)
    R.ck('limiter_lines', joints == sorted(H12_JOINTS),
         'one JointSaturationLimiter per joint, matched as a set because these come out in '
         'hash-map order; without it every clamp row below could be met by a limiters-off stack',
         log=limited_log or "''", joints=','.join(joints) or 'none',
         expected=','.join(sorted(H12_JOINTS)),
         hardware=','.join(sorted({hw for _, hw in found})) or 'none')
    # The position clamp. The final sample, after 2.5 s of post-roll: the arm was commanded 1.2
    # rad and the three candidate resting places are 0.8 (the rendered <limit>), 1.2 (no clamp at
    # all) and 1.570796 (the driver's own command-interface ceiling, which would mean the driver
    # clamped and the limiter did not). The nearest pair of those is 0.4 rad apart -- 261 encoder
    # ticks, 40x H12_POS_TOL -- so the row cannot confuse two of them. `max` is reported and not
    # gated: it says whether the approach ever went past the ceiling on its way (see the
    # docstring's list of what is not covered).
    arm = S.rec('pos_clamp')
    final, samples = final_of(arm, 'joint1'), series(arm, 'joint1')[1]
    R.ck('pos_clamp', abs(final - H12_ARM_LIMIT) <= H12_POS_TOL,
         'clamped at the rendered <limit>, not at the command and not at the driver ceiling',
         final=final, clamped_at=H12_ARM_LIMIT, tol=H12_POS_TOL, commanded=H12_ARM_COMMAND,
         driver_ceiling=POS_CMD_LIMIT, max=max(samples) if samples else NAN)
    # The velocity clamp, one row per wheel, measured the way H8.speed_wheel measures the same
    # stimulus against the driver's clamp: the arithmetic mean of the reported velocity between
    # t_cmd + 1.0 and t_stop - 0.5. Same window and same tolerance, so the two rows can be read
    # side by side -- there the wheel was commanded 8.0 rad/s into the driver's 2.0 rad/s
    # max_speed, here into the description's 2.0 rad/s <limit>. No stop is published in this
    # recording (the SIGKILL has to land while 8.0 still stands), so t_stop is t_cmd + hold and
    # there is no rest window to read; the stop is proved by wheels_stopped instead.
    wheels = S.rec('vel_clamp')
    for joint in ('joint3', 'joint4'):
        mean_v, n = mean_vel(wheels, joint, 1.0, 0.5)
        R.ck('vel_clamp.%s' % joint, abs(mean_v - H12_WHEEL_LIMIT) <= H12_VEL_TOL,
             'clamped at the rendered <limit velocity>, not at the command',
             mean=mean_v, clamped_at=H12_WHEEL_LIMIT, tol=H12_VEL_TOL,
             commanded=H12_WHEEL_COMMAND, driver_ceiling=V_CAP, n=n)
    # The same clamp, by a completely different path: the goal-speed register as the driver last
    # wrote it, read back off the servo after the stack was SIGKILLed. The rows above measure what
    # the encoder reported through the driver's READ path; this one is the driver's WRITE path,
    # so a clamp that shows up in both did not come from a misreporting state interface. Exactly
    # H8's register rows in shape, bound and tolerance (+-1 count of rounding): 2.0 rad/s is
    # 1303.797 counts, and an unclamped 8.0 rad/s would have been 5215 -- nowhere near.
    want = round(H12_WHEEL_LIMIT * STEPS_PER_RAD)
    for servo in (3, 4):
        raw = S.reg('vel_readback', servo, 'goal_speed_raw')
        R.ck('vel_register.id%d' % servo, abs(raw - want) <= 1,
             "the driver's own last write of the clamped command, independent of the read path",
             goal_speed_raw=raw, expected=want,
             unclamped=round(H12_WHEEL_COMMAND * STEPS_PER_RAD))
    # The row that keeps every row above readable. A limiter throw does not happen at activation
    # -- it happens the first time the limiter enforces -- so limited_stack was true either way
    # and the controller manager simply deactivates the arm controller mid-scenario. Then
    # pos_clamp reports wherever the arm was abandoned, which can be anywhere at all including
    # inside H12_POS_TOL of 0.8, and nothing else in the report would say why.
    state = S.fact('arm_state_after')
    R.ck('arm_state_after', state == 'active',
         'the arm controller survived the clamp; a trap-(c) throw deactivates it and makes every '
         'other H12 row unreadable',
         state=state or "''", expected='active')
    # Handback, part one: the wheels. Read off the bus by h_H12's vel_stop, i.e. after the limited
    # stack was SIGKILLed with 8.0 rad/s still commanded, which is the one place in this suite
    # where a latched goal speed could outlive its process.
    stopped = S.rec('vel_stop')
    R.ck('wheels_stopped', stopped.get('any_moving') is False,
         'the SIGKILL left a goal speed standing; this is the read that says it was cleared',
         any_moving=stopped.get('any_moving'))
    # Handback, part two: the arm, off its limit. park_final runs on an ordinary limiters-off
    # stack whose driver-side ceiling is +-1.570796, so it can always drive back from 0.8; the
    # park_back move inside the limited stack cannot be trusted to, because the case that matters
    # is the one where that stack's arm controller is already gone. All three of the stack's own
    # numbers ride along in this row's detail rather than in rows of their own: if the arm is not
    # parked they are the explanation, and if it is parked they add nothing to gate. All three,
    # deliberately -- a fact nothing reads is the shape H12.park exists to prevent, so
    # park_final_cm_exit_code is printed here beside its two siblings rather than left recorded
    # and unread.
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


CHECKERS = (('H1', h1), ('H1B', h1b), ('H2', h2), ('H3', h3), ('H4', h4), ('H5A', h5a),
            ('H5B', h5b), ('H5C', h5c), ('H6', h6), ('H7', h7), ('H8', h8), ('H9', h9),
            ('H10', h10), ('H11', h11), ('H12', h12))


# Directories that live beside the scenarios in the run directory and are not scenarios.
# `roslog` is created per scenario by scenario_begin (ROS_LOG_DIR="$OUT/roslog/$SCEN"); anything
# a self-test or a human drops in for scratch is expected to lead with '_' or '.'.
NOT_A_SCENARIO = ('roslog',)


def unchecked_rows(R, run_dir):
    """
    Fail loudly for a scenario directory that ran on the bench and that no checker claimed.

    run() walks CHECKERS, not the directory listing, so a scenario added to hil_check.sh's
    SCENARIOS list but never added to CHECKERS produces a directory full of recordings that
    nothing reads, no rows at all, and a report that looks complete. That is not hypothetical:
    H12 (jazzy.md section 6 step 8) was added to the shell, given a controller YAML, given its
    constants in this file and left out of CHECKERS, and every motorless check stayed green --
    including the self-test's own "every scenario reaches the report", which asks the question
    the wrong way round, of CHECKERS rather than of the bench.

    A scenario nothing checks is worse than a missing scenario: it drives the motors, costs the
    run its minutes, and buys nothing, while the summary line counts only the rows that exist.
    So the row is a FAIL and it names the directory.
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


def run(run_dir, allowed, seconds, port_free):
    """Check every scenario in the run directory, write the report, return the exit code."""
    report = Report(allowed)
    aborted = {}
    for line in _text(os.path.join(run_dir, 'aborted.txt')).splitlines():
        key, _, detail = line.partition('\t')
        if key.strip():
            aborted[key.strip()] = detail.strip()
    context = {}
    for name, checker in CHECKERS:
        scenario = Scenario(run_dir, name)
        report.prefix = ''
        if name in aborted:
            report.row(name, 'ABORTED', aborted[name])
        elif not scenario.present():
            report.row(name, 'SKIP', 'scenario was not run')
        else:
            report.prefix = name
            checker(report, scenario, context)
    report.prefix = ''
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

    # Defect 2: a frozen run and its catch-up step (PHASE2_SPEC 8.9, gated as H10.freeze).
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

    # The rows of 12.5 average the reported velocity, and a median is not an average: the
    # reported velocity is quantised at VQ, so a median snaps to a lattice value up to VQ/2
    # away -- nearly twice the +-0.020 rad/s those rows allow.
    lattice = [2.0 - 0.5 * VQ] * 5 + [2.0 + 0.5 * VQ] * 4
    expect(abs(mean(lattice) - 2.0) < 0.020 <= abs(median(lattice) - 2.0),
           'a quantised velocity needs its mean, not its median')
    window = [lattice[k % len(lattice)] for k in range(200)]
    fake = {'t_cmd': 0.0, 't_stop': 10.0,
            'joint_states': [[0.0, 0.01 * k, ['w'], [0.0], [window[k]], [0.0]]
                             for k in range(200)]}
    got, n = mean_vel(fake, 'w', 0.0, 0.0)
    expect(n == 200 and abs(got - mean(window)) < 1e-12, 'mean_vel is the arithmetic mean')

    # The H7 rows are built from INVERTED_FLIPS, so a wrong constant changes a gate (A9).
    expect(flips('position') and flips('velocity') and flips('load') and
           not flips('effort') and not flips('current') and not flips('torque'),
           'INVERTED_FLIPS drives the H7 rows')

    # The soak parser of PHASE3 5.14/15, clean and defective, the way every other gate here is
    # proved: a gate that cannot fire is not a gate.
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
    # The bracketed tail of L5 is required, not decorative (PHASE3 5.14, 5.30 row 23), so the
    # parser behind H11.per_servo is proved on a line that has one and a line that does not.
    expect(bus_totals_tail(two) == [(1, 0)], 'bus_totals_tail reads the last line per-servo tail')
    expect(bus_totals_tail(clean.replace(' [id1 0]', '')) == [],
           'a totals line that dropped its per-servo tail is caught')

    _self_test_limits_yaml(expect)
    _self_test_checkers(expect)

    print('hil_gates --self-test: %d check(s) failed' % len(failures) if failures
          else 'hil_gates --self-test: all checks passed')
    return 1 if failures else 0


def _self_test_limits_yaml(expect):
    """
    Prove test/hil/controllers/bench_limits.yaml is bench.yaml with exactly one line changed.

    The check bench_limits.yaml's own header promises, and which until Phase 5's review did not
    exist anywhere -- the header said "--self-test reads both files and fails unless their
    non-comment lines differ in exactly one place ... (_self_test_limits_yaml)" while --self-test
    never opened either file. The invariant held; the claim that it was checked did not.

    Why it is worth checking rather than trusting. H12 is the only scenario that runs against
    bench_limits.yaml, and the whole of its attribution rests on that file being the SAME bench as
    the other fourteen scenarios with enforce_command_limits flipped on. A controller setting added
    to bench.yaml and forgotten here (or the reverse) would silently give H12 a different stack,
    and no row in the report could see it: h12's clamp rows would keep passing, because a limiter
    clamping at 0.8 rad is exactly as true on a stack whose update_rate or arm controller type has
    drifted. The two files cannot be collapsed into one -- enforce_command_limits is read at
    hardware-component init, so it cannot be a runtime parameter override -- so the duplication
    stays and this is what holds it honest.

    Line numbers are carried through so a failure names the offending line in both files rather
    than only its text. Comments and blank lines are dropped on both sides, which is what lets the
    two files keep their own (very different) headers.
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
    # zip() stops at the shorter file, so a line one file has and the other does not would
    # otherwise slip past the comparison and be caught only by the count check above -- and only
    # when it happens to be appended at the end. The surplus is named here so "they differ in
    # exactly one place" means it whichever file is longer.
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
    Drive h1..h12 and the whole evaluator over a synthetic run tree (hil_fixture.py).

    The primitives above cannot see a defect that stops a checker from running at all -- a local
    rebinding a module-level helper, a missing key, a new file label that is never written. That
    is the class of bug that once let `colcon test` report GREEN while the evaluator could not
    process a single real run and wrote no report. These checks assert only that every checker
    runs and emits rows, and that run() writes hil_check.json with a row for every scenario; the
    verdicts are the bench's business, not the self-test's.
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
    Prove H11.wheels_turning fires, by re-running h11 over the fixture with the wheels stopped.

    The rest of _self_test_checkers deliberately ignores verdicts, but this one row exists only to
    catch a condition every other H11 row is blind to (PHASE3 5.13's turning wheels), so "the
    checker ran" is no evidence at all: the row has to be shown passing on the moving fixture and
    failing on a copy of it with joint3/joint4 held still. Copied into a sibling directory rather
    than mutated in place -- run() drives the fixture again afterwards. The leading underscore
    on the copy is what keeps it inert now that unchecked_rows() DOES read the directory
    listing: without it the copy would look like a scenario no checker claims, which is exactly
    the thing that row exists to fail on.
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
    Prove H1.activate fires for every id in EXAMPLE_IDS, one silent servo at a time.

    H1 is the only scenario that runs the shipped example.launch.py, and H1.activate is the only
    row there that can see a servo that did not answer on_configure's ping. That made it exactly
    the kind of row the rest of _self_test_checkers is blind to: it ignores verdicts, so between
    Phase 4 and here the id list in h1() read (1, 2, 3) against a four-joint example, a dead id 4
    scored `unable_to_ping=0`, and the whole self-test still printed "all checks passed". A row
    nothing can fail is not a gate, so the claim is made per id and asserted rather than assumed.

    Each id gets its own copy of the H1 directory with one warning line appended to log.txt, the
    same shape src/waveshare_servos.cpp:996 writes. Copies, not a mutation in place: run() drives
    the fixture again afterwards, and the leading underscore keeps the copies out of
    unchecked_rows()'s directory scan (the reasoning of _self_test_soak_load, which does the
    same).
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
    Prove H12's clamp rows fire, by re-running h12 over copies where the clamp did not happen.

    H12's two clamp rows are the only rows in the whole report that can see the controller
    manager's JointSaturationLimiter fail to clamp, which is the one thing jazzy.md section 6
    step 8 asks the bench to check. _self_test_checkers ignores verdicts, so "h12 ran and emitted
    rows" is no evidence about them at all -- the same blindness that let h1()'s id list read
    (1, 2, 3) against a four-joint example for a whole phase (see _self_test_h1_ping). So the
    baseline is asserted green first, and then each row is shown red on a copy of the fixture
    carrying exactly one defect: the command arrived unclamped.

    The defect is a single column of a single recording, which is what the failure would really
    look like: an arm that went all the way to the commanded 1.2 rad, and wheels that turned at
    the commanded 8.0 rad/s. The velocity injection rewrites the velocity column only and leaves
    the position column consistent with 2.0 rad/s, which would be a contradiction on the bench --
    it is not one here because h12 deliberately emits none of the consistency gates over these
    recordings (see h12's docstring on what is not covered), so nothing reads the wheels' position
    column and the defect stays confined to the rows under test.

    Copies in sibling directories, not mutations in place: run() drives the fixture again
    afterwards. Their names lead with an underscore so unchecked_rows() does not see them as
    scenarios nothing claims.
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
    # (label, the recording, the column index in a joint_states sample, the unclamped command,
    # the rows that and only that must go red). Sample layout is [rx, stamp, names, positions,
    # velocities, efforts], as hil_fixture.rec() builds it.
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


def main(argv):
    """Run the self-test, or check one run directory and write its report."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--run-dir')
    parser.add_argument('--allow-inconclusive', default='',
                        help='the frozen key list, passed in by hil_check.sh')
    parser.add_argument('--seconds', default='0s')
    parser.add_argument('--port-free', default='false')
    args = parser.parse_args(argv)
    if args.self_test:
        return _self_test()
    if not args.run_dir:
        parser.error('one of --self-test and --run-dir is required')
    return run(args.run_dir, args.allow_inconclusive.split(), args.seconds,
               args.port_free == 'true')


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
