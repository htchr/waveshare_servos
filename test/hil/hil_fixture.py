"""
A synthetic run tree for the hil_gates self-test (PHASE2_SPEC 12.3, 12.5).

`hil_gates.py --self-test` used to exercise only the gate primitives of 12.2 against synthetic
series. That left the per-scenario checkers h1..h17 -- the twenty entries of hil_gates.py's
CHECKERS, where the report of 12.4 is actually built -- entirely unexecuted, so a checker that
could not run at all still reported GREEN. This module writes a run directory with the same file
shapes hil_check.sh produces, thin enough to build in memory and complete enough that every
checker walks its whole body.

The data is mostly NOT tuned to make every row PASS. The self-test's claim is only that the
checkers run and emit rows; the verdicts themselves are what a real bench run decides. The
exception is a scenario whose rows are proved by DIFFERENTIAL injection -- H11's wheels_turning,
H1's activate, H12's clamp rows, and every row of the tool scenarios H13-H17 -- where the copy
carrying the defect has to be compared against a green baseline or the injection proves nothing;
see the H12 and H13-H17 entries below and hil_gates.py's _self_test_soak_load, _self_test_h1_ping,
_self_test_h12_clamps and _self_test_tools.
"""

import json
import os

# Kept in step with hil_gates.py, which imports this module rather than the other way round.
ENCODER_STEPS = 4096
STEPS_PER_RAD = ENCODER_STEPS / (2.0 * 3.141592653589793)
TICK = 2.0 * 3.141592653589793 / ENCODER_STEPS
VQ = 50 * TICK
CURRENT_PER_COUNT_A = 0.006
TORQUE_CONSTANT_NM_PER_A = 0.8825985
NM_PER_KGFCM = 0.0980665
OFFSET = 1.570796
IFACES = ('position', 'velocity', 'effort', 'current', 'voltage', 'temperature', 'load',
          'status', 'torque')
REAL = ('joint1', 'joint2', 'joint3', 'joint4')
BLOCK_KEYS = ('n', 'mode', 'pos_last_raw', 'pos_slope_ls_rad_s')

# A post-Phase-3-plausible bench, in microseconds (PHASE3 5.16 item 1). The pre-Phase-3 numbers
# here were 1500.0 [900.0 - 2800.0] read and 700.0 [400.0 - 1200.0] write, and 0.70 ms of write is
# now ABOVE WRITE_MS_AVG_MAX. This fixture is shared by h1, h9 and h11, and h1/h9 gate maxima too,
# so all four figures clear all four constants: 1.70 < 2.15, 2.60 < 4.00, 0.30 < 0.60, 0.90 < 1.50.
# The self-test's claim is never that these verdicts are right (see the docstring) -- but a fixture
# that reads like a broken bench misleads the next reader.
DIAGNOSTICS = (
    'name: waveshare_servos\n'
    '  read_cycle.execution_time\n'
    '   value: Avg: 1700.0 [1200.0 - 2600.0]\n'
    '  write_cycle.execution_time\n'
    '   value: Avg: 300.0 [150.0 - 900.0]\n')


def times(samples, hz=100.0):
    """Return a 100 Hz stamp column, the rate every bench scenario records at."""
    return [k / hz for k in range(samples)]


def turn(t, speed, start=0.0, effort=0.05):
    """Return a tick-quantised constant-speed column as (position, velocity, effort)."""
    quantised = round(speed / VQ) * VQ
    return ([round((start + quantised * x) / TICK) * TICK for x in t],
            [quantised] * len(t), [effort] * len(t))


def hold(t, position, effort=0.01):
    """Return a parked column as (position, velocity, effort)."""
    return ([position] * len(t), [0.0] * len(t), [effort] * len(t))


def join(*columns):
    """Concatenate columns sample-wise, so one joint can spin and then stop."""
    return tuple([x for column in columns for x in column[s]] for s in (0, 1, 2))


def rec(cols, t, **extra):
    """One recording. `cols` maps a joint name to its (position, velocity, effort) columns."""
    names = list(cols)
    out = {'joint_states': [[x + 1e6, x, names] +
                            [[cols[j][s][k] for j in names] for s in (0, 1, 2)]
                            for k, x in enumerate(t)]}
    out.update(extra)
    return out


def djs(t, cols):
    """`cols` maps a joint name to an ordered interface-name -> per-sample-values mapping."""
    names = list(cols)
    return [{'rx': x + 1e6, 'stamp': x, 'joint_names': names,
             'interfaces': [{'names': list(cols[j]),
                             'values': [cols[j][i][k] for i in cols[j]]} for j in names]}
            for k, x in enumerate(t)]


def nine(t, current=0.05, position=0.0, velocity=0.0):
    """Return the nine state interfaces of 12.5 H9, related as the driver relates them."""
    n = len(t)
    amps = [current] * n
    effort = [a * TORQUE_CONSTANT_NM_PER_A for a in amps]
    return {'position': [position] * n, 'velocity': [velocity] * n, 'effort': effort,
            'current': amps, 'voltage': [12.2] * n, 'temperature': [32.0] * n,
            'load': [0.10] * n, 'status': [0.0] * n,
            'torque': [e / NM_PER_KGFCM for e in effort]}


def ids(per_servo):
    """
    Return the readback shape hil_check.sh writes, one block per servo id.

    Every key in BLOCK_KEYS stays at the top of the block; the rest are nested under `registers`,
    which is where the readback helper puts what it read off the servo.
    """
    out = {}
    for servo, fields in per_servo.items():
        block = {'n': 5, 'mode': 1 if servo in (3, 4) else 0, 'pos_last_raw': 1024}
        registers = {}
        for key, value in fields.items():
            (block if key in BLOCK_KEYS else registers)[key] = value
        block['registers'] = registers
        out[str(servo)] = block
    return {'ids': out}


def readback(servos=(1, 2, 3, 4), **common):
    """Return a readback in which every servo carries the same registers."""
    return ids({servo: dict(common) for servo in servos})


#
# Phase 6 (PHASE6_SPEC E.2-E.8): the tools' scenarios H13-H17 and the bench's EEPROM snapshots.
#
# The four servos below are the golden baseline hil_eeprom took before any Phase 6 tool touched
# the bench (phase6_evidence/bench_eeprom_baseline.snap, E.0), so every snapshot and every scan
# table in this fixture reads like that bench and not like an invented one: firmware 3 20, model
# bytes 10 25 (the word 6410), modes 0 0 1 1, offsets raw 2044 / 2245 / 0 / 0 (+2044 and -197 on
# bit 11), torque and lock 1 everywhere. Register 2 is undefined and 5 is the id; both are filled
# in per servo.
BENCH_EEPROM = (3, 20, None, 10, 25, None, 0, 0, 1, 0, 0, 255, 15, 70, 140, 50, 232, 3, 45, 44, 44,
                32, 32, 0, 0, 0, 1, 1, 230, 0, 1, None, None, None, 20, 200, 80, 25, 250, 50)
BENCH = {1: {'offset_raw': 2044, 'mode': 0, 'position': 1413, 'volts_raw': 124, 'temp': 35},
         2: {'offset_raw': 2245, 'mode': 0, 'position': 1026, 'volts_raw': 123, 'temp': 35},
         3: {'offset_raw': 0, 'mode': 1, 'position': 340, 'volts_raw': 122, 'temp': 33},
         4: {'offset_raw': 0, 'mode': 1, 'position': 3594, 'volts_raw': 122, 'temp': 33}}
PORT = '/dev/ttyACM0'
CM_PID = '31337'          # the controller manager's pid while H14's stack holds the port
PROBE_PID = '4242'        # H14's flock-only holder, as its HOLDING line announces it
SCAN_HEADER = (' id  type  mode  model  baud_reg     baud  position  voltage_V  temp_C  status'
               '  offset')
USAGE = {'scan': 'usage: ros2 run waveshare_servos scan [--ros-args -p port:=/dev/ttyACM0 -p '
                 'baudrate:=1000000]',
         'set_id': 'usage: ros2 run waveshare_servos set_id --ros-args -p start_id:=<0..253> -p '
                   'new_id:=<1..253> [-p port:=...] [-p baudrate:=...]',
         'calibrate_midpoint': 'usage: ros2 run waveshare_servos calibrate_midpoint --ros-args -p '
                               'id:=<0..253> [-p port:=...] [-p baudrate:=...]'}


def bench_servo(bench_id, id_register=None, offset_raw=None, mode=None, torque=1, lock=1,
                goal=None, position=None):
    """
    Return one servo of a hil_eeprom snapshot, in the RESULT JSON shape (E.1).

    `bench_id` picks which of the four physical servos this is; `id_register` is what its register
    5 says, which differs only while H15 has moved servo 4 to 253.
    """
    bench = BENCH[bench_id]
    raw = bench['offset_raw'] if offset_raw is None else offset_raw
    values = list(BENCH_EEPROM)
    values[5] = bench_id if id_register is None else id_register
    values[31], values[32] = raw & 0xff, raw >> 8
    values[33] = bench['mode'] if mode is None else mode
    at = bench['position'] if position is None else position
    return {'n': 39, 'eeprom': {str(reg): value for reg, value in enumerate(values) if reg != 2},
            'sram': {'40': torque, '55': lock},
            # a wheel's goal position reads 0 on the bench (E.0); an arm's is where it holds
            'volatile': {'42': (at if values[33] == 0 else 0) if goal is None else goal, '56': at,
                         '62': bench['volts_raw'], '63': bench['temp'], '65': 0}}


def snapshot(servos, census=(1, 2, 3, 4)):
    """Return a whole snapshot: `servos` maps an id to bench_servo(); a None census: not taken."""
    return {'ok': True, 'census': None if census is None else list(census),
            'ids': {str(servo_id): block for servo_id, block in sorted(servos.items())}}


def bench_snapshot(census=(1, 2, 3, 4)):
    """Return the golden baseline itself."""
    return snapshot({i: bench_servo(i) for i in (1, 2, 3, 4)}, census)


def snap_text(snap):
    """Render a snapshot as the .snap file hil_eeprom writes with --out (E.1, DEVIATIONS D-5)."""
    ids = sorted(int(i) for i in snap['ids'])
    lines = ['hil_eeprom snapshot 1', 'ids ' + ' '.join(str(i) for i in ids),
             'ok ' + ('true' if snap['ok'] else 'false')]
    if snap.get('census') is not None:
        lines.append(' '.join(['census'] + [str(i) for i in snap['census']]))
    for servo_id in ids:
        block = snap['ids'][str(servo_id)]
        values = ['-' if reg == 2 else str(block['eeprom'].get(str(reg), 'x'))
                  for reg in range(40)]
        lines.append('servo %d eeprom %s' % (servo_id, ' '.join(values)))
        lines.append('servo %d sram 40=%s 55=%s' % (servo_id, block['sram']['40'],
                                                    block['sram']['55']))
        lines.append('servo %d volatile %s' % (servo_id, ' '.join(
            '%s=%s' % (reg, block['volatile'][reg]) for reg in ('42', '56', '62', '63', '65'))))
    return '\n'.join(lines) + '\n'


def snapshot_files(label, snap):
    """Return the two files `eeprom <label> snapshot ... --out <label>.snap` leaves behind."""
    return [(label + '.json', snap), (label + '.snap', snap_text(snap))]


def scan_row(servo_id, block):
    """One row of scan's table (C.1), printed from a snapshot block the way servo_tools does."""
    eeprom, volatile = block['eeprom'], block['volatile']
    mode = eeprom['33']
    raw = eeprom['31'] | eeprom['32'] << 8
    return '%3d  %-4s%6s%7s%10s%9s%10s%11s%8s%8s%8s' % (
        servo_id, {0: 'pos', 1: 'vel'}.get(mode, '-'), mode, eeprom['3'] | eeprom['4'] << 8,
        eeprom['6'], 1000000, volatile['56'], '%.1f' % (volatile['62'] * 0.1), volatile['63'],
        '0x%02x' % volatile['65'], -(raw & ~0x800) if raw & 0x800 else raw)


def scan_out(snap, seconds='3.9'):
    """Return the whole stdout of scan for a snapshot's servos: header, a row each, footer."""
    ids = sorted(int(i) for i in snap['ids'])
    return '\n'.join([SCAN_HEADER] + [scan_row(i, snap['ids'][str(i)]) for i in ids] + [
        'found %d servo(s) on %s at 1000000 baud: ids %s (pinged ids 0..253, 3 attempts each, '
        '%s s)' % (len(ids), PORT, ' '.join(str(i) for i in ids), seconds)]) + '\n'


SCAN_ERR = ('serial speed 1000000\nscan: use the id column as <param name="id"> and the type '
            'column as <param name="type">; see '
            'description/ros2_control/example.ros2_control.xacro\n')


def ran(label, rc, serial_speed_lines, holders='', seconds='0.412'):
    """Return the four facts hil_check.sh's tool() writes for every run, on every path."""
    return {'%s_rc' % label: str(rc), '%s_seconds' % label: seconds,
            '%s_serial_speed_lines' % label: str(serial_speed_lines),
            '%s_holders_after' % label: holders}


def refused(tool, message):
    """Return a usage refusal's stderr (A.1 step 6): the prefixed message, the bare usage."""
    return '%s: %s\n%s\n' % (tool, message, USAGE[tool])


def held(tool, pid, comm):
    """Return the C.0 port-held refusal as open_bus prints it through the prefixing stream."""
    return ("%s: port '%s' is held by another process (pid %s %s); refusing to share the bus -- "
            'stop that process first (a running controller manager holds the port for as long as '
            'its hardware component is configured). Nothing was sent to the servos.\n'
            % (tool, PORT, pid, comm))


def set_id_detail(start, new, verdict, writes=0, **fields):
    """set_id's detail line (C.2), `none` for what the run never measured (DEVIATIONS D-25)."""
    order = ('lock_before', 'unlock_read', 'id_write_ack', 'ack_ms', 'verify_ms', 'new_id_pings',
             'late_ack_from', 'late_ack_ms', 'old_id_silent', 'identity_same', 'lock_after')
    line = {key: 'none' for key in order}
    line['id_write_ack'] = 'not_sent'
    line.update(fields)
    return 'set_id: detail start_id=%d new_id=%d %s writes_sent=%d verdict=%s\n' % (
        start, new, ' '.join('%s=%s' % (key, line[key]) for key in order), writes, verdict)


def merged(*parts, **more):
    """Return one facts dict out of several, later keys winning."""
    out = {}
    for part in parts + (more,):
        out.update(part)
    return out


def gate_facts():
    """Return the facts guard_begin and guard_end leave behind on a clean run (E.2)."""
    return {'journal_written': 'true', 'journal_left': 'false', 'pre_eeprom_rc': '0',
            'post_eeprom_rc': '0', 'guard_compare_rc': '0'}


def build_tools(root):
    """
    Write H13-H17, a HEALTHY bench on which every non-NOTE row of h13..h17 passes.

    The departure from the module docstring's "not tuned to make every row PASS" is the H12 one:
    hil_gates.py's _self_test_tools proves every one of these rows by differential injection (E.8)
    -- a copy of one scenario directory carrying exactly one defect must turn exactly the listed
    rows red -- and that argument needs a baseline where each row is green.
    """
    bench = bench_snapshot()
    guard = snapshot_files('pre_eeprom', bench) + snapshot_files('post_eeprom', bench)

    # H13: scan, read-only. Four scans: explicit port, default port, stale name, positional.
    write(root, 'H13',
          facts=merged(gate_facts(), ran('scan', 0, 1, seconds='4.312'),
                       ran('scan_default', 0, 1, seconds='4.297'), ran('stale_scan', 64, 0),
                       ran('positional_scan', 64, 0)),
          files=guard + [
              ('scan.out.txt', scan_out(bench)), ('scan.err.txt', SCAN_ERR),
              ('scan_default.out.txt', scan_out(bench)), ('scan_default.err.txt', SCAN_ERR),
              ('stale_scan.out.txt', ''),
              ('stale_scan.err.txt', refused(
                  'scan', "parameter 'device_port' was renamed to 'port', the name the hardware "
                  'interface uses; refusing to run rather than ignore it and open the default '
                  "port '/dev/ttyACM0'")),
              ('positional_scan.out.txt', ''),
              ('positional_scan.err.txt', refused(
                  'scan', "unexpected argument '%s'; parameters go after --ros-args, for example: "
                  'ros2 run waveshare_servos scan --ros-args -p port:=%s' % (PORT, PORT))),
              ('guard_compare.json', {'equal': True, 'eeprom_only': False, 'diffs': []}),
              ('released.json', {'verdict': 'acquired'})])

    # H14: every tool refuses a held port, and the refusals write nothing. Stage A runs while the
    # controller manager (CM_PID) holds the port, stage B against a flock-only port_probe
    # (PROBE_PID), stage C with the port free and every id silent but 3.
    facts = merged(gate_facts(), controllers_active='true', cm_pids=CM_PID,
                   driver_warns_before='3', driver_warns_after='3', probe_pid=PROBE_PID)
    files = list(guard)
    for short, tool in (('scan', 'scan'), ('set_id', 'set_id'),
                        ('calibrate', 'calibrate_midpoint')):
        facts.update(ran('cm_' + short, 1, 0, holders=CM_PID))
        facts.update(ran('fl_' + short, 1, 0, holders=PROBE_PID))
        facts['fl_%s_holder_alive' % short] = 'true'
        files += [('cm_%s.out.txt' % short, ''),
                  ('cm_%s.err.txt' % short, held(tool, CM_PID, 'ros2_control_no')),
                  ('fl_%s.out.txt' % short, ''),
                  ('fl_%s.err.txt' % short, held(tool, PROBE_PID, 'hil_port_probe'))]
    usage = (
        ('stale', 'set_id', "parameter 'device_port' was renamed to 'port', the name the hardware "
         'interface uses; refusing to run rather than ignore it and open the default port '
         "'/dev/ttyACM0'"),
        ('bad_type', 'set_id', "parameter 'new_id' is 201.0 (a double), which is not an integer"),
        ('range', 'set_id', "parameter 'new_id' is 300, out of range; expected an integer between "
         '1 and 253 (the hardware interface accepts ids 1..253)'),
        ('cal_range', 'calibrate_midpoint', "parameter 'id' is 300, out of range; expected an "
         'integer between 0 and 253'),
        ('missing', 'set_id', "parameter 'new_id' is missing; expected an integer between 1 and "
         '253'),
        ('positional', 'set_id', "unexpected argument '%s'; parameters go after --ros-args, for "
         'example: ros2 run waveshare_servos set_id --ros-args -p port:=%s' % (PORT, PORT)),
        ('foreign_node', 'set_id', "parameter override(s) 'port' are addressed to node '/setid', "
         "but this tool's node is '/set_id'; rclcpp would drop them silently and use the "
         'defaults, so refusing to run. Give parameters without a node prefix: -p port:=...'))
    for label, tool, message in usage:
        facts.update(ran(label, 64, 0))
        files += [('%s.out.txt' % label, ''), ('%s.err.txt' % label, refused(tool, message))]
    facts.update(merged(ran('taken', 4, 1), ran('silent_start', 3, 1), ran('cal_silent', 3, 1)))
    files += [
        ('taken.out.txt', ''),
        ('taken.err.txt', "serial speed 1000000\nset_id: id 3 already answers on '%s'; refusing "
         'to give servo 200 an id that is taken -- two servos on one id answer on top of each '
         'other and cannot be told apart. Pick a free id; scan lists the taken ones. Nothing was '
         'written.\n' % PORT + set_id_detail(200, 3, 'refused')),
        ('silent_start.out.txt', ''),
        ('silent_start.err.txt', 'serial speed 1000000\nset_id: no servo answers at id 200 on '
         "'%s' at 1000000 baud (3 pings); nothing was written. scan lists the ids that do "
         'answer.\n' % PORT + set_id_detail(200, 201, 'no_answer')),
        ('cal_silent.out.txt', ''),
        ('cal_silent.err.txt', 'serial speed 1000000\ncalibrate_midpoint: no servo answers at id '
         "200 on '%s' at 1000000 baud (3 pings); nothing was written. scan lists the ids that do "
         'answer.\ncalibrate_midpoint: detail id=200 mode=none torque_before=none '
         'torque_written=false settle_ms=none position_before=none offset_raw_before=none '
         'unlock_read=none calibrate_ack=not_sent ack_ms=none late_ack_from=none late_ack_ms=none '
         'position_after=none offset_raw_after=none offset_sign=0 register40_after=none '
         'torque_final=none identity_same=none lock_after=none writes_sent=0 verdict=no_answer\n'
         % PORT),
        ('tools_window.json', rec({'joint1': hold(times(1200), 0.6),
                                   'joint2': hold(times(1200), -0.6)}, times(1200))),
        ('flock_holder.txt', 'HOLDING {"pid": %s, "port": "%s", "hold_s": 25, '
         '"no_exclusive": true}\n' % (PROBE_PID, PORT)),
        ('guard_compare.json', {'equal': True, 'eeprom_only': True, 'diffs': []}),
        ('released.json', {'verdict': 'acquired'})]
    write(root, 'H14', facts=facts, files=files)

    # H15: set_id 4 -> 253 -> 4. The moved snapshot is servo 4 answering at 253 with nothing but
    # register 5 changed; the tool's own way back leaves the bench as it was, so the restore after
    # it writes nothing at all (DEVIATIONS D-8).
    moved = snapshot({1: bench_servo(1), 2: bench_servo(2), 3: bench_servo(3),
                      253: bench_servo(4, id_register=253)}, census=(1, 2, 3, 253))
    back_detail = set_id_detail(
        253, 4, 'ok', writes=3, lock_before='1', unlock_read='0', id_write_ack='old_id',
        ack_ms='4', verify_ms='29', new_id_pings='2', old_id_silent='true',
        identity_same='true', lock_after='1')
    move_detail = back_detail.replace('start_id=253 new_id=4', 'start_id=4 new_id=253')
    write(root, 'H15',
          facts=merged(gate_facts(), ran('move', 0, 1), ran('moved_scan', 0, 1, seconds='4.301'),
                       ran('back', 0, 1), moved_eeprom_rc='0', back_eeprom_rc='0',
                       restore_rc='0'),
          files=guard + snapshot_files('moved_eeprom', moved) +
          snapshot_files('back_eeprom', bench) + [
              ('move.out.txt', 'servo 4 is now id 253: it answers at 253 and no longer at 4, its '
               'registers are otherwise unchanged, and its EEPROM lock is closed.\n'),
              ('move.err.txt', 'serial speed 1000000\nset_id: about to give servo 4 the id 253. '
               'If this run is interrupted or fails, run scan: the servo will answer at 4 or '
               '253.\n' + move_detail),
              ('moved_scan.out.txt', scan_out(moved)), ('moved_scan.err.txt', SCAN_ERR),
              ('back.out.txt', 'servo 253 is now id 4: it answers at 4 and no longer at 253, its '
               'registers are otherwise unchanged, and its EEPROM lock is closed.\n'),
              ('back.err.txt', 'serial speed 1000000\nset_id: about to give servo 253 the id 4. '
               'If this run is interrupted or fails, run scan: the servo will answer at 253 or '
               '4.\n' + back_detail),
              ('restore.json', {'ok': True, 'exit': 0, 'moved': None, 'writes': [], 'before': [],
                                'after': [], 'problems': [], 'signal_deferred': False}),
              ('guard_compare.json', {'equal': True, 'eeprom_only': False, 'diffs': []})])

    # H16: calibrate_midpoint on id 2, then the refusal on the wheel id 3. Id 2 rests at 1026 and
    # its offset reads raw 2245 (-197 on bit 11); after the tool it reads 2048 at the same shaft
    # angle, so its offset moved by 2048 - 1025 = 1023 ticks (the tool's own settled, torque-off
    # position_before is 1025) to raw 0x0cc4 = -1220. Torque stays OFF, the lock closed [Q2, Q3].
    calibrated = snapshot({1: bench_servo(1), 3: bench_servo(3), 4: bench_servo(4),
                           2: bench_servo(2, offset_raw=0x0cc4, torque=0, goal=1026,
                                          position=2048)})
    wheel = dict(calibrated, census=None)
    write(root, 'H16',
          facts=merged(gate_facts(), ran('cal', 0, 1), ran('wheel', 4, 1), cal_pos_rc='0',
                       cal_eeprom_rc='0', wheel_eeprom_rc='0', restore_rc='0'),
          files=guard + snapshot_files('cal_eeprom', calibrated) +
          snapshot_files('wheel_eeprom', wheel) + [
              ('cal.out.txt', 'servo 2 now reads 2048 at the position that read 1025; offset '
               'registers 31-32 went from -197 to -1220 (raw 0x08c5 -> 0x0cc4). Its torque is '
               'OFF; the hardware interface turns it on at activate. EEPROM lock closed. '
               'Power-cycle the servo and run scan to confirm the offset survived.\n'),
              ('cal.err.txt', 'serial speed 1000000\ncalibrate_midpoint: about to calibrate the '
               'midpoint of servo 2 (an EEPROM write of its offset, registers 31-32). If this run '
               'is interrupted or fails, run scan: the servo will answer at 2.\n'
               'calibrate_midpoint: detail id=2 mode=0 torque_before=1 torque_written=true '
               'settle_ms=120 position_before=1025 offset_raw_before=0x08c5 unlock_read=0 '
               'calibrate_ack=old_id ack_ms=5 late_ack_from=none late_ack_ms=none '
               'position_after=2048 offset_raw_after=0x0cc4 offset_sign=-1 register40_after=0 '
               'torque_final=0 identity_same=true lock_after=1 writes_sent=4 verdict=ok\n'),
              ('cal_pos.json', {'ok': True, 'id': 2, 'addr': 56, 'word': True, 'value': 2048}),
              ('wheel.out.txt', ''),
              ('wheel.err.txt', 'serial speed 1000000\ncalibrate_midpoint: servo 3 is in mode 1 '
               '(wheel); a midpoint only means something to a position servo (mode 0). Changing '
               'the mode is an EEPROM write this tool does not make: declare the joint type '
               "'pos' and let the hardware interface switch it at configure, then calibrate. "
               'Nothing was written.\ncalibrate_midpoint: detail id=3 mode=1 torque_before=1 '
               'torque_written=false settle_ms=none position_before=none offset_raw_before=0x0000 '
               'unlock_read=none calibrate_ack=not_sent ack_ms=none late_ack_from=none '
               'late_ack_ms=none position_after=none offset_raw_after=none offset_sign=0 '
               'register40_after=none torque_final=none identity_same=none lock_after=none '
               'writes_sent=0 verdict=refused\n'),
              ('restore.json', {'ok': True, 'exit': 0, 'moved': None, 'writes': [
                  {'id': 2, 'reg': 31, 'from': 0x0cc4, 'to': 2245, 'bytes': 2, 'kind': 'eeprom'},
                  {'id': 2, 'reg': 42, 'from': 1026, 'to': 1026, 'bytes': 2, 'kind': 'sram'},
                  {'id': 2, 'reg': 40, 'from': 0, 'to': 1, 'bytes': 1, 'kind': 'sram'}],
                  'before': [], 'after': [], 'problems': [], 'signal_deferred': False}),
              ('guard_compare.json', {'equal': True, 'eeprom_only': False, 'diffs': []})])

    # H17: the bench as found. initial_eeprom.snap is the pre-flight's snapshot and baseline.snap
    # the golden one of E.0, both copied in by h_H17; final_eeprom is taken there and then.
    write(root, 'H17',
          facts={'final_eeprom_rc': '0', 'journal_present': 'false',
                 'baseline_source': '/home/ubuntu/waveshare_ws/phase6_evidence/'
                                    'bench_eeprom_baseline.snap'},
          files=snapshot_files('final_eeprom', bench) + [
              ('initial_eeprom.snap', snap_text(bench)), ('baseline.snap', snap_text(bench))])


def write(root, name, facts=(), log='', files=()):
    """Write one scenario directory."""
    path = os.path.join(root, name)
    os.makedirs(path, exist_ok=True)
    base = {'port_free_before': 'true', 'port_free_after': 'true'}
    base.update(dict(facts))
    files = list(files) + [('facts.json', base), ('log.txt', log)]
    for filename, body in files:
        with open(os.path.join(path, filename), 'w') as handle:
            if isinstance(body, str):
                handle.write(body)
            else:
                json.dump(body, handle)
    return path


def build(root):
    """Write the whole run tree and return its path."""
    long_t, mid_t, short_t = times(1200), times(600), times(300)
    arm = {'joint1': hold(long_t, 0.6), 'joint2': hold(long_t, -0.6)}
    wheels = {'joint3': turn(long_t, 2.0), 'joint4': turn(long_t, -2.0)}
    spun = {j: join(turn(mid_t, 2.0), hold(mid_t, turn(mid_t, 2.0)[0][-1]))
            for j in ('joint3', 'joint4')}
    quiet = dict(arm, **wheels)

    write(root, 'H1',
          facts={'controllers_active': 'true', 'hw_state': 'active'},
          log="[INFO] Successful 'activate'\n[INFO] bus on '/dev/ttyACM0' at 1000000 baud\n",
          files=[('steady.json', rec(quiet, long_t)), ('diagnostics.txt', DIAGNOSTICS)])

    # H1B, the shipped example commanded rather than only watched. Its recordings deliberately
    # reuse the shapes of the bench scenarios whose bounds h1b borrows -- the arm columns are
    # H2's and the wheel column is H3's `spun` -- because the point of h1b is that the same
    # measurements are taken on the packaged stack, and a fixture that made them look like
    # different measurements would hide a checker that had drifted off them.
    write(root, 'H1B',
          facts={'controllers_active': 'true', 'hw_state': 'active',
                 'diff_drive_state': 'inactive', 'arm_state_after': 'active',
                 'hw_state_after_arm': 'active', 'vel_attempted': 'true',
                 'launch_exit_code': '0', 'port_free_within_10s': 'true',
                 'port_holders_after_exit': ''},
          log="[INFO] Successful 'deactivate'\n[INFO] Successful 'shutdown'\n",
          files=[('ex_move_to_0.json', rec({'joint1': hold(short_t, 0.0)}, short_t, t_cmd=0.0)),
                 ('ex_move_to_06.json', rec({'joint1': hold(short_t, 0.6)}, short_t, t_cmd=0.0)),
                 ('ex_vel_2.json', rec(spun, long_t, t_cmd=0.0, t_stop=6.0)),
                 ('after_exit.json', {'any_moving': False}),
                 ('probe_after_exit.json', {'verdict': 'acquired'})])

    write(root, 'H2',
          facts={'controllers_active': 'true'},
          files=[('move_to_0.json', rec({'joint1': hold(short_t, 0.0)}, short_t, t_cmd=0.0)),
                 ('move_to_06.json', rec({'joint1': hold(short_t, 0.6)}, short_t, t_cmd=0.0)),
                 ('post_readback.json',
                  ids({1: {'goal_position_raw': round((0.6 + OFFSET) * STEPS_PER_RAD)}}))])

    write(root, 'H3',
          files=[('vel_2.json', rec(spun, long_t, t_cmd=0.0, t_stop=6.0))])

    write(root, 'H4',
          files=[('spin.json', rec(wheels, long_t, t_cmd=0.0, t_stop=12.0)),
                 ('pre_readback.json', readback(pos_last_raw=0)),
                 ('post_readback.json', readback(pos_last_raw=1500))])

    write(root, 'H5A',
          facts={'controllers_active': 'false', 'spawner_rc': '1', 'hw_state': '',
                 'port_holders_running': '', 'cm_exit_code': '0'},
          log=("[FATAL] 1 of 5 servos did not answer: id 9 (joint 'joint5'); refusing to "
               "configure because 'allow_missing_servos' is false\n"),
          files=[('pre_readback.json', readback(torque_enable=1, acc=10,
                                                goal_position_raw=1024, goal_speed_raw=0)),
                 ('post_readback.json', readback(torque_enable=1, acc=10,
                                                 goal_position_raw=1024, goal_speed_raw=0))])

    write(root, 'H5B',
          facts={'controllers_active': 'true', 'spawner_rc': '0'},
          log="[WARN] unable to ping motor id '9'\n",
          files=[('wheels.json', rec(dict(wheels, joint5=hold(long_t, 0.0, effort=0.0)), long_t)),
                 ('diagnostics.txt', DIAGNOSTICS)])

    write(root, 'H5C',
          facts={'controllers_active': 'true', 'hw_state': 'active'},
          files=[('move_to_03.json', rec({'joint1': hold(short_t, 0.3)}, short_t, t_cmd=0.0))])

    holders = {}
    for label in ('flock_holder', 'excl_holder'):
        holders.update({'%s_probe_pid' % label: '4242',
                        '%s_refusal_within_hold' % label: 'true',
                        '%s_controllers_active' % label: 'false',
                        '%s_spawner_rc' % label: '1',
                        '%s_driver_refusals' % label: '1',
                        '%s_port_holders' % label: '4242',
                        '%s_serial_speed_lines' % label: '0'})
    write(root, 'H6',
          facts=dict({'driver_warns_before': '0', 'driver_warns_after': '0'}, **holders),
          files=[('probe_while_active.json',
                  {'verdict': 'refused_by_tiocexcl', 'open_errno': 16, 'flock_errno': 0}),
                 ('probe_window.json', rec(arm, long_t)),
                 ('flock_holder.json', {'verdict': 'acquired', 'no_exclusive': True}),
                 ('excl_holder.json', {'verdict': 'acquired', 'no_exclusive': False}),
                 ('flock_holder_pre.json', readback(torque_enable=1, acc=10)),
                 ('flock_holder_post.json', readback(torque_enable=1, acc=10)),
                 ('excl_holder_pre.json', readback(torque_enable=1, acc=10)),
                 ('excl_holder_post.json', readback(torque_enable=1, acc=10)),
                 ('probe_after_release.json', {'verdict': 'acquired', 'rc': 0})])

    load_cols = {j: nine(mid_t, current=0.05, velocity=2.0) for j in REAL}
    write(root, 'H7',
          files=[('pos_move.json', rec({'joint1': hold(short_t, 0.6),
                                        'joint2': hold(short_t, 0.6)}, short_t)),
                 ('pos_readback.json',
                  ids({1: {'goal_position_raw': round((0.6 + OFFSET) * STEPS_PER_RAD)},
                       2: {'goal_position_raw': round((-0.6 + OFFSET) * STEPS_PER_RAD)}})),
                 ('vel_spin.json', rec({'joint3': turn(mid_t, 2.0), 'joint4': turn(mid_t, 2.0)},
                                       mid_t, t_cmd=0.0, t_stop=6.0)),
                 ('vel_readback.json',
                  ids({3: {'goal_speed_raw': 1304, 'pos_slope_ls_rad_s': 1.99},
                       4: {'goal_speed_raw': -1304, 'pos_slope_ls_rad_s': -1.99}})),
                 ('load_step.json', rec({j: turn(mid_t, 2.0) for j in REAL}, mid_t, t_cmd=0.0,
                                        dynamic_joint_states=djs(mid_t, load_cols)))])

    write(root, 'H8',
          files=[('slow_readback.json', ids({1: {'goal_speed_raw': 652},
                                             3: {'goal_speed_raw': 1304}, 4: {'acc': 10}})),
                 ('fast_readback.json', ids({1: {'goal_speed_raw': 2608},
                                             3: {'goal_speed_raw': 6000}, 4: {'acc': 150}})),
                 ('slow_wheel.json', rec({'joint3': turn(mid_t, 2.0)}, mid_t, t_cmd=0.0,
                                         t_stop=6.0)),
                 ('settle_slow.json', rec({'joint1': hold(short_t, 1.0)}, short_t, t_cmd=0.0)),
                 ('settle_fast.json', rec({'joint1': hold(short_t, 1.0)}, short_t, t_cmd=0.0)),
                 ('t90_slow.json', rec({'joint4': turn(short_t, 3.0)}, short_t, t_cmd=0.0)),
                 ('t90_fast.json', rec({'joint4': turn(short_t, 3.0)}, short_t, t_cmd=0.0))])

    columns = {'joint1': nine(long_t), 'joint2': nine(long_t),
               'joint3': nine(long_t, velocity=2.0), 'joint4': nine(long_t, velocity=-2.0),
               'joint5': nine(long_t, current=0.0)}
    listed = '\n'.join('%s/%s' % (j, i) for j in REAL + ('joint5',) for i in IFACES)
    # t_cycle_done is 8.0 against a 12 s recording (long_t is 1200 samples at 100 Hz), so 400
    # samples follow the cycle and H9.cycle_recorded clears its 100-sample floor with room to
    # spare. It has to be a real margin rather than a value tuned to just pass: the row exists to
    # notice a recorder that stopped near the cycle, and a fixture sitting on the boundary would
    # make the red-first injection for it indistinguishable from fixture noise.
    write(root, 'H9',
          facts={'t_cycled': '2.0', 't_plus0': '1.0', 't_plus1': '3.0',
                 't_minus0': '5.0', 't_minus1': '7.0', 't_cycle_done': '8.0',
                 'cycle_inactive_rc': '0', 'cycle_active_rc': '0',
                 'hw_state_after_cycle': 'active'},
          log=''.join("[WARN] joint '%s' declares the deprecated 'torque' interface\n" % j
                      for j in REAL + ('joint5',)),
          files=[('interfaces.json', rec(quiet, long_t,
                                         dynamic_joint_states=djs(long_t, columns))),
                 ('arm_after_cycle.json', rec({'joint1': hold(short_t, 0.4),
                                               'joint2': hold(short_t, 0.4)}, short_t,
                                              t_cmd=0.0)),
                 ('arm_after_park.json', rec({'joint1': hold(short_t, 0.0),
                                              'joint2': hold(short_t, 0.0)}, short_t,
                                             t_cmd=0.0)),
                 ('hardware_interfaces.txt', listed),
                 ('diagnostics.txt', DIAGNOSTICS)])

    write(root, 'H10',
          facts={'exit_code': '0', 'term_exit_code': '0', 'port_free_within_10s': 'true',
                 'port_holders_after_exit': '', 'term_port_free': 'true',
                 'transitions': '1.0 2.0'},
          log=("[INFO] Successful 'deactivate'\n[INFO] Successful 'shutdown'\n"),
          files=[('after_exit.json', {'any_moving': False}),
                 ('probe_after_exit.json', {'verdict': 'acquired'}),
                 ('term_after_exit.json', {'any_moving': False}),
                 ('shutdown.json', rec(quiet, mid_t))])

    # H11, the soak of PHASE3 5.13/5.16. Written after H10 and not beside H9 because it reuses
    # H9's `columns` local: `nine:=true` means the soak recording carries all nine interfaces, so
    # h11's temperature NOTE has data. `columns` also carries a joint5 the real H11 has no
    # equivalent of -- h11 names its joints explicitly, so it is inert, and matching the shape the
    # rest of this fixture already uses is worth more than trimming it.
    #
    # The point of the entry is the bug this whole module exists to catch: a checker that reads a
    # file label or a fact key nobody writes emits a degenerate row and the self-test still says
    # GREEN. h11 reads `steady_soak.json`, `diagnostics.txt` and the `soak_s` fact, so all three
    # are here; `port_free_before` / `port_free_after` come from write()'s own base facts.
    write(root, 'H11',
          facts={'controllers_active': 'true', 'hw_state': 'active', 'soak_s': '600'},
          log=('[INFO] bus totals: transactions 60000, failed 0 (0.0 per million), '
               'worst consecutive 0, dropped 0 [id1 0, id2 0, id3 0, id4 0]\n'),
          files=[('steady_soak.json', rec(quiet, long_t,
                                          dynamic_joint_states=djs(long_t, columns))),
                 ('diagnostics.txt', DIAGNOSTICS)])

    # H12, the limits scenario of jazzy.md section 6 step 8: three stacks, and the middle one
    # renders +-0.8 rad / 2.0 rad/s <limit>s against bench_limits.yaml's enforce_command_limits,
    # so the controller manager's JointSaturationLimiter clamps an arm command of 1.2 rad and a
    # wheel command of 8.0 rad/s. Written last because it is the newest scenario, not because it
    # runs last on the bench (it runs before H11, so the soak's ten minutes are spent after every
    # fast row has reported).
    #
    # This entry is a HEALTHY H12 -- every h12 row passes over it -- which is a departure from the
    # module docstring's "not tuned to make every row PASS", and deliberate. Two of h12's rows are
    # proved by differential injection in hil_gates.py (_self_test_h12_clamps copies this
    # directory, unclamps one recording in it and asserts that exactly one row turns red), and
    # that argument only works against a baseline where the row in question is green: on a
    # fixture where the row was already red, an injection would prove nothing at all. The same
    # reasoning as _self_test_soak_load's H11 copy and _self_test_h1_ping's H1 copies.
    #
    # The numbers are the plausible ones rather than the exact ones wherever the bench has a
    # measured answer: the arm settles 0.0033 rad short of a commanded target (H2.target.move_to_06
    # read 0.5967 for 0.6 in the post-Phase-4 baseline), so the clamped arm rests at 0.7967 and
    # not at a suspiciously exact 0.8, and turn() quantises 2.0 rad/s to the 26-quantum lattice
    # value 1.99417 the same way a real reported velocity is quantised. Both sit inside their
    # tolerances with room to spare, so a red row here is a checker defect and never fixture noise.
    clamp_t = times(500)                      # 5 s: 0.5 pre + 2.0 move + 2.5 post, as h_H12 asks
    parked = round(OFFSET * STEPS_PER_RAD)    # 1024 ticks; the joint angle is raw * TICK - OFFSET
    arm_regs = {'torque_enable': 1, 'acc': 10, 'goal_position_raw': parked, 'goal_speed_raw': 0}
    # The limiter lines, and the reason this file writes three cm*.stdout files. h_H12 records
    # limited_cm_log so the gate reads the ONE log its limited stack wrote, and the two ordinary
    # stacks that bracket it run against bench.yaml, where enforce_command_limits is false and no
    # limiter is ever built. The order here is joint3, joint1, joint4, joint2 ON PURPOSE:
    # hardware_interface emits these in hash-map order (jazzy.md's Phase 4 amendment to section 6
    # step 8), so a checker that matched them by index would pass on a sorted fixture and fail on
    # the bench. Note that log.txt below does NOT carry them, where the real scenario_end would
    # (it concatenates every *.stdout into log.txt) -- that is what makes an injection into
    # cm2.stdout a single defect, and h12 reads the named file rather than log.txt anyway.
    limiter = ''.join('[INFO] [resource_manager]: Creating JointSaturationLimiter for joint '
                      "'%s' in hardware 'bench'\n" % j for j in ('joint3', 'joint1',
                                                                 'joint4', 'joint2'))
    write(root, 'H12',
          facts={'arm_pos_limit': '0.8', 'arm_command': '1.2',
                 'wheel_vel_limit': '2.0', 'wheel_command': '8.0',
                 'park_spawner_rc': '0', 'park_controllers_active': 'true',
                 'park_cm_exit_code': '0',
                 'limited_spawner_rc': '0', 'limited_controllers_active': 'true',
                 # 137 = 128 + SIGKILL, which is what stop_stack KILL must produce: the limited
                 # stack is killed with 8.0 rad/s still commanded so the goal-speed register
                 # survives. h12 reports it and gates nothing on it (a 0 here would mean the kill
                 # missed), exactly as H8's KILLed stack recorded cm_exit_code 137 on the bench.
                 'limited_cm_exit_code': '137', 'limited_cm_log': 'cm2.stdout',
                 'arm_state_after': 'active',
                 'park_final_spawner_rc': '0', 'park_final_controllers_active': 'true',
                 'park_final_cm_exit_code': '0'},
          log="[INFO] Successful 'activate'\n[INFO] Successful 'deactivate'\n",
          files=[('cm1.stdout', "[INFO] Successful 'activate'\n"),
                 ('cm2.stdout', limiter),
                 ('cm3.stdout', "[INFO] Successful 'activate'\n"),
                 ('arm_pre.json', ids({1: dict(arm_regs, pos_last_raw=parked),
                                       2: dict(arm_regs, pos_last_raw=parked)})),
                 ('pos_clamp.json', rec({'joint1': hold(clamp_t, 0.7967)}, clamp_t, t_cmd=0.0)),
                 ('vel_clamp.json', rec({'joint3': turn(mid_t, 2.0), 'joint4': turn(mid_t, 2.0)},
                                        mid_t, t_cmd=0.0, t_stop=6.0)),
                 ('vel_readback.json', ids({3: {'goal_speed_raw': 1304},
                                            4: {'goal_speed_raw': 1304}})),
                 ('vel_stop.json', dict(readback(servos=(3, 4), goal_speed_raw=0),
                                        any_moving=False)),
                 ('arm_post.json', ids({1: dict(arm_regs, pos_last_raw=parked),
                                        2: dict(arm_regs, pos_last_raw=parked)}))])

    # H13-H17, the tools' scenarios of Phase 6: a healthy bench, see build_tools.
    build_tools(root)
    return root
