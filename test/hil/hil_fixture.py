"""
A synthetic run tree for the hil_gates self-test (PHASE2_SPEC 12.3, 12.5).

`hil_gates.py --self-test` used to exercise only the gate primitives of 12.2 against synthetic
series. That left the twelve per-scenario checkers h1..h10 -- where the report of 12.4 is actually
built -- entirely unexecuted, so a checker that could not run at all still reported GREEN. This
module writes a run directory with the same file shapes hil_check.sh produces, thin enough to
build in memory and complete enough that every checker walks its whole body.

The data is deliberately NOT tuned to make every row PASS. The self-test's claim is only that the
checkers run and emit rows; the verdicts themselves are what a real bench run decides.
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
    write(root, 'H9',
          facts={'t_cycled': '2.0', 't_plus0': '1.0', 't_plus1': '3.0',
                 't_minus0': '5.0', 't_minus1': '7.0'},
          log=''.join("[WARN] joint '%s' declares the deprecated 'torque' interface\n" % j
                      for j in REAL + ('joint5',)),
          files=[('interfaces.json', rec(quiet, long_t,
                                         dynamic_joint_states=djs(long_t, columns))),
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
    return root
