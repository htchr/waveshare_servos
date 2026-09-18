"""
rclpy recorder for the waveshare_servos bench check (PHASE2_SPEC 12.3).

Trimmed from phase1_evidence/hil/hilctl.py: the /diagnostics subscription is gone (hil_check.sh
captures that topic through the CLI) and so are the unused --topics permutations. Every
subcommand records /joint_states, and optionally /dynamic_joint_states, at full rate while it
acts, so command times and joint data share one clock and one file. Output is one JSON file.

  wait_js  --timeout S                                          exit 0 once /joint_states arrives
  record   --duration S --out F [--djs]                          record only
  move     --controller C --joints a,b --positions x,y --duration T --out F
  vel      --topic T --values v1,v2 --hold H [--stop-values ...] --out F
  call     --out F [--timeout S] -- <command ...>                run a CLI while recording
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def stamp_to_float(stamp):
    """Convert a builtin_interfaces/Time to seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    """Subscribe to /joint_states, and optionally /dynamic_joint_states, and keep every sample."""

    def __init__(self, djs=False):
        super().__init__('hil_record_%d' % os.getpid())
        self.js = []
        self.djs = []
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        from sensor_msgs.msg import JointState
        self.create_subscription(JointState, '/joint_states', self._on_js, qos)
        if djs:
            from control_msgs.msg import DynamicJointState
            self.create_subscription(DynamicJointState, '/dynamic_joint_states', self._on_djs, qos)

    def _on_js(self, msg):
        self.js.append([time.time(), stamp_to_float(msg.header.stamp), list(msg.name),
                        list(msg.position), list(msg.velocity), list(msg.effort)])

    def _on_djs(self, msg):
        self.djs.append({
            'rx': time.time(), 'stamp': stamp_to_float(msg.header.stamp),
            'joint_names': list(msg.joint_names),
            'interfaces': [{'names': list(iv.interface_names), 'values': list(iv.values)}
                           for iv in msg.interface_values]})

    def spin_for(self, seconds):
        """Spin for a fixed time, keeping every sample that arrives."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, end - time.time())))

    def spin_until(self, predicate, timeout):
        """Spin until the predicate holds or the timeout expires; return whether it holds."""
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.01)
        return predicate()

    def write(self, path, data):
        """Write one recording, atomically, with the samples taken so far."""
        data.update({'joint_states': self.js, 'dynamic_joint_states': self.djs})
        with open(path + '.tmp', 'w') as handle:
            json.dump(data, handle)
        os.replace(path + '.tmp', path)


def floats(text):
    """Parse a comma separated list of floats."""
    return [float(x) for x in text.split(',') if x != '']


def cmd_wait_js(args):
    """Exit 0 once /joint_states arrives."""
    node = Recorder()
    ok = node.spin_until(lambda: len(node.js) > 0, args.timeout)
    node.destroy_node()
    return 0 if ok else 1


def cmd_record(args):
    """Record for a fixed duration, or until SIGTERM."""
    node = Recorder(djs=args.djs)
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    start = time.time()
    end = start + args.duration
    try:
        while time.time() < end and not stop['now']:
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    node.write(args.out, {'kind': 'record', 'label': args.label, 't_start': start,
                          't_end': time.time()})
    node.destroy_node()
    return 0


def cmd_move(args):
    """Publish one joint trajectory point and record the move."""
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    node = Recorder(djs=args.djs)
    topic = '/%s/joint_trajectory' % args.controller
    publisher = node.create_publisher(JointTrajectory, topic, 10)
    seen = node.spin_until(lambda: len(node.js) > 0, 15.0)
    matched = node.spin_until(lambda: publisher.get_subscription_count() > 0, 15.0)
    node.spin_for(args.pre)
    message = JointTrajectory()
    message.joint_names = args.joints.split(',')
    point = JointTrajectoryPoint()
    point.positions = floats(args.positions)
    whole = int(args.duration)
    point.time_from_start = Duration(sec=whole,
                                     nanosec=int(round((args.duration - whole) * 1e9)))
    message.points = [point]
    t_cmd = time.time()
    publisher.publish(message)
    # hil_check.sh waits for this line to place a SIGKILL inside the move rather than after it
    print('T_CMD %.6f' % t_cmd, flush=True)
    node.spin_for(args.duration + args.post)
    # rclpy stores a float64[] field as array.array('d', ...) and a string[] as a plain list;
    # neither is guaranteed JSON-serializable, so every message field is copied into a list
    # before it reaches json.dump (the same rule the Recorder callbacks follow).
    node.write(args.out, {'kind': 'move', 'label': args.label, 'controller': args.controller,
                          'joints': list(message.joint_names),
                          'targets': list(point.positions),
                          'duration': args.duration, 't_cmd': t_cmd,
                          't_stop': t_cmd + args.duration, 'got_joint_states': seen,
                          'subscriber_matched': matched})
    node.destroy_node()
    return 0 if (seen and matched) else 2


def cmd_vel(args):
    """Publish a velocity command, hold it, optionally publish a stop, and record all of it."""
    from std_msgs.msg import Float64MultiArray
    node = Recorder(djs=args.djs)
    publisher = node.create_publisher(Float64MultiArray, args.topic, 10)
    seen = node.spin_until(lambda: len(node.js) > 0, 15.0)
    matched = node.spin_until(lambda: publisher.get_subscription_count() > 0, 15.0)
    node.spin_for(args.pre)
    values = floats(args.values)
    t_cmd = time.time()
    publisher.publish(Float64MultiArray(data=values))
    print('T_CMD %.6f' % t_cmd, flush=True)
    node.spin_for(args.hold)
    t_stop, stop_values = None, None
    if args.stop_values is not None:
        stop_values = floats(args.stop_values)
        t_stop = time.time()
        publisher.publish(Float64MultiArray(data=stop_values))
        node.spin_for(args.stop_hold)
    node.write(args.out, {'kind': 'vel', 'label': args.label, 'topic': args.topic,
                          'joints': args.joints.split(',') if args.joints else [],
                          'values': values, 'hold': args.hold, 'stop_values': stop_values,
                          't_cmd': t_cmd, 't_stop': t_stop if t_stop else t_cmd + args.hold,
                          'got_joint_states': seen, 'subscriber_matched': matched})
    node.destroy_node()
    return 0 if (seen and matched) else 2


def cmd_call(args):
    """Run a command line while recording, so a CLI call and the joint data share one clock."""
    node = Recorder(djs=args.djs)
    seen = node.spin_until(lambda: len(node.js) > 0, args.wait_js)
    node.spin_for(args.pre)
    command = args.command[1:] if args.command and args.command[0] == '--' else args.command
    t_start = time.time()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    deadline, timed_out = t_start + args.timeout, False
    while process.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.01)
        if time.time() > deadline:
            process.kill()
            timed_out = True
            break
    out, err = process.communicate()
    t_end = time.time()
    node.spin_for(args.post)
    node.write(args.out, {'kind': 'call', 'label': args.label, 'command': command,
                          'rc': process.returncode, 'timed_out': timed_out, 'stdout': out,
                          'stderr': err, 't_cmd': t_start, 't_stop': t_end,
                          'got_joint_states': seen})
    node.destroy_node()
    return 0


def parse(argv):
    """Build the argument parser and parse one command line."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    sub = parser.add_subparsers(dest='cmd', required=True)
    for name in ('wait_js', 'record', 'move', 'vel', 'call'):
        one = sub.add_parser(name)
        one.add_argument('--label', default=name)
        one.add_argument('--djs', action='store_true', help='also record /dynamic_joint_states')
        if name != 'wait_js':
            one.add_argument('--out', required=True)
            one.add_argument('--pre', type=float, default=0.5)
            one.add_argument('--post', type=float, default=1.5)
        if name == 'wait_js':
            one.add_argument('--timeout', type=float, default=20.0)
        if name in ('record', 'call'):
            one.add_argument('--duration', type=float, default=10.0)
        if name == 'move':
            one.add_argument('--controller', required=True)
            one.add_argument('--joints', required=True)
            one.add_argument('--positions', required=True)
            one.add_argument('--duration', type=float, required=True)
        if name == 'vel':
            one.add_argument('--topic', required=True)
            one.add_argument('--joints', default='')
            one.add_argument('--values', required=True)
            one.add_argument('--hold', type=float, required=True)
            one.add_argument('--stop-values', default=None)
            one.add_argument('--stop-hold', type=float, default=2.0)
        if name == 'call':
            one.add_argument('--timeout', type=float, default=30.0)
            one.add_argument('--wait-js', type=float, default=5.0)
            one.add_argument('command', nargs=argparse.REMAINDER)
    return parser.parse_args(argv)


def main(argv):
    """Run one subcommand under rclpy."""
    args = parse(argv)
    rclpy.init()
    try:
        return {'wait_js': cmd_wait_js, 'record': cmd_record, 'move': cmd_move,
                'vel': cmd_vel, 'call': cmd_call}[args.cmd](args)
    finally:
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
