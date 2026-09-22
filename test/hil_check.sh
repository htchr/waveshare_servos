#!/usr/bin/env bash
# The waveshare_servos bench check (PHASE2_SPEC 12.3-12.6). Driven by ctest as `hil_check`, it
# skips unless WAVESHARE_HIL=1 and a real adapter is present, so a motorless machine records a
# skip rather than a failure. It drives the motors: nothing else may hold the port while it runs.
#
#   WAVESHARE_HIL=1 colcon test --packages-select waveshare_servos --ctest-args -R hil_check
#
# Environment: WAVESHARE_HIL_PORT (default /dev/ttyACM0), WAVESHARE_HIL_OUT (default
# ./hil_check.d), WAVESHARE_HIL_WS (default: the workspace this package was built from),
# WAVESHARE_HIL_HELPERS and WAVESHARE_HIL_STOP_WHEELS / _PORT_PROBE (set by CMake), and
# WAVESHARE_HIL_SOAK_S (H11's soak length in seconds, default 600, clamped to at least 40).
# Exit codes: 0 every check PASS/SKIP or an allowed INCONCLUSIVE; 1 a FAIL; 2 an abort; 77 skip.
#
# The safety machinery of PHASE2_SPEC 12.4 -- the clean-environment re-exec, HIL_TAG, killing
# only tagged processes, the port-holder check around every scenario, timeout on everything and
# stopping the wheels at the end of each scenario -- is carried from phase1_evidence/hil/
# common.sh:19-66, 130-149, 163-195 and run.sh:12-24, 83-94, where it was exercised for a whole
# phase. Never pkill -f ros2, never fuser -k, never a bare kill on a pgrep result.

PORT=${WAVESHARE_HIL_PORT:-/dev/ttyACM0}

HELPERS=${WAVESHARE_HIL_HELPERS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/hil" && pwd)}
STOP_WHEELS=${WAVESHARE_HIL_STOP_WHEELS:-}
PORT_PROBE=${WAVESHARE_HIL_PORT_PROBE:-}
OUT=${WAVESHARE_HIL_OUT:-$PWD/hil_check.d}
WS=${WAVESHARE_HIL_WS:-$(cd "$HELPERS/../../../.." && pwd)}

# INCONCLUSIVE is legal for exactly these two keys, because for those two the bench itself cannot
# supply the stimulus (PHASE2_SPEC 12.4/12.5). Adding a third is a spec change, reviewed as one,
# never a run-time decision. Every other row that would be INCONCLUSIVE is reported as FAIL.
HIL_INCONCLUSIVE_ALLOWED="H7.load_sign H8.accel_effect"

# The skip contract and the single clean-environment re-exec (12.3, run.sh:12-24). Both belong to
# the outer invocation: the re-exec's env -i PATH has no ros2 on it until setup.bash is sourced
# below, so running the contract again inside would skip every real run.
if [ -z "${HIL_CLEAN:-}" ]; then
  # Each line says why, and exits 77, CMake's skip code.
  [ "${WAVESHARE_HIL:-}" = 1 ] || { echo "SKIP: WAVESHARE_HIL is not 1"; exit 77; }
  [ -e "$PORT" ] || { echo "SKIP: $PORT does not exist"; exit 77; }
  { [ -r "$PORT" ] && [ -w "$PORT" ]; } ||
    { echo "SKIP: $PORT is not readable/writable by uid $(id -u)"; exit 77; }
  [ "$(id -u)" != 0 ] || { echo "SKIP: refusing to drive the bus as root"; exit 77; }
  command -v ros2 > /dev/null || { echo "SKIP: no ros2 on PATH"; exit 77; }
  # Canonicalise once, here: port_holders() matches the /proc fd symlink target literally, so a
  # port named through /dev/serial/by-id/... would make every port-freedom check say "free".
  PORT=$(readlink -f "$PORT" 2> /dev/null || echo "$PORT")
  mkdir -p "$OUT" || exit 2
  OUT=$(cd "$OUT" && pwd)
  exec env -i HOME="$HOME" USER="${USER:-ubuntu}" LANG=C.UTF-8 TERM=dumb \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    HIL_CLEAN=1 HIL_TAG="hil-$$-$(date +%s)" ROS_DOMAIN_ID=77 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONUNBUFFERED=1 \
    WAVESHARE_HIL=1 WAVESHARE_HIL_PORT="$PORT" WAVESHARE_HIL_OUT="$OUT" \
    WAVESHARE_HIL_WS="$WS" WAVESHARE_HIL_HELPERS="$HELPERS" \
    WAVESHARE_HIL_STOP_WHEELS="$STOP_WHEELS" WAVESHARE_HIL_PORT_PROBE="$PORT_PROBE" \
    WAVESHARE_HIL_SCENARIOS="${WAVESHARE_HIL_SCENARIOS:-}" \
    WAVESHARE_HIL_SOAK_S="${WAVESHARE_HIL_SOAK_S:-}" \
    bash --noprofile --norc "${BASH_SOURCE[0]}"
fi

export ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE HIL_TAG
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"

CM_BIN=/opt/ros/jazzy/lib/controller_manager/ros2_control_node
RSP_BIN=/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher
RECORD="python3 $HELPERS/hil_record.py"
GATES="python3 $HELPERS/hil_gates.py"
STARTED=$SECONDS
SCEN=setup
SDIR=$OUT

hil_log() {
  echo "[$(date +%H:%M:%S.%3N)] ${SCEN}: $*" | tee -a "$OUT/run.log"
}

# pids with an open fd on the port (common.sh:19-21; fuser/lsof are not installed here)
port_holders() {
  find /proc/[0-9]*/fd -lname "$PORT" 2> /dev/null | cut -d/ -f3 | sort -u | tr '\n' ' ' |
    sed 's/ $//'
}

wait_port_free() {
  local secs=${1:-10} i
  for ((i = 0; i < secs * 4; i++)); do
    [ -z "$(port_holders)" ] && return 0
    sleep 0.25
  done
  [ -z "$(port_holders)" ]
}

# only processes carrying this run's HIL_TAG are ever signalled (common.sh:33-51)
tagged_pids() {
  local pid
  for pid in $(pgrep -f "$1"); do
    if tr '\0' '\n' < "/proc/$pid/environ" 2> /dev/null | grep -qx "HIL_TAG=$HIL_TAG"; then
      echo "$pid"
    fi
  done
}

kill_tagged() {
  local sig=$1 pat pids
  shift
  for pat in "$@"; do
    pids=$(tagged_pids "$pat")
    [ -n "$pids" ] && kill "-$sig" $pids 2> /dev/null
  done
  return 0
}

STACK_PATTERNS=("^$CM_BIN" "^$RSP_BIN" "ros2 launch waveshare_servos" "controller_manager/spawner"
  "hil_record.py")

teardown_stack() {
  local i
  kill_tagged INT "${STACK_PATTERNS[@]}"
  for ((i = 0; i < 40; i++)); do
    [ -z "$(tagged_pids "^$CM_BIN")$(tagged_pids "^$RSP_BIN")" ] && break
    sleep 0.25
  done
  kill_tagged TERM "${STACK_PATTERNS[@]}"
  sleep 0.5
  kill_tagged KILL "^$CM_BIN" "^$RSP_BIN"
  wait_port_free 10
}

# ros2 CLI under a timeout, without the "waiting for service" chatter (common.sh:88-97)
r2() {
  local t=$1 rc ef
  shift
  ef=$(mktemp)
  timeout -s INT "$t" ros2 "$@" 2> "$ef"
  rc=$?
  grep -v 'waiting for service' "$ef" >&2
  rm -f "$ef"
  return $rc
}

fact() {
  local key=$1
  shift
  python3 - "$SDIR/facts.json" "$key" "$*" <<'EOF'
import json, os, sys
path, key, value = sys.argv[1], sys.argv[2], sys.argv[3]
d = json.load(open(path)) if os.path.exists(path) else {}
d[key] = value
json.dump(d, open(path, "w"), indent=1, sort_keys=True)
EOF
}

now() { date +%s.%N; }

controllers_snapshot() {
  r2 20 control list_controllers 2> /dev/null | awk 'NF>=3 && $1 !~ /^\[/ {print $1, $2, $NF}'
}

wait_controllers_active() {
  local secs=$1 start=$SECONDS snap c ok
  shift
  while ((SECONDS - start < secs)); do
    snap=$(controllers_snapshot)
    ok=1
    for c in "$@"; do
      echo "$snap" | grep -q "^$c .* active$" || ok=0
    done
    ((ok)) && return 0
    sleep 0.5
  done
  return 1
}

hw_state() {
  r2 20 control list_hardware_components 2> /dev/null | awk -v n="${1:-bench}" \
    '$1=="name:" {cur=$2} $1=="state:" && cur==n {sub(/^.*label=/, ""); print; exit}'
}

# stop_wheels, never while something holds the port. $1 = output name, rest = arguments
readback() {
  local out=$SDIR/$1.json txt rc
  shift
  if [ -n "$(port_holders)" ]; then
    echo "{\"ok\": false, \"error\": \"port_busy_skipped\"}" > "$out"
    return 1
  fi
  txt=$(timeout -s KILL 20 "$STOP_WHEELS" --port "$PORT" "$@" 2>&1)
  rc=$?
  echo "$txt" > "${out%.json}.txt"
  echo "$txt" | sed -n 's/^RESULT //p' | tail -1 > "$out"
  [ -s "$out" ] || echo '{"ok": false, "error": "no_result"}' > "$out"
  return $rc
}

# port_probe, the second opener of H6. $1 = output name, rest = arguments
probe() {
  local out=$SDIR/$1.json hold=15 txt
  shift
  case " $* " in *" --hold "*) hold=$(sed 's/.*--hold \([0-9]*\).*/\1/' <<< "$*");; esac
  txt=$(timeout -s KILL $((hold + 15)) "$PORT_PROBE" --port "$PORT" "$@" 2>&1)
  echo "$txt" | sed -n 's/^RESULT //p' | tail -1 > "$out"
  [ -s "$out" ] || echo '{"verdict": "no_result"}' > "$out"
}

# port_probe prints "HOLDING {json}" the moment it owns the port and before it sleeps. Echo the
# pid it announces, so the caller knows the hold window started rather than guessing it did.
wait_holding() {  # stdout-file [seconds]
  local i pid
  for ((i = 0; i < ${2:-20} * 10; i++)); do
    pid=$(sed -n 's/^HOLDING .*"pid": \([0-9]*\).*/\1/p' "$1" 2> /dev/null | head -1)
    [ -n "$pid" ] && { echo "$pid"; return 0; }
    sleep 0.1
  done
  # to stderr: the caller reads this function's stdout, and only the pid belongs there
  hil_log "no HOLDING line in $1 after ${2:-20}s" >&2
  return 1
}

abort_scenario() {
  printf '%s\t%s\n' "$1" "$2" >> "$OUT/aborted.txt"
  hil_log "ABORTED $1: $2"
}

# A holder that is not ours is not ours to kill (PHASE2_SPEC 12.4): stop, and say what to do.
port_rescue() {
  local pid tagged=1
  for pid in $(port_holders); do
    tr '\0' '\n' < "/proc/$pid/environ" 2> /dev/null | grep -qx "HIL_TAG=$HIL_TAG" || tagged=0
  done
  if [ "$tagged" = 0 ]; then
    hil_log "port $PORT is held by [$(port_holders)], and not by this suite:"
    for pid in $(port_holders); do
      hil_log "  pid $pid: $(tr '\0' ' ' < "/proc/$pid/cmdline" 2> /dev/null)"
    done
    hil_log "it is not this suite's process, so it is not this suite's to kill; stopping"
    return 1
  fi
  hil_log "escalating on our own tagged holders [$(port_holders)]"
  kill_tagged INT "${STACK_PATTERNS[@]}"
  sleep 10
  kill_tagged TERM "${STACK_PATTERNS[@]}"
  sleep 5
  kill_tagged KILL "${STACK_PATTERNS[@]}"
  wait_port_free 10
}

scenario_begin() {
  SCEN=$1
  SDIR=$OUT/$SCEN
  rm -rf "$SDIR"
  mkdir -p "$SDIR"
  export ROS_LOG_DIR="$OUT/roslog/$SCEN"
  rm -rf "$ROS_LOG_DIR"
  mkdir -p "$ROS_LOG_DIR"
  hil_log "begin"
  fact port_holders_before "$(port_holders)"
  local rescued=0
  if ! wait_port_free 15; then
    rescued=1
    port_rescue || { abort_scenario "$SCEN" "port held by [$(port_holders)] (not tagged)"; \
      return 2; }
    wait_port_free 5 || { fact port_free_before false
      abort_scenario "$SCEN" "port could not be freed"
      return 2; }
  fi
  # A rescue is not a clean start: 12.4 item 3 makes port_free_before a gated row, so a scenario
  # that needed one records false even though its own holders were ours to kill.
  if [ "$rescued" = 1 ]; then
    fact port_free_before false
  else
    fact port_free_before true
  fi
  # 12.4 item 9: record before you move, once per scenario, while the port is free.
  readback pre_readback --read-only --registers --ids 1,2,3,4 > /dev/null
  return 0
}

scenario_end() {
  teardown_stack
  cat "$SDIR"/*.stdout > "$SDIR/log.txt" 2> /dev/null
  fact port_holders_after "$(port_holders)"
  if [ -z "$(port_holders)" ]; then
    fact port_free_after true
    readback final_stop_wheels --ids 3,4 > /dev/null
  else
    fact port_free_after false
  fi
  hil_log "end (holders after: [$(port_holders)])"
  SCEN=run
  SDIR=$OUT
}

# ------------------------------------------------------------------ the stack under test
# $1 = xacro arguments, $2 = the wheels joint list for the controller, $3 = the stack watchdog in
# seconds (default 420). The watchdog is what SIGINTs a stack that outlives its scenario, so it
# has to be longer than the scenario: H11 soaks for ten minutes and passes its own (PHASE3 5.13).
# Every other scenario is far inside 420, and H9 owns the longest single stack. Phase 5 raised its
# recorder from 40 s to 70 s and added two post-cycle arm moves (see the `70 s, raised from 40`
# comment in h_H9), so the 56 s that stack lived in the archived post-Phase-4 run
# (phase5_evidence/post_phase4_baseline_2026-09-21_1659/run.log: `starting ros2_control_node` at
# 17:14:36.011, `end` at 17:15:32.403) becomes roughly 86 s -- the recorder, not the body, now
# sets that length (the budget block above h_H11 does the arithmetic). Still under a quarter of
# the 420 s default.
# H12 runs THREE stacks on this same default, none of them a minute long: the watchdog is per
# stack, a fresh `timeout` per call, never per scenario, so several short stacks in one scenario
# never walk up on it the way one long stack does.
#
# $4 = the controller YAML under hil/controllers (default bench.yaml), and it exists for H12
# alone. `enforce_command_limits` can only be set in that file -- the controller manager builds
# the JointSaturationLimiters at hardware-component init, so a `ros2 param set` on a running
# stack is accepted and does nothing (jazzy.md section 6 step 8, the Phase 4 amendment) -- and
# turning it on in bench.yaml would turn it on for every other scenario, whose gates were measured
# without it. A parameter here, rather than a variable a scenario sets: a global left set by one
# scenario would silently re-render the next one's stack.
start_stack() {
  local watchdog=${3:-420} yaml=${4:-bench.yaml}
  # port:= first, so a scenario argument can still override it deliberately
  xacro "$HELPERS/descriptions/bench.urdf.xacro" "port:=$PORT" $1 > "$SDIR/robot.urdf" || return 1
  sed "s/@WHEELS@/$2/" "$HELPERS/controllers/$yaml" > "$SDIR/cm.yaml"
  hil_log "starting ros2_control_node ($1) against $yaml"
  STACK_N=$((${STACK_N:-0} + 1))
  CM_LOG=$SDIR/cm$STACK_N.stdout
  timeout -s INT "$watchdog" "$CM_BIN" --ros-args --params-file "$SDIR/cm.yaml" > "$CM_LOG" 2>&1 &
  CM_WRAP=$!
  # robot_state_publisher takes the description through a parameter FILE, not through
  # -p robot_description:=<urdf>: rcl parses a parameter override as YAML, and a multi-line
  # XML document is not a YAML scalar, so the override form aborts the node before it starts.
  python3 -c 'import sys, yaml; yaml.safe_dump({"robot_state_publisher": {"ros__parameters":
    {"robot_description": open(sys.argv[1]).read()}}}, open(sys.argv[2], "w"))' \
    "$SDIR/robot.urdf" "$SDIR/rsp.yaml" || return 1
  timeout -s INT "$((watchdog + 10))" "$RSP_BIN" --ros-args --params-file "$SDIR/rsp.yaml" \
    > "$SDIR/rsp$STACK_N.stdout" 2>&1 &
  RSP_WRAP=$!
  timeout -s INT 90 ros2 run controller_manager spawner joint_state_broadcaster arm wheels \
    -c /controller_manager -p "$SDIR/cm.yaml" > "$SDIR/spawner$STACK_N.stdout" 2>&1
  SPAWNER_RC=$?
  fact spawner_rc "$SPAWNER_RC"
  if wait_controllers_active 60 joint_state_broadcaster arm wheels; then
    CONTROLLERS_ACTIVE=true
  else
    CONTROLLERS_ACTIVE=false
  fi
  fact controllers_active "$CONTROLLERS_ACTIVE"
  fact hw_state "$(hw_state)"
  fact port_holders_running "$(port_holders)"
  $RECORD wait_js --timeout 20 && fact joint_states_seen true || fact joint_states_seen false
}

# $1 = TERM (graceful, the driver deactivates) or KILL (the registers stay as the driver left them)
stop_stack() {
  local cm rc
  cm=$(pgrep -P "${CM_WRAP:-0}" 2> /dev/null)
  [ -n "$cm" ] && kill "-${1:-TERM}" "$cm" 2> /dev/null
  wait "$CM_WRAP" 2> /dev/null
  rc=$?
  fact cm_exit_code $rc
  # Also exposed as a global, for the reason start_stack exposes SPAWNER_RC and
  # CONTROLLERS_ACTIVE: a scenario that runs more than one stack has to copy each stack's number
  # under its own fact key before the next stop_stack overwrites the shared cm_exit_code
  # (h_H12 does exactly that, as park_cm_exit_code and limited_cm_exit_code).
  CM_EXIT_CODE=$rc
  local rsp
  rsp=$(pgrep -P "${RSP_WRAP:-0}" 2> /dev/null)
  [ -n "$rsp" ] && kill -TERM "$rsp" 2> /dev/null
  wait "$RSP_WRAP" 2> /dev/null
  wait_port_free 10 && fact port_released true || fact port_released false
}

# $6 is the controller whose /<controller>/joint_trajectory the point goes to, and it exists for
# H1B alone: the shipped example calls its arm controller `joint_trajectory_position_controller`
# (bringup/config/example_controllers.yaml:31-32), while every bench scenario calls it `arm`
# (hil/controllers/bench.yaml). A positional with a default rather than a variable a scenario
# sets, for the reason start_stack's $4 gives: a global left set by one scenario would silently
# retarget the next one's moves, and the failure would look like a dead controller.
move() {  # label joints positions duration [post] [controller]
  $RECORD move --controller "${6:-arm}" --joints "$2" --positions="$3" --duration "$4" --pre 0.5 \
    --post "${5:-1.5}" --label "$1" --out "$SDIR/$1.json" > "$SDIR/$1.stdout" 2>&1 ||
    hil_log "move $1 rc=$?"
}

# An empty stop-values publishes no stop at all, which is what the scenarios that SIGKILL the
# stack mid-command need: the controller holds the last command it was given (12.5, H7 and H8).
# $7 is the command topic, and like move's $6 it exists for H1B: the example's wheel controller is
# `joint_velocity_controller`, so its JointGroupVelocityController listens on
# /joint_velocity_controller/commands, not on the bench's /wheels/commands.
spin() {  # label values hold [stop-values [stop-hold]] [--djs] [topic]
  local stop=()
  [ -n "${4:-}" ] && stop=(--stop-values "$4" --stop-hold "${5:-2.0}")
  $RECORD vel --topic "${7:-/wheels/commands}" --values="$2" --hold "$3" "${stop[@]}" ${6:-} \
    --label "$1" --out "$SDIR/$1.json" > "$SDIR/$1.stdout" 2>&1 || hil_log "vel $1 rc=$?"
}

# Wait for a backgrounded recorder to say it has published, so a SIGKILL can be placed inside a
# move rather than after it. hil_record.py prints "T_CMD <epoch>" the moment it publishes.
wait_t_cmd() {  # stdout-file [seconds]
  local i
  for ((i = 0; i < ${2:-30} * 10; i++)); do
    grep -q '^T_CMD ' "$1" 2> /dev/null && return 0
    sleep 0.1
  done
  hil_log "no T_CMD in $1 after ${2:-30}s"
  return 1
}

diagnostics() {  # capture /diagnostics through the CLI (12.3: the recorder does not subscribe)
  r2 "${1:-12}" topic echo /diagnostics > "$SDIR/diagnostics.txt" 2>&1
  return 0
}

# ------------------------------------------------------------------ the ten scenarios (12.5)
h_H1() {
  # H1 is the shipped entry point: the one scenario that runs bringup/launch/example.launch.py and
  # its four-joint description, where every other scenario drives a bare ros2_control_node against
  # hil/descriptions/bench.urdf.xacro. It passes port:=$PORT (declared at example.launch.py:28-31,
  # forwarded into the xacro render at :73) so the suite follows WAVESHARE_HIL_PORT here like it
  # does everywhere else -- start_stack:319, the xacro line, passes the same "port:=$PORT" to its
  # own render.
  #
  # Until Phase 5 this scenario passed no port and called abort_scenario on any port but
  # /dev/ttyACM0, which cost more than the one scenario it looked like: run() turns an ABORTED row
  # into exit code 2 -- the line is 'if counts[...ABORTED...] or not port_free: return 2', at
  # hil_gates.py:1905-1906 -- so a bench on a second adapter reported the whole suite FAIL rather
  # than skipping the scenario it could not run.
  #
  # What that skip bought, and what passing the port gives up: the packaged default is no longer
  # exercised by being left alone. On the usual bench it is the same value either way -- both
  # description/urdf/example.urdf.xacro:6 and example.launch.py:30 default to /dev/ttyACM0 -- so a
  # regressed default would render identically here and pass. Nothing is lost by that, because
  # the default is a render-time fact about the shipped files and is already pinned where it can
  # be checked with no motors at all: test/test_urdf_xacro.py:86 asserts the three <xacro:arg>
  # declarations and their defaults verbatim, and :103 asserts that an argument-free render is
  # the real driver on /dev/ttyACM0 at 1 Mbaud.
  #
  # gui:=false stays: gui defaults to true (example.launch.py:45-49) and rviz2 has no display on
  # this bench, so the default would leave a failed node in the launch log of every run.
  scenario_begin H1 || return $?
  timeout -s INT 420 ros2 launch waveshare_servos example.launch.py "port:=$PORT" gui:=false \
    > "$SDIR/launch.stdout" 2>&1 &
  LAUNCH_WRAP=$!
  wait_controllers_active 90 joint_state_broadcaster joint_trajectory_position_controller \
    joint_velocity_controller && fact controllers_active true || fact controllers_active false
  fact hw_state "$(hw_state example_ws_ros2_control)"
  $RECORD wait_js --timeout 20 > /dev/null
  $RECORD record --duration 9 --label steady --out "$SDIR/steady.json"
  diagnostics 12
  kill_tagged INT "ros2 launch waveshare_servos"
  wait "$LAUNCH_WRAP" 2> /dev/null
  scenario_end
}

# H1B: the shipped example, COMMANDED. jazzy.md section 6 steps 1, 3, 4 and 7 are all written
# against bringup/launch/example.launch.py, but until Phase 5 H1 only watched that stack idle --
# it waited for the controllers, recorded 9 s of /joint_states and 12 s of /diagnostics, and
# SIGINTed. Every position, velocity and shutdown proof in this suite was therefore against
# hil/descriptions/bench.urdf.xacro and hil/controllers/bench.yaml, i.e. against a description
# and a controller set that ship with the TESTS and not with the package. The thing a user
# actually runs was never commanded.
#
# Why a separate scenario instead of more stimulus inside h_H1. H1's read_ms, write_ms, rate and
# per-joint gate rows are `[no-regression]` rows whose bounds were measured over an IDLE example
# stack: H1's 12 s /diagnostics capture is where READ_MS_AVG_MAX and friends are compared, and
# turning two servos inside that window changes the cost being measured. Re-basing a pre-existing
# no-regression term is not this pass's business (the argument h9's `cost` comment already makes
# about rate_hz), and a quiet stack is worth keeping as its own measurement. So H1 keeps its
# meaning and H1B pays a second launch startup, ~20 s, to get a clean one.
#
# SAFETY, and jazzy.md section 7: the example's joint_trajectory_position_controller claims BOTH
# position and velocity command interfaces (bringup/config/example_controllers.yaml:44-50), the
# configuration the joint_velocity_controller comment in that same file suspects of crashing
# joint_trajectory_controller on jazzy. The arm move below is the first thing in this repo that
# will ever command it on real hardware. Nothing here tries to resolve that. What it does do is
# fail loudly and safely if it happens:
#   - the arm is commanded FIRST, while the wheels are still stopped, so a crash during the arm
#     move cannot leave a wheel spinning;
#   - the wheel command is skipped outright if the component is no longer active afterwards, and
#     H1B.arm_survived turns that into a FAIL rather than a silent skip;
#   - hil_record.py never blocks forever -- it waits at most 15 s for a subscriber, then writes
#     its file and exits -- so a dead stack costs seconds, not a hung suite;
#   - if a crash does land mid-spin the servos keep their last goal speed in firmware, which is
#     why after_exit is read back and gated, and why scenario_end's final_stop_wheels and the
#     EXIT trap's cleanup_all both COMMAND zero afterwards.
h_H1B() {
  scenario_begin H1B || return $?
  timeout -s INT 420 ros2 launch waveshare_servos example.launch.py "port:=$PORT" gui:=false \
    > "$SDIR/launch.stdout" 2>&1 &
  LAUNCH_WRAP=$!
  wait_controllers_active 90 joint_state_broadcaster joint_trajectory_position_controller \
    joint_velocity_controller && fact controllers_active true || fact controllers_active false
  fact hw_state "$(hw_state example_ws_ros2_control)"
  # Phase 4 spawns diff_drive_controller --inactive deliberately (example.launch.py:162-168): it
  # claims the same joint3/joint4 velocity command interfaces as joint_velocity_controller, and
  # ros2_control hands each command interface to exactly one controller, so an active one would
  # take the wheels away from the controller this scenario commands. Nothing in the bench noticed
  # today if that regressed -- to active, or to absent because the separate apt package
  # (ros-jazzy-diff-drive-controller, package.xml:60) is not installed. An empty fact is the
  # "absent" case and H1B.diff_drive fails on it just as it fails on "active".
  fact diff_drive_state "$(controllers_snapshot | awk '$1=="diff_drive_controller" {print $NF}')"
  $RECORD wait_js --timeout 20 > /dev/null
  # Step 3: the arm, gated the way H2 gates it. Same two targets and same 2 s, so H1B.target and
  # H2.target are the same measurement on the two different stacks and can be read side by side.
  move ex_move_to_0 joint1 0.0 2.0 1.5 joint_trajectory_position_controller
  move ex_move_to_06 joint1 0.6 2.0 1.5 joint_trajectory_position_controller
  fact arm_state_after \
    "$(controllers_snapshot | awk '$1=="joint_trajectory_position_controller" {print $NF}')"
  local state
  state=$(hw_state example_ws_ros2_control)
  fact hw_state_after_arm "$state"
  # Step 4: the wheels, gated the way H3 gates it -- 2.0 rad/s for 6 s, then an explicit stop.
  # Guarded, because commanding wheels through a stack that has just died is how a bench ends up
  # with servos spinning on a latched goal speed and no process to stop them.
  if [ "$state" = active ]; then
    fact vel_attempted true
    spin ex_vel_2 "2.0,2.0" 6.0 "0.0,0.0" 2.5 "" /joint_velocity_controller/commands
  else
    fact vel_attempted false
    hil_log "component is '$state' after the arm move; skipping the wheel command (section 7)"
  fi
  # Step 7: the launch's own SIGINT teardown. H10 gates this shape for a bare ros2_control_node
  # under SIGINT and SIGTERM; this is the same proof for the wrapper a user actually runs, and
  # on a stack that has just driven both an arm joint and a wheel rather than an idle one.
  kill_tagged INT "ros2 launch waveshare_servos"
  wait "$LAUNCH_WRAP" 2> /dev/null
  fact launch_exit_code $?
  kill_tagged TERM "^$RSP_BIN"
  sleep 1.5
  readback after_exit --read-only --ids 3,4 > /dev/null
  wait_port_free 10 && fact port_free_within_10s true || fact port_free_within_10s false
  fact port_holders_after_exit "$(port_holders)"
  probe probe_after_exit
  scenario_end
}

h_H2() {
  scenario_begin H2 || return $?
  start_stack "" "[joint3, joint4]"
  move move_to_0 joint1 0.0 2.0
  move move_to_06 joint1 0.6 2.0
  stop_stack KILL          # the goal registers stay exactly as the driver last wrote them
  readback post_readback --read-only --registers --ids 1,2,3,4 > /dev/null
  scenario_end
}

h_H3() {
  scenario_begin H3 || return $?
  start_stack "" "[joint3, joint4]"
  spin vel_2 "2.0,2.0" 6.0 "0.0,0.0" 2.5
  stop_stack TERM
  scenario_end
}

h_H4() {
  scenario_begin H4 || return $?
  start_stack "" "[joint3, joint4]"
  spin spin "2.0,-2.0" 12.0 "0.0,0.0" 3.0
  stop_stack TERM
  readback post_readback --read-only --registers --ids 1,2,3,4 > /dev/null
  scenario_end
}

h_H5A() {
  scenario_begin H5A || return $?
  start_stack "phantom:=true allow_missing:=false" "[joint3, joint4, joint5]"
  stop_stack TERM
  readback post_readback --read-only --registers --ids 1,2,3,4 > /dev/null
  scenario_end
}

h_H5B() {
  scenario_begin H5B || return $?
  start_stack "phantom:=true allow_missing:=true" "[joint3, joint4, joint5]"
  diagnostics 12
  spin wheels "2.0,2.0,0.0" 6.0 "0.0,0.0,0.0" 2.5
  stop_stack TERM
  scenario_end
}

h_H5C() {
  scenario_begin H5C || return $?
  start_stack "allow_missing:=false" "[joint3, joint4]"
  move move_to_03 joint1 0.3 2.0
  stop_stack TERM
  scenario_end
}

h_H6() {
  scenario_begin H6 || return $?
  start_stack "" "[joint3, joint4]"          # the wheels stay at 0 throughout H6 (12.4 item 6)
  fact driver_warns_before "$(grep -c '\[WARN\]\|\[ERROR\]' "$CM_LOG")"
  $RECORD record --duration 4 --label probe_window --out "$SDIR/probe_window.json" &
  local recorder=$!
  sleep 1
  probe probe_while_active
  wait $recorder
  fact driver_warns_after "$(grep -c '\[WARN\]\|\[ERROR\]' "$CM_LOG")"
  stop_stack TERM
  # the driver must refuse to configure against a holder, once flock-only and once exclusive
  local kind flag holder probe_pid sampler next_log
  for kind in flock_holder excl_holder; do
    flag=$([ "$kind" = flock_holder ] && echo --no-exclusive)
    # The baseline for "the refused configure changed nothing" is taken per holder, immediately
    # before it and while the port is free -- not once at the top of the scenario, where H6's own
    # active stack legitimately wrote the goal registers, and not once for both holders, which
    # would stretch the comparison across two failed start_stacks of an unpowered arm.
    wait_port_free 15
    readback "${kind}_pre" --read-only --registers --ids 1,2,3,4 > /dev/null
    timeout -s KILL 60 "$PORT_PROBE" --port "$PORT" --hold 30 $flag > "$SDIR/$kind.txt" 2>&1 &
    holder=$!
    # The probe's RESULT line only exists once the hold is over; its HOLDING line exists as soon
    # as it owns the port, and carries the pid, so the hold window is observed and not assumed.
    probe_pid=$(wait_holding "$SDIR/$kind.txt" 20)
    fact "${kind}_probe_pid" "$probe_pid"
    # 12.5 wants the holders sampled once the driver's own configure has been refused -- a
    # leaked fd from the failed component is what makes it two -- and that is inside the hold,
    # not after start_stack returns, which can outlast a 30 s hold waiting on a spawner that
    # will never come up. The sampler waits for the refusal, then for the driver to let go.
    # It also records whether the holder was still alive when the refusal appeared: a holder that
    # expired first lets the driver succeed, and that is the sampler's fault, not the driver's.
    : > "$SDIR/$kind.holders"
    echo false > "$SDIR/$kind.within"
    next_log=$SDIR/cm$((${STACK_N:-0} + 1)).stdout
    (
      for ((i = 0; i < 160; i++)); do
        if grep -q 'refusing to share the bus' "$next_log" 2> /dev/null; then
          [ -n "$probe_pid" ] && [ -d "/proc/$probe_pid" ] && echo true > "$SDIR/$kind.within"
          sleep 1
          port_holders > "$SDIR/$kind.holders"
          exit 0
        fi
        # once the holder is gone there is nothing left to refuse, so stop waiting and let
        # ${kind}_refusal_within_hold report why, instead of blaming the driver 40 s later
        [ -n "$probe_pid" ] && [ ! -d "/proc/$probe_pid" ] && exit 0
        sleep 0.25
      done
    ) &
    sampler=$!
    start_stack "" "[joint3, joint4]"
    wait $sampler 2> /dev/null
    fact "${kind}_port_holders" "$(cat "$SDIR/$kind.holders")"
    fact "${kind}_refusal_within_hold" "$(cat "$SDIR/$kind.within")"
    fact "${kind}_spawner_rc" "$SPAWNER_RC"
    fact "${kind}_controllers_active" "$CONTROLLERS_ACTIVE"
    # Both refusals -- EBUSY from the exclusive holder and LOCK_FAILED from the flock-only one --
    # end in "refusing to share the bus" (src/waveshare_servos.cpp:1584-1587 and :1616-1620).
    fact "${kind}_driver_refusals" "$(grep -c 'refusing to share the bus' "$CM_LOG")"
    fact "${kind}_serial_speed_lines" "$(grep -c 'serial speed' "$CM_LOG")"
    stop_stack TERM
    wait $holder 2> /dev/null
    sed -n 's/^RESULT //p' "$SDIR/$kind.txt" | tail -1 > "$SDIR/$kind.json"
    [ -s "$SDIR/$kind.json" ] || echo '{"verdict": "no_result"}' > "$SDIR/$kind.json"
    wait_port_free 15
    readback "${kind}_post" --read-only --registers --ids 1,2,3,4 > /dev/null
  done
  probe probe_after_release
  scenario_end
}

h_H7() {
  scenario_begin H7 || return $?
  start_stack "inverted2:=true inverted4:=true" "[joint3, joint4]"
  move pos_move joint1,joint2 0.6,0.6 2.0
  stop_stack KILL
  readback pos_readback --read-only --registers --ids 1,2 > /dev/null
  start_stack "inverted2:=true inverted4:=true" "[joint3, joint4]"
  # No stop is published: the SIGKILL must land while +2.0 rad/s still stands, or the goal-speed
  # register reads 0 and the wheels have already ramped down (12.5, H7.vel_register/vel_physical).
  # The wheels are stopped immediately afterwards by vel_stop, by scenario_end and by the trap.
  spin vel_spin "2.0,2.0" 6.0
  stop_stack KILL          # killed while the wheels still turn: the raw slope is the evidence
  readback vel_readback --read-only --registers --ids 3,4 --samples 21 --interval-ms 50 > /dev/null
  readback vel_stop --ids 3,4 > /dev/null
  start_stack "inverted2:=true inverted4:=true" "[joint3, joint4]"
  spin load_step "6.0,6.0" 2.0 "0.0,0.0" 3.0 --djs
  stop_stack TERM
  scenario_end
}

h_H8() {
  scenario_begin H8 || return $?
  # max_accel is rad/s^2 and the ACC register is 100 steps/s^2 per count (include/units.hpp:66),
  # so the two values that land on 10 and 150 counts are 10*100*2pi/4096 and 150*100*2pi/4096.
  local slow="speed1:=1.0 speed3:=2.0 speed4:=9.2038847 accel4:=1.5339808"
  local fast="speed1:=4.0 speed3:=9.2038847 speed4:=9.2038847 accel4:=23.0097118"
  local stack wheel mover
  for stack in slow fast; do
    if [ "$stack" = slow ]; then
      start_stack "$slow" "[joint3, joint4]"
      wheel=8.0        # 4x joint3's 2.0 rad/s clamp
    else
      start_stack "$fast" "[joint3, joint4]"
      # joint3's own clamp, which is also the wheel command interface's declared max and the
      # 6000-count register ceiling: lround(9.2038847 * 651.8986) == 6000 either way, so the
      # register row of 12.5 does not need a command outside the interface to reach it
      wheel=9.2038847
    fi
    move "prep_$stack" joint1 0.0 2.0
    # 1.0 rad in 0.2 s (5 rad/s), so the servo's goal-speed clamp limits the move in both stacks.
    # The recording runs 2.5 s past the trajectory: H8.speed_arm allows t_slow up to 1.60 s, and
    # settle() reports NaN if the last sample is still outside the band.
    move "settle_$stack" joint1 1.0 0.2 2.5
    [ "$stack" = slow ] && spin slow_wheel "8.0,0.0" 4.0 "0.0,0.0" 3.0
    spin "t90_$stack" "0.0,3.0" 5.0 "0.0,0.0" 3.0
    # 12.5 reads the registers after a SIGKILL *mid-motion*: the driver writes the goal speed it
    # is pacing right now, so joint1 has to be inside a trajectory that demands more than its
    # max_speed (2.4 rad in 0.3 s = 8 rad/s) and joint3 still commanded at its own clamp. The
    # wheel command persists in the controller, so one --once publication is enough.
    r2 15 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
      "{data: [$wheel, 0.0]}" > /dev/null 2>&1
    move "kill_$stack" joint1 -1.4 0.3 3.0 &
    mover=$!
    wait_t_cmd "$SDIR/kill_$stack.stdout" 30
    sleep 0.3
    stop_stack KILL      # the goal-speed and ACC registers stay as the driver wrote them
    # read and stop before waiting on the recorder: nothing is driving the wheels now, and every
    # second between the kill and ${stack}_stop is a second the wheel keeps coasting
    readback "${stack}_readback" --read-only --registers --ids 1,3,4 > /dev/null
    readback "${stack}_stop" --ids 3,4 > /dev/null
    wait $mover 2> /dev/null
  done
  scenario_end
}

h_H9() {
  scenario_begin H9 || return $?
  start_stack "phantom:=true allow_missing:=true nine:=true" "[joint3, joint4, joint5]"
  r2 20 control list_hardware_interfaces > "$SDIR/hardware_interfaces.txt" 2>&1
  # 70 s, raised from 40 (Phase 5). The recording has to OUTLAST the inactive/active cycle, not
  # merely reach it, because the only gate that has ever caught the multi-turn reset of
  # jazzy.md section 6 step 6 is g1c.joint3/g1c.joint4 over this file, and g1c can only see a
  # reset in samples taken AFTER the component came back. The body ahead of the cycle already
  # measures ~38 s on this bench -- 15 s of wheel steps, two arm moves at ~6 s each and a 6 s
  # /diagnostics capture -- so at 40 s the cycle was landing within a couple of seconds of the
  # recorder's own deadline, and on the two archived runs it fitted only by luck. A recorder
  # that stops first does not make g1c fail; it makes g1c PASS over a window that cannot contain
  # the defect, which is the same vacuous-gate failure EXAMPLE_IDS had. H9.cycle_recorded gates
  # that it really did outlast the cycle, so this number cannot quietly become too small again.
  $RECORD record --duration 70 --djs --label interfaces --out "$SDIR/interfaces.json" &
  local recorder=$!
  sleep 5
  fact t_plus0 "$(now)"
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [3.0, 3.0, 0.0]}" > /dev/null 2>&1
  sleep 5
  fact t_plus1 "$(now)"
  fact t_minus0 "$(now)"
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [-3.0, -3.0, 0.0]}" > /dev/null 2>&1
  sleep 5
  fact t_minus1 "$(now)"
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0, 0.0]}" > /dev/null 2>&1
  move arm_fast joint1,joint2 0.5,0.5 1.0
  move arm_back joint1,joint2 0.0,0.0 1.0
  diagnostics 6
  fact t_cycled "$(now)"
  # The recovery half of jazzy.md section 6 step 6. The cycle itself is not new -- H9 has run it
  # since Phase 2 -- but until Phase 5 both transition results were thrown away and nothing after
  # the cycle was recorded, so the only consequence anything gated was the wheel-position
  # continuity g1c reads out of the recording above. The three facts below are what let
  # hil_gates.py h9() gate the other half: that the component came BACK.
  # stdout is chatter and goes to /dev/null; stderr is kept, per transition, in the scenario
  # directory. It used to go to /dev/null too, and that threw away the only sentence that says
  # WHICH half of a failed cycle failed: ros2controlcli prints the service name and the returned
  # `ok: false` / message on stderr, and cycle_inactive_rc / cycle_active_rc are bare exit codes
  # that cannot distinguish "the service was never there" from "the component refused the
  # transition". Without these two files a red H9.cycle_state has no explanation anywhere in the
  # run tree. The redirection is on the r2 wrapper, so `$?` on the next line is still r2's own
  # status (r2 returns the timeout/ros2 status at r2:139) and the recorded facts do not change.
  r2 30 control set_hardware_component_state bench inactive \
    > /dev/null 2> "$SDIR/cycle_inactive.stderr"
  fact cycle_inactive_rc $?
  sleep 1
  r2 30 control set_hardware_component_state bench active \
    > /dev/null 2> "$SDIR/cycle_active.stderr"
  fact cycle_active_rc $?
  # Taken after the transition rather than before it, because H9.cycle_recorded counts the
  # samples that follow it: those, and only those, are the ones g1c can see a reset in.
  fact t_cycle_done "$(now)"
  fact hw_state_after_cycle "$(hw_state)"
  # An `active` label is the controller manager's opinion of its own state machine; it is not
  # evidence that the bus came back. These two moves are that evidence, and they are the reason
  # the recorder above was lengthened: a component that reports active but whose write path is
  # dead answers this row and nothing else. 0.4 rad is inside the arm's <command_interface>
  # min/max on every bench render, and the park that follows leaves the arm where the next
  # scenario expects to find it (the discipline H12's park_final and H11's soak_park keep).
  move arm_after_cycle joint1,joint2 0.4,0.4 1.0
  move arm_after_park joint1,joint2 0.0,0.0 1.0
  wait $recorder
  stop_stack TERM
  scenario_end
}

h_H10() {
  scenario_begin H10 || return $?
  start_stack "" "[joint3, joint4]"
  $RECORD record --duration 20 --label shutdown --out "$SDIR/shutdown.json" &
  local recorder=$!
  sleep 1
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}" \
    > /dev/null 2>&1
  sleep 3
  fact transitions "$(now)"
  local cm
  cm=$(pgrep -P "$CM_WRAP" 2> /dev/null)
  [ -n "$cm" ] && kill -INT "$cm" 2> /dev/null
  wait "$CM_WRAP" 2> /dev/null
  fact exit_code $?
  wait $recorder
  kill_tagged TERM "^$RSP_BIN"
  sleep 1.5
  readback after_exit --read-only --ids 3,4 > /dev/null
  wait_port_free 10 && fact port_free_within_10s true || fact port_free_within_10s false
  fact port_holders_after_exit "$(port_holders)"
  probe probe_after_exit
  # The second stimulus of 12.5: SIGTERM, not SIGINT, to a bare ros2_control_node, again with a
  # wheel turning. The launch's own SIGINT teardown is the one H1 exercises, against the packaged
  # default port which that scenario deliberately does not override, so it is not repeated here.
  start_stack "" "[joint3, joint4]"
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 1.0]}" \
    > /dev/null 2>&1
  sleep 3
  cm=$(pgrep -P "$CM_WRAP" 2> /dev/null)
  [ -n "$cm" ] && kill -TERM "$cm" 2> /dev/null
  wait "$CM_WRAP" 2> /dev/null
  fact term_exit_code $?
  kill_tagged TERM "^$RSP_BIN"
  sleep 1.5
  readback term_after_exit --read-only --ids 3,4 > /dev/null
  wait_port_free 10 && fact term_port_free true || fact term_port_free false
  scenario_end
}

# H11: the soak of PHASE3 5.13. Ten minutes of the real Phase 3 cycle -- one sync read of four
# servos, one position sync write for the arm, one speed sync write for the wheels -- so the
# failed-transaction rate of jazzy.md item 3 is produced by this harness and not by a hand-run
# probe. The load matches probe 3 Q1 (100 Hz, 4 read, 2+2 written) except that the wheels turn:
# a stationary bus is not the bus a robot runs on, and probe 3 Q2 showed the timeout floor is
# measured under load. That the wheels really did turn is not assumed: h11's wheels_turning row
# reads it back out of the recording, because a publish that never matched a subscriber would
# otherwise buy a clean failed-transaction rate on a load 5.13 forbids. Only the last 20 s are
# recorded: the counters, not the samples, are the measurement, and a ten-minute /joint_states
# capture is tens of MB for nothing.
#
# THE RUNTIME BUDGET, against ctest's TIMEOUT 2400 (CMakeLists.txt:350). The baseline is not the
# 998 s of PHASE3 5.17 any more -- that arithmetic (998 + 600 + 40 = 1638) predates Phase 4. The
# measurement to reason from is the archived post-Phase-4 full run, which took 26m58s = 1618 s
# with SOAK_S at its 600 s default
# (phase5_evidence/post_phase4_baseline_2026-09-21_1659/hil_check.txt, summary line).
#
# Phase 5 spends some of the remaining margin in three places. All three are ESTIMATES, not
# measurements -- none has run on the bench -- so the first real run should replace them with
# observed begin/end deltas out of run.log. Each is built from segments that WERE measured in
# that archived run, and the segment is named so the estimate can be checked:
#   H1B, new: a second example.launch.py startup, two arm moves, a 6 s spin with its stop, the
#     SIGINT teardown and a readback/probe pair. ~75-95 s, kept as first written. The one measured
#     anchor is H1, the same launch idling: 36.4 s begin-to-end (16:59:35.910 -> 17:00:12.336),
#     which includes a 9 s recording and a 12 s /diagnostics capture H1B does not take and
#     excludes H1B's 8 s of moves, 9 s of spin-and-stop and its readback/probe pair (the probe
#     defaults to --hold 0, port_probe.cpp:47, so it costs nothing to speak of). That puts H1B
#     nearer 40 s than 95, so this bullet is the conservative end of the budget, not the likely
#     one; it stays until a run measures it.
#   H9, changed: the interfaces recording goes 40 s -> 70 s and the body gains two post-cycle arm
#     moves, and the body ends on `wait $recorder`. Which of the two sets the length flips here.
#     In the archived run the stack lived 56 s (17:14:36.011 -> 17:15:32.403): the recorder
#     started ~12 s in, ended at ~52 s, and the body was still running, so the wait cost almost
#     nothing. At 70 s the recorder ends at ~82 s while the body -- 44 s of steps, moves and
#     /diagnostics after the recorder starts, the two new moves included -- ends at ~56 s, so the
#     recorder now sets the length and the scenario runs ~86 s. ~+30 s.
#   H12, new: THREE bring-ups, four arm moves, a 6 s spin and four readbacks of its own (plus
#     scenario_begin's and scenario_end's, which every scenario pays). H7 is the scenario of the
#     same shape, and its three segments were measured at 12.3 s (stack + one 2 s move +
#     KILL stop + one readback), 22.1 s (stack + 6.5 s spin + KILL stop + two readbacks) and
#     19.8 s (stack + spin + TERM stop + scenario_end). H12's stack 1 and stack 3 are that first
#     shape, ~12 s each; stack 2 is the second shape plus two arm moves, ~31 s; scenario_begin
#     and scenario_end add ~5 s. ~60-80 s, the spread being the 9-12 s that one start_stack
#     itself varied by across H7's three calls.
# So the suite is about 1618 + 75 + 30 + 60 = 1783 s at the low end and 1618 + 95 + 30 + 80 =
# 1823 s at the high end: call it ~1780-1825 s, i.e. 575-620 s of margin under TIMEOUT 2400. (The
# figure this comment carried before Phase 5's review -- ~1720 s and ~680 s of margin -- had H12
# missing from it entirely and took H1B at its low end.) Still ample; not ample enough to keep
# adding scenarios without re-measuring.
#
# The SOAK_S rule follows from that margin: WAVESHARE_HIL_SOAK_S is 600 above, and raising it adds
# its own difference second for second. At 900 the run is ~2080-2125 s, 275-320 s inside the
# timeout, so 900 is the last value that fits -- above it, raise ctest's TIMEOUT 2400 with it.
h_H11() {
  scenario_begin H11 || return $?
  # The tail of this function -- the park move, the 20 s recording, the 12 s diagnostics capture
  # and the 2 s stop settle -- is about 38 s of the soak, so only the remainder is idled. Clamp
  # at 0: `sleep -10` is an error, not a short sleep, and a SOAK_S under 40 would otherwise run
  # the whole scenario with no soak in it and no sign that anything went wrong. 40 is also the
  # floor the variable itself is clamped to, below the scenario is not a soak at all.
  local soak=${WAVESHARE_HIL_SOAK_S:-600}
  [ "$soak" -ge 40 ] 2> /dev/null || soak=40
  local idle=$((soak - 38))
  [ "$idle" -lt 0 ] && idle=0
  # The stack's own watchdog must outlast the soak, or the controller manager takes a SIGINT
  # mid-idle and there is no driver left to record, to capture /diagnostics from, or to print the
  # totals line. 180 s of slack covers the stack start, the spawner and the teardown.
  start_stack "allow_missing:=false nine:=true" "[joint3, joint4]" $((soak + 180))
  fact soak_s "$soak"
  move soak_park joint1,joint2 0.0,0.0 2.0
  # Logged, unlike every other wheel publish in this file: this one IS the soak's load. If it
  # times out waiting for a matching subscription the ten minutes still run, on a stopped bus,
  # which is not the load PHASE3 5.13 specifies. h11's wheels_turning row is what FAILs the run;
  # this line is what tells the reader why, in run.log, without reading the recording.
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [1.0, 1.0]}" > /dev/null 2>&1 || hil_log "soak wheel command rc=$?"
  sleep "$idle"
  $RECORD record --duration 20 --djs --label steady_soak --out "$SDIR/steady_soak.json"
  diagnostics 12
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0]}" > /dev/null 2>&1 || hil_log "soak wheel stop rc=$?"
  sleep 2
  stop_stack TERM          # TERM, never KILL: on_deactivate prints the totals line (5.14)
  scenario_end             # ... and scenario_end stops the wheels again and proves the port free
}

# H12: jazzy.md section 6 step 8, the only step of the hardware recipe no other scenario touches.
# It runs the bench stack once with enforce_command_limits: true (test/hil/controllers/
# bench_limits.yaml) and proves that the controller manager's JointSaturationLimiter -- not the
# driver -- clamps a command that lies beyond the joint's <limit>.
#
# ATTRIBUTION, which is the whole design of this scenario and the reason it renders anything
# special. The driver clamps too, and it would clamp these same commands: a position command is
# held inside the command interface's own min/max (src/waveshare_servos.cpp:587-617) and a wheel's
# goal speed inside max_speed_counts, 6000 counts = 9.2038847 rad/s when no max_speed param is
# given (include/waveshare_servos.hpp:214, src/waveshare_servos.cpp:1936-1941). A command clamped
# at a number both mechanisms agree on is evidence for neither of them -- that is exactly what H8
# already measures on the driver's side. So the render tightens the two <limit> ceilings and
# leaves the driver's own where they are:
#
#   joint           <limit>          the driver's own ceiling      commanded here
#   joint3/joint4   velocity 2.0     9.2038847 rad/s               8.0 rad/s
#   joint1/joint2   +-0.8            +-1.570796 rad                1.2 rad
#
# Both commands are INSIDE what the driver would pass and OUTSIDE what the description allows, so
# a clamp landing on 2.0 rad/s or 0.8 rad can only have come from the limiter: nothing else in the
# stack knows those two numbers. The driver never reads a URDF <limit> at all -- it parses the
# <param>s and the command interface min/max, and nothing in src/waveshare_servos.cpp touches
# info_.limits -- so the two ceilings cannot be confused even in principle. The rendered proof is
# kept as well, and under its OWN name: this scenario runs three stacks and start_stack re-renders
# $SDIR/robot.urdf and $SDIR/cm.yaml on every call, so the tightened render is copied aside to
# $SDIR/robot_limited.urdf and $SDIR/cm_limited.yaml the moment the limited stack is up. Those two
# files carry the tightened <limit> next to the untouched <command_interface> min/max and the one
# controller YAML with enforce_command_limits on, for whoever reads a red row later; robot.urdf
# itself is whatever the LAST stack rendered, which is the final park's ordinary description.
#
# TRAP (c) of the Phase 4 amendment, and the reason for the first stack. With the flag on, an arm
# servo whose MEASURED position is more than 0.0087 rad outside its <limit>
# (joint_limits_helpers.hpp:32, OUT_OF_BOUNDS_EXCEPTION_TOLERANCE = 0.0087, "0.5 degrees") makes
# compute_position_limits throw, and the controller manager deactivates the arm controller instead
# of clamping anything -- a failure with nothing to do with what this scenario tests. So the arm
# is parked first by an ORDINARY stack with the limiters off, and where it actually came to rest
# is read off the servos while the port is free.
#
# That precondition has two halves and they are answered in two different places, because only
# one of them is something the shell can act on:
#   - did the park stack come up and drive the arm at all. The shell records park_spawner_rc,
#     park_controllers_active and park_cm_exit_code and, if that stack did not come up, ABORTS
#     before the limited stack ever starts (see the guard below for why an abort and not a FAIL).
#     Without it the park's evidence was not even retrievable: move() swallows a failed move into
#     a run.log line, and the second start_stack overwrites the shared spawner_rc /
#     controllers_active facts, so a silently failed park left the scenario looking healthy.
#   - where the arm actually rests, which no amount of guarding can force. That is the arm_pre
#     readback below, taken with the port free, and H12.arm_inside_limits is that reading: the one
#     precondition this scenario cannot control reports itself in its own row.
h_H12() {
  scenario_begin H12 || return $?
  # The stimulus, recorded rather than implied: hil_gates.py holds the same four numbers as
  # constants (hil_gates.py:760-763 -- H12_ARM_LIMIT, H12_ARM_COMMAND, H12_WHEEL_LIMIT,
  # H12_WHEEL_COMMAND, cited by name because that file is edited more often than this one) and
  # H12.stimulus compares every one of them against the fact recorded here, so the script and the
  # gate cannot drift apart silently the way a hard-coded expectation can (H8's register rows).
  # The two limit names, arm_pos_limit and wheel_vel_limit, are also the xacro arguments the
  # limited render is given below (declared at hil/descriptions/bench.urdf.xacro:27-28, whose
  # defaults are the UNtightened 1.570796 and 9.2038847), so a changed ceiling cannot reach the
  # description without reaching the fact the gate reads.
  fact arm_pos_limit 0.8
  fact arm_command 1.2
  fact wheel_vel_limit 2.0
  fact wheel_command 8.0
  # 1. park the arm inside the tightened limit with the limiters OFF, then read where it rests.
  #    The stack goes away first: nothing may hold the port during a readback (12.4 item 9).
  #    Both globals are cleared before the call because start_stack can return without setting
  #    either of them -- the xacro render and the rsp params file each `return 1` ahead of the
  #    spawner -- and H12 is the fourteenth scenario of the run, so the values H10's last stack
  #    left behind (spawner_rc=0, controllers_active=true) would read as a park that worked.
  SPAWNER_RC=
  CONTROLLERS_ACTIVE=
  start_stack "" "[joint3, joint4]"
  move park joint1,joint2 0.0,0.0 2.0
  stop_stack TERM
  # The park stack's own evidence, under keys of its own. start_stack and stop_stack write the
  # shared spawner_rc / controllers_active / cm_exit_code facts, and the two stacks below
  # overwrite them, so without this copy the park left no trace at all: move() ends in
  # `|| hil_log "move $1 rc=$?"` and swallows a failed park into one run.log line.
  fact park_spawner_rc "$SPAWNER_RC"
  fact park_controllers_active "$CONTROLLERS_ACTIVE"
  fact park_cm_exit_code "$CM_EXIT_CODE"
  # arm_pre keeps its label and its place: read with the port free, after the park stack is gone,
  # and BEFORE the guard below, so even an aborted H12 records where the arm was actually left.
  readback arm_pre --read-only --registers --ids 1,2 > /dev/null
  # The other half of the trap-(c) precondition, guarded the way h_H1B guards its own
  # (`if [ "$state" = active ]` before the wheel command): if the park stack never came up, the
  # arm was never driven and the limited stack would start with the arm wherever the previous
  # scenario left it. Outside the tightened +-0.8 <limit> that is not a clamp test at all --
  # compute_position_limits throws and the controller manager deactivates the arm controller, so
  # every clamp row of this scenario would go red for a reason that has nothing to do with the
  # limiter.
  #
  # ABORTED and not FAIL, because the two verdicts mean different things and only one of them is
  # true here: a FAIL says the code under test did the wrong thing, and this says the bench never
  # managed to ask it the question. An ABORTED row is louder than a FAIL in exactly the right way
  # -- run() returns 2 for any ABORTED row, not 1 ('if counts[...ABORTED...] or not port_free:
  # return 2', hil_gates.py:1905-1906) -- and it cannot be mistaken for a limiter regression.
  #
  # Returning 0 rather than 2 is deliberate. The run loop treats any rc=2 from a scenario as
  # "aborted_all" and aborts every scenario AFTER it with the message "an earlier scenario left
  # the port held by a process this suite may not kill" -- which is about a port this suite must
  # not touch, not about a park that did not run. H12 runs before H11, so returning 2 would cost
  # the ten-minute soak its run and label it with a false reason. Nothing here holds the port:
  # scenario_end below tears any stack down, stops the wheels and proves the port free, exactly
  # as it does on the success path, and the ABORTED row already carries the exit code.
  #
  # What the abort leaves behind, and why it is safe. The wheels are stopped (this scenario has
  # not commanded them yet, and scenario_end commands zero anyway) and the port is free, but the
  # arm is wherever the failed park left it -- which is the very thing being aborted for and
  # cannot be fixed by the stack that just failed to come up. It endangers nothing downstream:
  # the tightened +-0.8 <limit> is never rendered, because the abort happens before the limited
  # stack, and every other scenario renders the xacro's default +-1.570796 against bench.yaml,
  # which sets enforce_command_limits false (bench.yaml:7) -- so there is no limiter to throw,
  # the driver's own command-interface clamp still holds, and H11's soak_park parks the arm again
  # on its way past.
  if [ "$SPAWNER_RC" != 0 ] || [ "$CONTROLLERS_ACTIVE" != true ]; then
    abort_scenario H12 "the park stack did not come up (spawner_rc=[$SPAWNER_RC] \
controllers_active=[$CONTROLLERS_ACTIVE] cm_exit_code=[$CM_EXIT_CODE]); the arm was never parked, \
so the limited stack was not started -- see arm_pre.txt for where the arm actually is"
    scenario_end
    return 0
  fi
  # 2. the same bench stack, rendered with both ceilings tightened and driven by the one
  #    controller YAML that turns the limiters on.
  start_stack "arm_pos_limit:=0.8 wheel_vel_limit:=2.0" "[joint3, joint4]" "" bench_limits.yaml
  # The tightened render and the limiters-on YAML, copied before stack 3 re-renders both names
  # over them, plus the name of this stack's own controller-manager log: STACK_N counts stacks
  # across the whole run, so the limited stack's log is neither the first nor the last cm*.stdout
  # in this directory and the gate should not have to guess which of the three it is. The limiter
  # lines themselves ("Creating JointSaturationLimiter for joint ...") can only appear in this
  # one: bench_limits.yaml:20 sets enforce_command_limits true and bench.yaml:7, which the other
  # two stacks use, sets it false.
  cp "$SDIR/robot.urdf" "$SDIR/robot_limited.urdf"
  cp "$SDIR/cm.yaml" "$SDIR/cm_limited.yaml"
  fact limited_cm_log "$(basename "$CM_LOG")"
  # 2.5 s of post-roll, as H8's settle moves use: the clamp is read off the last sample, so the
  # recording has to outlast the trajectory rather than end inside its final approach.
  move pos_clamp joint1 1.2 2.0 2.5
  # Back inside the limit immediately, while there is still a controller to do it with: with a
  # working limiter the command above clamps at exactly 0.8 rad, and the trap-(c) throw fires
  # 0.0087 rad outside the <limit> -- 5.7 encoder ticks at 2*pi/4096 rad each -- so an arm left
  # sitting ON its limit is a hand-nudge away from breaking every scenario that follows.
  #
  # This move is best-effort and cannot be the whole mitigation: it publishes through the arm
  # controller of the LIMITED stack, and the case it most needs to cover is precisely the case
  # where that controller is already gone (the limiter threw and the controller manager
  # deactivated it), where move() publishes into nothing and logs `move park_back rc=...`. The
  # park that actually guarantees the bench state is park_final below, on an ordinary stack with
  # the limiters off, run unconditionally after this stack is torn down. Both are kept: this one
  # gets the arm off the edge seconds earlier on the healthy path, and it is the move
  # arm_state_after is read after.
  move park_back joint1,joint2 0.0,0.0 2.0
  # After both arm commands, because a limiter throw takes the arm controller down at the moment
  # it enforces, not at activation: start_stack's controllers_active was true either way.
  fact arm_state_after "$(controllers_snapshot | awk '$1=="arm" {print $NF}')"
  # No stop is published (the H7 pattern): the SIGKILL has to land while 8.0 rad/s still stands,
  # or the goal-speed register reads 0 and the clamped value is gone. The wheels are stopped
  # immediately afterwards by vel_stop, again by scenario_end and again by the exit trap.
  spin vel_clamp "8.0,8.0" 6.0
  stop_stack KILL          # the goal-speed registers stay exactly as the driver last wrote them
  # The limited stack's own numbers, under keys of its own, for the same reason the park's are:
  # the final park below runs a third start_stack/stop_stack pair and overwrites the shared
  # spawner_rc, controllers_active, hw_state, port_holders_running, joint_states_seen,
  # cm_exit_code and port_released facts. cm_exit_code here is a SIGKILL death by construction
  # (the line above), so it is recorded as the record of what this scenario did, not as a gate.
  fact limited_spawner_rc "$SPAWNER_RC"
  fact limited_controllers_active "$CONTROLLERS_ACTIVE"
  fact limited_cm_exit_code "$CM_EXIT_CODE"
  readback vel_readback --read-only --registers --ids 3,4 > /dev/null
  readback vel_stop --ids 3,4 > /dev/null
  # 3. the park that is not conditional on anything. Whatever happened above -- the limiter
  #    clamped and park_back worked, or the limiter threw, took the arm controller down and
  #    park_back published into a dead topic -- the arm may still be resting on 0.8 rad, which is
  #    5.7 ticks from the trap-(c) throw and would break the NEXT scenario that renders a limit.
  #    An ordinary stack has the limiters off and the driver's own +-1.570796 rad ceiling, so it
  #    can always drive the arm back to 0.0 from the edge; the limited stack could not be trusted
  #    to. This is the suite's park discipline paid where it cannot be skipped (H11's soak_park,
  #    H9's arm_after_park), and it is cheap: one bring-up, one 2 s move, one teardown and one
  #    readback is the shape H7's first segment measured at 12.3 s in the archived run
  #    (phase5_evidence/post_phase4_baseline_2026-09-21_1659/run.log, 17:12:05.861 -> :18.127).
  #
  #    Its stack facts land under park_final_*, and the shared keys named above are left holding
  #    THIS stack's values because it is the last one to run: read spawner_rc / controllers_active
  #    for H12 and you are reading the final park, not the limited stack.
  SPAWNER_RC=
  CONTROLLERS_ACTIVE=
  start_stack "" "[joint3, joint4]"
  move park_final joint1,joint2 0.0,0.0 2.0
  stop_stack TERM
  fact park_final_spawner_rc "$SPAWNER_RC"
  fact park_final_controllers_active "$CONTROLLERS_ACTIVE"
  fact park_final_cm_exit_code "$CM_EXIT_CODE"
  # Where the arm ended up, with the port free and every stack gone: the counterpart to arm_pre,
  # and the only reading that says the bench was handed back parked rather than on its limit.
  readback arm_post --read-only --registers --ids 1,2 > /dev/null
  scenario_end
}

# ------------------------------------------------------------------ pre-flight and the run
: > "$OUT/run.log"
: > "$OUT/aborted.txt"
hil_log "port=$PORT out=$OUT ws=$WS tag=$HIL_TAG"

prefix=$(ros2 pkg prefix waveshare_servos 2> /dev/null)
case "$prefix" in
  "$WS"/install*) ;;
  *)
    hil_log "waveshare_servos resolves to '$prefix', not inside $WS/install; refusing to run"
    exit 2
    ;;
esac
for binary in "$STOP_WHEELS" "$PORT_PROBE"; do
  [ -x "$binary" ] || { hil_log "missing helper binary '$binary'"; exit 2; }
done
command -v xacro > /dev/null || { hil_log "no xacro on PATH"; exit 2; }

cleanup_all() {
  SCEN=exit
  SDIR=$OUT
  teardown_stack
  if [ -z "$(port_holders)" ]; then
    timeout -s KILL 20 "$STOP_WHEELS" --port "$PORT" --ids 3,4 > "$OUT/final_stop_wheels.txt" 2>&1
    hil_log "final stop_wheels rc=$?"
  else
    hil_log "port still held by [$(port_holders)] at exit"
  fi
  timeout 20 ros2 daemon stop > /dev/null 2>&1
}
trap cleanup_all EXIT
trap 'hil_log "interrupted"; exit 130' INT TERM

timeout 20 ros2 daemon stop > /dev/null 2>&1
readback initial_readback --read-only --registers --ids 1,2,3,4 > /dev/null

# H11 goes last so its ten minutes are spent only after every fast row has reported, and so an
# abort in it costs nothing else (PHASE3 5.13). That is why H12 runs before it and not after it,
# even though the list then stops being in numeric order: the number is a name, the position is a
# cost decision, and H12's three short stacks are an estimated 60-80 s (the budget block above
# h_H11 shows the arithmetic), which is worth spending before the soak rather than after it.
#
# The report is NOT printed in this order, and nothing keeps the two orders in step. hil_gates.py's
# run() walks its own CHECKERS tuple and prints the rows in THAT order (`for name, checker in
# CHECKERS` at hil_gates.py:1880, the tuple at :1831-1833, which today is numeric); a scenario's
# position in the line below is a cost decision and says nothing about where its rows land in
# hil_check.txt. What the two lists do have to agree on is their MEMBERSHIP, and that is checked
# rather than assumed: a scenario that ran on the bench and that no checker in CHECKERS claims
# produces a FAIL row naming its directory (unchecked_rows(), hil_gates.py:1554-1580) -- the row
# that exists because H12 was added here, given a controller YAML and its constants, and gated by
# nothing at all.
SCENARIOS=${WAVESHARE_HIL_SCENARIOS:-"H1 H1B H2 H3 H4 H5A H5B H5C H6 H7 H8 H9 H10 H12 H11"}
aborted_all=0
for s in $SCENARIOS; do
  SCEN=run
  SDIR=$OUT
  if [ "$aborted_all" = 1 ]; then
    abort_scenario "$s" "an earlier scenario left the port held by a process this suite may not kill"
    continue
  fi
  hil_log "=== $s ==="
  "h_$s"
  rc=$?
  SCEN=run
  SDIR=$OUT
  hil_log "$s finished rc=$rc"
  [ "$rc" = 2 ] && aborted_all=1
done

SCEN=report
SDIR=$OUT
port_free=$([ -z "$(port_holders)" ] && echo true || echo false)
elapsed=$((SECONDS - STARTED))
$GATES --run-dir "$OUT" --allow-inconclusive "$HIL_INCONCLUSIVE_ALLOWED" \
  --seconds "$((elapsed / 60))m$((elapsed % 60))s" --port-free "$port_free" |
  tee "$OUT/hil_check.txt"
rc=${PIPESTATUS[0]}
hil_log "report written to $OUT/hil_check.txt and $OUT/hil_check.json (rc=$rc)"
exit "$rc"
