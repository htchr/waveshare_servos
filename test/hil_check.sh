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
# Every other scenario is far inside 420 -- H9's 40 s recording is the longest.
start_stack() {
  local watchdog=${3:-420}
  # port:= first, so a scenario argument can still override it deliberately
  xacro "$HELPERS/descriptions/bench.urdf.xacro" "port:=$PORT" $1 > "$SDIR/robot.urdf" || return 1
  sed "s/@WHEELS@/$2/" "$HELPERS/controllers/bench.yaml" > "$SDIR/cm.yaml"
  hil_log "starting ros2_control_node ($1)"
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
  local rsp
  rsp=$(pgrep -P "${RSP_WRAP:-0}" 2> /dev/null)
  [ -n "$rsp" ] && kill -TERM "$rsp" 2> /dev/null
  wait "$RSP_WRAP" 2> /dev/null
  wait_port_free 10 && fact port_released true || fact port_released false
}

move() {  # label joints positions duration [post]
  $RECORD move --controller arm --joints "$2" --positions="$3" --duration "$4" --pre 0.5 \
    --post "${5:-1.5}" --label "$1" --out "$SDIR/$1.json" > "$SDIR/$1.stdout" 2>&1 ||
    hil_log "move $1 rc=$?"
}

# An empty stop-values publishes no stop at all, which is what the scenarios that SIGKILL the
# stack mid-command need: the controller holds the last command it was given (12.5, H7 and H8).
spin() {  # label values hold [stop-values [stop-hold]] [--djs]
  local stop=()
  [ -n "${4:-}" ] && stop=(--stop-values "$4" --stop-hold "${5:-2.0}")
  $RECORD vel --topic /wheels/commands --values="$2" --hold "$3" "${stop[@]}" ${6:-} \
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
  # H1 is the shipped entry point. example.launch.py now has a port argument, but this scenario
  # deliberately does not pass one, so it always exercises the packaged default. Running it against
  # another adapter would silently test the wrong port, so say so and skip instead. (Passing
  # port:=$PORT and dropping this skip is a Phase 5 change.)
  if [ "$PORT" != /dev/ttyACM0 ]; then
    abort_scenario H1 "example.launch.py drives its packaged default port, not $PORT"
    return 0
  fi
  scenario_begin H1 || return $?
  timeout -s INT 420 ros2 launch waveshare_servos example.launch.py gui:=false \
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
  $RECORD record --duration 40 --djs --label interfaces --out "$SDIR/interfaces.json" &
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
  r2 30 control set_hardware_component_state bench inactive > /dev/null 2>&1
  sleep 1
  r2 30 control set_hardware_component_state bench active > /dev/null 2>&1
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
# If WAVESHARE_HIL_SOAK_S is ever raised above 900, raise ctest's TIMEOUT 2400 with it
# (CMakeLists.txt:278) -- see PHASE3 5.17 for the arithmetic: the measured full-suite duration is
# 998 s, so 998 + 600 + 40 = 1638 s against a 2400 s ctest timeout, i.e. 762 s of margin.
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
# abort in it costs nothing else (PHASE3 5.13).
SCENARIOS=${WAVESHARE_HIL_SCENARIOS:-"H1 H2 H3 H4 H5A H5B H5C H6 H7 H8 H9 H10 H11"}
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
