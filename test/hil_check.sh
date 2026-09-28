#!/usr/bin/env bash
# Bench check (ctest `hil_check`): drives the real servos; skips (exit 77) unless WAVESHARE_HIL=1.
# See docs/bench-check.md, "Run the bench check".

# Signal only processes tagged with this run's HIL_TAG: never pkill -f ros2, never fuser -k,
# never a bare kill on a pgrep result.

PORT=${WAVESHARE_HIL_PORT:-/dev/ttyACM0}

HELPERS=${WAVESHARE_HIL_HELPERS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/hil" && pwd)}
STOP_WHEELS=${WAVESHARE_HIL_STOP_WHEELS:-}
PORT_PROBE=${WAVESHARE_HIL_PORT_PROBE:-}
OUT=${WAVESHARE_HIL_OUT:-$PWD/hil_check.d}
WS=${WAVESHARE_HIL_WS:-$(cd "$HELPERS/../../../.." && pwd)}
# Forward each of these through the env -i re-exec below, or it is lost there.
# TOOLS stays empty unless given; the pre-flight fills in the install tree.
EEPROM=${WAVESHARE_HIL_EEPROM:-}
JOURNAL=${WAVESHARE_HIL_JOURNAL:-$HOME/.local/state/waveshare_servos/hil_eeprom_journal.snap}
# The port the journal was taken on; the pre-flight applies a journal only on that port.
# See docs/bench-check.md, "EEPROM journal".
JOURNAL_PORT=$JOURNAL.port
BASELINE=${WAVESHARE_HIL_EEPROM_BASELINE:-}
TOOLS=${WAVESHARE_HIL_TOOLS:-}
JOURNAL_OURS=0          # 1 from the moment guard_begin writes the journal, 0 once it is removed

# Only these rows may be INCONCLUSIVE (the bench cannot always supply their stimulus).
# Any other INCONCLUSIVE row is reported as FAIL.
HIL_INCONCLUSIVE_ALLOWED="H7.load_sign H8.accel_effect"

# Skip checks and the clean-env re-exec run only in the outer call: inside, ros2 is not on PATH
# until setup.bash is sourced, so the checks would skip every real run.
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
    WAVESHARE_HIL_EEPROM="$EEPROM" WAVESHARE_HIL_JOURNAL="$JOURNAL" \
    WAVESHARE_HIL_EEPROM_BASELINE="$BASELINE" WAVESHARE_HIL_TOOLS="$TOOLS" \
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

# pids with an open fd on the port (fuser/lsof are not installed on the bench)
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

# only processes carrying this run's HIL_TAG are ever signalled
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

# The tools and hil_eeprom are here so port_rescue can stop them if they hang (tagged only).
# Binaries from a WAVESHARE_HIL_TOOLS override do not match the tools pattern.
STACK_PATTERNS=("^$CM_BIN" "^$RSP_BIN" "ros2 launch waveshare_servos" "controller_manager/spawner"
  "hil_record.py" "lib/waveshare_servos/(scan|set_id|calibrate_midpoint)" "hil_eeprom")

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

# ros2 CLI under a timeout, without the "waiting for service" chatter
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

# Never signal a port holder without this run's HIL_TAG: log it and stop.
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
  # A rescue is not a clean start: port_free_before is a gated row, so it records false.
  if [ "$rescued" = 1 ]; then
    fact port_free_before false
  else
    fact port_free_before true
  fi
  # Read the registers once per scenario, before any move, while the port is free.
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

# start_stack XACRO_ARGS WHEELS [WATCHDOG_S=420] [YAML=bench.yaml]. The watchdog SIGINTs a stack
# that outlives its scenario. YAML is an argument, not a global, for H12's limiters only.
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
  # The URDF goes in a params FILE: rcl parses a -p value as YAML, and a multi-line XML
  # document aborts the node.
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
  # Also a global: a scenario with several stacks (H12) copies it under its own fact key
  # before the next stop_stack overwrites cm_exit_code.
  CM_EXIT_CODE=$rc
  local rsp
  rsp=$(pgrep -P "${RSP_WRAP:-0}" 2> /dev/null)
  [ -n "$rsp" ] && kill -TERM "$rsp" 2> /dev/null
  wait "$RSP_WRAP" 2> /dev/null
  wait_port_free 10 && fact port_released true || fact port_released false
}

# $6 = controller (default arm; H1B: joint_trajectory_position_controller). An argument, not a
# global, so one scenario cannot retarget the next one's moves.
move() {  # label joints positions duration [post] [controller]
  $RECORD move --controller "${6:-arm}" --joints "$2" --positions="$3" --duration "$4" --pre 0.5 \
    --post "${5:-1.5}" --label "$1" --out "$SDIR/$1.json" > "$SDIR/$1.stdout" 2>&1 ||
    hil_log "move $1 rc=$?"
}

# Empty stop-values publish no stop (H7 and H12 SIGKILL the stack while the command stands).
# $7 = topic (default /wheels/commands; H1B: /joint_velocity_controller/commands).
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

diagnostics() {  # capture /diagnostics through the CLI (hil_record.py does not subscribe)
  r2 "${1:-12}" topic echo /diagnostics > "$SDIR/diagnostics.txt" 2>&1
  return 0
}

# ------------------------------------------------------------------ tools and EEPROM

# tool LABEL NAME ARGV...: output goes to *.out.txt/*.err.txt, not *.stdout (log.txt collects
# those). Every fact is written on every path, so a missing fact never reads as a pass.
tool() {
  local label=$1 name=$2 t0 rc
  shift 2
  if [ ! -x "$TOOLS/$name" ]; then
    : > "$SDIR/$label.out.txt"
    echo "no binary $TOOLS/$name" > "$SDIR/$label.err.txt"
    fact "${label}_rc" 127
    fact "${label}_seconds" 0
    fact "${label}_serial_speed_lines" none       # not a number: every gate reading it FAILs
    fact "${label}_holders_after" "$(port_holders)"
    return 127
  fi
  t0=$(now)
  timeout -k 5 -s INT 60 "$TOOLS/$name" "$@" > "$SDIR/$label.out.txt" 2> "$SDIR/$label.err.txt"
  rc=$?
  fact "${label}_rc" "$rc"
  fact "${label}_seconds" "$(python3 -c "print('%.3f' % ($(now) - $t0))")"
  fact "${label}_serial_speed_lines" \
    "$(cat "$SDIR/$label.out.txt" "$SDIR/$label.err.txt" | grep -c 'serial speed')"
  fact "${label}_holders_after" "$(port_holders)"
  return $rc
}

# $1 = label, rest = hil_eeprom arguments; refuses while the port is held, as readback does
eeprom() {
  local label=$1 rc
  shift
  local lim=(-s KILL 60)
  [ "$1" = restore ] && lim=(-k 30 -s TERM 90)   # restore defers TERM to the end of a sequence
  if [ -n "$(port_holders)" ]; then
    echo '{"ok": false, "error": "port_busy_skipped"}' > "$SDIR/$label.json"
    fact "${label}_rc" busy
    return 1
  fi
  timeout "${lim[@]}" "$EEPROM" --port "$PORT" "$@" > "$SDIR/$label.txt" 2>&1
  rc=$?
  sed -n 's/^RESULT //p' "$SDIR/$label.txt" | tail -1 > "$SDIR/$label.json"
  [ -s "$SDIR/$label.json" ] || echo '{"ok": false, "error": "no_result"}' > "$SDIR/$label.json"
  fact "${label}_rc" "$rc"
  return $rc
}

eeprom_offline() {  # compare only: opens no port, so no port check
  local label=$1 rc
  shift
  timeout -s KILL 30 "$EEPROM" "$@" > "$SDIR/$label.txt" 2>&1
  rc=$?
  sed -n 's/^RESULT //p' "$SDIR/$label.txt" | tail -1 > "$SDIR/$label.json"
  [ -s "$SDIR/$label.json" ] || echo '{"equal": false, "error": "no_result"}' > "$SDIR/$label.json"
  fact "${label}_rc" "$rc"
  return $rc
}

snap_is_bench() {  # $1 = snapshot RESULT json. True only for a complete snapshot of the bench.
  python3 - "$1" << 'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
ids = d.get('ids', {})
ok = (d.get('ok') is True and d.get('census') == [1, 2, 3, 4]
      and all(ids.get(str(i), {}).get('n') == 39 for i in (1, 2, 3, 4)))
sys.exit(0 if ok else 1)
EOF
}

# guard_begin journals the pre snapshot. Returns 0 go on, 1 abort this scenario, 2 abort the run.
# See docs/bench-check.md, "EEPROM journal".
guard_begin() {
  local why= rc
  if [ -e "$JOURNAL" ]; then
    ABORT_REASON="journal $JOURNAL exists and is not this scenario's; an unknown journal is never \
overwritten -- the next run's pre-flight restores it, or see it by hand"
    abort_scenario "$SCEN" "$ABORT_REASON"
    return 2
  fi
  eeprom pre_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/pre_eeprom.snap"
  rc=$?
  if [ "$rc" != 0 ]; then
    why="the pre_eeprom snapshot failed (rc $rc; see pre_eeprom.txt, or port_busy_skipped in \
pre_eeprom.json)"
  elif ! snap_is_bench "$SDIR/pre_eeprom.json"; then
    why="pre_eeprom is not a complete snapshot of the bench (ok, census exactly 1 2 3 4, 39 \
readable EEPROM bytes each)"
  elif [ ! -s "$SDIR/pre_eeprom.snap" ] || grep -Eq '(^| )x( |$)' "$SDIR/pre_eeprom.snap"; then
    why="pre_eeprom.snap is empty or holds an unreadable byte"
  else
    # Mark the journal ours before writing it, so an interrupt here still restores it.
    # The port record goes first: a journal never exists without one.
    JOURNAL_OURS=1
    if ! { mkdir -p "$(dirname "$JOURNAL")" && printf '%s\n' "$PORT" > "$JOURNAL_PORT" &&
      cp "$SDIR/pre_eeprom.snap" "$JOURNAL.tmp" && mv "$JOURNAL.tmp" "$JOURNAL"; }; then
      rm -f "$JOURNAL.tmp" "$JOURNAL_PORT"
      JOURNAL_OURS=0
      why="could not write the journal $JOURNAL"
    elif ! cmp -s "$SDIR/pre_eeprom.snap" "$JOURNAL"; then
      rm -f "$JOURNAL" "$JOURNAL_PORT"   # written a moment ago by this function; no tool has run
      JOURNAL_OURS=0
      why="the journal $JOURNAL does not read back as pre_eeprom.snap"
    fi
  fi
  if [ -n "$why" ]; then
    abort_scenario "$SCEN" "guard_begin: $why; no tool was called"
    return 1
  fi
  fact journal_written true
  return 0
}

# $1 = full (EEPROM, 40, 55 and census) or eeprom_only (H14, whose stack legitimately changes SRAM
# 40 and 55). Called on EVERY return path after a successful guard_begin.
guard_end() {
  local mode=()
  [ "${1:-full}" = eeprom_only ] && mode=(--eeprom-only)
  eeprom post_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/post_eeprom.snap"
  if eeprom_offline guard_compare compare "${mode[@]}" "$SDIR/pre_eeprom.snap" \
    "$SDIR/post_eeprom.snap"; then
    rm -f "$JOURNAL" "$JOURNAL_PORT"
    JOURNAL_OURS=0
    fact journal_left false
    return 0
  fi
  # No --allow-regs: the journal is this scenario's own, so every difference is ours to undo.
  # The scenario's rows still compare against the first post snapshot and still FAIL.
  hil_log "the bench EEPROM differs from pre_eeprom; restoring from the journal"
  eeprom guard_restore restore --from "$JOURNAL"
  eeprom post2_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/post2_eeprom.snap"
  if eeprom_offline guard_compare2 compare "${mode[@]}" "$SDIR/pre_eeprom.snap" \
    "$SDIR/post2_eeprom.snap"; then
    rm -f "$JOURNAL" "$JOURNAL_PORT"
    JOURNAL_OURS=0
    fact journal_left false
    return 0
  fi
  fact journal_left true
  ABORT_REASON="bench EEPROM not restored; journal $JOURNAL kept"
  abort_scenario "$SCEN" "$ABORT_REASON"
  return 2
}

holder_alive() {  # $1 = a pid announced by a HOLDING line; true while that process still lives
  [ -n "$1" ] && [ -d "/proc/$1" ] && echo true || echo false
}

# ------------------------------------------------------------------ scenarios
h_H1() {
  # H1: the shipped example.launch.py, idle; gui:=false because the bench has no display.
  # See docs/bench-check.md, "Example stack (H1 and H1B)".
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

# H1B: the shipped example, commanded. Arm first; wheels only if the component is still active.
# See docs/bench-check.md, "Example stack (H1 and H1B)".
h_H1B() {
  scenario_begin H1B || return $?
  timeout -s INT 420 ros2 launch waveshare_servos example.launch.py "port:=$PORT" gui:=false \
    > "$SDIR/launch.stdout" 2>&1 &
  LAUNCH_WRAP=$!
  wait_controllers_active 90 joint_state_broadcaster joint_trajectory_position_controller \
    joint_velocity_controller && fact controllers_active true || fact controllers_active false
  fact hw_state "$(hw_state example_ws_ros2_control)"
  # diff_drive_controller must be loaded but inactive (it claims the wheel command interfaces);
  # an empty state means absent, and H1B.diff_drive fails on both.
  fact diff_drive_state "$(controllers_snapshot | awk '$1=="diff_drive_controller" {print $NF}')"
  $RECORD wait_js --timeout 20 > /dev/null
  # The arm, as H2 moves it: same targets and 2 s, so H1B.target and H2.target compare
  # directly.
  move ex_move_to_0 joint1 0.0 2.0 1.5 joint_trajectory_position_controller
  move ex_move_to_06 joint1 0.6 2.0 1.5 joint_trajectory_position_controller
  fact arm_state_after \
    "$(controllers_snapshot | awk '$1=="joint_trajectory_position_controller" {print $NF}')"
  local state
  state=$(hw_state example_ws_ros2_control)
  fact hw_state_after_arm "$state"
  # The wheels, as H3: 2.0 rad/s for 6 s, then a stop. Skipped if the component died, so no
  # wheel is left on a latched goal speed with no process to stop it.
  if [ "$state" = active ]; then
    fact vel_attempted true
    spin ex_vel_2 "2.0,2.0" 6.0 "0.0,0.0" 2.5 "" /joint_velocity_controller/commands
  else
    fact vel_attempted false
    hil_log "component is '$state' after the arm move; skipping the wheel command"
  fi
  # The launch's own SIGINT teardown, after an arm and a wheel moved (H10 covers a bare node).
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
  start_stack "" "[joint3, joint4]"          # the wheels stay at 0 throughout H6
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
    # Baseline per holder, just before it and with the port free (H6's active stack above
    # legitimately wrote the goal registers).
    wait_port_free 15
    readback "${kind}_pre" --read-only --registers --ids 1,2,3,4 > /dev/null
    timeout -s KILL 60 "$PORT_PROBE" --port "$PORT" --hold 30 $flag > "$SDIR/$kind.txt" 2>&1 &
    holder=$!
    # The probe's RESULT line only exists once the hold is over; its HOLDING line exists as soon
    # as it owns the port, and carries the pid, so the hold window is observed and not assumed.
    probe_pid=$(wait_holding "$SDIR/$kind.txt" 20)
    fact "${kind}_probe_pid" "$probe_pid"
    # Sample the holders inside the hold, 1 s after the refusal (a leaked driver fd makes two),
    # and record if the holder was alive then: an expired hold lets configure succeed.
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
    # Both refusals (EBUSY from an exclusive holder, LOCK_FAILED from a flock-only one) log
    # "refusing to share the bus".
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
  # No stop: the SIGKILL must land while +2.0 rad/s stands, or the goal-speed register reads 0.
  # vel_stop, scenario_end and the EXIT trap stop the wheels afterwards.
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
  # max_accel is rad/s^2 and the ACC register is 100 steps/s^2 per count (include/units.hpp),
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
      # joint3's clamp = the interface max = the 6000-count register ceiling
      # (lround(9.2038847 * 651.8986) == 6000), so no command outside the interface is needed.
      wheel=9.2038847
    fi
    move "prep_$stack" joint1 0.0 2.0
    # 1.0 rad in 0.2 s, so the servo's speed clamp limits the move. 2.5 s post-roll: H8.speed_arm
    # allows t_slow up to 1.60 s, and settle() is NaN if the last sample is outside the band.
    move "settle_$stack" joint1 1.0 0.2 2.5
    [ "$stack" = slow ] && spin slow_wheel "8.0,0.0" 4.0 "0.0,0.0" 3.0
    spin "t90_$stack" "0.0,3.0" 5.0 "0.0,0.0" 3.0
    # SIGKILL mid-motion: joint1 in a 2.4 rad / 0.3 s (8 rad/s) move, over its max_speed, and
    # joint3 at its clamp. The controller keeps the wheel command, so one --once publish is enough.
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
  # 70 s: the recording must outlast the component cycle, or g1c sees no post-cycle samples.
  # See docs/bench-check.md, "Component cycle (H9)".
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
  # Cycle the component. Keep each transition's stderr: ros2controlcli prints the failing service
  # and its message there, which the bare exit codes cannot tell apart.
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
  # 'active' does not prove the bus is back; these moves do. 0.4 rad is inside every render's
  # command limits, and the park leaves the arm at 0 for the next scenario.
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
  # Second stimulus: SIGTERM (not SIGINT) to a bare ros2_control_node with a wheel turning.
  # H1B covers the launch's own SIGINT teardown.
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

# H11: a soak of the real 100 Hz cycle with the wheels turning; only the last 20 s are recorded.
# See docs/bench-check.md, "Soak (H11)".

# Runtime: a full run takes ~1880 s at SOAK_S=600, under ctest's TIMEOUT 2400. Each soak second
# adds one; above SOAK_S=750, raise the TIMEOUT too.
h_H11() {
  scenario_begin H11 || return $?
  # The tail (park, 20 s recording, 12 s /diagnostics, 2 s settle) takes ~38 s, so idle the rest.
  # SOAK_S is at least 40, and idle at least 0 (`sleep -N` is an error, not a short sleep).
  local soak=${WAVESHARE_HIL_SOAK_S:-600}
  [ "$soak" -ge 40 ] 2> /dev/null || soak=40
  local idle=$((soak - 38))
  [ "$idle" -lt 0 ] && idle=0
  # Watchdog = soak + 180 s (start, spawner, teardown), or the CM gets SIGINT mid-soak and
  # prints no totals line.
  start_stack "allow_missing:=false nine:=true" "[joint3, joint4]" $((soak + 180))
  fact soak_s "$soak"
  move soak_park joint1,joint2 0.0,0.0 2.0
  # Logged: this publish IS the soak's load. If it fails, the soak runs on a stopped bus and
  # h11's wheels_turning row FAILs; this line says why.
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [1.0, 1.0]}" > /dev/null 2>&1 || hil_log "soak wheel command rc=$?"
  sleep "$idle"
  $RECORD record --duration 20 --djs --label steady_soak --out "$SDIR/steady_soak.json"
  diagnostics 12
  r2 10 topic pub --once /wheels/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.0, 0.0]}" > /dev/null 2>&1 || hil_log "soak wheel stop rc=$?"
  sleep 2
  stop_stack TERM          # TERM, never KILL: on_deactivate prints the totals line
  scenario_end             # ... and scenario_end stops the wheels again and proves the port free
}

# H12: the controller manager's JointSaturationLimiter, not the driver, clamps a command past a
# URDF <limit>. See docs/bench-check.md, "Command limits (H12)".

# Only the <limit>s are tightened (the driver never reads them), so a clamp there is the limiter's.
# The arm is parked first with the limiters off: 0.0087 rad outside a <limit>, the limiter throws.
h_H12() {
  scenario_begin H12 || return $?
  # The stimulus as facts: H12.stimulus compares them with hil_gates.py's H12_* constants, and
  # the limits are also the xacro arguments below, so script, render and gate cannot drift apart.
  fact arm_pos_limit 0.8
  fact arm_command 1.2
  fact wheel_vel_limit 2.0
  fact wheel_command 8.0
  # 1. Park the arm with the limiters OFF, then read where it rests. Clear both globals first:
  #    start_stack can return early and leave the previous scenario's values.
  SPAWNER_RC=
  CONTROLLERS_ACTIVE=
  start_stack "" "[joint3, joint4]"
  move park joint1,joint2 0.0,0.0 2.0
  stop_stack TERM
  # The park stack's facts under own keys: the next two stacks overwrite the shared ones, and
  # move() only logs a failure.
  fact park_spawner_rc "$SPAWNER_RC"
  fact park_controllers_active "$CONTROLLERS_ACTIVE"
  fact park_cm_exit_code "$CM_EXIT_CODE"
  # arm_pre keeps its label and its place: read with the port free, after the park stack is gone,
  # and BEFORE the guard below, so even an aborted H12 records where the arm was actually left.
  readback arm_pre --read-only --registers --ids 1,2 > /dev/null
  # No park stack: the arm was never parked, so the limiter could throw. Record ABORTED (the bench
  # could not ask), not FAIL; return 0, not 2, so the later scenarios (the soak) still run.
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
  # Keep the tightened render, the limiters-on YAML and this stack's CM log name before stack 3
  # re-renders; only this log can hold the 'Creating JointSaturationLimiter' lines.
  cp "$SDIR/robot.urdf" "$SDIR/robot_limited.urdf"
  cp "$SDIR/cm.yaml" "$SDIR/cm_limited.yaml"
  fact limited_cm_log "$(basename "$CM_LOG")"
  # 2.5 s of post-roll, as H8's settle moves use: the clamp is read off the last sample, so the
  # recording has to outlast the trajectory rather than end inside its final approach.
  move pos_clamp joint1 1.2 2.0 2.5
  # Best effort: back inside the limit while a controller exists (0.8 rad is 5.7 ticks from the
  # throw). If the limiter threw, this publishes into nothing; park_final below is the guarantee.
  move park_back joint1,joint2 0.0,0.0 2.0
  # After both arm commands, because a limiter throw takes the arm controller down at the moment
  # it enforces, not at activation: start_stack's controllers_active was true either way.
  fact arm_state_after "$(controllers_snapshot | awk '$1=="arm" {print $NF}')"
  # No stop (as H7): the SIGKILL must land while 8.0 rad/s stands, so the goal-speed register
  # keeps the clamped value. vel_stop, scenario_end and the EXIT trap stop the wheels.
  spin vel_clamp "8.0,8.0" 6.0
  stop_stack KILL          # the goal-speed registers stay exactly as the driver last wrote them
  # The limited stack's facts under own keys (the final park overwrites the shared ones).
  # Its cm_exit_code is a SIGKILL by design: recorded, not gated.
  fact limited_spawner_rc "$SPAWNER_RC"
  fact limited_controllers_active "$CONTROLLERS_ACTIVE"
  fact limited_cm_exit_code "$CM_EXIT_CODE"
  readback vel_readback --read-only --registers --ids 3,4 > /dev/null
  readback vel_stop --ids 3,4 > /dev/null
  # 3. Unconditional park on an ordinary stack (limiters off): the arm may still rest on 0.8 rad.
  #    The shared stack facts of H12 hold THIS stack's values.
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

# ------------------------------------------------------------------ tool scenarios H13-H17

# Each runs between guard_begin and guard_end. See docs/bench-check.md, "EEPROM journal".
# The tools run directly from $TOOLS, not through `ros2 run`, so their exit codes are their own.

# H13: scan, read-only: explicit port, no parameter (only on /dev/ttyACM0), the stale
# device_port name beside the new one, and a positional port.
h_H13() {
  scenario_begin H13 || return $?
  guard_begin
  local g=$?
  if [ "$g" != 0 ]; then scenario_end; [ "$g" = 2 ] && return 2; return 0; fi
  tool scan scan --ros-args -p port:="$PORT"
  if [ "$PORT" = /dev/ttyACM0 ]; then
    tool scan_default scan
  else
    fact scan_default_skipped "the port under test is $PORT, and a bare scan opens /dev/ttyACM0"
  fi
  tool stale_scan scan --ros-args -p device_port:="$PORT" -p port:="$PORT"
  tool positional_scan scan "$PORT" --ros-args -p port:="$PORT"
  guard_end full
  g=$?
  probe released
  scenario_end
  return $g
}

# H14: every tool refuses a held port and writes nothing. Only silent ids (200, 201, 300 -> 44),
# after a census of exactly 1-4. See docs/bench-check.md, "Tool scenarios (H13 to H17)".
h_H14() {
  scenario_begin H14 || return $?
  guard_begin
  local g=$?
  if [ "$g" != 0 ]; then scenario_end; [ "$g" = 2 ] && return 2; return 0; fi
  # Stage A: the controller manager holds the port, and the tools must not disturb its bus.
  start_stack "" "[joint3, joint4]"
  fact driver_warns_before "$(grep -c '\[WARN\]\|\[ERROR\]' "$CM_LOG")"
  fact cm_pids "$(port_holders)"
  $RECORD record --duration 12 --label tools_window --out "$SDIR/tools_window.json" &
  local rec=$!
  sleep 1
  tool cm_scan scan --ros-args -p port:="$PORT"
  tool cm_set_id set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=201
  tool cm_calibrate calibrate_midpoint --ros-args -p port:="$PORT" -p id:=200
  # Wait on the recorder's pid, never a bare `wait` (that also waits for the 420 s watchdog).
  wait "$rec"
  fact driver_warns_after "$(grep -c '\[WARN\]\|\[ERROR\]' "$CM_LOG")"
  stop_stack TERM
  # Stage B: a flock-only holder. A tool that opens with a raw SMS_STS::begin gets past it and
  # prints `serial speed`; one that goes through ServoBus is refused first (via_servobus).
  wait_port_free 15
  timeout -s KILL 60 "$PORT_PROBE" --port "$PORT" --hold 25 --no-exclusive \
    > "$SDIR/flock_holder.txt" 2>&1 &
  local holder=$! probe_pid
  probe_pid=$(wait_holding "$SDIR/flock_holder.txt" 20)
  fact probe_pid "$probe_pid"
  tool fl_scan scan --ros-args -p port:="$PORT"
  fact fl_scan_holder_alive "$(holder_alive "$probe_pid")"
  tool fl_set_id set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=201
  fact fl_set_id_holder_alive "$(holder_alive "$probe_pid")"
  tool fl_calibrate calibrate_midpoint --ros-args -p port:="$PORT" -p id:=200
  fact fl_calibrate_holder_alive "$(holder_alive "$probe_pid")"
  wait "$holder"
  # Stage C: refusals with the port free; the expected exit code of each is on its line.
  tool stale set_id --ros-args -p device_port:="$PORT" -p port:="$PORT" -p start_id:=200 \
    -p new_id:=201                                                                        # 64
  tool bad_type set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=201.0      # 64
  tool range set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=300           # 64
  tool cal_range calibrate_midpoint --ros-args -p port:="$PORT" -p id:=300               # 64
  tool missing set_id --ros-args -p port:="$PORT" -p start_id:=200                       # 64
  tool positional set_id "$PORT" --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=201  # 64
  tool foreign_node set_id --ros-args -p port:="$PORT" -p setid:port:="$PORT" -p start_id:=200 \
    -p new_id:=201                                                                        # 64
  # 4: id 3 answers. A tool without the taken check reaches the silent 200 and exits 3 instead;
  # nothing is written either way.
  tool taken set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=3
  tool silent_start set_id --ros-args -p port:="$PORT" -p start_id:=200 -p new_id:=201    # 3
  tool cal_silent calibrate_midpoint --ros-args -p port:="$PORT" -p id:=200              # 3
  guard_end eeprom_only
  g=$?
  probe released
  scenario_end
  return $g
}

# H15: set_id round trip 4 -> 253 -> 4, journaled. 253 is the top of scan's range, so moved_scan
# proves that scan reaches it.
h_H15() {
  scenario_begin H15 || return $?
  guard_begin
  local g=$?
  if [ "$g" != 0 ]; then scenario_end; [ "$g" = 2 ] && return 2; return 0; fi
  # The precondition guard_begin already checked, repeated next to the write it protects: never
  # write on an unexpected bench, and 253 is silent only when the census is exactly 1 2 3 4.
  if ! snap_is_bench "$SDIR/pre_eeprom.json"; then
    abort_scenario H15 "pre_eeprom is not a complete snapshot of exactly ids 1-4, so 253 may not \
be silent; nothing was written"
    guard_end full
    g=$?
    scenario_end
    return $g
  fi
  tool move set_id --ros-args -p port:="$PORT" -p start_id:=4 -p new_id:=253
  eeprom moved_eeprom snapshot --ids 1,2,3,253 --census --out "$SDIR/moved_eeprom.snap"
  tool moved_scan scan --ros-args -p port:="$PORT"
  tool back set_id --ros-args -p port:="$PORT" -p start_id:=253 -p new_id:=4
  # The state the tool itself left, BEFORE the helper's restore can mask it (restored_by_tool).
  eeprom back_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/back_eeprom.snap"
  eeprom restore restore --from "$SDIR/pre_eeprom.snap"
  guard_end full
  g=$?
  scenario_end
  return $g
}

# $1 = pre_eeprom's RESULT json. True when H16's calibration is observable: the snapshot is sound,
# id 2 is in mode 0 and rests at least 100 ticks from 2048 (about 1026 on this bench).
h16_observable() {
  python3 - "$1" << 'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
two = d.get('ids', {}).get('2', {})
at = two.get('volatile', {}).get('56')
if isinstance(at, int) and at & 0x8000:
    at = -(at & 0x7fff)
sys.exit(0 if d.get('ok') is True and two.get('eeprom', {}).get('33') == 0 and
         isinstance(at, int) and abs(at - 2048) >= 100 else 1)
EOF
}

# H16: calibrate_midpoint on id 2, and the refusal on the wheel id 3, journaled.
h_H16() {
  scenario_begin H16 || return $?
  guard_begin
  local g=$?
  if [ "$g" != 0 ]; then scenario_end; [ "$g" = 2 ] && return 2; return 0; fi
  if ! h16_observable "$SDIR/pre_eeprom.json"; then
    abort_scenario H16 "id 2 is not in mode 0, or rests within 100 ticks of 2048, in \
pre_eeprom: the calibration would be unobservable, so this scenario may never PASS; nothing was \
written"
    guard_end full
    g=$?
    scenario_end
    return $g
  fi
  tool cal calibrate_midpoint --ros-args -p port:="$PORT" -p id:=2
  # Immediately: the independent position read, before anything can let the servo creep.
  eeprom cal_pos read --id 2 --addr 56 --word
  eeprom cal_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/cal_eeprom.snap"
  tool wheel calibrate_midpoint --ros-args -p port:="$PORT" -p id:=3                     # 4
  eeprom wheel_eeprom snapshot --ids 1,2,3,4 --out "$SDIR/wheel_eeprom.snap"
  # restore writes the offset, then the goal at the new present position, then the torque.
  eeprom restore restore --from "$SDIR/pre_eeprom.snap"
  guard_end full
  g=$?
  scenario_end
  return $g
}

# H17: the bench as found. Last in the list: every other scenario has run by now.
h_H17() {
  scenario_begin H17 || return $?
  eeprom final_eeprom snapshot --ids 1,2,3,4 --census --out "$SDIR/final_eeprom.snap"
  cp "$OUT/initial_eeprom.snap" "$SDIR/initial_eeprom.snap" ||
    hil_log "no initial_eeprom.snap from the pre-flight; H17.eeprom_as_found has nothing to match"
  fact baseline_source "$BASELINE"
  if [ -n "$BASELINE" ]; then
    cp "$BASELINE" "$SDIR/baseline.snap" || hil_log "could not copy the baseline '$BASELINE'"
  fi
  fact journal_present "$([ -e "$JOURNAL" ] && echo true || echo false)"
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
  # Restore a journal THIS run wrote that no guard_end removed; remove it only on success.
  # A SIGKILL or a ctest timeout skips this trap; the next run's pre-flight handles that.
  if [ "$JOURNAL_OURS" = 1 ] && [ -e "$JOURNAL" ] && [ -z "$(port_holders)" ]; then
    if eeprom exit_restore restore --from "$JOURNAL"; then
      rm -f "$JOURNAL" "$JOURNAL_PORT"
      hil_log "restored the bench EEPROM from this run's journal"
    else
      hil_log "restoring from this run's journal failed (see exit_restore.txt); $JOURNAL is kept \
for the next pre-flight"
    fi
  fi
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

# Tools and EEPROM pre-flight: after the traps, before anything reads the bench.
[ -x "$EEPROM" ] || { hil_log "missing helper binary '$EEPROM'"; exit 2; }
# A WAVESHARE_HIL_TOOLS override can run older binaries that ignore `port` and always open
# /dev/ttyACM0, so it is refused on any other port.
if [ -n "$TOOLS" ] && [ "$PORT" != /dev/ttyACM0 ]; then
  hil_log "the tools override is for the older tool binaries, which ignore 'port' and always \
open /dev/ttyACM0; refusing to run them against $PORT"
  exit 2
fi
# A journal left by a killed run: restore it only on its own port and only registers 5, 31, 32,
# 33, 40, 55; else keep it and stop. See docs/bench-check.md, "Recover an interrupted run".
if [ -e "$JOURNAL" ]; then
  hil_log "journal from $(date -r "$JOURNAL" '+%F %T') found; bench EEPROM was left changed by an \
earlier run"
  journal_port=$(head -n 1 "$JOURNAL_PORT" 2> /dev/null)
  if [ "$journal_port" != "$PORT" ]; then
    hil_log "the journal $JOURNAL was taken on port '${journal_port:-(no record in $JOURNAL_PORT)}', \
and this run's port is '$PORT'; a journal is applied only to the bench it came from, so it was NOT \
applied and is kept -- run on that port, or check the journal and remove it by hand"
    exit 2
  fi
  # A held port is reported as a held port, not as a journal that cannot be applied.
  if [ -n "$(port_holders)" ]; then
    hil_log "port $PORT is held by [$(port_holders)]; the journal $JOURNAL was NOT applied and is \
kept -- stop that process and run again"
    exit 2
  fi
  rm -f "$SDIR/preflight_restore.txt" "$SDIR/preflight_restore.json"
  if eeprom preflight_restore restore --from "$JOURNAL" --allow-regs 5,31,32,33,40,55; then
    rm -f "$JOURNAL" "$JOURNAL_PORT"
    hil_log "restored the bench EEPROM from the journal and removed it"
  elif [ ! -e "$SDIR/preflight_restore.txt" ]; then
    hil_log "port $PORT became busy [$(port_holders)] before the restore; the journal $JOURNAL was \
NOT applied and is kept -- stop that process and run again"
    exit 2
  else
    while IFS= read -r line; do
      hil_log "  $line"
    done < <(grep -v '^RESULT ' "$SDIR/preflight_restore.txt" 2> /dev/null)
    hil_log "bench EEPROM differs from the journal outside the registers the tools write (or the \
journal is unreadable); not restoring automatically -- see $JOURNAL"
    exit 2
  fi
fi
[ -n "$TOOLS" ] || TOOLS=$prefix/lib/waveshare_servos
hil_log "tools from $TOOLS"
# H17.eeprom_as_found compares with THIS file: remove one an earlier run left in $OUT.
rm -f "$OUT/initial_eeprom.snap" "$OUT/initial_eeprom.txt" "$OUT/initial_eeprom.json"
if ! eeprom initial_eeprom snapshot --ids 1,2,3,4 --census --out "$OUT/initial_eeprom.snap"; then
  rm -f "$OUT/initial_eeprom.snap"
  hil_log "pre-flight initial_eeprom snapshot failed (see initial_eeprom.txt/.json); the start of \
this run was not read, so H17.eeprom_as_found will FAIL"
fi

timeout 20 ros2 daemon stop > /dev/null 2>&1
readback initial_readback --read-only --registers --ids 1,2,3,4 > /dev/null

# Order is a cost decision: fast rows first, journaled tool scenarios before the soak H11, H17
# last. It must name exactly hil_gates.py's CHECKERS. See docs/bench-check.md, "Scenarios".
SCENARIOS=${WAVESHARE_HIL_SCENARIOS:-"H1 H1B H2 H3 H4 H5A H5B H5C H6 H7 H8 H9 H10 H12 H13 H14 H15 H16 H11 H17"}
aborted_all=0
abort_all_reason=
for s in $SCENARIOS; do
  SCEN=run
  SDIR=$OUT
  if [ "$aborted_all" = 1 ]; then
    abort_scenario "$s" "$abort_all_reason"
    continue
  fi
  hil_log "=== $s ==="
  ABORT_REASON=
  "h_$s"
  rc=$?
  SCEN=run
  SDIR=$OUT
  hil_log "$s finished rc=$rc"
  # 127 = bash's "command not found": a SCENARIOS name with no h_ function. Abort it.
  if [ "$rc" = 127 ]; then
    if declare -F "h_$s" > /dev/null; then
      abort_scenario "$s" "h_$s returned 127"
    else
      abort_scenario "$s" "no h_$s function"
    fi
  fi
  # A 2 aborts every later scenario, carrying the reason the scenario gave (guard_end's "bench
  # EEPROM not restored", guard_begin's unknown journal) rather than always the port-held text.
  if [ "$rc" = 2 ]; then
    aborted_all=1
    abort_all_reason="after $s: ${ABORT_REASON:-an earlier scenario left the port held by a \
process this suite may not kill}"
  fi
done

SCEN=report
SDIR=$OUT
port_free=$([ -z "$(port_holders)" ] && echo true || echo false)
elapsed=$((SECONDS - STARTED))
# --expected: a scenario in the list that left no directory and no abort is FAIL did_not_run.
$GATES --run-dir "$OUT" --allow-inconclusive "$HIL_INCONCLUSIVE_ALLOWED" \
  --seconds "$((elapsed / 60))m$((elapsed % 60))s" --port-free "$port_free" \
  --expected "$SCENARIOS" |
  tee "$OUT/hil_check.txt"
rc=${PIPESTATUS[0]}
hil_log "report written to $OUT/hil_check.txt and $OUT/hil_check.json (rc=$rc)"
exit "$rc"
