# Development and release

This page tells a maintainer how to change and release the package safely. There is no CI, by
decision, so you do the release checks by hand.

## Development rules

- Work from the workspace root of [Set up](../README.md#set-up). Build with `--symlink-install`.
- Write the test first. Make sure that it fails for the expected reason.
- Put each test file in `test/`. Register it in `CMakeLists.txt` inside `if(BUILD_TESTING)`. Lint
  is part of the suite.
- Every test passes with no motors.
- Drive `ServoBus` through a pseudo-terminal. Do not mock the vendored library, and never edit a
  vendored file ([the rule](../THIRD_PARTY.md#the-rule-these-files-are-not-edited)).
- Search `test/` before you change a log string. `test_readme.py`, `test/hil/hil_gates.py` (the
  `bus totals:` line), the lifecycle test and the load test match some log text exactly.
- Declare each test dependency as a `<test_depend>`, unless it is an `<exec_depend>`
  (`rosdep install` installs all types except `doc`). Do not also declare `controller_manager` as
  a `<test_depend>`, although `test_example_launch.py` imports it. As a `<test_depend>`, its CMake
  config prints a `tl_expected` deprecation warning through
  `ament_lint_auto_find_test_dependencies()` (944 bytes of build stderr on Jazzy, September 2026).
  Then release check 1 fails.

## Run the motorless tests

WARNING: Make sure that `WAVESHARE_HIL` is not set in your shell. When it is `1`, `colcon test`
runs the bench check, which moves the servos on `/dev/ttyACM0` and writes their EEPROM.

Stop every program that uses `/dev/ttyACM0`. Then run these commands from the workspace root:

```bash
colcon build --packages-select waveshare_servos
colcon test --packages-select waveshare_servos
colcon test-result --all --verbose
```

- `colcon test` exits 0 also when a test fails, unless you add `--return-code-on-test-failure`.
  `colcon test-result --all --verbose` gives the result.
- `test_tools_cli` holds the port for its full run, and `test_hil_eeprom` for one case. If a
  different program holds the port for 30 s, `test_tools_cli` fails.
- To run one test, add `--ctest-args -R <name>` to `colcon test`.
- Without `WAVESHARE_HIL=1`, `hil_check` prints `SKIP: WAVESHARE_HIL is not 1` (a ctest skip).
- On Ubuntu 24.04, set `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS`, or `ament_cppcheck` skips every file.

## What the tests cover

`test_servo_bus`, `test_lifecycle_over_pty`, `test_servo_tools`, `test_tools_cli` and
`test_hil_eeprom` use the [fake servo bus](#fake-servo-bus). `test_lifecycle_over_pty` replaces
two bench tests: a servo that stops its replies during a run, and all 256 status bytes. Unit
tests, [launch and render tests](#launch-and-render-tests), the gate self-test and lint (not of the
vendored files) need no bus. Only `hil_check` needs the [bench](bench-check.md).

`test_readme` does the [documentation checks](#documentation-checks), and `test_vendored_files`
does the [checksum check](../THIRD_PARTY.md#checksums). `test_release_metadata` checks
`package.xml` (format 3, license texts) and `CHANGELOG.rst` (it parses, the versions decrease, and
the top version is the `package.xml` version). It rejects placeholders and development pointers
in the files that a clone reads first. No test checks `docs/` for them, so check it by hand.

## Documentation checks

The docs use ASD-STE100 Simplified Technical English. `test_readme.py` compares eight reference
blocks with the code. Each block starts at a line `<!-- reference:<name>:begin -->` and stops at a
line `<!-- reference:<name>:end -->`. A missing block fails, and never skips.

| Page | Blocks | Compared with |
| --- | --- | --- |
| [configuration.md](configuration.md#hardware-parameters) | `hardware-parameters`, `joint-parameters`, `state-interfaces`, `status-bits` | The name tables of the driver, the defaults in `src/driver_defaults.hpp`, the bit names in bit order |
| [setup.md](setup.md#launch-arguments) | `launch-arguments` | Names and defaults in `example.launch.py` |
| [tools.md](tools.md#tool-parameters) | `tool-parameters`, `exit-codes` | Accepted names, id ranges, `enum class Exit` |
| [operation.md](operation.md#log-messages) | `log-lines`, exactly 65 lines (`EXPECTED_LOG_LINES`) | C++ string literals in `src/` and `include/`: each fragment of 8 or more characters |

Joint defaults, units and prose are not checked. Every relative link and GitHub anchor in
`README.md`, `THIRD_PARTY.md` and `docs/*.md` must resolve. `README.md` must link `LICENSE`,
`CHANGELOG.rst` and `THIRD_PARTY.md`. No document cites a package file by line number, except
the vendored files.

A code comment cites a section as `See docs/<file>.md, "<heading>".` (or `THIRD_PARTY.md`) on one
line. The test finds each heading in the cited file, and pins the count (`EXPECTED_DOC_QUOTES`)
and the list (`CITED_HEADINGS`). Rename a heading only with its citations and the two constants.

## Fake servo bus

`test/fake_servo_bus.hpp` is a fake SMS/STS servo bus on an `openpty()` pair, so the vendored
packet code runs unchanged. Each servo is a 256-byte register file. A register changes only when a
test sets it or a packet writes it, so each case replays byte for byte.

- The fake acks each write that is not a broadcast. The vendored library waits a full timeout for
  each missing ack.
- The responder thread is an RAII member, so a failed `ASSERT_*` still joins it.
- All knobs are off by default. The tool knobs, for example the EEPROM lock, RESET and the baud
  model, act only on packets that the driver never sends.
- The fake servos have firmware 3.6 and model 777, unlike the [bench](setup.md#the-reference-bench).
- The lock policy sets what an EEPROM WRITE does while register 55 is 1. `apply_always` (the
  default, for the driver suites) ignores the lock, and `drop_when_locked` refuses the write.
  `volatile_when_locked` (the memory-table rule, for the tool suites) loses it at `power_cycle()`.
- In the tool tests, the witness is the frame log and the byte count, never the tool report.
- All state is behind one mutex. Use `snapshot()`, and keep no `FakeServo &` across a driver call.
- A sync write gets no ack, so call `wait_quiet()` before you read it. It waits for two empty 1 ms
  poll windows, with a 2 s deadline. `TearDown` asserts that `quiet_timeouts()` is 0.

## Keep tests off the bench

The default port of the driver and the tools is the bench adapter. These rules keep tests off it:

- Each `test_lifecycle_over_pty` case that reaches `on_configure` sets `port` to its own
  pseudo-terminal. `TearDown` checks that no serial port is open before it destroys the resource
  manager, which closes the port.
- `test_load_waveshare_servos` never configures the driver. It reads the default port from a log
  line.
- `test_tools_cli` runs the built tools from `WAVESHARE_TOOL_*` (a missing path fails) on fake ids
  11-14, not 1-4. `DefaultPortGuard` holds `/dev/ttyACM0` with `flock` and `TIOCEXCL`, and sends
  no byte. A tool that falls back to it gets `EBUSY`. The suite fails unless the port is absent,
  held by the guard or refused (`EACCES`). A port that another process holds is not safe, because
  that process can release it during the suite. As root, the suite fails if the port exists (root
  ignores `TIOCEXCL`).
- `test_tools_cli` checks each exit with `WIFEXITED` and `WEXITSTATUS`. A shell shows 130 also
  for a SIGINT kill.
- The launch test gives `use_mock_hardware:=true` and a nonexistent port. Both go to the xacro
  through the same `Command` in the launch file. If an edit there drops them, the xacro uses its
  defaults: the real driver on `/dev/ttyACM0`. Thus two checks are necessary:
  `assert_description_renders_to_the_mock()` before a process starts (it fails closed), and
  `test_2_hardware_component_is_the_mock` on the loaded component after start-up.
- `gui:=false` is not a safety measure. It only keeps `rviz2` out of the launch.

## Launch and render tests

`test_urdf_xacro` renders the installed share copy of `example.urdf.xacro`, and opens no port.
With `use_mock_hardware` true, `<hardware>` holds `mock_components/GenericSystem` and no
`<param>`. With it false, it holds `waveshare_servos/WaveshareServos` with the test `port` and
`baudrate`. An unknown value of `use_mock_hardware` stops the render. With `--symlink-install`
the share file links to the source, so only a copy install (release check 4) tests the `install()`
rules.

`test_example_launch` starts the example on mock hardware and checks the stack while it runs. A
healthy run takes about 7 s. In the worst case, the waits add up to about 295 s. That is 45 s for
`ReadyToTest`, seven waits of `STARTUP_TIMEOUT` (30 s each) and 40 s for `/joint_states`.
`check_controllers_running` waits two times, and `check_if_js_published` waits two times for a
fixed 20 s. Keep the ctest `TIMEOUT` (330 s) above that sum, because a ctest timeout hides the
failed check.

All controller spawners on a machine share one lock file, so the test is `RUN_SERIAL`. A spawner
stops after about 115 s without the lock: `Failed to acquire lock after multiple attempts.`

Each ROS test has `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and its own `ROS_DOMAIN_ID`: 77
`hil_check.sh`, 85 `test_tool_params`, 86 `test_tools_cli` and 88 `test_example_launch`. Never
give a new test 77, where it finds the controller manager of the bench check.

## Release checklist

Do these checks before a release, in this order:

1. Build the package. Make sure that `log/latest_build/waveshare_servos/stderr.log` is empty.
2. [Run the motorless tests](#run-the-motorless-tests).
3. Run rosdep in a clean environment, from the workspace root:
   ```bash
   env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep keys --from-paths src --ignore-src
   env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep check --from-paths src --ignore-src --rosdistro jazzy
   env -i HOME="$HOME" PATH=/usr/bin:/bin rosdep install --from-paths src --ignore-src --rosdistro jazzy --simulate --reinstall -y
   ```
4. Do a copy install in a new workspace. Run it from the package directory of a clean clone.
   ```bash
   mkdir -p /tmp/release_ws/src && cp -r . /tmp/release_ws/src/waveshare_servos
   cd /tmp/release_ws && env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c '
     source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash &&
     colcon test && colcon test-result --all --verbose &&
     timeout -s INT 20 ros2 launch waveshare_servos example.launch.py use_mock_hardware:=true gui:=false'
   ```
5. Build on a clean machine in Docker. Only this check finds an undeclared dependency. Run it from
   the package directory of a clean clone of the release commit.
   ```bash
   docker run --rm -v "$PWD":/ws/src/waveshare_servos:ro ros:jazzy-ros-core bash -c '
     set -e; apt-get update && apt-get install -y python3-rosdep python3-colcon-common-extensions build-essential
     rosdep init && rosdep update
     cd /ws && . /opt/ros/jazzy/setup.sh
     rosdep install --from-paths src --ignore-src --rosdistro jazzy -y          # no -r
     colcon build && . install/setup.sh
     colcon test --ctest-args -E "^(test_tools_cli|test_hil_eeprom|hil_check)$"
     colcon test-result --all --verbose'
   ```
   It passes if `docker run` exits 0 and `colcon test-result` shows more than 0 tests, 0 errors
   and 0 failures. Read the output: `set -e` does not stop the recipe after a failed build.
6. Run the full [bench check](bench-check.md#run-the-bench-check) on the reference bench.
7. Increase the version in `package.xml`. Add a new top section to `CHANGELOG.rst`.
8. Do the `sha256sum -c` hand check in [THIRD_PARTY.md](../THIRD_PARTY.md#checksums).

## Versioning

The version is in `package.xml`, and [CHANGELOG.rst](../CHANGELOG.rst) lists every version.
`ros2 pkg xml waveshare_servos -t version` prints the installed version.

Semantic versioning covers the plugin name and the hardware and joint parameters (names, types,
defaults, ranges). It also covers the command and state interfaces (names, units), the tools
(names, parameters, exit codes) and the example launch arguments. A breaking change to one of
them increases the major version. The installed C++ headers put generic names such as `units.hpp`
and `SCS.h` on the include path. The version does not cover them, `ServoBus`, a direct link to the
plugin library or the vendored library. Log text (also the `bus totals:` line), the example values
and the test harness can change in any version.
