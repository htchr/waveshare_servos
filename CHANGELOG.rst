^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waveshare_servos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-09-23)
------------------
* Ported to ROS 2 Jazzy. Built and tested against ros2_control 4.48.0 and ros2_controllers 4.42.1
  on Ubuntu 24.04. This version does not build on Humble; 0.1.0 on the ``humble`` branch is the
  Humble version.
* ``package.xml`` declares the license as ``GPL-3.0-or-later`` instead of ``GPL-3.0-only``. The
  ``LICENSE`` file is unchanged.
* Breaking changes (upgrading from 0.1.0):

  * Requires the Jazzy ros2_control API (``on_init(HardwareComponentInterfaceParams)``, interface
    handles created by the framework).
  * ``allow_missing_servos`` defaults to ``false``: a servo that does not answer its ping makes
    ``on_configure`` fail, and with the controller manager's defaults ``ros2_control_node`` does
    not start. 0.1.0 logged a warning and carried on. Set it to ``true`` for the old behaviour.
  * ``on_init`` rejects descriptions that 0.1.0 accepted or crashed on, with one FATAL message.
    ``id`` must be present, 1..253 and unique in the ``<ros2_control>`` block. A declared
    ``type`` must agree with the command interfaces: ``pos`` needs a ``position`` command
    interface and ``vel`` must not have one. Position ``min``/``max`` plus ``offset`` must fit
    the servo's single-turn range (ticks 0..4095). ``baudrate`` must be one of the seven rates
    the servo library supports.
  * The ``torque`` state interface is deprecated in favour of ``effort`` (N m). ``torque`` still
    reports kg cm and logs one deprecation warning per joint.
  * The serial port is opened exclusively (``TIOCEXCL`` and ``flock``). ``on_configure`` fails
    while another process holds the port exclusively or holds its lock, and while the driver has
    the port, any other program that tries to open it is refused (one running as root is not). A
    program that already had the port open without either (a serial monitor started first) is not
    stopped: the driver configures and logs a warning naming it. The tools refuse to run while the
    controller manager holds the port.
  * Tools: ``device_port`` and ``baud_rate`` are renamed ``port`` and ``baudrate``. The old
    names, unknown names and parameters addressed to another node are refused with exit code 64,
    not ignored. ``set_id`` requires ``start_id`` and ``new_id``, and ``calibrate_midpoint``
    requires ``id``; there are no defaults of 1 any more. Values are checked too, also with exit
    code 64: ``start_id`` and ``id`` must be 0..253, ``new_id`` 1..253 and different from
    ``start_id``, and ``baudrate`` one of the seven supported rates, and a positional argument is
    refused. 0.1.0 cut an id to 8 bits (``new_id`` 300 wrote id 44), fell back to 115200 for an
    unsupported rate, and ignored positional arguments.
  * ``calibrate_midpoint`` refuses a servo that is not in mode 0 instead of writing the mode
    register, and it leaves the servo's torque off.
  * Example controllers: ``joint_trajectory_velocity_controller`` (a JointTrajectoryController)
    is replaced by ``joint_velocity_controller``
    (``velocity_controllers/JointGroupVelocityController``), which takes
    ``std_msgs/Float64MultiArray`` on ``/joint_velocity_controller/commands``. The trajectory
    controller uses ``interpolate_from_desired_state`` instead of ``open_loop_control``.
  * Example description: the ``prefix`` xacro argument and the ``prefix`` macro parameter are
    removed. They never worked, because the URDF joints were not prefixed. The
    ``example_ws_ros2_control`` macro takes ``name``, ``port``, ``baudrate`` and
    ``use_mock_hardware``. The example has four joints, declares ``effort`` instead of
    ``torque``, and uses new ``<limit>`` values (9.2 rad/s, 1.0 N m).
  * ``libwaveshare_servos.so`` no longer contains the whole vendored SCServo library; the
    SCSCL, SMSBL and SMSCL classes are gone from it. Linking the plugin library to use the servo
    classes directly is not supported.
  * If you used a pre-release ``jazzy`` snapshot that already had ``io_timeout_ms``: the default
    changed from 20 to 5 ms, and 1 is now refused (range 2..1000).

* Behaviour changes with an unchanged description:

  * Torque is enabled on every servo when the hardware activates.
  * Deactivating the hardware stops the wheels and parks each position joint that answers at its
    measured position. 0.1.0 stopped the wheels and re-sent the position joints' last goals with
    goal speed 0, which the servo treats as full speed.
  * The driver clamps position commands to the position command interface's ``min``/``max``. A
    joint that starts outside them holds its position until it is commanded back inside. Wheel
    speeds are clamped to ``max_speed``, which defaults to 9.2 rad/s.
  * The goal speed of a position joint is paced so it reaches each setpoint in one control
    period, and it is never 0 (which the servo treats as full speed). The ``velocity`` command of
    a ``pos`` joint is only used when no measurement is available.
  * The mode register (EEPROM) is written at configure, and at an activation that re-adds a
    missing or dropped servo, only when it differs from the joint's type, with the EEPROM lock
    opened around the write and the mode read back. 0.1.0 wrote it on every configure without
    opening the lock.
  * The tools exit non-zero on failure (documented exit codes). 0.1.0 always exited 0.
  * A ``vel`` joint's ``position`` is a multi-turn count by default (``unwrap`` defaults to
    ``true`` for ``vel`` joints) and no longer wraps at one revolution. Set ``unwrap`` to
    ``false`` for 0.1.0's single-turn value.

* New features:

  * Hardware parameters ``port``, ``baudrate``, ``io_timeout_ms``, ``ping_attempts``,
    ``max_read_fails``, ``allow_missing_servos``, ``protocol``, ``feedback_mode``,
    ``encoder_steps``, ``current_per_count_a`` and ``torque_constant_nm_per_a``. They replace
    the hard-coded ``/dev/ttyACM0`` and 1 Mbaud. Unknown parameters are logged and ignored.
  * Joint parameters ``inverted``, ``max_speed`` (rad/s), ``max_accel`` (rad/s^2) and
    ``unwrap``. ``type`` is optional: a joint with only a velocity command is ``vel``, otherwise
    it is ``pos``.
  * State interfaces ``effort`` (N m), ``current`` (A), ``voltage`` (V), ``load`` and ``status``
    (the servo's status byte). A joint may declare any subset of the nine interfaces, in any
    order. They are exported in the order the description lists them.
  * By default, wheel positions are unwrapped into a multi-turn count, which is kept when the
    hardware is cycled inactive and active. ``diff_drive_controller`` odometry no longer jumps
    once per revolution.
  * Batched bus traffic. Feedback is fetched with one sync read (``INST_SYNC_READ``) per cycle,
    or one per 30 servos on a larger bus, with a per-servo fallback. Goals still go out as
    broadcast sync writes, as in 0.1.0, now split at 30 position or 82 speed records per packet,
    and the wheels' acceleration register is written at four edges instead of by an acknowledged
    write per wheel in every cycle. Every frame of a sync-read reply is checked for header, id,
    length and checksum.
  * Faults in the status byte are logged by name, and torque is re-enabled when a fault clears.
  * Recovery: after ``max_read_fails`` failed reads in a row, a servo is dropped from the read
    cycle with one ERROR. ``ros2 control set_hardware_component_state <name> inactive``, then
    ``active``, pings it again and adds it back.
  * ``on_shutdown`` and ``on_error`` stop the wheels, park the position joints and close the
    port. At deactivation the driver logs a ``bus totals:`` line with failed-transaction counts
    per servo.
  * New tools: ``scan`` lists every servo that answers on ids 0..253, and ``factory_reset``
    restores one servo's EEPROM to factory values, except its id. ``set_id`` and
    ``calibrate_midpoint`` now check the servo before they write and verify the result
    afterwards.
  * Example launch arguments ``port``, ``baudrate`` and ``use_mock_hardware``, which swaps in
    ``mock_components/GenericSystem`` and opens no port. The example also gets a
    ``diff_drive_controller``, which is loaded inactive.
  * A devcontainer for Jazzy. It puts the container user in ``dialout`` explicitly and passes
    ``--ulimit rtprio=99`` and ``--ulimit memlock=-1`` (and ``--cap-add=sys_nice``), which raise
    RLIMIT_RTPRIO to 99 and RLIMIT_MEMLOCK to unlimited, the limits real-time scheduling and
    memory locking need.

* Documentation:

  * The README documents every hardware and joint parameter, the command and state interfaces,
    start-up with missing servos, recovering a dropped servo, real-time permissions, sizing the
    bus, the tools, troubleshooting and the tests.
  * ``THIRD_PARTY.md`` records where the vendored SCServo library comes from, what was changed
    at import and since, its checksums and how to refresh it.

* Packaging:

  * ``package.xml``: version 1.0.0, repository and bug tracker URLs; license files named, and an
    Apache-2.0 entry for the two files that keep their upstream Apache-2.0 headers (see
    ``THIRD_PARTY.md``).
  * ``package.xml``: declared what was only satisfied transitively. ``launch`` and
    ``launch_ros`` for the example launch file;
    ``ros2run`` and ``ros2topic`` for the README commands; and for the tests,
    ``ament_cmake_cpplint``, ``builtin_interfaces``, ``lifecycle_msgs``, ``rcutils``,
    ``python3-catkin-pkg-modules``, ``python3-docutils``, ``python3-pytest`` and
    ``python3-yaml``.
  * ``package.xml`` declares ``controller_manager`` as an exec dependency only, not a test
    dependency; the reason is recorded beside ``ament_lint_auto_find_test_dependencies()`` in
    ``CMakeLists.txt``.
  * ``LICENSE``, ``LICENSES/Apache-2.0.txt`` and ``THIRD_PARTY.md`` are installed to
    ``share/waveshare_servos``.
  * The README's and the devcontainer's ``rosdep install`` do not pass ``-r``, which would turn
    an unresolvable dependency into success; the devcontainer's start-up stops with rosdep's
    error in that case, before the build.

* Fixes:

  * ``torque`` truncated the current to whole amperes, so it read 0 below 1 A.
  * A joint without an ``id`` or ``type`` parameter crashed the driver.
  * An absent servo stayed on the bus: each cycle it cost four reads, each waiting out the
    library's 100 ms timeout.
  * Position moves lurched at the start and end of trajectories.
  * The installed example did not launch: the launch file looked for the URDF in the wrong
    place, and the RViz configuration was not installed. RViz now also shows the robot model.
  * Vendored ``SCSerial``: ``readSCS()`` could write before the start of its buffer on a read
    error, ``wFlushSCS()`` truncated partially written frames, and ``end()`` leaked the file
    descriptor, so the serial port was released only when the process exited.
  * ``package.xml`` did not declare the build dependencies (``hardware_interface``,
    ``pluginlib``, ``rclcpp``, ``rclcpp_lifecycle``). The unused ``joint_state_publisher_gui``
    dependency is removed.
  * The example's ``<limit>`` velocities (0.5 and 1.0 rad/s) would have capped the joints as
    soon as ``enforce_command_limits`` was enabled, and its effort limit (1000) was three orders
    of magnitude too high.

* Performance (four ST3025 servos at 1 Mbaud and 100 Hz on the reference bench described in the
  README; details in its section "Sizing the bus"):

  * ``read()`` takes 1.645 ms instead of 3.064 ms per cycle, and ``write()`` takes 0.014 ms
    instead of 1.447 ms. That is 16.6 % of the 10 ms period instead of 45 %. The comparison is
    with the branch's earlier path, which read each servo separately.
  * Each extra servo costs about 0.29 ms, so about twelve joints fit at 100 Hz with 50 %
    headroom, up from five. Only one to four servos were measured; beyond that it is a model.
  * There were 0 failed transactions in 244 158 over four ten-minute runs, which bounds the rate
    below 13 per million at 95 % confidence.

* Tests (0.1.0 had none):

  * A motorless suite: gtest/gmock cases in which the driver and the tools talk to a simulated
    servo bus on a pseudo-terminal, xacro render tests, a launch test of the example on mock
    hardware, checks that the README's reference tables (their names, and the hardware and launch
    defaults) and its list of quoted messages agree with the code, that the vendored files match
    the checksums in ``THIRD_PARTY.md``, and that ``package.xml`` and ``CHANGELOG.rst`` are
    well-formed and name the same version, and ament lint with the vendored files excluded.
  * ``test/hil_check.sh`` checks the real servos in 20 scenarios on the reference bench and runs
    only when ``WAVESHARE_HIL=1`` is set.

* Known limitations:

  * ``read()`` and ``write()`` always return OK, so a bus failure never triggers ``on_error``:
    silent servos are dropped after ``max_read_fails`` cycles and the component stays active.
  * Activation enables torque before the first goal is written. After ``calibrate_midpoint``,
    or on a cold start (goal register 0), a joint can move toward a stale goal for about one
    control period.
  * ``effort`` and ``current`` are approximate, and they are never negative: this firmware
    never sets the sign of the current. Status bits 4, 6 and 7 are unverified.
  * A servo that loses power while running comes back with torque off, and answering again does
    not re-enable it: the driver enables torque only at activation and when a status fault clears,
    so it is re-enabled only if it reported a fault just before or after the outage. Otherwise
    deactivate and activate the hardware to recover it; that activation also enables torque
    before a goal is written, and from the first cycle the still-active controller drives the
    joint to its last command at up to ``max_speed``, which can be a fast move (worked out from
    the code), so keep clear of the joint. Its wheel acceleration register (SRAM) is rewritten
    at four edges, but whether they catch every brown-out is untested. Inferred from the measured
    power-up state and the code; a mid-run brown-out was not measured.
  * ``protocol`` accepts only ``sms_sts``; the SCS series (``scscl``) is not implemented.
  * JointTrajectoryController 4.42.1 crashes on velocity-only joints, so wheels need a velocity
    controller.
  * ``factory_reset`` was verified on a single servo and has no bench scenario.
  * The ``io_timeout_ms`` default and floor are sized for 1 Mbaud and do not scale with
    ``baudrate``.
  * With ``port`` set to a ``/dev/serial/by-id`` link, the driver's "port busy" error cannot
    name the process that holds the port (the tools can).
  * The mode write is read back, but the results of the EEPROM unlock and lock are not checked.
  * ``test/hil_check.sh`` assumes the reference bench and does not check the bus before its
    first scenarios.
  * A disconnected USB adapter is not reported as a failure, and deactivating and activating the
    hardware does not reopen the port (set the hardware ``unconfigured``, re-plug the adapter,
    then set it ``active``; that route was measured with the adapter left plugged in, the unplug
    itself was not; or restart the launch).
  * Torque stays on after shutdown.

0.1.0 (2025-07-18)
------------------
* Humble version: a ros2_control ``SystemInterface`` for Waveshare/Feetech ST servos using the
  SMS/STS protocol, on ``/dev/ttyACM0`` at 1 Mbaud (hard-coded).
* Joint parameters ``id``, ``type`` (``pos``: servo mode 0, ``vel``: wheel mode 1) and
  ``offset`` (rad). The command interfaces are ``position`` and ``velocity``. Every joint
  declares exactly four state interfaces, in this order: ``position``, ``velocity``, ``torque``
  (kg cm) and ``temperature``.
* Position commands start from the measured position, so the robot does not move when it
  activates.
* ``set_id`` and ``calibrate_midpoint`` tools (parameters ``device_port`` and ``baud_rate``).
* Example description with three joints (two position, one wheel), with a launch file and a
  controller configuration.
* Includes the vendored Feetech/Waveshare SCServo library.
