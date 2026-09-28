^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package waveshare_servos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-09-23)
------------------
* The package now runs on ROS 2 Jazzy. It is built and tested with ros2_control 4.48.0 and
  ros2_controllers 4.42.1 on Ubuntu 24.04. This version does not build on Humble. The Humble
  version is 0.1.0 on the ``humble`` branch.
* ``package.xml`` declares the license as ``GPL-3.0-or-later`` instead of ``GPL-3.0-only``. The
  ``LICENSE`` file stays the same.
* Breaking changes (upgrading from 0.1.0):

  * The driver uses the Jazzy ros2_control API: ``on_init(HardwareComponentInterfaceParams)``, and
    interface handles that the framework creates.
  * ``allow_missing_servos`` defaults to ``false``. If a servo does not answer its ping,
    ``on_configure`` fails. With the defaults of the controller manager, ``ros2_control_node``
    then exits. 0.1.0 logged a warning and continued. Set it to ``true`` to get the old behaviour.
  * ``on_init`` rejects descriptions that 0.1.0 accepted or crashed on, and logs one FATAL
    message. Each joint must have an ``id`` in 1..253 that is unique in the ``<ros2_control>``
    block. A declared ``type`` must agree with the command interfaces. A ``pos`` joint needs a
    ``position`` command interface, and a ``vel`` joint must not have one. Position ``min`` and
    ``max`` plus ``offset`` must fit in the single-turn range of the servo (ticks 0..4095).
    ``baudrate`` must be one of the seven rates that the servo library supports.
  * The ``torque`` state interface is deprecated. Use ``effort`` (N m) instead. ``torque`` still
    reports kgf cm and logs one deprecation warning for each joint.
  * The driver opens the serial port exclusively (``TIOCEXCL`` and ``flock``). ``on_configure``
    fails if another process holds the port exclusively or holds its lock. While the driver has
    the port, other programs cannot open it, but a program that runs as root can. The driver does
    not stop a program that already had the port open without either lock, for example a serial
    monitor that started first. The driver then configures and logs a warning that names that
    program. The tools refuse to run while the controller manager holds the port.
  * Tool parameter names: ``device_port`` and ``baud_rate`` are now ``port`` and ``baudrate``.
    The tools do not ignore the old names, unknown names or parameters for another node. They
    refuse them with exit code 64. ``set_id`` needs ``start_id`` and ``new_id``, and
    ``calibrate_midpoint`` needs ``id``. These parameters no longer default to 1.
  * Tool parameter values: the tools check the values and refuse a bad value with exit code 64.
    ``start_id`` and ``id`` must be in 0..253. ``new_id`` must be in 1..253 and different from
    ``start_id``. ``baudrate`` must be one of the seven supported rates. The tools also refuse a
    positional argument with exit code 64. 0.1.0 cut an id to 8 bits (``new_id`` 300 wrote id
    44), used 115200 for an unsupported rate, and ignored positional arguments.
  * ``calibrate_midpoint`` refuses a servo that is not in mode 0, and does not write the mode
    register. It leaves the torque of the servo off.
  * Example controllers: ``joint_velocity_controller``
    (``velocity_controllers/JointGroupVelocityController``) replaces
    ``joint_trajectory_velocity_controller`` (a JointTrajectoryController). It takes
    ``std_msgs/Float64MultiArray`` on ``/joint_velocity_controller/commands``. The trajectory
    controller uses ``interpolate_from_desired_state`` instead of ``open_loop_control``.
  * Example description: the ``prefix`` xacro argument and the ``prefix`` macro parameter are
    gone. They never worked, because the URDF joints did not have the prefix. The
    ``example_ws_ros2_control`` macro takes ``name``, ``port``, ``baudrate`` and
    ``use_mock_hardware``. The example has four joints and declares ``effort`` instead of
    ``torque``. Its ``<limit>`` values are new (9.2 rad/s, 1.0 N m).
  * ``libwaveshare_servos.so`` no longer contains all of the vendored SCServo library. The
    SCSCL, SMSBL and SMSCL classes are not in it. The package does not support a program that
    links the plugin library to use the servo classes directly.
  * Pre-release ``jazzy`` snapshots with ``io_timeout_ms``: the default is now 5 ms instead of
    20 ms. The range is 2..1000, so the driver now refuses 1.

* Behaviour changes with an unchanged description:

  * The driver enables torque on every servo when the hardware activates.
  * When the hardware deactivates, the driver stops the wheels. It parks each position joint that
    answers at its measured position. 0.1.0 stopped the wheels and sent the last goals of the
    position joints again with goal speed 0. The servo treats goal speed 0 as full speed.
  * The driver clamps position commands to the ``min`` and ``max`` of the ``position`` command
    interface. A joint that starts outside them holds its position until a command brings it
    back inside. The driver clamps wheel speeds to ``max_speed`` (default 9.2 rad/s).
  * The driver sets the goal speed of a position joint so that the joint reaches each setpoint
    in one control period. This goal speed is never 0, which the servo treats as full speed. The
    driver uses the ``velocity`` command of a ``pos`` joint only when no measurement is
    available.
  * The driver writes the mode register (EEPROM) only when it differs from the joint type. It
    does this at configure, and at an activation that adds back a missing or dropped servo. It
    opens the EEPROM lock around the write and reads the mode back. 0.1.0 wrote the mode on
    every configure and did not open the lock.
  * The tools exit with a non-zero code on failure. ``docs/tools.md``, section "Exit codes",
    lists the codes. 0.1.0 always exited 0.
  * The ``position`` of a ``vel`` joint is a multi-turn count by default (``unwrap`` defaults to
    ``true`` for ``vel`` joints). It no longer wraps at one revolution. Set ``unwrap`` to
    ``false`` to get the single-turn value of 0.1.0.

* New features:

  * The hardware parameters ``port``, ``baudrate``, ``io_timeout_ms``, ``ping_attempts``,
    ``max_read_fails``, ``allow_missing_servos``, ``protocol``, ``feedback_mode``,
    ``encoder_steps``, ``current_per_count_a`` and ``torque_constant_nm_per_a`` replace the
    hard-coded ``/dev/ttyACM0`` and 1 Mbaud. The driver logs and ignores an unknown parameter.
  * The joint parameters ``inverted``, ``max_speed`` (rad/s), ``max_accel`` (rad/s^2) and
    ``unwrap`` are new. The ``type`` parameter is optional. A joint with only a velocity command
    interface is ``vel``, and any other joint is ``pos``.
  * The state interfaces ``effort`` (N m), ``current`` (A), ``voltage`` (V), ``load`` and
    ``status`` (the status byte of the servo) are new. A joint can declare any subset of the nine
    interfaces, in any order. The driver exports them in the order of the description.
  * By default, the driver unwraps wheel positions into a multi-turn count. The count stays when
    the hardware goes inactive and then active again. ``diff_drive_controller`` odometry no
    longer jumps once per revolution.
  * Batched feedback: the driver gets the feedback with one sync read (``INST_SYNC_READ``) per
    cycle, or one per 30 servos on a larger bus. A read for each servo is the fallback. The
    driver checks the header, id, length and checksum of every frame in a sync-read reply.
  * Batched goals: goals still go out as broadcast sync writes, as in 0.1.0. The driver now
    splits them at 30 position records or 82 speed records per packet. The driver writes the
    acceleration register of the wheels at four edges, not with an acknowledged write per wheel
    in every cycle.
  * The driver logs status-byte faults by name. It enables torque again when a fault clears.
  * Recovery: after ``max_read_fails`` failed reads in a row, the driver drops a servo from the
    read cycle and logs one ERROR. ``ros2 control set_hardware_component_state <name> inactive``,
    then ``active``, pings the servo again and adds it back.
  * ``on_shutdown`` and ``on_error`` stop the wheels, park the position joints and close the
    port. At deactivation, the driver logs a ``bus totals:`` line with the failed-transaction
    counts of each servo.
  * New tools: ``scan`` lists every servo that answers on ids 0..253. ``factory_reset`` sets the
    EEPROM of one servo back to factory values, except its id. ``set_id`` and
    ``calibrate_midpoint`` now check the servo before they write, and verify the result after.
  * New example launch arguments ``port``, ``baudrate`` and ``use_mock_hardware``.
    ``use_mock_hardware`` puts ``mock_components/GenericSystem`` in place of the driver and opens
    no port. The example also has a ``diff_drive_controller``, which the launch file loads
    inactive.
  * A devcontainer for Jazzy. It puts the container user in ``dialout`` explicitly. It passes
    ``--ulimit rtprio=99``, ``--ulimit memlock=-1`` and ``--cap-add=sys_nice``. The two
    ``--ulimit`` options raise RLIMIT_RTPRIO to 99 and RLIMIT_MEMLOCK to unlimited. Real-time
    scheduling and memory locking need these limits.

* Documentation:

  * ``README.md`` gives the set-up and the usage.
  * Eight pages in ``docs/`` give the details: ``setup.md``, ``configuration.md``,
    ``bus-timing.md``, ``operation.md``, ``tools.md``, ``bench-check.md``, ``design.md`` and
    ``development.md``. They document every parameter, the interfaces, start-up and recovery,
    real-time set-up, bus sizing, the tools, troubleshooting and the tests.
  * ``THIRD_PARTY.md`` records where the vendored SCServo library comes from, and what changed
    at import and after. It also gives the checksums and the procedure to refresh the library.

* Packaging:

  * ``package.xml`` has version 1.0.0 and the repository and bug tracker URLs. It names the
    license files. An Apache-2.0 entry covers the two files that keep their upstream Apache-2.0
    headers (see ``THIRD_PARTY.md``).
  * ``package.xml`` now declares dependencies that the package got only through other packages
    before. ``launch`` and ``launch_ros`` are for the example launch file. ``ros2run`` and
    ``ros2topic`` are for the commands in ``README.md`` and ``docs/``. The tests use
    ``ament_cmake_cpplint``, ``builtin_interfaces``, ``lifecycle_msgs``, ``rcutils``,
    ``python3-catkin-pkg-modules``, ``python3-docutils``, ``python3-pytest`` and
    ``python3-yaml``.
  * ``package.xml`` declares ``controller_manager`` as an exec dependency only, not as a test
    dependency. ``docs/development.md``, section "Development rules", gives the reason.
  * The package installs ``LICENSE``, ``LICENSES/Apache-2.0.txt`` and ``THIRD_PARTY.md`` to
    ``share/waveshare_servos``.
  * The ``rosdep install`` commands of ``README.md`` and of the devcontainer do not pass ``-r``.
    With ``-r``, rosdep reports success for a dependency that it cannot resolve. Without ``-r``,
    an unresolved dependency stops the devcontainer start-up with the rosdep error, before the
    build.

* Fixes:

  * ``torque`` cut the current to whole amperes, so it read 0 below 1 A.
  * A joint without an ``id`` or ``type`` parameter crashed the driver.
  * An absent servo stayed on the bus. In each cycle it cost four reads, and each read waited
    for the 100 ms timeout of the library.
  * Position moves jerked at the start and end of trajectories.
  * The installed example did not launch. The launch file looked for the URDF in the wrong
    place, and the package did not install the RViz configuration. RViz now also shows the robot
    model.
  * Vendored ``SCSerial``: after a read error, ``readSCS()`` wrote before the start of its
    buffer. ``wFlushSCS()`` cut partially written frames short. ``end()`` leaked the file
    descriptor, so the serial port became free only when the process stopped.
  * ``package.xml`` did not declare the build dependencies (``hardware_interface``,
    ``pluginlib``, ``rclcpp``, ``rclcpp_lifecycle``). It no longer declares the unused
    ``joint_state_publisher_gui``.
  * The example had ``<limit>`` velocities of 0.5 and 1.0 rad/s. These limits capped the joints
    if ``enforce_command_limits`` was on. Its effort limit (1000) was three orders of magnitude
    too high.

* Performance, with four ST3025 servos at 1 Mbaud and 100 Hz on the reference bench that
  ``docs/setup.md`` describes. ``docs/bus-timing.md``, section "Cycle cost", gives the details:

  * ``read()`` takes 1.645 ms per cycle instead of 3.064 ms. ``write()`` takes 0.014 ms instead
    of 1.447 ms. Together they use 16.6 % of the 10 ms period instead of 45 %. The comparison is
    with the earlier path of the branch, which read each servo separately.
  * Each extra servo costs about 0.29 ms. Thus about twelve joints fit at 100 Hz with 50 %
    headroom, up from five. The measurements used one to four servos. Above four, the figures
    come from a model.
  * Four ten-minute runs had 0 failed transactions in 244 158. This bounds the failure rate
    below 13 per million at 95 % confidence.

* Tests (0.1.0 had none):

  * A motorless suite. Its gtest/gmock cases let the driver and the tools talk to a simulated
    servo bus on a pseudo-terminal. It also has xacro render tests and a launch test of the
    example on mock hardware. ament lint runs with the vendored files excluded.
  * Document checks compare the reference tables in ``docs/`` (their names, and the hardware
    and launch defaults) and the quoted-message list with the code. They make sure that the
    vendored files match the checksums in ``THIRD_PARTY.md``. They also make sure that
    ``package.xml`` and ``CHANGELOG.rst`` are well-formed and name the same version.
  * ``test/hil_check.sh`` checks the real servos in 20 scenarios on the reference bench. It runs
    only with ``WAVESHARE_HIL=1``.

* Known limitations:

  * ``read()`` and ``write()`` always return OK, so a bus failure never triggers ``on_error``.
    The component stays active.
  * Activation enables torque before the first goal goes out. After ``calibrate_midpoint`` or a
    cold start, a joint can move toward an old goal for about one control period.
  * ``effort`` and ``current`` are approximate and never negative. Status bits 4, 6 and 7 are
    not verified.
  * A servo that loses power while it runs comes back with torque off. To recover it, deactivate
    and activate the hardware, and keep clear of the joint.
  * ``protocol`` accepts only ``sms_sts``. The driver does not support the SCS series
    (``scscl``).
  * JointTrajectoryController 4.42.1 crashes on velocity-only joints, so wheels need a velocity
    controller.
  * The check of ``factory_reset`` used one servo only, and the bench check has no scenario for
    it.
  * The ``io_timeout_ms`` default and floor are for 1 Mbaud and do not change with ``baudrate``.
  * If ``port`` is a ``/dev/serial/by-id`` link, the driver cannot name the holder of a busy
    port. The tools can.
  * The driver reads the mode write back, but does not check the results of the EEPROM unlock
    and lock.
  * ``test/hil_check.sh`` assumes the reference bench and does not check the bus before its
    first scenarios.
  * The driver does not report a disconnected USB adapter as a failure. Deactivation and
    activation do not open the port again.
  * Torque stays on after shutdown.

  ``docs/operation.md``, section "Known issues", is the maintained list, with the details and
  the workarounds.

0.1.0 (2025-07-18)
------------------
* The Humble version: a ros2_control ``SystemInterface`` for Waveshare/Feetech ST servos that
  use the SMS/STS protocol. The port ``/dev/ttyACM0`` and the rate of 1 Mbaud are hard-coded.
* Joint parameters ``id``, ``type`` (``pos``: servo mode 0, ``vel``: wheel mode 1) and
  ``offset`` (rad). The command interfaces are ``position`` and ``velocity``. Every joint
  declares exactly four state interfaces, in this order: ``position``, ``velocity``, ``torque``
  (kgf cm) and ``temperature``.
* Position commands start from the measured position, so the robot does not move when it
  activates.
* The ``set_id`` and ``calibrate_midpoint`` tools, with the parameters ``device_port`` and
  ``baud_rate``.
* An example description with three joints (two position joints and one wheel), with a launch
  file and a controller configuration.
* The vendored Feetech/Waveshare SCServo library.
