cmake_minimum_required(VERSION 3.16)
project(waveshare_servos LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the same behavior for Windows as it is on Linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Deprecated ros2_control / rclcpp API in the driver-owned targets fails the build.
# Switch off only to build an unmigrated driver, or against a newer ros2_control.
# This is a cache variable: once set to OFF it stays OFF until set back to ON.
option(WAVESHARE_SERVOS_WERROR_DEPRECATED
  "Treat deprecation warnings in the driver targets as errors" ON)

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Vendored Waveshare/Feetech SCServo library: third-party, read-only sources.
# Built on its own so its warnings can be handled here instead of in the sources.
# It is linked into the plugin and the tools, never installed or exported.
add_library(scservo STATIC
  src/SCS.cpp
  src/SCSCL.cpp
  src/SCSerial.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
  src/SMS_STS.cpp
)
set_target_properties(scservo PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(scservo PUBLIC cxx_std_17)
target_include_directories(scservo PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # the sync-write helpers size a variable length array from the servo count
  target_compile_options(scservo PRIVATE -Wno-vla)
endif()

# The package's only door to the serial bus: the vendored packet layer plus exclusive access to
# the tty. Driver-owned, so it is linted and gets the package-wide warnings in full -- no -Wno-vla,
# it contains no variable length array. Like scservo it is neither installed nor exported; the
# plugin links it privately and the pty test links it directly.
add_library(servo_bus STATIC src/servo_bus.cpp)
set_target_properties(servo_bus PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(servo_bus PUBLIC cxx_std_17)
target_include_directories(servo_bus PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(servo_bus PUBLIC scservo)

# ros2_control hardware plugin
add_library(waveshare_servos SHARED
  src/units.cpp
  src/waveshare_servos.cpp
)
target_compile_features(waveshare_servos PUBLIC cxx_std_17)
target_include_directories(waveshare_servos PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/waveshare_servos>
)
target_link_libraries(waveshare_servos
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
    servo_bus
)

# Tools (jazzy.md Phase 6). Their logic is a ROS-free library the tests drive against the fake
# bus in-process; each executable is one line. They reach the bus only through servo_bus, which
# PUBLIC-links the same scservo target as the plugin (Phase 6 item 4), so they take the same
# exclusive lock and are refused while the controller manager holds the port.
add_library(servo_tools STATIC src/servo_tools.cpp)
set_target_properties(servo_tools PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(servo_tools PUBLIC cxx_std_17)
target_include_directories(servo_tools PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
target_link_libraries(servo_tools PUBLIC servo_bus)

# tool_main.cpp reads the global parameter overrides through rcl's own API
# (rcl_arguments_get_param_overrides, rcl_yaml_node_struct_fini), so it names rcl and
# rcl_yaml_param_parser itself rather than leaning on rclcpp to carry them (review fix F33).
find_package(rcl REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
add_library(tool_cli STATIC src/tool_params.cpp src/tool_main.cpp)
target_link_libraries(tool_cli PUBLIC servo_tools rclcpp::rclcpp rcl::rcl
  rcl_yaml_param_parser::rcl_yaml_param_parser)

foreach(tool IN ITEMS scan set_id calibrate_midpoint factory_reset)
  add_executable(${tool} src/${tool}.cpp)
  target_link_libraries(${tool} PRIVATE tool_cli)
endforeach()

if(WAVESHARE_SERVOS_WERROR_DEPRECATED AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(driver_target IN ITEMS
      waveshare_servos servo_bus servo_tools tool_cli scan set_id calibrate_midpoint factory_reset)
    target_compile_options(${driver_target} PRIVATE -Werror=deprecated-declarations)
  endforeach()
endif()

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface waveshare_servos.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/waveshare_servos
)
install(
  DIRECTORY description/
  DESTINATION share/${PROJECT_NAME}/description
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS waveshare_servos
  EXPORT export_waveshare_servos
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS scan set_id calibrate_midpoint factory_reset
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # This used to sit next to hil_gates_self_test, its only user; it is up here now because the
  # pytest test below needs it too and needs it first. ament_add_pytest_test() defaults
  # PYTHON_EXECUTABLE to the imported target Python3::Interpreter and resolves it through
  # get_executable_path(... BUILD) (ament_add_pytest_test.cmake:73-77 under
  # /opt/ros/jazzy/share/ament_cmake_pytest/cmake). Measured, not assumed: a missing target is
  # NOT a configure error there -- get_executable_path() hands back whatever it was given when the
  # name is not a target (get_executable_path.cmake:49-50), so the ctest command would quietly
  # become the literal string "Python3::Interpreter" and only fail when the test runs. And a
  # second measurement: find_package(ament_cmake) at the top of this file already defines
  # Python3::Interpreter, so the order is not load-bearing today. Both facts say the same thing --
  # keep the find_package explicit and keep it above its first user, rather than resting on an
  # accident of what ament_cmake happens to pull in.
  # add_launch_test() needs none of this: it falls back to the plain ${PYTHON_EXECUTABLE} variable
  # that the PythonExtra module sets, which launch_testing_ament_cmake-extras.cmake includes
  # (add_launch_test.cmake:62-69).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # The sources carry no license header, so the two linters that ask for one are
  # switched off: ament_copyright entirely, and cpplint's legal/copyright rule.
  # The license of the package is declared in package.xml and LICENSE.
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)

  # The vendored SCServo files are third-party and are not edited, so the linters
  # that honor this list (cppcheck, cpplint, flake8, uncrustify) skip them.
  # Paths are relative to this directory; each entry must name a file.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    include/INST.h
    include/SCS.h
    include/SCSCL.h
    include/SCSerial.h
    include/SCServo.h
    include/SMSBL.h
    include/SMSCL.h
    include/SMS_STS.h
    src/SCS.cpp
    src/SCSCL.cpp
    src/SCSerial.cpp
    src/SMSBL.cpp
    src/SMSCL.cpp
    src/SMS_STS.cpp
  )
  ament_lint_auto_find_test_dependencies()

  # cpplint, registered here rather than by ament_lint_auto so its copyright rule
  # can be dropped, over the same vendored exclusions. ament_cpplint() itself is
  # not used: its EXCLUDE is a one-value keyword, so it would place --filters in
  # the middle of the exclusion list and cpplint would then lint the vendored
  # files as positional arguments. --filters therefore comes before --exclude.
  find_package(ament_cmake_cpplint REQUIRED)
  find_program(AMENT_CPPLINT_BIN NAMES "ament_cpplint" REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/ament_cpplint")
  ament_add_test(
    cpplint
    COMMAND "${AMENT_CPPLINT_BIN}"
      "--xunit-file" "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/cpplint.xunit.xml"
      "--filters=-legal/copyright"
      "--exclude" ${AMENT_LINT_AUTO_FILE_EXCLUDE}
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/ament_cpplint/cpplint.txt"
    RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/cpplint.xunit.xml"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    TIMEOUT 120
  )
  set_tests_properties(cpplint PROPERTIES LABELS "cpplint;linter")

  # Loads the plugin through pluginlib from the ament index (the installed
  # package), so the test is not linked against it. It never configures the
  # plugin: the port is still hard-coded and configure would open it.
  # test/test_helpers.hpp and test/test_support.hpp are the shared test helpers; the
  # include directory is named so a test in a subdirectory can include them too.
  ament_add_gmock(test_load_waveshare_servos test/test_load_waveshare_servos.cpp)
  target_link_libraries(test_load_waveshare_servos
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_include_directories(test_load_waveshare_servos PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # The units are pure arithmetic: no ros2_control, no bus, no motors. src/units.cpp is compiled
  # into the test rather than linked from the plugin, so the test needs none of its dependencies.
  ament_add_gmock(test_units test/test_units.cpp src/units.cpp)
  target_include_directories(test_units PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # This target links nothing, so it inherits no standard: without this it would compile the shared
  # src/units.cpp at whatever the compiler's default happens to be.
  target_compile_features(test_units PRIVATE cxx_std_17)

  # The unwrapper is header-only and pure: integer arithmetic over a tick sequence, with no
  # ros2_control, no rclcpp, no bus and no motors. Only the include directory is needed -- there is
  # no source file to compile in and nothing to link.
  ament_add_gmock(test_position_unwrapper test/test_position_unwrapper.cpp)
  target_include_directories(test_position_unwrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # This target links nothing, so it inherits no standard from a dependency.
  target_compile_features(test_position_unwrapper PRIVATE cxx_std_17)

  # The parameter getters are pure string handling: no ROS objects, no URDF, no bus, no motors.
  # The link line is required, not optional: hardware_interface::stod and ::parse_bool are
  # exported functions in libhardware_interface.so, not header-only. src/ is on the include path
  # because src/param_parsing.hpp is driver-owned and deliberately not installed.
  ament_add_gmock(test_param_parsing test/test_param_parsing.cpp)
  target_link_libraries(test_param_parsing hardware_interface::hardware_interface)
  target_include_directories(test_param_parsing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # Compiles the public header the way a dependent package with its own
  # visibility_controls.h does. Links the plugin target for its include
  # directories and dependencies; the test calls no plugin code.
  ament_add_gmock(test_visibility_controls test/test_visibility_controls.cpp)
  target_link_libraries(test_visibility_controls waveshare_servos)

  # Drives ServoBus through an openpty() pair so the vendored packet code runs unchanged: no
  # ros2_control, no plugin, no motors. openpty has been in libc since glibc 2.34, so do NOT add
  # `util` -- there is no libutil.so dev stub installed here and -lutil would fail to link.
  ament_add_gmock(test_servo_bus test/test_servo_bus.cpp)
  target_link_libraries(test_servo_bus servo_bus)
  target_include_directories(test_servo_bus PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # Drives the whole component lifecycle -- the plugin loaded through pluginlib from the ament
  # index, behind a ResourceManager -- against the fake servo bus of test/fake_servo_bus.hpp on an
  # openpty() pair. No motors, no adapter, no /dev/ttyACM0: every case passes its own
  # <param name="port">. Like test_servo_bus it must NOT link `util`; openpty has been in libc
  # since glibc 2.34 and there is no libutil.so dev stub installed here.
  ament_add_gmock(test_lifecycle_over_pty test/test_lifecycle_over_pty.cpp)
  target_link_libraries(test_lifecycle_over_pty
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_include_directories(test_lifecycle_over_pty PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # Its rclcpp-backed cases create a node and a context, so it gets a private graph like the
  # other ROS tests: 85 is not 77 (hil_check.sh), 86 (test_tools_cli) or 88 (test_example_launch).
  ament_add_gmock(test_tool_params test/test_tool_params.cpp
    ENV ROS_DOMAIN_ID=85 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_tool_params tool_cli)

  # The tools' bus logic against the fake bus, in-process. TIMEOUT 240, not the default 60: every
  # scan case pings all 254 ids three times at 5 ms (C.1, about 3.8 s a scan), and there are a
  # dozen of them.
  ament_add_gmock(test_servo_tools test/test_servo_tools.cpp TIMEOUT 240)
  target_link_libraries(test_servo_tools servo_tools)
  target_include_directories(test_servo_tools PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # Spawns the BUILT executables against a FakeBus pty (fake ids 11-14, never the bench's 1-4) and
  # holds /dev/ttyACM0 with DefaultPortGuard while it does. Domain 86: not 77 (hil_check.sh), not 88
  # (test_example_launch). Do NOT link util (openpty is in libc). src/ is named because the test
  # reads defaults::kPort and servo_bus exports only include/.
  ament_add_gmock(test_tools_cli test/test_tools_cli.cpp TIMEOUT 240
    ENV ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        "WAVESHARE_TOOL_SCAN=$<TARGET_FILE:scan>" "WAVESHARE_TOOL_SET_ID=$<TARGET_FILE:set_id>"
        "WAVESHARE_TOOL_CALIBRATE_MIDPOINT=$<TARGET_FILE:calibrate_midpoint>"
        "WAVESHARE_TOOL_FACTORY_RESET=$<TARGET_FILE:factory_reset>")
  target_link_libraries(test_tools_cli servo_bus)
  target_include_directories(test_tools_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_dependencies(test_tools_cli scan set_id calibrate_midpoint factory_reset)

  # The example description renders to different XML depending on use_mock_hardware, and which
  # <plugin> ros2_control ends up loading is decided there, at xacro render time, before any
  # ResourceManager exists -- so no gmock test above can reach it. This one renders the text and
  # reads it back: no ROS graph, no plugin, no port.
  # TIMEOUT 60 is ament_add_test's own default written out (ament_add_test.cmake:75-77); a render
  # of description/urdf/example.urdf.xacro measures 0.07 s here for 8,891 bytes of URDF, so the
  # timeout bounds a hung import, not the work. Note the file has to exist when cmake runs:
  # ament_add_pytest_test() checks the path itself and stops the configure with a FATAL_ERROR if
  # it is missing (ament_add_pytest_test.cmake:64-71).
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py TIMEOUT 60)

  # Brings the example up on mock hardware and waits for the controllers to become active. It is
  # the only motorless test that runs the real bringup -- ros2_control_node, robot_state_publisher
  # and the spawners -- which is what the ENV and RUN_SERIAL below are for.
  #
  # ENV gives it a private ROS graph. The domain must NOT be 77: test/hil_check.sh:69 re-execs
  # itself with ROS_DOMAIN_ID=77, and on a shared domain this test would discover the bench's
  # controller manager (or the bench ours) and assert against the wrong stack. LOCALHOST discovery
  # keeps it off the LAN as well. ENV is not an add_launch_test keyword -- that function parses
  # only TARGET, TIMEOUT, PYTHON_EXECUTABLE, ARGS and LABELS (add_launch_test.cmake:52-55) and
  # forwards everything else verbatim to ament_add_test, where ENV is a multi-value VAR=value
  # list (ament_add_test.cmake:56-59).
  #
  # TIMEOUT 330 replaces the 60 s default (ament_add_test.cmake:75-77), and the number is chosen
  # to sit ABOVE this test's own worst case rather than below it. test/test_example_launch.py
  # budgets ~295 s for a run in which nothing comes up at all: 45 s of ReadyToTest plus five test
  # methods, two of whose helpers (check_controllers_running, check_if_js_published) spend their
  # timeout twice over -- the arithmetic is written out at the STARTUP_TIMEOUT constant there.
  # This was TIMEOUT 180, which broke the invariant that same comment states: a total failure
  # would have been SIGKILLed by ctest at 180 s and reported as "test timed out", throwing away
  # the assertion message that says WHICH of the five things failed. Raising the ctest bound
  # rather than shortening the in-test waits is the deliberate half of that trade: the waits are
  # long on purpose, because ctest may run this on a loaded machine and because every controller
  # spawner on the machine serialises on one lock file (see RUN_SERIAL below), so a shorter wait
  # would buy a readable failure by manufacturing flaky ones. The 35 s above 295 is headroom for
  # launch shutdown and the post-shutdown test; none of it is paid in the normal case, where that
  # same comment records a measured 7 s end to end. Keep the two numbers in step: the sum lives
  # in test_example_launch.py, the bound lives here.
  #
  # TARGET is given explicitly because the ctest name is what set_tests_properties below needs:
  # left to itself add_launch_test() derives that name from the path and this test would be
  # called test_test_example_launch (add_launch_test.cmake:84-90).
  add_launch_test(test/test_example_launch.py
    TARGET test_example_launch
    TIMEOUT 330
    ENV "ROS_DOMAIN_ID=88" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
  )
  # RUN_SERIAL for the same reason hil_check has it, but over a different resource: every
  # controller spawner on this machine serialises on one lock file,
  # ~/.ros/locks/ros2-control-controller-spawner.lock, and holds it while it waits for its
  # controller manager (see bringup/launch/example.launch.py:156-161). Two tests spawning
  # controllers at once would queue behind each other inside their own timeouts.
  set_tests_properties(test_example_launch PROPERTIES RUN_SERIAL TRUE)

  # Bench check. Runs only with WAVESHARE_HIL=1 and a real adapter; on any other
  # machine it exits 77 and ctest records a skip. It drives the motors, so it is
  # never part of a default build's expectations.
  #
  # Its two helper binaries speak to the tty without ROS, so the shell script can act while the
  # port is free. hil_stop_wheels needs the vendored packet layer, which the scservo target
  # already compiles with -Wno-vla for the VLA at src/SMS_STS.cpp:56; port_probe opens the tty
  # and nothing else. NEITHER IS INSTALLED: they are test fixtures, not package tools.
  add_executable(hil_stop_wheels test/hil/stop_wheels.cpp)
  target_link_libraries(hil_stop_wheels scservo)

  add_executable(hil_port_probe test/hil/port_probe.cpp)
  target_compile_features(hil_port_probe PRIVATE cxx_std_17)

  # The bench's EEPROM oracle and repair tool. Independent of servo_tools ON PURPOSE: a bug shared
  # by the tools and the repair would hide itself. Not installed.
  add_executable(hil_eeprom test/hil/eeprom.cpp test/hil/eeprom_core.cpp)
  target_link_libraries(hil_eeprom servo_bus)
  target_include_directories(hil_eeprom PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/hil)
  # TIMEOUT 180, not the default 60: every restore case pays for hil_eeprom's own census, 254 ids
  # at two 5 ms pings each (E.1), before and after its writes -- 2.5 s a census, 56 s in all,
  # measured.
  ament_add_gmock(test_hil_eeprom test/test_hil_eeprom.cpp test/hil/eeprom_core.cpp TIMEOUT 180
    ENV "WAVESHARE_HIL_EEPROM_BIN=$<TARGET_FILE:hil_eeprom>")
  target_link_libraries(test_hil_eeprom servo_bus)
  target_include_directories(test_hil_eeprom PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test ${CMAKE_CURRENT_SOURCE_DIR}/test/hil)
  add_dependencies(test_hil_eeprom hil_eeprom)
  # Both take the real /dev/ttyACM0 (test_tools_cli's DefaultPortGuard for its whole run,
  # compare_opens_no_port for one case), so `ctest -j` must never run them side by side: one would
  # find the port busy because of the other. hil_check is RUN_SERIAL already.
  set_tests_properties(test_tools_cli test_hil_eeprom
    PROPERTIES RESOURCE_LOCK waveshare_default_port)

  # The gate logic is the part of the bench check that can be proved without a bench: the
  # self-test runs the three invariant gates over synthetic series with injected defects and
  # needs no hardware, no ROS and no run directory (PHASE2_SPEC 11.2, 12.3).
  ament_add_test(hil_gates_self_test
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/hil/hil_gates.py"
            --self-test
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/hil_gates_self_test.xunit.xml"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )

  file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/hil_check")
  ament_add_test(hil_check
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/hil_check.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    OUTPUT_FILE "${CMAKE_BINARY_DIR}/hil_check/hil_check.txt"
    RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/hil_check.xunit.xml"
    ENV "WAVESHARE_HIL_STOP_WHEELS=$<TARGET_FILE:hil_stop_wheels>"
        "WAVESHARE_HIL_PORT_PROBE=$<TARGET_FILE:hil_port_probe>"
        "WAVESHARE_HIL_EEPROM=$<TARGET_FILE:hil_eeprom>"
        "WAVESHARE_HIL_HELPERS=${CMAKE_CURRENT_SOURCE_DIR}/test/hil"
    SKIP_RETURN_CODE 77
    # Added in Phase 5, and it fixes a real defect rather than tidying one. The script writes its
    # own report (hil_check.txt / hil_check.json) but never a JUnit file, so the RESULT_FILE above
    # was always absent afterwards -- and without this flag ament treats an absent result file as
    # an error even when the command exited 0, synthesising
    # `hil_check.xunit.missing_result` (ament_cmake_test/__init__.py:263, and the template at
    # :364). The effect was that a bench run in which every row PASSed still came back through
    # `colcon test-result` as "1 error", which is the worst possible reading of a green run: it
    # trains you to ignore the one test that drives the hardware. With the flag, rc 0 writes a
    # passing result (__init__.py:237-242) and rc 77 still writes a skipped one (:257); a genuine
    # non-zero exit, which is how this script reports a FAIL or an abort, keeps falling through to
    # the failure path, so a red run stays red and carries the tail of the output with it.
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 2400
  )
  # RUN_SERIAL because nothing else may touch the port; SKIP_RETURN_CODE 77 is what makes a
  # motorless `colcon test` report a skip rather than a failure (it reaches ament_add_test too,
  # so the xunit file says skipped as well); the label runs or excludes it with -L / -LE hil.
  set_tests_properties(hil_check PROPERTIES RUN_SERIAL TRUE LABELS "hil")
endif()

# Exports
ament_export_targets(export_waveshare_servos HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
