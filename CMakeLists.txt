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

# Tools
add_executable(set_id src/set_id.cpp)
target_link_libraries(set_id PRIVATE scservo rclcpp::rclcpp)

add_executable(calibrate_midpoint src/calibrate_midpoint.cpp)
target_link_libraries(calibrate_midpoint PRIVATE scservo rclcpp::rclcpp)

if(WAVESHARE_SERVOS_WERROR_DEPRECATED AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(driver_target IN ITEMS waveshare_servos servo_bus set_id calibrate_midpoint)
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
  TARGETS set_id calibrate_midpoint
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

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

  # The gate logic is the part of the bench check that can be proved without a bench: the
  # self-test runs the three invariant gates over synthetic series with injected defects and
  # needs no hardware, no ROS and no run directory (PHASE2_SPEC 11.2, 12.3).
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
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
        "WAVESHARE_HIL_HELPERS=${CMAKE_CURRENT_SOURCE_DIR}/test/hil"
    SKIP_RETURN_CODE 77
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
