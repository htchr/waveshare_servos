cmake_minimum_required(VERSION 3.16)
project(waveshare_servos LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the same behavior for Windows as it is on Linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# ON: deprecated API in the driver targets fails the build. Set OFF for a newer ros2_control;
# it is a cache variable, so it stays OFF until you set it ON again.
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

# Vendored SCServo library (read-only), built apart so its warnings are handled here. Not
# installed, but its headers are: waveshare_servos.hpp -> servo_bus.hpp -> SMS_STS.h.
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

# The only access to the serial bus: vendored packets plus the exclusive port lock. Linted, full
# warnings (no VLA), not installed. See docs/design.md, "Code layout".
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

# ROS-free tool logic, tested in-process on the fake bus; each tool main is one line. The tools
# share servo_bus's port lock, so they are refused while a controller manager holds the port.
add_library(servo_tools STATIC src/servo_tools.cpp)
set_target_properties(servo_tools PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(servo_tools PUBLIC cxx_std_17)
target_include_directories(servo_tools PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>)
target_link_libraries(servo_tools PUBLIC servo_bus)

# tool_main.cpp calls rcl's parameter-override API directly, so it links rcl and
# rcl_yaml_param_parser itself instead of through rclcpp.
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
install(FILES LICENSE THIRD_PARTY.md DESTINATION share/${PROJECT_NAME})
install(FILES LICENSES/Apache-2.0.txt DESTINATION share/${PROJECT_NAME}/LICENSES)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  # Keep above the first pytest test: ament_add_pytest_test() runs Python3::Interpreter, and a
  # missing target is no configure error; the test fails only when it runs.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # The package's own sources carry no license header (two keep an upstream Apache-2.0 header),
  # so ament_copyright is off and cpplint runs below without legal/copyright.
  set(ament_cmake_copyright_FOUND TRUE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)

  # The vendored files are not edited, so cppcheck, cpplint, flake8 and uncrustify skip them.
  # Paths are relative to this directory; each entry names one file.
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
  # Finds every <test_depend>. controller_manager is only an exec_depend: as a test_depend its
  # CMake config warns. See docs/development.md, "Development rules".
  ament_lint_auto_find_test_dependencies()

  # cpplint by hand, without legal/copyright. Not ament_cpplint(): its one-value EXCLUDE would put
  # --filters inside the exclusion list, and cpplint would lint the vendored files.
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

  # Loads the installed plugin through pluginlib but never configures it, because configure would
  # open /dev/ttyACM0. test/ holds the shared test helpers.
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

  # The unwrapper is header-only: an include path, nothing to compile in or link.
  ament_add_gmock(test_position_unwrapper test/test_position_unwrapper.cpp)
  target_include_directories(test_position_unwrapper PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  # This target links nothing, so it inherits no standard from a dependency.
  target_compile_features(test_position_unwrapper PRIVATE cxx_std_17)

  # Links hardware_interface for its exported stod and parse_bool (not header-only). src/ for
  # param_parsing.hpp, which is not installed.
  ament_add_gmock(test_param_parsing test/test_param_parsing.cpp)
  target_link_libraries(test_param_parsing hardware_interface::hardware_interface)
  target_include_directories(test_param_parsing PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

  # Compiles the public header after a consumer's own visibility_controls.h guard. Links the
  # plugin only for its include paths and dependencies; it calls no plugin code.
  ament_add_gmock(test_visibility_controls test/test_visibility_controls.cpp)
  target_link_libraries(test_visibility_controls waveshare_servos)

  # ServoBus over an openpty() pair, vendored code unchanged; no plugin, no motors. openpty is in
  # libc since glibc 2.34, so do not link util.
  ament_add_gmock(test_servo_bus test/test_servo_bus.cpp)
  target_link_libraries(test_servo_bus servo_bus)
  target_include_directories(test_servo_bus PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # The full lifecycle (pluginlib and a ResourceManager) against the fake servo bus on a pty.
  # Every case sets its own port, so /dev/ttyACM0 is never opened.
  ament_add_gmock(test_lifecycle_over_pty test/test_lifecycle_over_pty.cpp)
  target_link_libraries(test_lifecycle_over_pty
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
  target_include_directories(test_lifecycle_over_pty PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # Its rclcpp cases create a node, so it gets a private ROS graph (domain 85, one per test).
  # See docs/development.md, "Launch and render tests".
  ament_add_gmock(test_tool_params test/test_tool_params.cpp
    ENV ROS_DOMAIN_ID=85 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  target_link_libraries(test_tool_params tool_cli)

  # Tool logic on the fake bus, in-process. TIMEOUT 240: each scan pings 254 ids 3 times at 5 ms
  # (about 3.8 s), and about a dozen cases scan.
  ament_add_gmock(test_servo_tools test/test_servo_tools.cpp TIMEOUT 240)
  target_link_libraries(test_servo_tools servo_tools)
  target_include_directories(test_servo_tools PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)

  # Runs the built tools against a fake bus (ids 11-14, not the bench's 1-4) and holds
  # /dev/ttyACM0 meanwhile. Domain 86; src/ for defaults::kPort.
  ament_add_gmock(test_tools_cli test/test_tools_cli.cpp TIMEOUT 240
    ENV ROS_DOMAIN_ID=86 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
        "WAVESHARE_TOOL_SCAN=$<TARGET_FILE:scan>" "WAVESHARE_TOOL_SET_ID=$<TARGET_FILE:set_id>"
        "WAVESHARE_TOOL_CALIBRATE_MIDPOINT=$<TARGET_FILE:calibrate_midpoint>"
        "WAVESHARE_TOOL_FACTORY_RESET=$<TARGET_FILE:factory_reset>")
  target_link_libraries(test_tools_cli servo_bus)
  target_include_directories(test_tools_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/test)
  add_dependencies(test_tools_cli scan set_id calibrate_midpoint factory_reset)

  # Renders the example xacro and checks which <plugin> it selects; no ROS graph, no port. A
  # render takes about 0.07 s, so TIMEOUT 60 only bounds a hung import.
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py TIMEOUT 60)

  # The release documents, checked against the code and the vendored files: no motors, no port,
  # no ROS graph. Each reads the source tree through its own __file__.
  ament_add_pytest_test(test_release_metadata test/test_release_metadata.py TIMEOUT 60)
  ament_add_pytest_test(test_readme test/test_readme.py TIMEOUT 60)
  ament_add_pytest_test(test_vendored_files test/test_vendored_files.py TIMEOUT 60)

  # Real bringup on mock hardware, private graph (domain 88, never hil_check's 77). TIMEOUT 330
  # stays above the test's ~295 s worst case. See docs/development.md, "Launch and render tests".
  add_launch_test(test/test_example_launch.py
    TARGET test_example_launch
    TIMEOUT 330
    ENV "ROS_DOMAIN_ID=88" "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
  )
  # RUN_SERIAL: all controller spawners on a machine share one lock file, so a parallel bringup
  # waits, and its spawner gives up after about 115 s.
  set_tests_properties(test_example_launch PROPERTIES RUN_SERIAL TRUE)

  # Bench check: runs only with WAVESHARE_HIL=1 and an adapter, else exits 77 (skip); it moves
  # the motors. Its helpers talk to the tty without ROS; test fixtures, not installed.
  add_executable(hil_stop_wheels test/hil/stop_wheels.cpp)
  target_link_libraries(hil_stop_wheels scservo)

  add_executable(hil_port_probe test/hil/port_probe.cpp)
  target_compile_features(hil_port_probe PRIVATE cxx_std_17)

  # The bench's EEPROM oracle and repair tool. Independent of servo_tools ON PURPOSE: a bug shared
  # by the tools and the repair would hide itself. Not installed.
  add_executable(hil_eeprom test/hil/eeprom.cpp test/hil/eeprom_core.cpp)
  target_link_libraries(hil_eeprom servo_bus)
  target_include_directories(hil_eeprom PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test/hil)
  # TIMEOUT 180: each restore case runs hil_eeprom's census (254 ids x 2 pings at 5 ms, ~2.5 s)
  # before and after its writes; 56 s in all, measured.
  ament_add_gmock(test_hil_eeprom test/test_hil_eeprom.cpp test/hil/eeprom_core.cpp TIMEOUT 180
    ENV "WAVESHARE_HIL_EEPROM_BIN=$<TARGET_FILE:hil_eeprom>")
  target_link_libraries(test_hil_eeprom servo_bus)
  target_include_directories(test_hil_eeprom PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test ${CMAKE_CURRENT_SOURCE_DIR}/test/hil)
  add_dependencies(test_hil_eeprom hil_eeprom)
  # Both hold the real /dev/ttyACM0 (test_tools_cli for its whole run, test_hil_eeprom for one
  # case), so ctest -j must not run them together. hil_check is RUN_SERIAL already.
  set_tests_properties(test_tools_cli test_hil_eeprom
    PROPERTIES RESOURCE_LOCK waveshare_default_port)

  # The bench-check gate logic, proved without a bench on synthetic series with injected
  # defects. No hardware, no ROS, no run directory.
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
    # The script writes no JUnit file: with this flag rc 0 records a pass, not a missing-result
    # error. rc 77 is still a skip, and any other rc a failure.
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 2400
  )
  # RUN_SERIAL: nothing else may use the port. Exit 77 is a skip, in the xunit file too.
  # ctest -L hil selects this test; -LE hil leaves it out.
  set_tests_properties(hil_check PROPERTIES RUN_SERIAL TRUE LABELS "hil")
endif()

# Exports
ament_export_targets(export_waveshare_servos HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
