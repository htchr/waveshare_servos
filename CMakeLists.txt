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

# ros2_control hardware plugin
add_library(waveshare_servos SHARED
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
    scservo
)

# Tools
add_executable(set_id src/set_id.cpp)
target_link_libraries(set_id PRIVATE scservo rclcpp::rclcpp)

add_executable(calibrate_midpoint src/calibrate_midpoint.cpp)
target_link_libraries(calibrate_midpoint PRIVATE scservo rclcpp::rclcpp)

if(WAVESHARE_SERVOS_WERROR_DEPRECATED AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  foreach(driver_target IN ITEMS waveshare_servos set_id calibrate_midpoint)
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
  ament_add_gmock(test_load_waveshare_servos test/test_load_waveshare_servos.cpp)
  target_link_libraries(test_load_waveshare_servos
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )

  # Compiles the public header the way a dependent package with its own
  # visibility_controls.h does. Links the plugin target for its include
  # directories and dependencies; the test calls no plugin code.
  ament_add_gmock(test_visibility_controls test/test_visibility_controls.cpp)
  target_link_libraries(test_visibility_controls waveshare_servos)
endif()

# Exports
ament_export_targets(export_waveshare_servos HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
