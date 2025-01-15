cmake_minimum_required(VERSION 3.16)
project(waveshare_servos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the same behavior for Windows as it is on Linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

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

# Compile
add_library(
  waveshare_servos
  SHARED
  src/waveshare_servos.cpp
  src/SCSCL.cpp
  src/SCS.cpp
  src/SCSerial.cpp
  src/SMSBL.cpp
  src/SMSCL.cpp
  src/SMS_STS.cpp
)
target_compile_features(waveshare_servos PUBLIC cxx_std_17)
target_include_directories(waveshare_servos PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/waveshare_servos>
)

ament_target_dependencies(
  waveshare_servos PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Add the SetId node
add_executable(set_id src/set_id.cpp)
target_include_directories(set_id PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(set_id waveshare_servos)
ament_target_dependencies(set_id rclcpp)

# Add the CalibMid node
add_executable(calibrate_midpoint src/calibrate_midpoint.cpp)
target_include_directories(calibrate_midpoint PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(calibrate_midpoint waveshare_servos)
ament_target_dependencies(calibrate_midpoint rclcpp)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface waveshare_servos.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/waveshare_servos
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/waveshare_servos
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/waveshare_servos
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

# Exports
ament_export_targets(export_waveshare_servos HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
