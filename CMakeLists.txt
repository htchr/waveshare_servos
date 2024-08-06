cmake_minimum_required(VERSION 3.16)
project(waveshare_servos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS 
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# compile
add_library(
  waveshare_servos
  SHARED
  hardware/waveshare_servos.cpp
  hardware/SCSCL.cpp
  hardware/SCS.cpp
  hardware/SCSerial.cpp
  hardware/SMSBL.cpp
  hardware/SMSCL.cpp
  hardware/SMS_STS.cpp
)
target_compile_features(waveshare_servos PUBLIC cxx_std_17)
target_include_directories(waveshare_servos PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/waveshare_servos>
)
ament_target_dependencies(
  waveshare_servos PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface waveshare_servos.xml)

# install
install(
  DIRECTORY hardware/include/
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

# if(BUILD_TESTING)
# endif()

# exports
ament_export_targets(export_waveshare_servos HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
