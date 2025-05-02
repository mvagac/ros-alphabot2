cmake_minimum_required(VERSION 3.8)
project(alphabot2-diffdrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pal_statistics_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)

add_library(alphabot2_diffdrive SHARED src/alphabot2_diffdrive.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_include_directories(
  alphabot2_diffdrive
  PRIVATE
  include
)

ament_target_dependencies(
  alphabot2_diffdrive PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(alphabot2_diffdrive PRIVATE
  pigpio gpiod
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(
  TARGETS alphabot2_diffdrive
  EXPORT alphabot2_diffdrive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  alphabot2_diffdrive
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


ament_package()
