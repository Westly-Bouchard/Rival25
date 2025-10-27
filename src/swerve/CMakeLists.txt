cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(swerve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(odesc_driver REQUIRED)

add_executable(testPublisher src/test_publisher.cpp)
ament_target_dependencies(testPublisher rclcpp std_msgs sensor_msgs)

install(TARGETS testPublisher DESTINATION lib/${PROJECT_NAME})

# Compile
add_library(
  swerve
  SHARED
  src/swerve_module_interface.cpp
  src/swerve_controller.cpp
)

target_compile_features(swerve PUBLIC cxx_std_17)

target_include_directories(swerve PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/swerve>
)

target_link_libraries(swerve PUBLIC
  hardware_interface::hardware_interface
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  odesc_driver::odesc_driver
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface swerve_plugin.xml)

# Export controller plugin
pluginlib_export_plugin_description_file(controller_interface swerve_plugin.xml)

# Install
install(
  DIRECTORY include
  DESTINATION include/swerve
)

install(TARGETS swerve
  EXPORT export_swerve
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY test/config test/launch test/urdf
  DESTINATION share/swerve
)

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

ament_export_targets(export_swerve HAS_LIBRARY_TARGET)
ament_package()
