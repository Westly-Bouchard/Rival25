cmake_minimum_required(VERSION 3.15)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

project(odesc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hardware_interface)
find_package(pluginlib)
find_package(rclcpp)
find_package(rclcpp_lifecycle)

# Compile
add_library(
  odesc
  SHARED
  src/odesc_hw_interface.cpp
)
target_compile_features(odesc PUBLIC cxx_std_17)
target_include_directories(odesc PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/odesc>
)
target_link_libraries(odesc PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface odesc_plugin.xml)

# Install
install(
  DIRECTORY include
  DESTINATION include/odesc
)
install(TARGETS odesc
  EXPORT export_odesc
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

# Exports
ament_export_targets(export_odesc HAS_LIBRARY_TARGET)
ament_package()
