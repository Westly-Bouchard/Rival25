cmake_minimum_required(VERSION 3.15)

# This is just so the CI runner won't complain
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Start ROS Package Build
project(odesc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(odescAxisEncoderOnlyNode
src/axis_rx_interface.cpp
src/axis_encoder_only_node.cpp
src/mixins/util.cpp
)
ament_target_dependencies(odescAxisEncoderOnlyNode rclcpp std_msgs)

add_executable(odescAxisNode
src/axis_rx_interface.cpp
src/axis_trx_interface.cpp
src/axis_node.cpp
src/mixins/util.cpp
)
ament_target_dependencies(odescAxisNode rclcpp std_msgs)

install(TARGETS
  odescAxisEncoderOnlyNode
  odescAxisNode
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
