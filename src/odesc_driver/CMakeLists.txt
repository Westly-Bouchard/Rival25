cmake_minimum_required(VERSION 3.15)

# This is just so the CI runner won't complain
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Start ROS Package Build
project(odesc_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_library(DBCPPP_LIB dbcppp PATHS /usr/local/lib)

add_executable(odescAxisNode
src/axis_rx_interface.cpp
src/axis_encoder_only_interface.cpp
src/axis_encoder_only_node.cpp
src/mixins/util.cpp
)
ament_target_dependencies(odescAxisNode rclcpp std_msgs ament_index_cpp)
target_link_libraries(odescAxisNode ${DBCPPP_LIB})
target_include_directories(odescAxisNode PRIVATE /usr/local/include)

install(TARGETS
  odescAxisNode
  DESTINATION lib/${PROJECT_NAME})

install(
  FILES dbc/odrive-cansimple.dbc
  DESTINATION share/${PROJECT_NAME}/dbc
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
