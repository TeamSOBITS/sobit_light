cmake_minimum_required(VERSION 3.8)
project(sobit_light_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
# find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(rclcpp_components REQUIRED) # 追加
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sobits_msgs REQUIRED)

# Find libraries
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/sobit_light_joint_controller.cpp
  src/sobit_light_wheel_controller.cpp
)

# rclcpp_components_register_nodes(${PROJECT_NAME}
#   "sobit_light::JointController"
#   "sobit_light::WheelController"
# )

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "sobit_light::JointController"
#   EXECUTABLE joint_controller
# )

# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "sobit_light::WheelController"
#   EXECUTABLE wheel_controller
# )

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_msgs
)


# Executables
add_executable(test_control_wheel example/test_control_wheel.cpp)
target_link_libraries(test_control_wheel ${PROJECT_NAME})
ament_target_dependencies(test_control_wheel
  rclcpp
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_msgs
)

#############
## Install ##
#############

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  test_control_wheel
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
