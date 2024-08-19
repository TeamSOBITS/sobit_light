cmake_minimum_required(VERSION 3.8)
# cmake_minimum_required(VERSION 3.5)
project(sobit_light_library)

# add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_PREFIX_PATH "/home/sobits/colcon_ws/src" ${CMAKE_PREFIX_PATH})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED) # 追加
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sobits_msgs REQUIRED)


# Specify additional locations of header files
# include_directories(
#   include
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${rclcpp_INCLUDE_DIRS}
#   ${std_msgs_INCLUDE_DIRS}
#   ${sobits_msgs_INCLUDE_DIRS} 
#   ${ament_index_build_path}/sobits_msgs/include
#   ${PROJECT_SOURCE_DIR}/install/sobits_msgs/include
# )

# include_directories(${CMAKE_SOURCE_DIR}/install/sobits_msgs/include)

# include_directories(
#   ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/sobits_msgs
# )


# Library
add_library(${PROJECT_NAME} SHARED
  src/sobit_light_joint_controller.cpp
  src/sobit_light_wheel_controller.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "sobit_light::JointController"
  "sobit_light::WheelController"
)

# target_compile_options(${PROJECT_NAME} PUBLIC -Wall)

ament_target_dependencies(${PROJECT_NAME}
  pybind11
  rclcpp
  rclcpp_components  # 追加
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# if(NOT WIN32)
#   ament_environment_hooks(
#     "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
#   )
# endif()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pybind11
  rclcpp
  rclcpp_components  # 追加
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_msgs
)


# # Pybind11 module
add_library(sobit_light_module SHARED
  src/sobit_light_library.cpp
  src/sobit_light_joint_controller.cpp
  src/sobit_light_wheel_controller.cpp
)

set_target_properties(sobit_light_module PROPERTIES
  PREFIX ""
  # SUFFIX ".so"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

# target_compile_options(sobit_light_module PUBLIC -Wall)

ament_target_dependencies(sobit_light_module
  pybind11
  rclcpp
  rclcpp_components  # 追加
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  trajectory_msgs
  geometry_msgs
  nav_msgs
  sobits_msgs
)

target_include_directories(sobit_light_module
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Executables
add_executable(test_control_wheel example/test_control_wheel.cpp)
target_link_libraries(test_control_wheel ${PROJECT_NAME})
ament_target_dependencies(test_control_wheel
  pybind11
  rclcpp
  rclcpp_components  # 追加
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
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  sobit_light_module
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  test_control_wheel
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  example/test_control_wheel.py
  DESTINATION lib/${PROJECT_NAME}
)

# ament_python_install_package(sobit_light_module)

# install(TARGETS test_control_arm test_control_head test_control_wheel test_grasp test_place
#   DESTINATION lib/${PROJECT_NAME}
# )


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