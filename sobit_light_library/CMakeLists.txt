# cmake_minimum_required(VERSION 3.8)
# project(sobit_light_library)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()





cmake_minimum_required(VERSION 3.5)
project(sobit_light_library)

## Compile as C++14
add_compile_options(-std=c++17)

# set(CMAKE_PREFIX_PATH "/home/sobits/colcon_ws/src" ${CMAKE_PREFIX_PATH})


## Find ament macros and libraries
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED) # 追加
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pybind11 REQUIRED)
# find_package(current_state_array REQUIRED)

# find_package(dynamixel_sdk REQUIRED)
find_package(sobits_msgs REQUIRED)


#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

###################################
## ament specific configuration ##
###################################


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
)

include_directories(${CMAKE_SOURCE_DIR}/install/sobits_msgs/include)

# include_directories(
#   ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/sobits_msgs
# )

## Library
add_library(${PROJECT_NAME} SHARED 
                                   src/sobit_light_joint_controller.cpp
                                   src/sobit_light_wheel_controller.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components  # 追加
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  trajectory_msgs
  pybind11
  nav_msgs
  # dynamixel_sdk
  sobits_msgs
)

## Pybind11 module
add_library(sobit_light_module SHARED src/sobit_light_library.cpp
                                     src/sobit_light_joint_controller.cpp
                                     src/sobit_light_wheel_controller.cpp)
set_target_properties(sobit_light_module PROPERTIES
    PREFIX ""
    SUFFIX ".so"
)
ament_target_dependencies(sobit_light_module
  rclcpp
  rclcpp_components  # 追加
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  # dynamixel_sdk
  sobits_msgs
  pybind11
  trajectory_msgs
  # current_state_array
)

# ## Executables
# add_executable(test_control_arm example/test_control_arm.cpp)
# target_link_libraries(test_control_arm ${PROJECT_NAME})
# ament_target_dependencies(test_control_arm
#   "rclcpp"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "dynamixel_sdk"
#   "sobits_msgs"
# )

# add_executable(test_control_head example/test_control_head.cpp)
# target_link_libraries(test_control_head ${PROJECT_NAME})
# ament_target_dependencies(test_control_head
#   "rclcpp"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "dynamixel_sdk"
#   "sobits_msgs"
# )

# add_executable(test_control_wheel example/test_control_wheel.cpp)
# target_link_libraries(test_control_wheel ${PROJECT_NAME})
# ament_target_dependencies(test_control_wheel
#   "rclcpp"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "dynamixel_sdk"
#   "sobits_msgs"
# )

# add_executable(test_grasp example/test_grasp.cpp)
# target_link_libraries(test_grasp ${PROJECT_NAME})
# ament_target_dependencies(test_grasp
#   "rclcpp"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "dynamixel_sdk"
#   "sobits_msgs"
# )

# add_executable(test_place example/test_place.cpp)
# target_link_libraries(test_place ${PROJECT_NAME})
# ament_target_dependencies(test_place
#   "rclcpp"
#   "tf2"
#   "tf2_ros"
#   "tf2_geometry_msgs"
#   "geometry_msgs"
#   "nav_msgs"
#   "dynamixel_sdk"
#   "sobits_msgs"
# )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sobit_light_module
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# install(TARGETS test_control_arm test_control_head test_control_wheel test_grasp test_place
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()