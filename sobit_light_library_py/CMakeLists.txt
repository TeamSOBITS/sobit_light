# cmake_minimum_required(VERSION 3.8)
# project(sobit_light_library_py)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()
cmake_minimum_required(VERSION 3.5)
project(sobit_light_library_py)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(sobit_light_library REQUIRED)

# Find python before pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sobits_msgs REQUIRED)


include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Add the pybind11 modules
pybind11_add_module(_arm SHARED
  src/sobit_light_library_py/_arm.cpp
)
target_link_libraries(_arm PUBLIC
  sobit_light_library::sobit_light_library
)
# ament_target_dependencies(_arm PUBLIC
#   "sobit_light_library"
# )

pybind11_add_module(_wheel SHARED
  src/sobit_light_library_py/_wheel.cpp
)
target_link_libraries(_wheel PUBLIC
  sobit_light_library::sobit_light_library
)
# ament_target_dependencies(_wheel PUBLIC
#   "sobit_light_library"
# )

# Install cython modules as sub-modules of the project
install(
  TARGETS
    _wheel
    _arm
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  # DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/test_control_wheel.py
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
