cmake_minimum_required(VERSION 3.5)
project(robot_control_pubsub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

add_executable(user_node src/user_node.cpp)
ament_target_dependencies(user_node rclcpp std_msgs)

add_executable(motor_node src/motor_node.cpp)
target_link_libraries(motor_node TrajectoryGenerator-lib)
ament_target_dependencies(motor_node rclcpp std_msgs)

add_executable(epos_node src/epos_node.cpp)
target_link_libraries(epos_node MotorController-lib)
ament_target_dependencies(epos_node rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  user_node
  motor_node
  epos_node
  DESTINATION lib/${PROJECT_NAME})

include(${CMAKE_SOURCE_DIR}/MotorController/CMakeLists.txt)
include(${CMAKE_SOURCE_DIR}/TrajectoryGenerator/CMakeLists.txt)

ament_package()
