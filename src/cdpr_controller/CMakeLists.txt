# cmake_minimum_required(VERSION 3.5)
# project(cdpr_controller)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

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
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

cmake_minimum_required(VERSION 3.5)
project(cdpr_controller)

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
find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )

# ----------------------------------------------------------
# User Node
add_executable(userNode src/UserNode.cpp)
ament_target_dependencies(userNode rclcpp std_msgs geometry_msgs)

# ----------------------------------------------------------
# Robot Node
add_executable(robotNode src/RobotNode.cpp)
target_link_libraries(robotNode TrajectoryGenerator-lib RobotModel-lib)
ament_target_dependencies(robotNode rclcpp std_msgs geometry_msgs)

# ----------------------------------------------------------
# Motor Node
add_executable(motorNode src/MotorNode.cpp)
target_link_libraries(motorNode MotorController-lib)
ament_target_dependencies(motorNode rclcpp std_msgs geometry_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  userNode
  robotNode
  motorNode
  DESTINATION lib/${PROJECT_NAME})

include(${CMAKE_SOURCE_DIR}/MotorController/CMakeLists.txt)
include(${CMAKE_SOURCE_DIR}/TrajectoryGenerator/CMakeLists.txt)
include(${CMAKE_SOURCE_DIR}/RobotModel/CMakeLists.txt)

ament_package()
