cmake_minimum_required(VERSION 2.8.12)
project (RobotModelModule)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package (Eigen3 3.3.7 REQUIRED)

add_library                (RobotModel-lib STATIC ${CMAKE_SOURCE_DIR}/RobotModel/src/RobotModel.cpp)
target_link_libraries      (RobotModel-lib Eigen3::Eigen)
target_include_directories (RobotModel-lib PUBLIC ${CMAKE_SOURCE_DIR}/RobotModel/include/)

install(TARGETS
    RobotModel-lib
    DESTINATION lib/${PROJECT_NAME})