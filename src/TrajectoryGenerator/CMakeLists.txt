cmake_minimum_required(VERSION 2.8.12)
project (TrajectoryGeneratorModule)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package (Eigen3 3.3.7 REQUIRED)

add_library                (TrajectoryGenerator-lib STATIC ${CMAKE_SOURCE_DIR}/src/TrajectoryGenerator/src/TrajectoryGenerator.cpp)
target_link_libraries      (TrajectoryGenerator-lib Eigen3::Eigen)
target_include_directories (TrajectoryGenerator-lib PUBLIC ${CMAKE_SOURCE_DIR}/src/TrajectoryGenerator/include/)

install(TARGETS
    TrajectoryGenerator-lib
    DESTINATION lib/${PROJECT_NAME})