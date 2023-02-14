cmake_minimum_required(VERSION 2.8.12)
project (MotorContollerModule)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(epos-lib SHARED IMPORTED)
set_target_properties(epos-lib PROPERTIES IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/src/MotorController/src/libEposCmd.so.6.8.1.0)

add_library(MotorController-lib STATIC ${CMAKE_SOURCE_DIR}/src/MotorController/src/MotorController.cpp)
target_link_libraries(MotorController-lib epos-lib)
target_include_directories(MotorController-lib PUBLIC ${CMAKE_SOURCE_DIR}/src/MotorController/include/)

install(TARGETS
    MotorController-lib
    DESTINATION lib/${PROJECT_NAME})