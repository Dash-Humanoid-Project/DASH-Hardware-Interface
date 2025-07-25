cmake_minimum_required(VERSION 3.10)
project(dash_hardware_interface)

option(UPXTREME_i14 ON)
option(ENABLE_TIME_BENCHMARK OFF)

message(UPXTREME_i14="${UPXTREME_i14}")
message(ENABLE_TIME_BENCHMARK="${ENABLE_TIME_BENCHMARK}")

if(${UPXTREME_i14})
    add_definitions(-DUPXTREME_i14)
endif()

if(${ENABLE_TIME_BENCHMARK})
    add_definitions(-DENABLE_TIME_BENCHMARK)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Asio library
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

#include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ASIO_INCLUDE_DIR})

# source files
set(SRC_FILES
    src/MsgBase.cpp
    src/UPXtreme.cpp
)

# library
add_library(dash_hardware_interface SHARED ${SRC_FILES})
target_link_libraries(dash_hardware_interface Threads::Threads)

# executable
add_executable(closed_loop_test test/ClosedLoopControlTest.cpp)
target_link_libraries(closed_loop_test dash_hardware_interface)

# copy selected files to teensy/
file(GLOB HEADER_FILES_TO_COPY ${CMAKE_SOURCE_DIR}/include/*.h)

set(SRC_FILES_TO_COPY
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_SOURCE_DIR}/src/MsgBase.cpp
)

set(FILES_TO_COPY ${HEADER_FILES_TO_COPY} ${SRC_FILES_TO_COPY})

set(TEENSY_DIR
    ${CMAKE_SOURCE_DIR}/teensy
)

add_custom_target(copy_to_teensy_dir ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${FILES_TO_COPY} ${TEENSY_DIR}
    COMMENT "Copying selected files to teensy/ directory"
)

add_custom_target(copy_teensy_CAN_dir ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/include/Param.h ${CMAKE_SOURCE_DIR}/test/CAN_cyclic_msg_benchmark
    COMMENT "Copying selected files to test/CAN_cyclic_msg_benchmark/ directory"
)
