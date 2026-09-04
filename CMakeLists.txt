cmake_minimum_required(VERSION 3.10)
project(dash_hardware_interface)

option(UPXTREME_i14 "Build for UPXtreme i14 (PC) platform" ON)
option(ENABLE_TIME_BENCHMARK "Enable UDP time benchmarking" OFF)

message(UPXTREME_i14="${UPXTREME_i14}")
message(ENABLE_TIME_BENCHMARK="${ENABLE_TIME_BENCHMARK}")

if(${UPXTREME_i14})
    add_definitions(-DUPXTREME_i14)
endif()

if(${ENABLE_TIME_BENCHMARK})
    add_definitions(-DENABLE_TIME_BENCHMARK)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Asio library
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

# VectorNav SDK (vnproglib) — used by Imu.cpp. Point this at the cpp/
# directory of a vnproglib checkout; built as a subdirectory so it's
# compiled with the same toolchain/flags as the rest of this project.
set(VNPROGLIB_DIR "/home/dvolpi/Source/vnproglib/cpp" CACHE PATH
    "Path to vnproglib's cpp/ directory (VectorNav SDK)")
if(NOT EXISTS ${VNPROGLIB_DIR}/CMakeLists.txt)
    message(FATAL_ERROR "VNPROGLIB_DIR (${VNPROGLIB_DIR}) does not contain vnproglib's cpp/ sources. "
                         "Set -DVNPROGLIB_DIR=/path/to/vnproglib/cpp")
endif()
# dash_hardware_interface is a SHARED lib, so libvncxx (linked into it) must be -fPIC too.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(${VNPROGLIB_DIR} ${CMAKE_BINARY_DIR}/vnproglib)

#include directories
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${ASIO_INCLUDE_DIR})
include_directories(${VNPROGLIB_DIR}/include)

# source files
set(SRC_FILES
    src/MsgBase.cpp
    src/UPXtreme.cpp
    src/SimUPXtreme.cpp
    src/Leg.cpp
    src/HardwareBridge.cpp
    src/Imu.cpp
    src/PeriodicTimer.cpp
    src/LegController.cpp
    src/KeyboardInput.cpp
    src/ModeDispatcher.cpp
)

# library
add_library(dash_hardware_interface SHARED ${SRC_FILES})
target_link_libraries(dash_hardware_interface Threads::Threads libvncxx)

# executable
add_executable(closed_loop_test
    test/ClosedLoopControlTest.cpp
    test/TestUtils.cpp
    test/PositionMode.cpp
    test/ImpedanceMode.cpp
    test/CartesianMode.cpp
    test/TrajectoryPlayback.cpp
)
target_link_libraries(closed_loop_test dash_hardware_interface)

# Executable for UDP benchmark test
add_executable(test_udp_benchmark test/udp_benchmark/test_udp_benchmark.cpp)
target_link_libraries(test_udp_benchmark dash_hardware_interface)

# Standalone IMU hardware verification test
add_executable(imu_test test/ImuTest.cpp)
target_link_libraries(imu_test dash_hardware_interface)

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

set(TEENSY2_DIR
    ${CMAKE_SOURCE_DIR}/teensy2
)

set(TEENSY3_DIR
    ${CMAKE_SOURCE_DIR}/teensy3
)

set(TEENSY4_DIR
    ${CMAKE_SOURCE_DIR}/teensy4
)

set(TEENSY_DIR_PARAM_ONLY
    ${CMAKE_SOURCE_DIR}/test/CAN_cyclic_msg_benchmark
    ${CMAKE_SOURCE_DIR}/test/CAN_noncyclic_msg_benchmark
)

add_custom_target(copy_to_teensy_dir ALL
    COMMAND ${CMAKE_COMMAND} -E copy ${FILES_TO_COPY} ${TEENSY_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${FILES_TO_COPY} ${TEENSY2_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${FILES_TO_COPY} ${TEENSY3_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy ${FILES_TO_COPY} ${TEENSY4_DIR}
    COMMENT "Copying selected files to teensy/, teensy2/, teensy3/, and teensy4/ directories"
)

add_custom_target(copy_teensy_CAN_dir ALL
    COMMENT "Copying Param.h to CAN benchmark directories"
)

foreach(dst IN LISTS TEENSY_DIR_PARAM_ONLY)
    add_custom_command(TARGET copy_teensy_CAN_dir PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_SOURCE_DIR}/include/Param.h ${dst}/Param.h
    )
endforeach()
