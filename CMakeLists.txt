cmake_minimum_required(VERSION 3.10)
project(dash_hardware_interface)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find the Asio library
find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

add_library(upxtreme SHARED UPXtreme.cpp MsgBase.cpp)
target_include_directories(upxtreme PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(upxtreme Threads::Threads)

# Add executable
add_executable(closed_loop_test ClosedLoopControlTest.cpp)
target_link_libraries(closed_loop_test upxtreme)
target_include_directories(closed_loop_test PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
