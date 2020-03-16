cmake_minimum_required(VERSION 3.9)

project(ftc265 VERSION 2020.1.0)

# Find librealsense2
# Note that we package the realsense2 sysroot and all of its headers+cmake configs in this repo
# This is kind of a hack, but it makes it so that this repo builds all in one
message(WARNING ${CMAKE_CXX_COMPILER_TARGET})
set(realsense2_DIR "${CMAKE_CURRENT_LIST_DIR}/sysroot_${CMAKE_CXX_COMPILER_TARGET}/usr/local/lib/cmake/realsense2/")
find_package(realsense2 NO_DEFAULT_PATH REQUIRED)

# We only support C++17 and above
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add sources
add_library(${PROJECT_NAME} SHARED
    t265wrapper.hpp
    t265wrapper.cpp
)

# Link to librealsense2
target_link_libraries(${PROJECT_NAME}
    ${realsense2_LIBRARY}
)
