cmake_minimum_required(VERSION 3.9)

project(ftc265 VERSION 2020.1.0)

# Add jni headers (may not be needed for Android; TODO)
#find_package(realsense2 REQUIRED)
#if (NOT CMAKE_TOOLCHAIN_FILE)
#    find_package(JNI REQUIRED)
#    include_directories(${JNI_INCLUDE_DIRS})
#else()
#    include_directories("jni/")
#endif()

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
