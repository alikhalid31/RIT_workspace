cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(raw_framevsframe_change)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (raw_framevsframe_change raw_framevsframe_change.cpp)
target_link_libraries (raw_framevsframe_change ${PCL_LIBRARIES})
