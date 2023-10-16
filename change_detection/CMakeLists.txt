cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(change_detection_frameVSmap)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (change_detection_frameVSmap change_detection_frameVSmap.cpp)
target_link_libraries (change_detection_frameVSmap ${PCL_LIBRARIES})
