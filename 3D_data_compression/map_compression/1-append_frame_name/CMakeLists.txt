cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(append_frame_name)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (append_frame_name append_frame_name.cpp)
target_link_libraries (append_frame_name ${PCL_LIBRARIES})

