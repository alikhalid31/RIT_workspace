cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(DIFF1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (fuse fuse.cpp)
target_link_libraries (fuse ${PCL_LIBRARIES})

