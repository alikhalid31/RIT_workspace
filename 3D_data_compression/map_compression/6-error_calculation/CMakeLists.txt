cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(error_calculation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (error_calculation error_calculation.cpp)
target_link_libraries (error_calculation ${PCL_LIBRARIES})

