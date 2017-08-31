cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(OFF2PCD)

find_package(PCL 1.2 REQUIRED)

include_directories(${include})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(main main.cpp)

target_link_libraries(main ${PCL_LIBRARIES})
