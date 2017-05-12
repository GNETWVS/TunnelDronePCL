cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(TunnelDronePCL C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cleanup_pcd point_cloud_cleanup.cpp)
target_link_libraries (cleanup_pcd ${PCL_LIBRARIES})
#target_link_libraries (cleanup_pcd ~/usr/local/lib/)
