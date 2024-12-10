cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_viewer)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_compression pcl_compression.cpp)
target_link_libraries(pcl_compression ${PCL_LIBRARIES})

add_executable(octree_search octree_search.cpp)
target_link_libraries(octree_search ${PCL_LIBRARIES})

add_executable(octree_change_detect octree_change_detect.cpp)
target_link_libraries(octree_change_detect ${PCL_LIBRARIES})