cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_viewer)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(k_nearest_neighbor_search k_nearest_neighbor_search.cpp)
target_link_libraries(k_nearest_neighbor_search ${PCL_LIBRARIES})

add_executable(neighbors_with_radius_search neighbors_with_radius_search.cpp)
target_link_libraries(neighbors_with_radius_search ${PCL_LIBRARIES})

add_executable(neighbors_with_voxel_search neighbors_with_voxel_search.cpp)
target_link_libraries(neighbors_with_voxel_search ${PCL_LIBRARIES})
