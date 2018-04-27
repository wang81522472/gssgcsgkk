cmake_minimum_required(VERSION 2.8.3)
project(costmap_to_gridmap)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  grid_map_core
  grid_map_costmap_2d
  grid_map_msgs
  grid_map_ros
  grid_map_visualization
  nav_msgs
  std_msgs
  tf
  visualization_msgs
        roscpp
)

catkin_package(
  INCLUDE_DIRS #include
  LIBRARIES costmap_to_gridmap
  CATKIN_DEPENDS costmap_2d geometry_msgs grid_map_core grid_map_costmap_2d grid_map_msgs grid_map_ros grid_map_visualization nav_msgs std_msgs tf visualization_msgs roscpp
  #DEPENDS system_lib
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(costmap_to_gridmap src/costmap_to_gridmap.cpp)
target_link_libraries(costmap_to_gridmap ${catkin_LIBRARIES})
add_dependencies(costmap_to_gridmap ${catkin_EXPORTED_TARGETS})