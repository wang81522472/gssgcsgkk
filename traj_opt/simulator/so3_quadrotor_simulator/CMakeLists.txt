#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
#find_package(Eigen3 REQUIRED)

#include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(include/odeint-v2)

#rosbuild_add_boost_directories()
#rosbuild_add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)
#rosbuild_add_link_flags(quadrotor_dynamics -Wl,--as-needed)

#rosbuild_add_executable(test_dynamics src/test_dynamics/test_dynamics.cpp)
#target_link_libraries(test_dynamics quadrotor_dynamics rt)

#rosbuild_add_executable(quadrotor_simulator_so3
#  src/quadrotor_simulator_so3.cpp)
#target_link_libraries(quadrotor_simulator_so3 quadrotor_dynamics)
#rosbuild_add_link_flags(quadrotor_simulator_so3 -Wl,--as-needed)

#----------------------------------
cmake_minimum_required(VERSION 2.8.3)
project(so3_quadrotor_simulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
##
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include/odeint-v2)
##
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(irobot_msgs
#   src/${PROJECT_NAME}/irobot_msgs.cpp
# )
add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

## Declare a cpp executable
#add_executable(odom_visualization src/odom_visualization.cpp)
add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3 
   ${catkin_LIBRARIES}
   quadrotor_dynamics
)
