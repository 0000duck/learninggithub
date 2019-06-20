cmake_minimum_required(VERSION 2.8.3)
project(teleop_controller)
set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(catkin_deps
  joy
  roscpp
  std_msgs
  tf)

find_package(catkin REQUIRED COMPONENTS
  ${catkin_deps})

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

##find_package(IPOPT REQUIRED)


catkin_package(
          INCLUDE_DIRS include
        #  LIBRARIES hello
        CATKIN_DEPENDS ${catkin_deps}
        #   DEPENDS ipopt
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)

add_executable(tank_teleop_joy
  src/tank_teleop_joy.cpp)

target_link_libraries(tank_teleop_joy
  ${catkin_LIBRARIES}
  ${IPOPT_LIBRARIES})

message(STATUS "IPOPT LIBRARIES:" ${IPOPT_LIBRARIES})
