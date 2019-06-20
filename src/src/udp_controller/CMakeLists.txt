cmake_minimum_required(VERSION 2.8.3)
project(udp_controller)

set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## find ros packages and combine them with catkin
set(catkin_deps
        roscpp
        std_msgs)

find_package(catkin REQUIRED COMPONENTS
        ${catkin_deps}
        )

## ros specific cmake command
catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS ${catkin_deps}
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

#####################################################################
####   add a ros node executable file
#####################################################################
add_executable(udp_controller
        udp_controller.cpp
        )


target_link_libraries(udp_controller
        ${catkin_LIBRARIES}
        ${IPOPT_LIBRARIES}
        ${LINKER_LIBS}
        gflags
        glog
        )
