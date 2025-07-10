cmake_minimum_required(VERSION 3.0.2)
project(px4_tyrion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
            roscpp
            std_msgs
            tf2
            tf2_ros 
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS roscpp
              std_msgs
              tf2
              tf2_ros 
#  INCLUDE_DIRS include
#  LIBRARIES px4_tyrion
#  CATKIN_DEPENDS px4
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(velocity_command src/velocity_command.cpp)
add_executable(ros_pose_to_mavros src/ros_pose_to_mavros.cpp)
add_executable(tyrion_commander src/tyrion_commander.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(velocity_command ${catkin_LIBRARIES})
target_link_libraries(ros_pose_to_mavros ${catkin_LIBRARIES})
target_link_libraries(tyrion_commander ${catkin_LIBRARIES})


