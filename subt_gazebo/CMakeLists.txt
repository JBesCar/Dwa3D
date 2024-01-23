cmake_minimum_required(VERSION 3.5.1)
project(subt_gazebo)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

set (SUBT_MAJOR_VERSION 0)
set (SUBT_MINOR_VERSION 1)
set (SUBT_PATCH_VERSION 0)
set (SUBT_VERSION_FULL
  ${SUBT_MAJOR_VERSION}.${SUBT_MINOR_VERSION}.${SUBT_PATCH_VERSION})

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_ros
    gazebo_plugins
    roscpp
    geometry_msgs
    std_msgs
    std_srvs

)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(gazebo REQUIRED)

########################
## Message generation ##
########################

# Add here SubT ROS messages.

###########
## Build ##
###########

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_srvs
  INCLUDE_DIRS
  ${CATKIN_DEVEL_PREFIX}/include
  CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)


