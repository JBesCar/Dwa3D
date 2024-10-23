cmake_minimum_required(VERSION 3.0.2)
project(octomap_height_estimation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    std_msgs 
    tf 
    tf2_sensor_msgs  
    tf2_ros
    octomap_server
)


find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp std_msgs tf tf2_sensor_msgs tf2_ros message_generation octomap_server)

###########
## Build ##
###########

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(octomap_height_estimation src/octomap_height_estimation.cpp src/octomap_height_estimation_node.cpp)
target_link_libraries(octomap_height_estimation ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})
