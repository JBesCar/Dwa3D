cmake_minimum_required(VERSION 2.8.3)
project(tyrion_octomap_global_planning)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp   
    std_msgs 
    actionlib
    tf 
    tf2_sensor_msgs  
    tf2_ros
    octomap_server
)

find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp std_msgs tf tf2_sensor_msgs tf2_ros)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ompl_planning_example src/ompl_planning_example.cpp)
target_link_libraries(ompl_planning_example ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES})

add_executable(global_planning src/global_planning.cpp src/global_planning_node.cpp)
target_link_libraries(global_planning ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${OMPL_LIBRARIES} ${PCL_LIBRARIES})

add_executable(naive_planning src/naive_planning.cpp src/naive_planning_node.cpp)
target_link_libraries(naive_planning ${catkin_LIBRARIES})
