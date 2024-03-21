cmake_minimum_required(VERSION 2.8.3)
project(tyrion_dwa)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS 
    roscpp  
    std_msgs 
    mav_msgs
    mav_planning_msgs
    tf 
    tf2_sensor_msgs  
    tf2_ros
    hector_moveit_actions
)

add_message_files(
  DIRECTORY msg
  FILES DynamicWindowMsg.msg
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp std_msgs mav_msgs mav_planning_msgs tf tf2_sensor_msgs tf2_ros message_generation)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${catkin_INCLUDE_DIRS})


find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# add_executable(point_cloud_merger_node src/point_cloud_merger_node.cpp)
# target_link_libraries(point_cloud_merger_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(tyrion_dwa src/tyrion_dwa.cpp src/tyrion_dwa_node.cpp)
target_link_libraries(tyrion_dwa ${PCL_LIBRARIES} ${catkin_LIBRARIES})


catkin_install_python(PROGRAMS scripts/Dwa_Visual_Info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/Dwa_numeric_info.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)