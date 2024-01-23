cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_exploration)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    hector_moveit_actions
    actionlib
)

find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs hector_moveit_actions)

if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})


add_executable(hector_explorer_dwa src/Explorer_dwa.cpp src/explorer_node_dwa.cpp)
target_compile_options(hector_explorer_dwa PRIVATE ${OpenMP_FLAGS} ${CMAKE_CXX_FLAGS})
target_link_libraries(hector_explorer_dwa ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenMP_LIBS})
add_dependencies(hector_explorer_dwa hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages)


find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(point_cloud_merger_node src/point_cloud_merger_node.cpp)
target_link_libraries(point_cloud_merger_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_executable(simulation_point_cloud_merger_node src/simulation_point_cloud_merger_node.cpp)
target_link_libraries(simulation_point_cloud_merger_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})


add_executable(tf_listener_test src/tf_listener_test.cpp)
target_link_libraries(tf_listener_test ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(navigator_dwa src/Navigator_dwa.cpp src/navigator_node_dwa.cpp)
target_link_libraries(navigator_dwa ${catkin_LIBRARIES})
