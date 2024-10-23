cmake_minimum_required(VERSION 2.8.3)
project(ouster_pointcloud_filter)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
)

find_package(OpenMP REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

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
add_executable(near_filter src/near_filter.cpp)
target_link_libraries(near_filter ${PCL_LIBRARIES} ${catkin_LIBRARIES})

