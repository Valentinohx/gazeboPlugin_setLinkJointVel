cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build the plugin
add_library(JointVel_plugin SHARED src/JointVel_plugin.cc)
target_link_libraries(JointVel_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(LinkVelSet_plugin SHARED src/LinkVel_plugin.cc)
target_link_libraries(LinkVelSet_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

