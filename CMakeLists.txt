cmake_minimum_required(VERSION 3.0.2)
project(group_3_rwa1)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nist_gear
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sdata src/sub_sensor.cpp)
target_link_libraries(sdata ${catkin_LIBRARIES})

add_executable(agvs_controller src/ariac_agv.cpp src/agvs_controller.cpp)
target_link_libraries(agvs_controller ${catkin_LIBRARIES})
