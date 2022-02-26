cmake_minimum_required(VERSION 2.8.3)
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

include_directories(include ${catkin_INCLUDE_DIRS} src include)

add_executable(sd src/main.cpp src/sub_sensor.cpp)
add_dependencies(sd ${catkin_EXPORTED_TARGETS})
target_link_libraries(sd ${catkin_LIBRARIES})

