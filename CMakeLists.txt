cmake_minimum_required(VERSION 2.8.3)
project(group3_rwa4)

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

add_executable(sensors_controller src/sensors.cpp src/sensors_controller.cpp)
add_dependencies(sensors_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensors_controller ${catkin_LIBRARIES})

add_executable(agvs_controller src/ariac_agv.cpp src/agvs_controller.cpp)
target_link_libraries(agvs_controller ${catkin_LIBRARIES})

add_executable(agility_controller src/agility.cpp src/agility_controller.cpp)
target_link_libraries(agility_controller ${catkin_LIBRARIES})
