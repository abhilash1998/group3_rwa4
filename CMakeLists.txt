cmake_minimum_required(VERSION 2.8.3)
project(group3_rwa2)

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

add_executable(rwa2_main src/ariac_agv.cpp src/sensors.cpp src/rwa2_main.cpp)
target_link_libraries(rwa2_main ${catkin_LIBRARIES})