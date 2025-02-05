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
  eigen_conversions
  control_msgs
  geometry_msgs
  moveit_msgs
  rosgraph_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  moveit_core
  moveit_ros_perception
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_controller src/arm.cpp src/ariac_agv.cpp src/agility.cpp src/utils.cpp src/arm_controller.cpp)
target_link_libraries(arm_controller ${catkin_LIBRARIES})

add_executable(blackout_controller src/blackout.cpp src/blackout_controller.cpp)
target_link_libraries(blackout_controller ${catkin_LIBRARIES})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  config/robot_workcell_specs.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
