#include "sensors.hpp"
/// @fn  Sensors(ros::NodeHandle* const)
/// @brief Sensor class constructor
///
/// @pre
/// @post
/// @param nh
Sensors::Sensors(ros::NodeHandle* const nh){
n=nh;
}
/// @fn void break_beam_callback(const nist_gear::Proximity::ConstPtr&)
/// @brief Breakbeam0 Trigger
///
/// @pre
/// @post
/// @param msg
void Sensors::break_beam_callback(const nist_gear::Proximity::ConstPtr& msg) {
    if (msg->object_detected) {  
      ROS_INFO_STREAM("Callback triggered for Topic /ariac/breakbeam_0");
    }
}

/// @fn void break_beam_change_callback(const nist_gear::Proximity::ConstPtr&)
/// @brief break_beam_change trigger
///
/// @pre
/// @post
/// @param msg
void Sensors::break_beam_change_callback(const nist_gear::Proximity::ConstPtr& msg) {
    if (msg->object_detected) {  
      ROS_INFO_STREAM("Callback triggered for Topic /ariac/breakbeam_0_change");
    }
}

/// @fn void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr&)
/// @brief laser_profiler trigger
///
/// @pre
/// @post
/// @param msg
void Sensors::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  
  if (msg->ranges[200] < 0.62)
  {
     ROS_INFO_STREAM("Callback triggered for Topic /ariac/laser_profiler_0");
  }
}
/// @fn void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr&)
/// @brief proximity_sensor trigger
///
/// @pre
/// @post
/// @param msg
void Sensors::proximity_sensor_callback(const sensor_msgs::Range::ConstPtr& msg)
{
  if ((msg->range) < 0.14)
  {  
    ROS_INFO_STREAM("Callback triggered for Topic /ariac/proximity_sensor_0");
  }
}

/// @fn void quality_callback1(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief Quality sensor trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::quality_callback1(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "quality_control_sensor_1: '" << image_msg->models.size() << "' objects.");
  }
/// @fn void quality_callback2(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief Quality sensor trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::quality_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "quality_control_sensor_2: '" << image_msg->models.size() << "' objects.");
  }
/// @fn void quality_callback3(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief Quality sensor trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::quality_callback3(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "quality_control_sensor_3: '" << image_msg->models.size() << "' objects.");
  }


/// @fn void quality_callback4(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief Quality sensor trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::quality_callback4(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "quality_control_sensor_4: '" << image_msg->models.size() << "' objects.");
  }


/// @fn void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief logical_camera trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "logical_camera_bins0 camera: '" << image_msg->models.size() << "' objects.");
  }
/// @fn void logical_camera_callback2(const nist_gear::LogicalCameraImage::ConstPtr&)
/// @brief logical_camera trigger
///
/// @pre
/// @post
/// @param image_msg
void Sensors::logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "logical_camera_station2: '" << image_msg->models.size() << "' objects.");
  }

/// @fn void startdetect()
/// @brief To call all the functions
///
/// @pre
/// @post
void Sensors::startdetect()
{
  breakbeam0_sub = n->subscribe("/ariac/breakbeam_0", 1, &Sensors::break_beam_callback,this);

breakbeam0_change_sub = n->subscribe("/ariac/breakbeam_0_change", 1, &Sensors::break_beam_change_callback,this);

  logical_camera_bins0_sub = n->subscribe("/ariac/logical_camera_bins0", 1, &Sensors::logical_camera_callback,this);

  logical_camera_station2_sub = n->subscribe("/ariac/logical_camera_station2", 1, &Sensors::logical_camera_callback2,this);

  proximity_sensor_0_sub = n->subscribe("/ariac/proximity_sensor_0", 1, &Sensors::proximity_sensor_callback,this);

  laser_profiler_0_sub = n->subscribe("/ariac/laser_profiler_0", 1, &Sensors::laser_profiler_callback,this);

  quality_control_sensor_1_sub = n->subscribe("/ariac/quality_control_sensor_1", 1, &Sensors::quality_callback1,this);

  quality_control_sensor_2_sub = n->subscribe("/ariac/quality_control_sensor_2", 1, &Sensors::quality_callback2,this);

  quality_control_sensor_3_sub = n->subscribe("/ariac/quality_control_sensor_3", 1, &Sensors::quality_callback3, this);

  quality_control_sensor_4_sub = n->subscribe("/ariac/quality_control_sensor_4", 1, &Sensors::quality_callback4, this);

  ros::spin();

}
