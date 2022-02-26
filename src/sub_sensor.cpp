#include "sub_sensor.hpp"

Sensors::Sensors(ros::NodeHandle nh)
{
  n=nh;
}

void Sensors::break_beam_callback(const nist_gear::Proximity::ConstPtr& msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_INFO_STREAM("Callback triggered for Topic /ariac/breakbeam_0");
    }
}

void Sensors::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  
  if (msg->ranges[200] < 0.62)
  {
     ROS_INFO_STREAM("Laser profiler sees something.");
  }
}

void Sensors::proximity_sensor_callback(const sensor_msgs::Range::ConstPtr& msg)
{
  if ((msg->range) < 0.14)
  {  // If there is an object in proximity.
    ROS_INFO_STREAM("Proximity sensor sees something.");
  }
}


void Sensors::quality_callback1(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Quality camera: '" << image_msg->models.size() << "' objects.");
  }

void Sensors::quality_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Quality camera: '" << image_msg->models.size() << "' objects.");
  }

void Sensors::quality_callback3(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Quality camera: '" << image_msg->models.size() << "' objects.");
  }

void Sensors::quality_callback4(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Quality camera: '" << image_msg->models.size() << "' objects.");
  }



void Sensors::logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "logical_camera_bins0 camera: '" << image_msg->models.size() << "' objects.");
  }

void Sensors::logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "logical_camera_station2: '" << image_msg->models.size() << "' objects.");
  }


void Sensors::startdetect()
{
  ros::Subscriber sub1 = n.subscribe("/ariac/breakbeam_0", 1, &Sensors::break_beam_callback,this);
  ros::Subscriber sub2 = n.subscribe("/ariac/logical_camera_bins0", 1, &Sensors::logical_camera_callback,this);
  ros::Subscriber sub3 = n.subscribe("/ariac/logical_camera_station2", 1, &Sensors::logical_camera_callback2,this);
  ros::Subscriber sub4 = n.subscribe("/ariac/proximity_sensor_0", 1, &Sensors::proximity_sensor_callback,this);
  ros::Subscriber sub5 = n.subscribe("/ariac/laser_profiler_0", 1, &Sensors::laser_profiler_callback,this);
  ros::Subscriber sub6 = n.subscribe("/ariac/quality_control_sensor_1", 1, &Sensors::quality_callback1,this);
  ros::Subscriber sub7 = n.subscribe("/ariac/quality_control_sensor_2", 1, &Sensors::quality_callback2,this);
  ros::Subscriber sub8 = n.subscribe("/ariac/quality_control_sensor_3", 1, &Sensors::quality_callback3, this);
  ros::Subscriber sub9 = n.subscribe("/ariac/quality_control_sensor_4", 1, &Sensors::quality_callback4, this);
  ros::spin();

}
