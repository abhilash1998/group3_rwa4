#include <algorithm>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
class Sensors {
public:
explicit Sensors(ros::NodeHandle);

void break_beam_callback(const nist_gear::Proximity::ConstPtr& msg);

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr& msg);

void quality_callback1(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback3(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void quality_callback4(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void logical_camera_callback(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void logical_camera_callback2(
    const nist_gear::LogicalCameraImage::ConstPtr& image_msg);

void startdetect();

private:

ros::NodeHandle n;

};
