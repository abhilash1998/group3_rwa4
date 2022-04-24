#include <ros/subscriber.h>
#include <ros/timer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/String.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>

#include <string>
#include <array>
#include <vector>

class AgilityChallenger
{
protected:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    ros::Subscriber competition_state_sub;
    ros::Subscriber orders_subs;
    ros::Subscriber logical_camera_1_sub;
    ros::Subscriber logical_camera_2_sub;
    std::array<ros::Subscriber, 4> quality_control_sensor_subs;
    ros::Timer watch_for_blackouts_tmr;

    std::string current_competition_state;
    std::vector<nist_gear::KittingShipment> current_kitting_shipments;
    std::array<std::vector<std::string>, 2> current_parts_found;
    bool sensors_started;
    bool in_sensor_blackout;

    void help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx);
    void help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int agv_idx);

    void annouce_world_tf(const std::string part_name, const std::string frame);
    void competition_state_callback(const std_msgs::String::ConstPtr& msg);
    void order_callback(const nist_gear::Order::ConstPtr& msg);
    void logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void sensor_blackout_detected_callback(const ros::TimerEvent& evt);

public:
    AgilityChallenger(ros::NodeHandle* const nh);
    ~AgilityChallenger();
};
