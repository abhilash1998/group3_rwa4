#include <ros/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
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

    ros::Subscriber orders_subs;
    ros::Subscriber blackout_sub;
    std::array<ros::Subscriber, 4> logical_camera_subs;
    std::array<ros::Subscriber, 4> quality_control_sensor_subs;

    std::vector<nist_gear::KittingShipment> current_kitting_shipments;
    std::array<std::vector<std::string>, 4> current_logical_camera_data;
    bool in_sensor_blackout;

    // The most recently heard quality control updates heard from QC cameras
    // 1-4 (with indices [0,3])
    std::array<nist_gear::LogicalCameraImage, 4> current_qc_results;

    void help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx);
    void help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int lc_idx);

    void annouce_world_tf(const std::string part_name, const std::string frame);
    void order_callback(const nist_gear::Order::ConstPtr& msg);
    void blackout_status_callback(const std_msgs::Bool::ConstPtr& msg);
    void logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void logical_camera_image4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);
    void quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg);

public:
    AgilityChallenger(ros::NodeHandle* const nh);
    ~AgilityChallenger();

    std::vector<nist_gear::KittingShipment> get_current_kitting_shipments() const;
    std::vector<int> get_camera_indices_of(const std::string& product_type) const;
    std::string get_logical_camera_contents() const;

    // If there are any faulty parts, get the pick pose for one of them.
    // @param pick_frame If this method returns true, then this value is
    // overwritten with the pose of a faulty part resolved in world frame,
    // if false then this value is not overwritten
    // @return True if there is a faulty part, false otherwise.
    bool get_agv_faulty_part(geometry_msgs::Pose& pick_frame) const;
};
