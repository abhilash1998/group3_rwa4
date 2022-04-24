#include "agility.hpp"

#include <ros/node_handle.h>

void AgilityChallenger::order_callback(const nist_gear::Order::ConstPtr& msg)
{
    const std::string order_id = msg->order_id;
    // Temporary functionality, where order_1 is 'hardcoded' to be a high
    // priority order, others are not
    ROS_INFO_STREAM("Received order with ID '"
                    << order_id
                    << "'"
                    << ((order_id == "order_1") ? " (High-Priority!)" : ""));

    assert(!msg->kitting_shipments.empty());
    current_kitting_shipments = msg->kitting_shipments;
}

void AgilityChallenger::blackout_status_callback(const std_msgs::Bool::ConstPtr& msg)
{
    in_sensor_blackout = msg->data;
    ROS_INFO_STREAM("Blackout status updated: " << std::to_string(in_sensor_blackout));
}

void AgilityChallenger::help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx)
{
    // Clear the list of parts that this camera currently sees, and repopulate
    // it with updated data
    // Notice the vector is a reference
    std::vector<std::string>& current_parts_bin_idx = current_parts_found[bin_idx];
    current_parts_bin_idx.clear();
    for (auto iter_model = msg->models.begin(); iter_model != msg->models.end(); ++iter_model)
    {
        current_parts_bin_idx.push_back(iter_model->type);
    }
}

void AgilityChallenger::help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int agv_idx)
{
    if (msg->models.size() > 0)
    {
        ROS_ERROR_STREAM("FOUND FAULTY PART ON AGV" << agv_idx);
    }
}

void AgilityChallenger::logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1 with index 0
    help_logical_camera_image_callback(msg, 0);
}

void AgilityChallenger::logical_camera_image2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #2 with index 1
    help_logical_camera_image_callback(msg, 1);
}

void AgilityChallenger::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #1 with index 0
    help_quality_control_sensor_callback(msg, 0);
}

void AgilityChallenger::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #2 with index 1
    help_quality_control_sensor_callback(msg, 1);
}

void AgilityChallenger::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #3 with index 2
    help_quality_control_sensor_callback(msg, 2);
}

void AgilityChallenger::quality_control_sensor4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #4 with index 3
    help_quality_control_sensor_callback(msg, 3);
}

AgilityChallenger::AgilityChallenger(ros::NodeHandle* const nh) :
    tf_listener(tf_buffer),
    in_sensor_blackout(false)
{
    orders_subs = nh->subscribe<nist_gear::Order>(
        "/ariac/orders",
        1,
        &AgilityChallenger::order_callback,
        this
    );
    blackout_sub = nh->subscribe<std_msgs::Bool>(
        "/group3/in_sensor_blackout",
        1,
        &AgilityChallenger::blackout_status_callback,
        this
    );
    logical_camera_1_sub = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_1",
        1,
        &AgilityChallenger::logical_camera_image1_callback,
        this
    );
    logical_camera_2_sub = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_2",
        1,
        &AgilityChallenger::logical_camera_image2_callback,
        this
    );
    quality_control_sensor_subs[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_1",
        1,
        &AgilityChallenger::quality_control_sensor1_callback,
        this
    );
    quality_control_sensor_subs[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_2",
        1,
        &AgilityChallenger::quality_control_sensor2_callback,
        this
    );
    quality_control_sensor_subs[2] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_3",
        1,
        &AgilityChallenger::quality_control_sensor3_callback,
        this
    );
    quality_control_sensor_subs[3] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_4",
        1,
        &AgilityChallenger::quality_control_sensor4_callback,
        this
    );
}

AgilityChallenger::~AgilityChallenger()
{
}
