#include "agility.hpp"

#include <ros/node_handle.h>

#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <numeric>
#include <cmath>

void AgilityChallenger::order_callback(const nist_gear::Order::ConstPtr& msg)
{
    // Temporary functionality, where order_1 is 'hardcoded' to be a high
    // priority order, others are not
    pending_order = *msg;
    pending_order_priority = (pending_order.order_id == "order_1") ? 2 : 1;

    ROS_INFO_STREAM("Received order with ID '"
                    << pending_order.order_id
                    << "' with priority "
                    << pending_order_priority);
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
    std::vector<std::string>& current_parts_bin_idx = current_detected_parts[bin_idx];
    current_parts_bin_idx.clear();
    for (auto iter_model = msg->models.begin(); iter_model != msg->models.end(); ++iter_model)
    {
        current_parts_bin_idx.push_back(iter_model->type);
    }
}

void AgilityChallenger::help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int lc_idx)
{
    const nist_gear::LogicalCameraImage new_results = *msg;
    if (new_results.models.size() != current_qc_results[lc_idx].models.size())
    {
        ROS_INFO_STREAM("Number of faulty models from logical camera #"
                        << lc_idx+1
                        << " changed from "
                        << current_qc_results[lc_idx].models.size()
                        << " to "
                        << new_results.models.size());
    }

    current_qc_results[lc_idx] = new_results;

    const std::string agv_id = std::string("agv") + std::to_string(lc_idx+1);

    // Loop through each of the existing parts that need verification. Create a
    // new list, where parts are only added if they show up as faulty by the QC
    // sensor.
    std::vector<PartForFaultVerification> new_list;
    for (const PartForFaultVerification& part : parts_for_fault_verification[agv_id])
    {
        // We have the part already resolved in world frame, store its 3D
        // position in ppt
        bool found_part = false;
        const geometry_msgs::Point ppt = part.second.position;
        for (const nist_gear::Model model : new_results.models)
        {
            // Get this model in world frame, store its 3D position in mpt
            geometry_msgs::Pose mpose;
            tf2::Transform qc_camera_tf, part_tf;
            tf2::fromMsg(current_qc_results[lc_idx].pose, qc_camera_tf);
            tf2::fromMsg(model.pose, part_tf);
            tf2::toMsg(qc_camera_tf * part_tf, mpose);
            const geometry_msgs::Point mpt = mpose.position;

            // If its within (0.1m, 0.1m) on the X/Y plane, consider it the same model
            const double dx = std::abs(mpt.x - ppt.x);
            const double dy = std::abs(mpt.y - ppt.y);
            ROS_DEBUG_STREAM("ppt=[x:" << ppt.x << ",y:" << ppt.y << "] vs. mpt=[x:" << mpt.x << ",y:" << mpt.y << "]");
            ROS_DEBUG_STREAM("dx=" << dx << ", dy=" << dy);
            if ((dx < 0.1) && (dy < 0.1))
            {
                // We found the part we were interested in
                found_part = true;
                new_list.push_back(part);
                break;
            }
        }
        if (!found_part)
        {
            // If we didn't find the part, it means the model wasn't picked up
            // by the QC camera, meaning that either 1.) it was never faulty
            // and this confirmed that, or 2.) it was faulty but was removed
            // from the tray by the robot
            ROS_INFO_STREAM("Removed part at [x:" << ppt.x << ",y:" << ppt.y << "] from faulty part verification queue");
        }
    }
    parts_for_fault_verification[agv_id] = new_list;
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

void AgilityChallenger::logical_camera_image3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #3 with index 2
    help_logical_camera_image_callback(msg, 2);
}

void AgilityChallenger::logical_camera_image4_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #4 with index 3
    help_logical_camera_image_callback(msg, 3);
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
    pending_order_priority(0),
    in_sensor_blackout(false)
{
    orders_subs = nh->subscribe<nist_gear::Order>(
        "/ariac/orders",
        1,
        &AgilityChallenger::order_callback,
        this
    );
    blackout_sub = nh->subscribe<std_msgs::Bool>(
        "/group3/blackout_active",
        1,
        &AgilityChallenger::blackout_status_callback,
        this
    );
    logical_camera_subs[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_1",
        1,
        &AgilityChallenger::logical_camera_image1_callback,
        this
    );
    logical_camera_subs[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_2",
        1,
        &AgilityChallenger::logical_camera_image2_callback,
        this
    );
    logical_camera_subs[2] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_3",
        1,
        &AgilityChallenger::logical_camera_image3_callback,
        this
    );
    logical_camera_subs[3] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_4",
        1,
        &AgilityChallenger::logical_camera_image4_callback,
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

    parts_for_fault_verification = {
        {"agv1", {}},
        {"agv2", {}},
        {"agv3", {}},
        {"agv4", {}}
    };
}

AgilityChallenger::~AgilityChallenger()
{
}

bool AgilityChallenger::is_sensor_blackout_active() const
{
    return in_sensor_blackout;
}

void AgilityChallenger::queue_for_fault_verification(const std::string& agv_id,
                                                     const std::string& product_type,
                                                     const geometry_msgs::Pose& objective_pose_in_world)
{
    ROS_INFO_STREAM("Queueing part for fault verification: "
                    << agv_id
                    << ", "
                    << product_type
                    << ", "
                    << objective_pose_in_world);
    parts_for_fault_verification[agv_id].push_back(std::make_pair(product_type, objective_pose_in_world));
}

bool AgilityChallenger::needs_fault_verification(const std::string& agv_id)
{
    return !parts_for_fault_verification[agv_id].empty();
}

int AgilityChallenger::consume_pending_order(nist_gear::Order& order)
{
    // 'Consume' the current order
    order = pending_order;
    pending_order = nist_gear::Order();

    // 'Consume' its priority
    const int priority = pending_order_priority;
    pending_order_priority = 0;

    return priority;
}

bool AgilityChallenger::higher_priority_order_requested(const int current_priority) const
{
    return pending_order_priority > current_priority;
}

std::vector<int> AgilityChallenger::get_camera_indices_of(const std::string& product_type) const
{
    std::vector<int> indices;
    for (int i = 0; i < current_detected_parts.size(); i++)
    {
        const std::vector<std::string>& lcd = current_detected_parts[i];
        if (lcd.cend() != std::find(lcd.cbegin(), lcd.cend(), product_type))
        {
            indices.push_back(i+1);
        }
    }
    return indices;
}

std::string AgilityChallenger::get_logical_camera_contents() const
{
    std::string str = "{";
    for (int i = 0; i < current_detected_parts.size(); i++)
    {
        if (i != 0)
        {
            str += ", ";
        }

        const std::vector<std::string>& lcd = current_detected_parts[i];
        str += (std::to_string(i+1) + ": [");
        if (!lcd.empty())
        {
            str += std::accumulate(
                std::next(lcd.begin()),
                lcd.end(),
                lcd[0],
                [](const std::string& a, const std::string& b) { return std::move(a) + ',' + b; }
            );
        }
        str += "]";
    }
    return str;
}

bool AgilityChallenger::get_agv_faulty_part(std::string& agv_id,
                                            std::string& product_type,
                                            geometry_msgs::Pose& pick_frame) const
{
    for (auto iter = parts_for_fault_verification.cbegin(); iter != parts_for_fault_verification.cend(); ++iter)
    {
        const std::vector<PartForFaultVerification> iter_parts = iter->second;
        if (!iter_parts.empty())
        {
            agv_id = iter->first;
            std::tie(product_type, pick_frame) = iter_parts.front();
            return true;
        }
    }
    return false;
}
