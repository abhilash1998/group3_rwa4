#include "agility.hpp"

#include <ros/node_handle.h>
#include <ros/duration.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Matrix3x3.h>

void AgilityChallenger::annouce_world_tf(const std::string part_name, const std::string frame)
{
    const geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform("world", frame, ros::Time(0));

    //convert Quaternion to Euler angles
    const tf2::Matrix3x3 m(tf2::Quaternion(
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z,
        transformStamped.transform.rotation.w
    ));
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Part '%s' in /world frame: [%f,%f,%f] [%f,%f,%f]",
            part_name.c_str(),
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z,
            roll,
            pitch,
            yaw);
}

void AgilityChallenger::competition_state_callback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data != current_competition_state)
    {
        current_competition_state = msg->data;
        ROS_INFO_STREAM("Competition state updated: " << current_competition_state);
    }
}

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
    const nist_gear::KittingShipment ks = msg->kitting_shipments[0];

    const std::string shipment_type = ks.shipment_type;
    current_product0_type = ks.products[0].type;
    current_product1_type = ks.products[1].type;
    current_parts_found[0].clear();
    current_parts_found[1].clear();
    announced_part_frames = false;
    //ROS_INFO_STREAM("Product_0 : '"<< current_product0_type<< "'");
    //ROS_INFO_STREAM("Product_1 : '"<< current_product1_type<< "'");

    // If all of the parts are not currently available, check back again in
    // 20 seconds
    if (evaluate_if_all_parts_found(-1))
    {
        ROS_INFO_STREAM("All parts for " << order_id << " were found!");
    }
    else
    {
        ROS_WARN_STREAM("Did not immediately find all parts for " << order_id << "!");
        watch_for_order_parts_tmr.start();
    }
}

void AgilityChallenger::help_logical_camera_image_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int bin_idx)
{
    // First, we received sensor data, so we are not in a sensor blackout. We
    // stop the timer, and then at the end of the function, we restart it so we
    // can continue to monitor for blackouts. If a sensor blackout never
    // occurs, then that means this callback is called often enough that the
    // timer never fires. Also, 'sensors_started' is used only to indicate that
    // one or more callbacks to this function have been received. This deals
    // with the time delay that comes with creating a pub/sub connection.
    sensors_started = true;
    in_sensor_blackout = false;
    watch_for_blackouts_tmr.stop();

    // Clear the list of parts that this camera currently sees, and repopulate
    // it with updated data
    // Notice the vector is a reference
    std::vector<std::string>& current_parts_bin_idx = current_parts_found[bin_idx];
    current_parts_bin_idx.clear();
    for (auto iter_model = msg->models.begin(); iter_model != msg->models.end(); ++iter_model)
    {
        current_parts_bin_idx.push_back(iter_model->type);
    }

    evaluate_if_all_parts_found(bin_idx);

    // As mentioned above, we restart the timer here to continue monitoring for
    // sensor blackouts
    watch_for_blackouts_tmr.start();
}

void AgilityChallenger::help_quality_control_sensor_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg, const int agv_idx)
{
    if (msg->models.size() > 0)
    {
        ROS_ERROR_STREAM("FOUND FAULTY PART ON AGV" << agv_idx);
    }
}

bool AgilityChallenger::evaluate_if_all_parts_found(const int bin_idx)
{
#define BUILD_FRAME(p, i, b) \
    "logical_camera_bins" + std::to_string(b) + "_" + p + "_" + std::to_string(i) + "_frame"

    // Count the number of each required product type that was found
    bool did_announcement = false;
    int current_product0_type_found = 0;
    int current_product1_type_found = 0;
    for (int i = 0; i < 2; i++)
    {
        // Get the list of parts captured by camera #i
        const std::vector<std::string>& current_parts_bin_i = current_parts_found[i];
        for (auto part_iter = current_parts_bin_i.begin(); part_iter != current_parts_bin_i.end(); ++part_iter)
        {
            const std::string part = *part_iter;
            if(part.compare(current_product0_type)==0)
            {
                // We found an instance of product0
                // Only print its pose relative to the world frame if it is in
                // the bin of the camera that issued this callback
                current_product0_type_found++;
                if (!announced_part_frames && (bin_idx == i))
                {
                    annouce_world_tf(part, BUILD_FRAME(part,current_product0_type_found,i));
                    did_announcement = true;
                }
            }
            else if(part.compare(current_product1_type)==0)
            {
                // Same as above, but for product1
                current_product1_type_found++;
                if (!announced_part_frames && (bin_idx == i))
                {
                    annouce_world_tf(part, BUILD_FRAME(part,current_product1_type_found,i));
                    did_announcement = true;
                }
            }
        }
    }

    announced_part_frames |= did_announcement;
    // ros::Duration(0.25).sleep();
    // ros::spinOnce();

#undef BUILD_FRAME

    // This checks if the most recently collected models seen by the camera
    // could complete the order if they are different types
    const bool diff_parts_found = (current_product0_type_found > 0) && (current_product1_type_found > 0);
    // This is for when the parts of the same type
    const bool same_parts_found = (current_product0_type == current_product1_type) && (current_product0_type_found > 1);
    // If either of those are true, then all parts are found
    return diff_parts_found || same_parts_found;
}

void AgilityChallenger::logical_camera_image0_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #0
    help_logical_camera_image_callback(msg, 0);
}

void AgilityChallenger::logical_camera_image1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for bin #1
    help_logical_camera_image_callback(msg, 1);
}

void AgilityChallenger::quality_control_sensor0_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #0
    help_quality_control_sensor_callback(msg, 0);
}

void AgilityChallenger::quality_control_sensor1_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #1
    help_quality_control_sensor_callback(msg, 1);
}

void AgilityChallenger::quality_control_sensor2_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #2
    help_quality_control_sensor_callback(msg, 2);
}

void AgilityChallenger::quality_control_sensor3_callback(const nist_gear::LogicalCameraImage::ConstPtr& msg)
{
    // Callback for agv #3
    help_quality_control_sensor_callback(msg, 3);
}

void AgilityChallenger::sensor_blackout_detected_callback(const ros::TimerEvent& evt)
{
    if ((current_competition_state == "go") && sensors_started && !in_sensor_blackout)
    {
        in_sensor_blackout = true;
        ROS_ERROR_STREAM("SENSOR BLACK OUT");
    }
}

void AgilityChallenger::handle_check_for_order_parts(const ros::TimerEvent& evt)
{
    if (evaluate_if_all_parts_found(-1))
    {
        ROS_INFO_STREAM("Found all parts for the order!");
    }
    else
    {
        ROS_ERROR_STREAM("INSUFFICIENT PRODUCTS FOUND!");
    }
    watch_for_order_parts_tmr.stop();
}

AgilityChallenger::AgilityChallenger(ros::NodeHandle* const nh) :
    tf_listener(tf_buffer),
    current_competition_state(""),
    current_product0_type(""),
    current_product1_type(""),
    sensors_started(false),
    in_sensor_blackout(false),
    announced_part_frames(false)
{
    competition_state_sub = nh->subscribe<std_msgs::String>(
        "/ariac/competition_state",
        1,
        &AgilityChallenger::competition_state_callback,
        this
    );
    orders_subs = nh->subscribe<nist_gear::Order>(
        "/ariac/orders",
        1,
        &AgilityChallenger::order_callback,
        this
    );
    logical_camera_bins0_sub = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_bins0",
        1,
        &AgilityChallenger::logical_camera_image0_callback,
        this
    );
    logical_camera_bins1_sub = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/logical_camera_bins1",
        1,
        &AgilityChallenger::logical_camera_image1_callback,
        this
    );
    quality_control_sensor_subs[0] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_0",
        1,
        &AgilityChallenger::quality_control_sensor0_callback,
        this
    );
    quality_control_sensor_subs[1] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_1",
        1,
        &AgilityChallenger::quality_control_sensor1_callback,
        this
    );
    quality_control_sensor_subs[2] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_2",
        1,
        &AgilityChallenger::quality_control_sensor2_callback,
        this
    );
    quality_control_sensor_subs[3] = nh->subscribe<nist_gear::LogicalCameraImage>(
        "/ariac/quality_control_sensor_3",
        1,
        &AgilityChallenger::quality_control_sensor3_callback,
        this
    );
    watch_for_blackouts_tmr = nh->createTimer(
        ros::Duration(1.0),
        &AgilityChallenger::sensor_blackout_detected_callback,
        this
    );
    watch_for_order_parts_tmr = nh->createTimer(
        ros::Duration(20.0),
        &AgilityChallenger::handle_check_for_order_parts,
        this,
        false,
        false
    );
}

AgilityChallenger::~AgilityChallenger()
{
}
