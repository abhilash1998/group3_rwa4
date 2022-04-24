#include "agility.hpp"

#include <ros/node_handle.h>
#include <ros/duration.h>

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
    current_kitting_shipments = msg->kitting_shipments;
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

AgilityChallenger::AgilityChallenger(ros::NodeHandle* const nh) :
    tf_listener(tf_buffer),
    current_competition_state(""),
    sensors_started(false),
    in_sensor_blackout(false)
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
}

AgilityChallenger::~AgilityChallenger()
{
}
