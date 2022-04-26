#include <ros/subscriber.h>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Product.h>

#include <string>
#include <array>
#include <vector>
#include <unordered_map>

class AgilityChallenger
{
protected:
    using PartForFaultVerification = std::pair<nist_gear::Product, geometry_msgs::Pose>;

    ros::Subscriber orders_subs;
    ros::Subscriber blackout_sub;
    std::array<ros::Subscriber, 4> logical_camera_subs;
    std::array<ros::Subscriber, 4> quality_control_sensor_subs;

    // The relative priority of the order at \a pending_order. If this is 0,
    // the \a pending_order is not populated / not a valid order.
    int pending_order_priority;

    // An order that was received and is pending to be catered. This is
    // consumed by callers of consume_pending_order(nist_gear::Order&).
    nist_gear::Order pending_order;

    // The parts currently detected by the logical cameras
    std::array<std::vector<std::string>, 4> current_detected_parts;

    // True if a sensor blackout was detected right now, false otherwise
    bool in_sensor_blackout;

    // 
    std::unordered_map<std::string, std::vector<PartForFaultVerification>> parts_for_fault_verification;

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

    // Getter for \a sensor_blackout_active.
    // @return The value of \a sensor_blackout_active (true if a blackout is
    // active, false otherwise).
    bool is_sensor_blackout_active() const;

    // 
    void queue_for_fault_verification(const std::string& agv_id,
                                      const nist_gear::Product& product,
                                      const geometry_msgs::Pose& objective_pose_in_world);

    // 
    bool needs_fault_verification(const std::string& agv_id);

    // Pass ownership of \a pending_order off to the caller of this method if
    // it is populated.
    // @param order If there is a pending order (\a pending_order_priority is
    // nonzero), this is overwritten with \a pending_order.
    // @return The current value of \a pending_order_priority.
    int consume_pending_order(nist_gear::Order& order);

    // Get whether or not there is an order with a priority that is higher than
    // the given one.
    // @param current_priority The priority of the order being catered by the
    // caller of this method.
    // @return Whether or not \a pending_order_priority is greater than
    // \a current_priority.
    bool higher_priority_order_requested(const int current_priority) const;

    std::vector<int> get_camera_indices_of(const std::string& product_type) const;
    std::string get_logical_camera_contents() const;

    // If there are any faulty parts, get the pick pose for one of them.
    // @param pick_frame If this method returns true, then this value is
    // overwritten with the pose of a faulty part resolved in world frame,
    // if false then this value is not overwritten
    // @return True if there is a faulty part, false otherwise.
    bool get_agv_faulty_part(std::string& agv_id,
                             nist_gear::Product& product,
                             geometry_msgs::Pose& pick_frame) const;
};
