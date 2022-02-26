#include "sub_agv.hpp"

#include <ros/ros.h>

#include <vector>

#define NUM_AGVS 4

void AriacAgv::state_callback(const std_msgs::String::ConstPtr& msg)
{
    curr_state = msg->data;
    ROS_INFO_STREAM("Received updated state message: " << curr_state);
}
void AriacAgv::station_callback(const std_msgs::String::ConstPtr& msg)
{
    curr_station = msg->data;
    ROS_INFO_STREAM("Received updated station message: " << curr_station);
}

AriacAgv::AriacAgv(ros::NodeHandle* const nh, const int agv_number)
{
    const std::string prefix = std::string("/ariac/agv") + std::to_string(agv_number);
    state_sub = nh->subscribe(
        prefix + "/state",
        1,
        &AriacAgv::state_callback,
        this
    );
    station_sub = nh->subscribe(
        prefix + "/station",
        1,
        &AriacAgv::station_callback,
        this
    );
}
AriacAgv::~AriacAgv()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agv_handler");
    ros::NodeHandle nh;
    std::vector<AriacAgv> agvs;
    for (int i = 0; i < NUM_AGVS; i++)
    {
        // AGV topics use identifiers in the range [1,NUM_AGVS], but this loop
        // is [0,NUM_AGVS-1], so add 1 to all indices when creating them here
        agvs.push_back(AriacAgv(&nh, i+1));
    }
    ros::spin();
    return 0;
}
