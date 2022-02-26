#include "ariac_agv.hpp"

#include <ros/ros.h>

#include <memory>

#define NUM_AGVS 4

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agvs_controller");
    ros::NodeHandle nh;
    std::vector<std::unique_ptr<AriacAgv>> agvs;
    for (int i = 0; i < NUM_AGVS; i++)
    {
        // AGV topics use identifiers in the range [1,NUM_AGVS], but this loop
        // is [0,NUM_AGVS-1], so add 1 to all indices when creating them here
        agvs.push_back(std::unique_ptr<AriacAgv>(new AriacAgv(&nh, i+1)));
    }
    ros::spin();
    return 0;
}
