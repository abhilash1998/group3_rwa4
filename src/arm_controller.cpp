#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "arm.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    Arm arm;
    arm.goToPresetLocation("home1");
    arm.goToPresetLocation("home2");

    return 0;
}
