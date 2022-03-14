#include <ros/ros.h>
#include <ros/timer.h>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include "agility.hpp"

#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agility_controller");
    ros::NodeHandle nh;

    // Create our node to monitor agility challenges
    AgilityChallenger agility_challenger(&nh);

	ros::Duration wait_for_competition_state(0.1);
    bool competition_state_valid = false;
    std::string competition_state;
    ros::Subscriber competition_state_sub = nh.subscribe<std_msgs::String>(
		"/ariac/competition_state",
		1,
		[&](const std_msgs::String::ConstPtr& msg) 
		{
            competition_state = msg->data;
            competition_state_valid = true;
        }
    );

	//
	// Start the competition
	//

    // Spin the node until we've collected the competition state
    while (!competition_state_valid)
    {
        wait_for_competition_state.sleep();
        ros::spinOnce();
    }

    // If the competition has not started, we place a request to start it
    if (competition_state == "go")
    {
        ROS_INFO_STREAM("Competition is already started.");
    }
    else
    {
        // Create the client
        assert(competition_state == "init");
        ros::ServiceClient start_competition_scl = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // Wait a very long time, because this service server may take a while
        // to be created
        assert(start_competition_scl.waitForExistence(ros::Duration(30.0)));

        // Place the request
        std_srvs::Trigger start_competition_srv;
        if (!(start_competition_scl.call(start_competition_srv) && start_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to start the competition: '"<< start_competition_srv.response.message);
            std::cout << "Broke 1" << std::endl;
            return 1;
        }

        // Wait a little and collect the updated competition state
        competition_state_valid = false;
        ros::Duration(0.25).sleep();
        ros::spinOnce();
        if (competition_state == "go")
        {
            ROS_INFO_STREAM("Competition was started.");
        }
        else
        {
            ROS_ERROR_STREAM("Competition state did not update as expected: '"<< competition_state);
            std::cout << "Broke 2" << std::endl;
            return 2;
        }
    }

    // Spin indefinitely
    ros::spin();

    return 0;
}
