#include "ariac_agv.hpp"

#include <ros/ros.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <assert.h>

#define NUM_AGVS 4

/// @fn int main(int, char**)
/// @brief Main avg function to start the competition and take oreders
///
/// @pre
/// @post
/// @param argc
/// @param argv
/// @return
int main(int argc, char **argv)
{
    // Start the AGVs controller node
    ros::init(argc, argv, "agvs_controller");
    ros::NodeHandle nh;

    // Create interfaces to each of the AGVs
    std::vector<std::unique_ptr<AriacAgv>> agvs;
    for (int i = 0; i < NUM_AGVS; i++)
    {
        // AGV topics use identifiers in the range [1,NUM_AGVS], but this loop
        // is [0,NUM_AGVS-1], so add 1 to all indices when creating them here
        agvs.push_back(std::unique_ptr<AriacAgv>(new AriacAgv(&nh, i+1)));
    }

    // (This may be moved elsewhere eventually)
    // Manage the competition state and submit AGV shipments as appropriate

    // Create a sub to collect the current competition state ASAP
    ros::Duration wait_for_competition_state(0.1);
    bool competition_state_valid = false;
    std::string competition_state;
    ros::Subscriber competition_state_sub = nh.subscribe<std_msgs::String>(
        "/ariac/competition_state",
        1,
        [&](const std_msgs::String::ConstPtr& msg) {
            competition_state = msg->data;
            competition_state_valid = true;
        }
    );

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
        ros::ServiceClient start_competition_scl = nh.serviceClient<std_srvs::Trigger>(
            "/ariac/start_competition"
        );
        // Wait a very long time, because this service server may take a while
        // to be created
        assert(start_competition_scl.waitForExistence(ros::Duration(30.0)));

        // Place the request
        std_srvs::Trigger start_competition_srv;
        if (!(start_competition_scl.call(start_competition_srv)
              && start_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to start the competition: '"
                             << start_competition_srv.response.message
            );
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
            ROS_ERROR_STREAM("Competition state did not update as expected: '"
                             << competition_state
            );
            return 2;
        }
    }

    // Create a sub to collect orders
    ros::Duration wait_for_orders(0.1);
    bool order_valid = false;
    int agv_number;
    std::string assembly_station_name;
    std::string shipment_type;
    ros::Subscriber orders_sub = nh.subscribe<nist_gear::Order>(
        "/ariac/orders",
        1,
        [&](const nist_gear::Order::ConstPtr& msg) {
            ROS_INFO_STREAM("Received order with ID '"
                            << msg->order_id
                            << "'"
            );

            assert(!msg->kitting_shipments.empty());
            const nist_gear::KittingShipment ks = msg->kitting_shipments[0];

            agv_number = std::stoi(ks.agv_id.substr(3)); // Assume the AGV ID is agvX, with X in [1,NUM_AGVS]
            assembly_station_name = ks.station_id;
            shipment_type = ks.shipment_type;
            order_valid = true;
        }
    );

    // Wait, then submit the shipment. Subtract one from from the AGV number to
    // shift the range from [1,NUM_AGVS] to [0,NUM_AGVS-1].
    ros::Duration(60.0).sleep();
    ros::spinOnce();
    if (!order_valid)
    {
        ROS_ERROR_STREAM("No orders were found!");
        return 3;
    }
    agvs[agv_number-1]->submit_shipment(
        assembly_station_name,
        shipment_type
    );

    // Let the AGV move around a little before shutting down
    ros::Duration(10.0).sleep();

    // End the competition if it hasn't been already. Spin the node until we've
    // collected the competition state. Reuse the variables from above.
    competition_state_valid = false;
    while (!competition_state_valid)
    {
        wait_for_competition_state.sleep();
        ros::spinOnce();
    }

    // If the competition has not ended, we place a request to end it
    if (competition_state == "done")
    {
        ROS_INFO_STREAM("Competition is already ended.");
    }
    else
    {
        // Create the client
        assert(competition_state == "go");
        ros::ServiceClient end_competition_scl = nh.serviceClient<std_srvs::Trigger>(
            "/ariac/end_competition"
        );
        // Wait a shorter amount of time, because it should be created already
        assert(end_competition_scl.waitForExistence(ros::Duration(0.5)));

        // Place the request
        std_srvs::Trigger end_competition_srv;
        if (!(end_competition_scl.call(end_competition_srv)
              && end_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to end the competition: '"
                             << end_competition_srv.response.message
            );
            return 4;
        }

        // Wait a little and collect the updated competition state
        competition_state_valid = false;
        ros::Duration(0.25).sleep();
        ros::spinOnce();
        if (competition_state == "done")
        {
            ROS_INFO_STREAM("Competition was ended.");
        }
        else
        {
            ROS_ERROR_STREAM("Competition state did not update as expected: '"
                             << competition_state
            );
            return 5;
        }
    }

    ros::shutdown();

    return 0;
}
