#include "ariac_agv.hpp"
#include "sensors.hpp"

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <nist_gear/Order.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <memory>
#include <assert.h>

#define NUM_AGVS 4

int start(ros::NodeHandle* const nh)
{
    ros::Duration wait_for_competition_state(0.1);
    bool competition_state_valid = false;
    std::string competition_state;
    ros::Subscriber competition_state_sub = nh->subscribe<std_msgs::String>("/ariac/competition_state",1,[&](const std_msgs::String::ConstPtr& msg) 
    {
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
        ros::ServiceClient start_competition_scl = nh->serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // Wait a very long time, because this service server may take a while
        // to be created
        assert(start_competition_scl.waitForExistence(ros::Duration(30.0)));

        // Place the request
        std_srvs::Trigger start_competition_srv;
        if (!(start_competition_scl.call(start_competition_srv) && start_competition_srv.response.success))
        {
            ROS_ERROR_STREAM("Failed to start the competition: '"<< start_competition_srv.response.message);
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
            return 2;
        }
    }

}

int end(ros::NodeHandle* const nh)
{
     // End the competition if it hasn't been already. Spin the node until we've
    // collected the competition state. Reuse the variables from above.
    ros::Duration wait_for_competition_state(0.1);
    std::string competition_state;
    bool competition_state_valid = false;
    ros::Subscriber competition_state_sub = nh->subscribe<std_msgs::String>("/ariac/competition_state",1,[&](const std_msgs::String::ConstPtr& msg) 
    {
            competition_state = msg->data;
            competition_state_valid = true;
        }
    );
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
        ros::ServiceClient end_competition_scl = nh->serviceClient<std_srvs::Trigger>(
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
}

void world_tf(std::string p_type, int i, int b)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);
    ros::Duration timeout(5.0);
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer.lookupTransform("world", "logical_camera_bins"+std::to_string(b)+"_"+p_type+"_"+std::to_string(i)+"_frame",ros::Time(0), timeout);
    tf2::Quaternion q(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);

    //convert Quaternion to Euler angles
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Part in /world frame: [%f,%f,%f] [%f,%f,%f]",
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z,
            roll,
            pitch,
            yaw);
}


int order_av(ros::NodeHandle* const nh, int i)
{
    // Create a sub to collect orders
    ros::Duration wait_for_orders(0.1);
    bool order_valid = false;
    int agv_number;
    std::string assembly_station_name;
    std::string shipment_type;
    std::string product1_type;
    std::string product2_type;
    std::string order_id;
    bool new_order = true;
    ros::Subscriber orders_sub = nh->subscribe<nist_gear::Order>("/ariac/orders",1,[&](const nist_gear::Order::ConstPtr& msg) 
    {
            order_id =  msg-> order_id;
            if(i==1 && order_id.compare("order_0")==0)
            {
                new_order = false;
                return 0;
            }
            else if(order_id.compare("order_1")==0)
            {
                ROS_INFO_STREAM("High-priority order is announced");
            }
            ROS_INFO_STREAM("Received order with ID '"<< order_id<< "'");

            assert(!msg->kitting_shipments.empty());
            const nist_gear::KittingShipment ks = msg->kitting_shipments[0];

            agv_number = std::stoi(ks.agv_id.substr(3)); // Assume the AGV ID is agvX, with X in [1,NUM_AGVS]
            assembly_station_name = ks.station_id;
            shipment_type = ks.shipment_type;
            product1_type = ks.products[0].type;
            product2_type = ks.products[1].type;
            // ROS_INFO_STREAM("Product_1 : '"<< product1_type<< "'");
            // ROS_INFO_STREAM("Product_2 : '"<< product2_type<< "'");
            order_valid = true;

        }
    );
    //if (order_id.compare(porder_id)==0)
    
    int len = 0;
    bool found_1 = false;
    bool found_2 = false;
    std::string part_detected;

    ros::Subscriber logical_camera_bins0_sub = nh->subscribe<nist_gear::LogicalCameraImage>("/ariac/logical_camera_bins0", 1, [&](const nist_gear::LogicalCameraImage::ConstPtr& msg)
    {
        len = msg->models.size();
        int p1 = 0;
        int p2 = 0;
        for(int i=0;i<len;i++)
        {
            part_detected = msg->models[i].type;
            if (product1_type.compare(product2_type)==0)
            {
                if(part_detected.compare(product1_type)==0)
                {
                    p1=p1+1;
                    world_tf(part_detected,p1,0);
                    found_1 = true;
                    if (p1>1){found_2 = true;}
                }
            }
            else{
                if(part_detected.compare(product1_type)==0)
                {
                    //ROS_INFO_STREAM("Product 1 Found: "<<part_detected);
                    p1=p1+1;
                    world_tf(part_detected,p1,0);
                    found_1 = true;
                }
                if(part_detected.compare(product2_type)==0)
                {
                    //ROS_INFO_STREAM("Product 2 Found: "<<part_detected);
                    p2=p2+1;
                    world_tf(part_detected,p2,0);
                    found_2 = true;
                }
            }
        }

    }
    );

    ros::Subscriber logical_camera_bins1_sub = nh->subscribe<nist_gear::LogicalCameraImage>("/ariac/logical_camera_bins1", 1, [&](const nist_gear::LogicalCameraImage::ConstPtr& msg)
    {
        len = msg->models.size();
        int p1 = 0;
        int p2 = 0;
        for(int i=0;i<len;i++)
        {
            part_detected = msg->models[i].type;
            if (product1_type.compare(product2_type)==0)
            {
                if(part_detected.compare(product1_type)==0)
                {
                    p1=p1+1;
                    world_tf(part_detected,p1,1);
                    found_1 = true;
                    if (p1>1){found_2 = true;}
                }
            }
            else{
                if(part_detected.compare(product1_type)==0)
                {
                    //ROS_INFO_STREAM("Product 1 Found: "<<part_detected);
                    p1=p1+1;
                    world_tf(part_detected,p1,1);
                    found_1 = true;
                }
                if(part_detected.compare(product2_type)==0)
                {
                    //ROS_INFO_STREAM("Product 2 Found: "<<part_detected);
                    p2=p2+1;
                    world_tf(part_detected,p2,0);
                    found_2 = true;
                }
            }
        }

    }
    );


    ros::Duration(1).sleep();
    ros::spinOnce();
    if(!new_order){return 0;}
    if(!found_1 || !found_2)
    {
        // ros::Duration(20.0).sleep();
        // ros::spinOnce();
        ROS_INFO_STREAM("Insufficient parts to complete "<< order_id);
    }

    if (!order_valid)
    {
        ROS_ERROR_STREAM("No orders were found!");
        return 3;
    }
}

void quality(ros::NodeHandle* const nh)
{
    for (int i=1;i<=4;i++){
    ros::Subscriber quality_control_sensor_1_sub = nh->subscribe<nist_gear::LogicalCameraImage>("/ariac/quality_control_sensor_"+std::to_string(i), 1, [&](const nist_gear::LogicalCameraImage::ConstPtr& msg) 
    {
        // ROS_INFO_STREAM("AGV"<<i<<" :"<<msg->models.size());
        if(msg->models.size() > 0)
        {
            ROS_ERROR_STREAM("FOUND FAULTY PART ON AGV"<<i);
        }
    });
    ros::Duration(1).sleep();
    ros::spinOnce();
    }
}

void sensor_fail(ros::NodeHandle* const nh)
{
    bool state = false;
    ros::Subscriber logical_camera_bins1_sub = nh->subscribe<nist_gear::LogicalCameraImage>("/ariac/logical_camera_bins1", 1, [&](const nist_gear::LogicalCameraImage::ConstPtr& msg)
    {
       state = true;
    }
    );

    ros::Duration(1).sleep();
    ros::spinOnce();
    if(!state)
    {
        ROS_ERROR_STREAM("SENSOR BLACK OUT");
    }
}

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
    std::string porder_id;
    // Start the AGVs controller node
    ros::init(argc, argv, "rwa2");
    ros::NodeHandle nh;

    start(&nh);
    order_av(&nh,0);
    while(true)
    {
        sensor_fail(&nh);
        quality(&nh);
        if(!porder_id.compare("order_1")==0){ order_av(&nh,1);}
    }
    end(&nh);

    ros::shutdown();

    return 0;
}
