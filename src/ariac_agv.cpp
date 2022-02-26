#include "ariac_agv.hpp"

#include <ros/node_handle.h>
#include <nist_gear/AGVToAssemblyStation.h>

#include <vector>

void AriacAgv::state_callback(const std_msgs::String::ConstPtr& msg)
{
    curr_state = msg->data;
    ROS_DEBUG_STREAM("AGV #"
                     << number
                     << " received updated state message: "
                     << curr_state
    );
}
void AriacAgv::station_callback(const std_msgs::String::ConstPtr& msg)
{
    curr_station = msg->data;
    ROS_INFO_STREAM("AGV #"
                    << number
                    << " received updated station message: "
                    << curr_station
    );
}

AriacAgv::AriacAgv(ros::NodeHandle* const nh, const int agv_number) :
        number(agv_number)
{
    const std::string prefix = std::string("/ariac/agv") + std::to_string(number);
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
    submit_shipment_scl = nh->serviceClient<nist_gear::AGVToAssemblyStation>(
        prefix + "/submit_shipment"
    );
}
AriacAgv::~AriacAgv()
{
}

bool AriacAgv::submit_shipment(const std::string& assembly_station_name,
                               const std::string& shipment_type)
{
    nist_gear::AGVToAssemblyStation submit_srv;
    submit_srv.request.assembly_station_name = assembly_station_name;
    submit_srv.request.shipment_type = shipment_type;

    bool rc = false;
    if (submit_shipment_scl.call(submit_srv))
    {
        rc = submit_srv.response.success;
        std::string log_message = "AGV #"
            + std::to_string(number)
            + " submitted shipment ["
            + submit_srv.request.assembly_station_name
            + ","
            + submit_srv.request.shipment_type
            + ","
            + (rc ? "SUCCESS" : "FAILURE")
            + "]"
        ;
        if (!submit_srv.response.message.empty())
        {
            log_message += ": " + submit_srv.response.message;
        }
        if (rc)
        {
            ROS_INFO_STREAM(log_message);
        }
        else
        {
            ROS_ERROR_STREAM(log_message);
        }
    }
    else
    {
        ROS_ERROR_STREAM("AGV #"
                         << number
                         << " shipment submission failed!"
        );
    }

    return rc;
}
