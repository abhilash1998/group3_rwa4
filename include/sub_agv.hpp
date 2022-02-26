#include <ros/subscriber.h>
#include <std_msgs/String.h>

#include <string>

// Forward declare
namespace ros {
    class NodeHandle;
}

class AriacAgv
{
protected:
    ros::Subscriber state_sub;
    ros::Subscriber station_sub;

    std::string curr_state;
    std::string curr_station;

    void state_callback(const std_msgs::String::ConstPtr& msg);
    void station_callback(const std_msgs::String::ConstPtr& msg);

public:
    AriacAgv(ros::NodeHandle* const nh, const int agv_number);
    ~AriacAgv();
};
