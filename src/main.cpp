#include "sub_sensor.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_subscriber");
  ros::NodeHandle n;
  Sensors s(n);
  s.startdetect();
  ros::spin();

  return 0;
}
