#include <ros/ros.h>
#include "data_logger/data_logger.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "data_logger_node");

  ros::NodeHandle nh("~");

  DataLogger datalogger(nh);

  ROS_INFO("START data_logger_node");

  ros::spin();

  return 0;
}
