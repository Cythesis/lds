#include <ros/ros.h>
#include "lds/request.h"
#include "controller.cpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
  // Initialization
  ros::init(argc, argv, "create_environment_server");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::Rate rate(10.0);
  Controller ur5;
  ros::ServiceServer service = n.advertiseService("create_environment", &Controller::createEnvironmentCallback, &ur5);
  ROS_INFO("Client ready to receive commands.");
  ros::waitForShutdown();
  return 0;
}