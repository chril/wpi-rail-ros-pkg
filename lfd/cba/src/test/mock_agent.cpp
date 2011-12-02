/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <lfd_common/demonstration.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

#include <iostream>

using namespace std;

void execute_callback(const std_msgs::Int32::ConstPtr &msg)
{

}

bool demonstration_callback(lfd_common::demonstration::Request &req, lfd_common::demonstration::Response &resp)
{
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_state_publisher");
  // a handle for this ROS node
  ros::NodeHandle node;

  // create services and topics
  ros::Subscriber execute = node.subscribe<std_msgs::Int32> ("execute", 1, execute_callback);
  ros::Publisher a_complete = node.advertise<std_msgs::Bool> ("a_complete", 1);
  ros::ServiceServer dem = node.advertiseService("demonstration", demonstration_callback);

  ROS_INFO("Mock Agent Initialized");

  ros::spin();

  return EXIT_SUCCESS;
}
