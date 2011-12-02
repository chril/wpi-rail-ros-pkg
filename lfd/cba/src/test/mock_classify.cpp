/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <lfd_common/state.h>
#include <lfd_common/classification_point.h>
#include <lfd_common/conf_classification.h>
#include <ros/ros.h>

using namespace std;

bool classify_callback(lfd_common::conf_classification::Request &req, lfd_common::conf_classification::Response &resp)
{
  resp.c = 0.123;
  resp.l = 10;
  resp.db = -58;
  return true;
}

void add_point_callback(const lfd_common::classification_point::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_classify");
  // a handle for this ROS node
  ros::NodeHandle node;

  // create services and topics
  ros::ServiceServer classify = node.advertiseService("classify", classify_callback);
  ros::Subscriber add_point = node.subscribe<lfd_common::classification_point> ("add_point", -1, add_point_callback);

  ROS_INFO("Mock Classifier Initialized");

  // run the node
  ros::spin();

  return EXIT_SUCCESS;
}
