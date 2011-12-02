#include <lfd_common/state.h>
#include <lfd_common/conf_classification.h>
#include <ros/ros.h>
#include <iostream>
using namespace std;

bool classify_callback(lfd_common::conf_classification::Request &req, lfd_common::conf_classification::Response &resp)
{
  resp.c = 0.123;
  resp.l = 10;
  resp.db = -58;
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "cba_test_mock_classify");
  // a handle for this ROS node
  ros::NodeHandle node;

  // published topics
  ros::ServiceServer classify = node.advertiseService("classify", classify_callback);

  ros::spin();

  return EXIT_SUCCESS;
}
