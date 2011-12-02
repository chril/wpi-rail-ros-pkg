/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <lfd_common/state.h>
#include <lfd_common/conf_classification.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_state_publisher");
  // a handle for this ROS node
  ros::NodeHandle node;

  // published topics
  ros::Publisher update_state = node.advertise<lfd_common::state> ("update_state", 1);

  ROS_INFO("Mock State Publisher Initialized");

  // publish states continuously
  while (ros::ok())
  {
    lfd_common::state s;
    for (int j = 0; j < 5; j++)
      s .state_vector.push_back(j * j);
    update_state.publish(s);
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}
