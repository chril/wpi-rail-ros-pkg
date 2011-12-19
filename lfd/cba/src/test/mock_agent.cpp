/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <lfd_common/demonstration.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <iostream>

using namespace std;

bool dem; /*!< if a demonstration should be given */
ros::ServiceClient a_complete; /*!< the a_complete service */

void execute_callback(const std_msgs::Int32::ConstPtr &msg)
{
  // lets execute the command
  cout << "Execution command '" << msg->data << "' received... ";

  // sleep some time and then report the action finished
  usleep(msg->data * 10000);

  // report the action now done
  cout << "execution finished!" << endl;
  std_srvs::Empty srv;
  a_complete.call(srv);
}

bool demonstration_callback(lfd_common::demonstration::Request &req, lfd_common::demonstration::Response &resp)
{
  // only send a demonstration every other request
  if (dem)
  {
    // based on what the mock classifier is going to do, give a demonstration
    if (req.s.state_vector.at(2) < 5)
      resp.a = 5;
    else if (req.s.state_vector.at(2) < 10)
      resp.a = 10;
    else if (req.s.state_vector.at(2) < 15)
      resp.a = 15;
    else if (req.s.state_vector.at(2) < 20)
      resp.a = 20;
    else
      resp.a = 25;

    cout << "Request for demonstration received -- giving demonstration '" << resp.a << "'." << endl;
  }

  // set the valid flag and flip the demonstration boolean
  resp.valid = dem;
  dem = !dem;

  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "mock_agent");
  // a handle for this ROS node
  ros::NodeHandle node;

  // create services and topics
  ros::Subscriber execute = node.subscribe<std_msgs::Int32> ("execute", 1, execute_callback);
  a_complete = node.serviceClient<std_srvs::Empty> ("a_complete");
  ros::ServiceServer dem = node.advertiseService("demonstration", demonstration_callback);

  ROS_INFO("Mock Agent Initialized");

  ros::spin();

  return EXIT_SUCCESS;
}
