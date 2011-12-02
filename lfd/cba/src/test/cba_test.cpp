#include <lfd_common/state.h>
#include <lfd_common/conf_classification.h>
#include <ros/ros.h>
#include <iostream>
using namespace std;

bool classify_callback(lfd_common::conf_classification::Request &req, lfd_common::conf_classification::Response &resp)
{
  cout << "HERE!!!" << endl;
  resp.c = 0.123;
  resp.l = 10;
  resp.db = -58;
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "cba_test");
  // a handle for this ROS node
  ros::NodeHandle node;

  // published topics
  ros::Publisher update_state = node.advertise<lfd_common::state> ("update_state", 10);


  // give the CBA node some time to start
  sleep(1);

  // publish a few states
  for (int i = 0; i < 10; i++)
  {
    lfd_common::state s;
    for (int j = 0; j < 5; j++)
      s .state_vector.push_back(i);
    update_state.publish(s);
    sleep(1);
  }

  return EXIT_SUCCESS;
}
