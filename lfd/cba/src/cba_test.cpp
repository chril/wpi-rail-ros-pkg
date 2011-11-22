#include <lfd_common/state.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "cba");
  // a handle for this ROS node
  ros::NodeHandle node;

  // published topics
  ros::Publisher update_state = node.advertise<lfd_common::state> ("update_state", 10);

  // give the CBA node some time to start
  sleep(1);

  // publish a few states
  for (int i = 0; i < 5; i++)
  {
    lfd_common::state s;
    for (int j = 0; j < 10; j++)
      s .state_vector.push_back(j * i);
    update_state.publish(s);
  }

  sleep(1);

  return EXIT_SUCCESS;
}
