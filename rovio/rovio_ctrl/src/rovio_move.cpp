/*
 * rovio_ctrl.cpp
 *
 *  Created on: Aug 2, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/rovio_http.h>
#include <sstream>
#include <string>

using namespace std;

class move_controller
{
public:
  move_controller();
  ~move_controller();

private:
  // subscription callbacks
  void man_drv_callback(const rovio_shared::man_drv::ConstPtr &msg);

  // host location of the Rovio
  string host;
  // communicates with the Rovio
  rovio_http *rovio;
  // a handle for the node
  ros::NodeHandle node;

  // subscriptions
  ros::Subscriber man_drv;
};

move_controller::move_controller()
{
  string user;
  string pass;

  // check for all the correct parameters
  if (!node.getParam(USER, user))
  {
    ROS_ERROR("Parameter %s not found.", USER);
    exit(-1);
  }
  if (!node.getParam(PASS, pass))
  {
    ROS_ERROR("Parameter %s not found.", PASS);
    exit(-1);
  }
  if (!node.getParam(HOST, host))
  {
    ROS_ERROR("Parameter %s not found.", HOST);
    exit(-1);
  }

  // create the communication object to talk to Rovio
  rovio = new rovio_http(user, pass);

  // add subscriptions that will control the Rovio
  man_drv = node.subscribe<rovio_shared::man_drv> ("man_drv", 8, &move_controller::man_drv_callback, this);

  ROS_INFO("Rovio Move Controller Initialized");
}

move_controller::~move_controller()
{
  // free up the rovio_http object
  delete rovio;
}

void move_controller::man_drv_callback(const rovio_shared::man_drv::ConstPtr &msg)
{
  // check to see if all the requests are valid
  if (msg->drive < rovio_shared::man_drv::MIN_DRIVE_VAL || msg->drive > rovio_shared::man_drv::MAX_DRIVE_VAL)
  {
    ROS_ERROR("Manual Drive 'drive' value of %i out of range [%i,%i].", msg->drive, rovio_shared::man_drv::MIN_DRIVE_VAL, rovio_shared::man_drv::MAX_DRIVE_VAL);
    return;
  }
  if (msg->speed < rovio_shared::man_drv::MIN_SPEED_VAL || msg->speed > rovio_shared::man_drv::MAX_SPEED_VAL)
  {
    ROS_ERROR("Manual Drive 'speed' value of %i out of range [%i,%i].", msg->speed, rovio_shared::man_drv::MIN_SPEED_VAL, rovio_shared::man_drv::MAX_SPEED_VAL);
    return;
  }

  // build the URL command and send it
  stringstream ss;
  ss << "http://" << host.c_str() << "/rev.cgi?Cmd=nav&action=18&drive=" << (int)msg->drive << "&speed="
      << (int)msg->speed;

  rovio_response *buf = rovio->send(ss.str().c_str());
  rovio_response_clean(buf);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_move");

  // initialize the Rovio controller
  move_controller controller;

  // update at 5 Hz
  ros::Rate loop_rate(5);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
