/*
 * trick_or_treater.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <NaoCPP/naocpp_motion.h>
#include <nao_trick_or_treat/trick_or_treater.h>
#include <string>

using namespace std;

trick_or_treater::trick_or_treater()
{
  // grab parameters
  string host;
  int port;

  // check for all the correct parameters
  if (!node.getParam(HOST, host))
  {
    ROS_ERROR("Parameter %s not found.", HOST);
    exit(-1);
  }
  if (!node.getParam(PORT, port))
  {
    ROS_ERROR("Parameter %s not found.", PORT);
    exit(-1);
  }

  //connect to the Nao
  motion = new naocpp_motion(host, port);

  // turn on the motors we need
  motion->set_stiffnesses("Head", 0.8);

  //create the vector for head position names
  head.push_back("HeadYaw");
  head.push_back("HeadPitch");

  // start at 0, 0
  vector<float> head_angles;
  head_angles.push_back(0);
  head_angles.push_back(0);
  motion->set_angles(head, head_angles, 0.5);

  //subscribe to the joystick
  joy_sub = node.subscribe<sensor_msgs::Joy> ("joy", 10, &trick_or_treater::joy_cback, this);

  ROS_INFO("Trick-or-Treater Initialized");
}

trick_or_treater::~trick_or_treater()
{
  // reset all of the stiffnesses
  motion->set_stiffnesses("Body", 0);
  delete motion;
}

void trick_or_treater::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // get the right joystick (the head controller)
  vector<float> head_yaw_pitch;
  head_yaw_pitch.push_back(joy->axes.at(2));
  head_yaw_pitch.push_back(joy->axes.at(3));
  //set the head position
  motion->set_angles(head, head_yaw_pitch, 0.75);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "trick_or_treater");

  // initialize the controller
  trick_or_treater tot;

  // update at 5 Hz
  ros::Rate loop_rate(5);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
}
