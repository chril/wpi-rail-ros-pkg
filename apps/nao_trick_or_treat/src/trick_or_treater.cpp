/*
 * trick_or_treater.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <NaoCPP/naocpp_motion.h>
#include <nao_trick_or_treat/trick_or_treater.h>
#include <string>

using namespace std;

trick_or_treater::trick_or_treater()
{
  // grab parameters
  string host, naoqi;
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
  if (!node.getParam(NAOQI, naoqi))
  {
    ROS_ERROR("Parameter %s not found.", NAOQI);
    exit(-1);
  }

  //connect to the Nao
  motion = new naocpp_motion(host, port, naoqi);
  tts = new naocpp_text_to_speech(host, port, naoqi);

  // turn on the motors we need
  motion->set_stiffnesses("Body", 0.75);
  tts->set_volume(0.75);

  // get up in the correct position
  motion->sit();
  motion->stand();

  //create the vector for head position names
  head.push_back("HeadYaw");
  head.push_back("HeadPitch");

  // start at 0, 0
  vector<float> head_angles;
  head_angles.push_back(0);
  head_angles.push_back(0);
  motion->set_angles(head, head_angles, 0.65);

  //subscribe to the joystick
  joy_sub = node.subscribe<sensor_msgs::Joy> ("joy", 10, &trick_or_treater::joy_cback, this);

  ROS_INFO("Trick-or-Treater Initialized");
}

trick_or_treater::~trick_or_treater()
{
  // sit back down
  motion->sit();
  // reset all of the stiffnesses
  motion->set_stiffnesses("Body", 0);
  delete motion;
}

void trick_or_treater::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // set the walk speed
  float theta = joy->axes.at(2) / (M_PI / 2.0);
  motion->set_walk_velocity(joy->axes.at(1), joy->axes.at(0), theta, 1);

  vector<float> head_yaw_pitch;
  if (theta == 0)
  {
    // get the head positions for the walk
    head_yaw_pitch.push_back(joy->axes.at(2));
    head_yaw_pitch.push_back(joy->axes.at(3));
  }
  else
  {
    // get the head positions from the D-pad
    head_yaw_pitch.push_back(joy->axes.at(4));
    head_yaw_pitch.push_back(joy->axes.at(5));
  }
  //set the head position
  motion->set_angles(head, head_yaw_pitch, 0.4);

  // check for any TTS buttons
  if (joy->buttons.at(0) == 1)
    tts->say("Trick-or-Treat!");
  else if (joy->buttons.at(1) == 1)
    tts->say("Happy Halloween!");
  else if (joy->buttons.at(2) == 1)
      tts->say("Hey everybody, check out how adorable I am!");
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
