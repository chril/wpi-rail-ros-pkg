/*
 * trick_or_treater.h
 *
 *  Created on: Oct 6, 2011
 *      Author: rctoris
 */

#ifndef TRICK_OR_TREATER_H_
#define TRICK_OR_TREATER_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <NaoCPP/naocpp_motion.h>
#include <NaoCPP/naocpp_text_to_speech.h>

#define HOST "/nao_trick_or_treat/host"
#define PORT "/nao_trick_or_treat/port"
#define NAOQI "/nao_trick_or_treat/naoqi"

class trick_or_treater
{
public:
  trick_or_treater();
  virtual ~trick_or_treater();

private:
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  // a handle for the node
  ros::NodeHandle node;
  // subscription to the joystick
  ros::Subscriber joy_sub;

  naocpp_motion *motion;
  vector<string> head;
  naocpp_text_to_speech *tts;
};

#endif
