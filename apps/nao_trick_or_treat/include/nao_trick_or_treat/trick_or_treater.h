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
#include <NaoCPP/naocpp_video.h>
#include <opencv2/opencv.hpp>

#define HOST "/nao_trick_or_treat/host"
#define PORT "/nao_trick_or_treat/port"
#define NAOQI "/nao_trick_or_treat/naoqi"

#define VIDEO_SUB "trick_or_treat_interface"
#define WINDOW_NAME "Nao Trick-Or-Treater"

class trick_or_treater
{
public:
  trick_or_treater();
  virtual ~trick_or_treater();
  void update_gui();

private:
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  // a handle for the node
  ros::NodeHandle node;
  // subscription to the joystick
  ros::Subscriber joy_sub;

  naocpp_motion *motion;
  vector<string> head;
  naocpp_text_to_speech *tts;

  naocpp_video *video;
  cv::Mat *img;
  int layers, frame_padding_x, frame_padding_y;
};

#endif
