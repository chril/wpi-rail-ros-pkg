/*
 * rovio_audio.cpp
 *
 *  Created on: Aug 21, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>

class audio_controller
{
public:
  audio_controller();
  ~audio_controller();
};

audio_controller::audio_controller()
{
  // TODO Auto-generated constructor stub

}

audio_controller::~audio_controller()
{
  // TODO Auto-generated destructor stub
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_audio");

  // initialize the Rovio controller
  audio_controller controller;

  // update at 5 Hz
  ros::Rate loop_rate(5);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
