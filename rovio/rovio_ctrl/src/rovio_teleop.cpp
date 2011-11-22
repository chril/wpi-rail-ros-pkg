/*!
 * \file rovio_teleop.h
 * \brief Allows for control of the Rovio with a joystick.
 *
 * rovio_teleop creates a ROS node that allows the control of a Rovio with a joystick.
 * This node listens to a joy topic and sends messages to the cmd_vel topic in the rovio_move node and head_ctrl service in the rovio_head node.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <rovio_shared/head_ctrl.h>
#include <rovio_ctrl/rovio_teleop.h>
#include <rovio_shared/man_drv.h>
#include <sensor_msgs/Joy.h>
#include <string>

using namespace std;

teleop_controller::teleop_controller()
{
  // create the published topic and client
  cmd_vel = node.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
  head_ctrl = node.serviceClient<rovio_shared::head_ctrl> ("head_ctrl");

  //subscribe to the joystick
  joy_sub = node.subscribe<sensor_msgs::Joy> ("joy", 10, &teleop_controller::joy_cback, this);
  ROS_INFO("Rovio Teleop Started");
}

void teleop_controller::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  // create the message for a speed message and request for the head
  rovio_shared::man_drv drv;
  rovio_shared::head_ctrl head;

  // check for any head control buttons
  if (joy->buttons.at(0) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_DOWN;
  else if (joy->buttons.at(1) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_MIDDLE;
  else if (joy->buttons.at(2) == 1)
    head.request.head_pos = rovio_shared::head_ctrl::Request::HEAD_UP;
  else
    head.request.head_pos = -1;

  // check if a head request was made
  if (head.request.head_pos != -1)
    // send the request
    head_ctrl.call(head);

  // create the twist message
  geometry_msgs::Twist twist;
  // left joystick controls the linear movement
  twist.linear.x = joy->axes.at(1);
  twist.linear.y = -joy->axes.at(0);
  twist.linear.z = 0;
  // right joystick controls the angular movement
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = -joy->axes.at(2);
  // send the twist command
  cmd_vel.publish(twist);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_teleop");

  // initialize the Rovio controller
  teleop_controller controller;

  // continue until a ctrl-c has occurred
  ros::spin();
}
