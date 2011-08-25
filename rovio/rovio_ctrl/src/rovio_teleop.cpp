/*
 * rovio_ctrl.cpp
 *
 *  Created on: Aug 2, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <rovio_ctrl/head_ctrl.h>
#include <rovio_shared/man_drv.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <termios.h>

using namespace std;

// keycodes for the controller
#define KEY_W 0x77
#define KEY_A 0x61
#define KEY_S 0x73
#define KEY_D 0x64
#define KEY_Q 0x71
#define KEY_E 0x65
#define KEY_1 0x31
#define KEY_2 0x32
#define KEY_3 0x33
#define KEY_4 0x34
#define KEY_5 0x35
#define KEY_6 0x36
#define KEY_7 0x37
#define KEY_8 0x38
#define KEY_9 0x39
#define KEY_0 0x30
#define KEY_LT 0x2C
#define KEY_GT 0x2E
#define KEY_FSLASH 0x2F

// terminal modes for reading input
struct termios buffered, raw;

void set_terminal_buffered()
{
  tcsetattr(0, TCSANOW, &buffered);
}

class teleop_controller
{
public:
  teleop_controller();
  void run();

private:
  // a handle for the node
  ros::NodeHandle node;

  // published topics and clients
  ros::Publisher man_drive;
  ros::ServiceClient head_ctrl;

  int speed;
};

teleop_controller::teleop_controller()
{
  // create the published topic and client
  man_drive = node.advertise<rovio_shared::man_drv> ("man_drv", 8);
  head_ctrl = node.serviceClient<rovio_ctrl::head_ctrl> ("head_ctrl");

  // start off with a speed of 5
  speed = 5;

  // put the terminal in raw mode
  tcgetattr(0, &buffered);
  memcpy(&raw, &buffered, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(0, TCSANOW, &raw);

  ROS_INFO("Rovio Keyboard Teleop Started");
}

void teleop_controller::run()
{
  // keystroke to be read
  char c;

  // continue until we have an input error
  while (read(0, &c, 1))
  {
    // create the message for a speed message and request for the head
    rovio_shared::man_drv msg;
    rovio_ctrl::head_ctrl srv;

    // parse the message
    switch (c)
    {
      case KEY_W:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::FORWARD;
        break;
      case KEY_A:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::STRAIGHT_LEFT;
        break;
      case KEY_S:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::BACKWARD;
        break;
      case KEY_D:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::STRAIGHT_RIGHT;
        break;
      case KEY_Q:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::ROTATE_LEFT;
        break;
      case KEY_E:
        srv.request.head_pos = -1;
        msg.drive = rovio_shared::man_drv::ROTATE_RIGHT;
        break;
      case KEY_1:
        srv.request.head_pos = -1;
        msg.drive = -1;
        msg.speed = 1;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_2:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 2;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_3:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 3;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_4:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 4;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_5:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 5;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_6:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 6;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_7:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 7;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_8:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 8;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_9:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 9;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_0:
        srv.request.head_pos = -1;
        msg.drive = -1;
        speed = 10;
        ROS_INFO("Speed set to %i", speed);
        break;
      case KEY_LT:
        msg.drive = -1;
        srv.request.head_pos = rovio_ctrl::head_ctrl::Request::HEAD_DOWN;
        break;
      case KEY_GT:
        msg.drive = -1;
        srv.request.head_pos = rovio_ctrl::head_ctrl::Request::HEAD_MIDDLE;
        break;
      case KEY_FSLASH:
        msg.drive = -1;
        srv.request.head_pos = rovio_ctrl::head_ctrl::Request::HEAD_UP;
        break;
      default:
        // not a valid key
        msg.drive = -1;
        srv.request.head_pos = -1;
        break;
    }
    //set the speed
    msg.speed = speed;

    // check if a valid key was pressed
    if (msg.drive != -1)
      // publish the message
      man_drive.publish(msg);
    else if (srv.request.head_pos != -1)
      // send the request
      head_ctrl.call(srv);
  }
}

void end(int signal)
{
  // reset the terminal mode back
  tcsetattr(0, TCSANOW, &buffered);
  // shutdown ROS and exit once finished
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_teleop");

  // initialize the Rovio controller
  teleop_controller controller;

  // handle a ctrl-c
  signal(SIGINT, end);

  // run the controller
  controller.run();
}
