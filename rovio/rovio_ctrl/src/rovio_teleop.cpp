/*
 * rovio_ctrl.cpp
 *
 *  Created on: Aug 2, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <rovio_shared/man_drv.h>
#include <rovio_shared/rovio_http.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <termios.h>

using namespace std;

// keycodes for the controller
#define W 0x77
#define A 0x61
#define S 0x73
#define D 0x64

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
  ~teleop_controller();
  void run();

private:
  // host location of the Rovio
  string host;
  // communicates with the Rovio
  rovio_http *rovio;
  // a handle for the node
  ros::NodeHandle node;

  // published topics
  ros::Publisher man_drive;
};

teleop_controller::teleop_controller()
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

  // create the published topic
  man_drive = node.advertise<rovio_shared::man_drv> ("man_drv", 8);

  // put the terminal in raw mode
  tcgetattr(0, &buffered);
  memcpy(&raw, &buffered, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(0, TCSANOW, &raw);

  ROS_INFO("Rovio Teleop Controller Initialized");
}

teleop_controller::~teleop_controller()
{
  // free up the rovio_http object
  delete rovio;
}

void teleop_controller::run()
{
  // keystroke to be read
  char c;

  // continue until we have an input error
  while (read(0, &c, 1))
  {
    // create the message
    rovio_shared::man_drv msg;

    //TODO: let user control speed
    msg.speed = 5;

    // parse the message
    switch (c)
    {
      case W:
        msg.drive = 1;
        break;
      case A:
        msg.drive = 3;
        break;
      case S:
        msg.drive = 2;
        break;
      case D:
        msg.drive = 4;
        break;
      default:
        //TODO: other commands
        // not a valid key
        msg.drive = -1;
        break;
    }

    // check if a valid key was pressed
    if (msg.drive != -1)
      // publish the message
      man_drive.publish(msg);
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
