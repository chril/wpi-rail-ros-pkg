/*
 * nao_terminal_talk.cpp
 *
 *  Created on: Nov 10, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nao_tools/nao_terminal_talk.h>
#include <string>
#include <iostream>

using namespace std;

nao_terminal_talk::nao_terminal_talk()
{
  // create the published topic
  man_drive = node.advertise<std_msgs::String> ("speech", 1);
  // set with the 'exit' command
  exit = false;

  ROS_INFO("Rovio Keyboard Teleop Started");
}

void process_command()
{
  // print out the start of the line
  cout << NAO_TERMINAL_PREFIX;

  // wait for the command
  string cmd;
  getline(cin, cmd);

  // check if it is the exit command
  if (cmd.compare(EXIT_COMMAND) == 0)
    exit = true;
  else
  {
    cout << cmd << endl;
  }
}

bool nao_terminal_talk::exit()
{
  return exit;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "nao_terminal_talk");

  // initialize the controller
  nao_terminal_talk talk;

  /*
   * Continue until a ctrl-c has occurred.
   * Note that since process_command blocks, a ctrl-c will only be
   * processed after each command is handled. Alternatively, an
   * 'exit' command can be used to quit.
   */
  while (ros::ok() && !talk.exit())
  {
    // for good practice
    ros::spinOnce();
  }
}
