/*!
 * \file nao_terminal_talk.cpp
 * \brief Control of the text-to-speech module over the terminal
 *
 * nao_terminal_talk allows you to type text into the terminal and send it to the Nao's text-to-speech module. Volume control is also enabled in this node with use of the volume() command.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#include <iostream>
#include <nao_tools/nao_terminal_talk.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <string>

using namespace std;

nao_terminal_talk::nao_terminal_talk()
{
  // create the published topics
  speech = node.advertise<std_msgs::String> ("nao_say", 1);
  volume = node.advertise<std_msgs::Float32> ("nao_set_volume", 1);
  // set with the 'exit()' command
  exit_flag = false;

  ROS_INFO("Nao Terminal Talk Started");

  // print the exit information to the terminal
  cout << "Enter text for the Nao to speak. "
      << "To exit this node, use the 'exit()' command." << endl;
}

void nao_terminal_talk::process_command()
{
  // print out the start of the line
  cout << NAO_TERMINAL_PREFIX;

  // wait for the command
  string cmd;
  float v;
  getline(cin, cmd);

  // check if it is the exit command
  if (cmd.compare(EXIT_COMMAND) == 0)
    exit_flag = true;
  else if (sscanf(cmd.c_str(), VOLUME_COMMAND, &v) == 1)
  {
    // send the volume command
    std_msgs::Float32 vol;
    vol.data = v;

    volume.publish(vol);
  }
  else
  {
    // send the text to the nao_ctrl node
    std_msgs::String msg;
    msg.data = cmd;

    speech.publish(msg);
  }
}

bool nao_terminal_talk::exit()
{
  // just return the flag
  return exit_flag;
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
   * 'exit()' command can be used to quit.
   */
  while (ros::ok() && !talk.exit())
  {
    talk.process_command();
    // for good practice
    ros::spinOnce();
  }
}
