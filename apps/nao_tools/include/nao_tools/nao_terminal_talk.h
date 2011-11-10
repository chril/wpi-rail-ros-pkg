/*
 * nao_terminal_talk.h
 *
 *  Created on: Nov 10, 2011
 *      Author: rctoris
 */

#ifndef NAO_TERMINAL_TALK_H_
#define NAO_TERMINAL_TALK_H_

#include <ros/ros.h>

// printed at the start of each new line
#define NAO_TERMINAL_PREFIX "> "
// the command used to exit or set the volume
#define EXIT_COMMAND "exit()"
#define VOLUME_COMMAND "volume(%f)"

class nao_terminal_talk
{
public:
  nao_terminal_talk();
  void process_command();
  bool exit();

private:
  // a handle for the node
  ros::NodeHandle node;

  // published topics
  ros::Publisher speech, volume;

  // used to check if the user used the 'exit' command
  bool exit_flag;
};

#endif
