/*!
 * \file nao_terminal_talk.h
 * \brief Control of the text-to-speech module over the terminal
 *
 * nao_terminal_talk allows you to type text into the terminal and send it to the Nao's text-to-speech module. Volume control is also enabled in this node with use of the volume() command.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date November 22, 2011
 */

#ifndef NAO_TERMINAL_TALK_H_
#define NAO_TERMINAL_TALK_H_

#include <ros/ros.h>

/*!
 * \def NAO_TERMINAL_PREFIX
 * Printed at the start of each new line in the terminal
 */
#define NAO_TERMINAL_PREFIX "> "

// the command used to exit or set the volume

/*!
 * \def EXIT_COMMAND
 * The command used to exit
 */
#define EXIT_COMMAND "exit()"

/*!
 * \def VOLUME_COMMAND
 * The command used to set the volume
 */
#define VOLUME_COMMAND "volume(%f)"

/*!
 * \class nao_terminal_talk
 * \brief A terminal-based node that allows for control of the Nao's text-to-speech module.
 *
 * The nao_terminal_talk handles communication to the Nao's text-to-speech module. ROS nodes and topics are created and maintained within this object.
 */
class nao_terminal_talk
{
public:
  /*!
   * \brief Creates a nao_terminal_talk.
   *
   * Creates a nao_terminal_talk object that can be used control of the Nao's text-to-speech module.
   */
  nao_terminal_talk();

  /*!
   * \brief Process a line of input from standard-in.
   *
   * Based on current line of input from standard-in, the command will be sent to the Nao's text-to-speech module.
   * If an exit() command is seen, the node is shut down. If the volume() command is seen, the volume is changed. All other input is sent directly to the module.
   */
  void process_command();

  /*!
   * \brief Check the exit flag.
   *
   * Check's if an exit command has been seen yet.
   * \return if an exit command has been seen yet
   */
  bool exit();

private:
  ros::NodeHandle node; /*!< a handle for this ROS node */
  ros::Publisher speech, volume; /*!< the nao_say and nao_set_volume topics */

  bool exit_flag; /*!< used to check if the user used the 'exit' command */
};

#endif
