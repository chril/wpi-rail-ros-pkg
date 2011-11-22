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

#ifndef ROVIO_TELEOP_H_
#define ROVIO_TELEOP_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

/*!
 * \class teleop_controller
 * \brief Provides a bridge between the joy topic and the Rovio nodes.
 *
 * The teleop_controller handles the translation between joystick commands and communication to the Rovio nodes.
 */
class teleop_controller
{
public:
  /*!
   * \brief Creates a teleop_controller.
   *
   * Creates a teleop_controller object that can be used control the Rovio with a joystick. ROS nodes, services, and publishers are created and maintained within this object.
   */
  teleop_controller();

private:
  /*!
   * \brief joy topic callback function.
   *
   * Process the joystick command and send messages/service requests to the appropriate Rovio nodes.
   *
   * \param joy the message for the joy topic
   */
  void joy_cback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher cmd_vel; /*!< the cmd_vel topic */
  ros::ServiceClient head_ctrl; /*!< the head_ctrl service */
  ros::Subscriber joy_sub; /*!< the joy topic */
};

/*!
 * Creates and runs the rovio_teleop node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
