/*
 * cba.h
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#ifndef CBA_H_
#define CBA_H_

#include <ros/ros.h>
#include <lfd_common/state.h>
#include <vector>

class cba_learner
{
public:
  cba_learner();
  virtual ~cba_learner();
  void step();

private:
  void update_state_callback(const lfd_common::state::ConstPtr &msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber update_state; /*!< the update_state topic */

  float *current_state; /*!< the current state of CBA */
  int s_size; /*!< the size of the state vector */
  bool action_complete; /*!< if the action has been reported by the agent as finished */
};

/*!
 * Creates and runs the cba node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv);

#endif
