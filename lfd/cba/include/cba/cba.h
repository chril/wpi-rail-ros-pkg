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

class cba_learner
{
public:
  cba_learner();
  virtual ~cba_learner();
  void step();

private:
  void query_cba_callback(const lfd_common::state::ConstPtr &msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber query_cba; /*!< the query_cba topic */
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
