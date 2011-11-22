/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <cba/cba.h>
#include <lfd_common/state.h>
#include <ros/ros.h>

cba_learner::cba_learner()
{
  // add subscriptions
  man_drv = node.subscribe<lfd_common::state> ("query_cba", 1, &cba_learner::query_cba_callback, this);
}

cba_learner::~cba_learner()
{
  // TODO Auto-generated destructor stub
}

void cba_learner::step()
{

}

void cba_learner::query_cba_callback(const lfd_common::state::ConstPtr &msg)
{

}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "cba");

  // initialize the CBA learner
  cba_learner cba;

  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    // step through the algorithm once
    cba.step();
  }

  return EXIT_SUCCESS;
}
