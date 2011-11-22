/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <cba/cba.h>
#include <lfd_common/state.h>
#include <ros/ros.h>
#include <vector>

#include <iostream>

using namespace std;

cba_learner::cba_learner()
{
  // add subscriptions
  update_state = node.subscribe<lfd_common::state> ("update_state", 1, &cba_learner::update_state_callback, this);

  // initial CBA values
  s = NULL;
  s_size = -1;
  action_complete = true;

  ROS_INFO("CBA Learner Initialized");
}

cba_learner::~cba_learner()
{
  // clear the state vector if it exists
  if (s != NULL)
    free( s);
}

void cba_learner::step()
{
  // check if the state exists
  if (s == NULL)
    return;

  // check if the agent has reported their action finished
  if (action_complete)
  {

  }
  else
  {

  }
}

void cba_learner::update_state_callback(const lfd_common::state::ConstPtr &msg)
{
  // set the state size if it is not yet set
  if (s_size == -1)
    s_size = msg->state_vector.size();

  // check if the state sizes match
  if (s_size != (int)msg->state_vector.size())
    ROS_WARN("WARNING: State sizes do not match -- Ignoring current state.");

  // check if the state size is allocated yet
  if (s == NULL)
    s = (float *)malloc(sizeof(float) * s_size);

  // copy the state vector
  for (int i = 0; i < s_size; i++)
    s[i] = msg->state_vector[i];
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
