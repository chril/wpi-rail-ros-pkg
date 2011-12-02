/*
 * cba.cpp
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#include <ANN/ANN.h>
#include <cba/cba.h>
#include <lfd_common/conf_classification.h>
#include <lfd_common/state.h>
#include <limits>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <vector>

#include <iostream>

using namespace std;

cba_learner::cba_learner()
{
  // add subscriptions
  state_listener = node.subscribe<lfd_common::state> ("update_state", 1, &cba_learner::state_listener_callback, this);
  a_complete = node.subscribe<std_msgs::Bool> ("a_complete", 1, &cba_learner::a_complete_callback, this);

  // check for the maximum number of data points to allocate
  ros::param::param<int>(MAX_DATA_POINTS, max_pts, DEFAULT_MAX_POINTS);

  // create the classification topics and services
  classify = node.serviceClient<lfd_common::conf_classification> ("classify");

  // initial CBA values
  kd_tree = NULL;
  s = NULL;
  s_size = -1;
  pts = 0;
  action_complete = true;
  autonomous_action = false;
  dist_thresh = 0;

  ROS_INFO("CBA Learner Initialized");
}

cba_learner::~cba_learner()
{
  // clear the state vector if it exists
  if (s != NULL)
    free(s);
  // cleanup ANN
  if (kd_tree != NULL)
    delete kd_tree;
  annClose();
}

void cba_learner::step()
{
  // check if the state exists
  if (s == NULL)
    return;

  // check if the agent has reported their action finished
  if (action_complete)
  {
    // request a prediction from the classifier
    prediction *p = classify_state(s, s_size);
  }
  else if (autonomous_action)
  {

  }
}

prediction *cba_learner::classify_state(float *s, int size)
{
  // allocate the prediction
  prediction *p = (prediction *)malloc(sizeof(prediction));

  // check for the classifier
  if (classify.exists())
  {
    // create the service request
    lfd_common::conf_classification cc;
    for (int i = 0; i < size; i++)
      cc.request.s.state_vector.push_back(s[i]);
cout << "CALLING!!" << endl;

    // send the service request
    classify.call(cc);

    // fill in the struct
    p->c = cc.response.c;
    p->l = cc.response.l;
    p->db = cc.response.db;
  }
  else
  {
    ROS_WARN("Could not connect to classifier");
    // fill the prediction with negative infinity confidence
    p->c = -numeric_limits<float>::infinity();
    p->db = -1;
    p->l = -1;
  }

  cout << p->c << " " << p->l << " " << p->db <<endl;

  return p;
}

void cba_learner::state_listener_callback(const lfd_common::state::ConstPtr &msg)
{
  // set the state size if it is not yet set
  if (s_size == -1)
  {
    s_size = msg->state_vector.size();
    // allocate space for ANN
    data = annAllocPts(max_pts, s_size);
  }

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

void cba_learner::a_complete_callback(const std_msgs::Bool::ConstPtr &msg)
{
  // set the action complete value
  action_complete = msg->data;
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
    sleep(2);
  }

  return EXIT_SUCCESS;
}
