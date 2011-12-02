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
  annDeallocPts(data);
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
    prediction *p = classify_state();
    // calculate the nearest neighbor distance
    double d = nearest_neighbor();
    // check against the thresholds
    if (p->c > conf_thresh(p->l, p->db) && d < dist_thresh)
    {
      // report the action to be executed

    }
    else
    {
      // request a demonstration
    }

    // cleanup
    free(p);
  }
  else if (autonomous_action)
  {

  }
}

prediction *cba_learner::classify_state()
{
  // allocate the prediction
  prediction *p = (prediction *)malloc(sizeof(prediction));

  // check for the classifier
  if (classify.exists())
  {
    // create the service request
    lfd_common::conf_classification cc;
    for (int i = 0; i < s_size; i++)
      cc.request.s.state_vector.push_back(s[i]);

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

  return p;
}

double cba_learner::nearest_neighbor()
{
  // check if we have any data points yet
  if (pts == 0)
    return numeric_limits<float>::infinity();
  else
  {
    // calculate NN using ANN
    ANNidxArray index = new ANNidx[1];
    ANNdistArray dists = new ANNdist[1];

    // create the search structure
    ANNkd_tree *kd_tree = new ANNkd_tree(data, pts, s_size);

    // create the data point
    ANNpoint pt = annAllocPt(s_size);
    for (int i = 0; i < s_size; i++)
      pt[i] = s[i];

    // calculate nearest neighbor
    kd_tree->annkSearch(pt, 1, index, dists, ANN_EPSILON);
    // unsquare the distance
    double d = sqrt(dists[0]);

    // cleanup
    annDeallocPt(pt);
    delete kd_tree;

    return d;
  }
}

double cba_learner::conf_thresh(int l, int db)
{
  // check the thresholds for the given action label and decision boundary pair
  for (uint i = 0; i < conf_thresholds.size(); i++)
    if (conf_thresholds.at(i)->l == l && conf_thresholds.at(i)->db == db)
      return conf_thresholds.at(i)->thresh;

  // no threshold found for the given pair -- we return infinity
  return numeric_limits<float>::infinity();
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
