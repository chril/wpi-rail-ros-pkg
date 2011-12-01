/*
 * cba.h
 *
 *  Created on: Nov 22, 2011
 *      Author: rctoris
 */

#ifndef CBA_H_
#define CBA_H_

#include <ANN/ANN.h>
#include <ros/ros.h>
#include <lfd_common/state.h>
#include <std_msgs/Bool.h>
#include <vector>

/*!
 * \def MAX_DATA_POINTS
 * The max data points ROS parameter name
 */
#define MAX_DATA_POINTS "~max_data_points"
/*!
 * \def DEFAULT_MAX_POINTS
 * The default number of max data points to allocate
 */
#define DEFAULT_MAX_POINTS 1024

class cba_learner
{
public:
  cba_learner();
  virtual ~cba_learner();
  void step();

private:
  void update_state_callback(const lfd_common::state::ConstPtr &msg);
  void a_complete_callback(const std_msgs::Bool::ConstPtr &msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Subscriber update_state, a_complete; /*!< the update_state and a_complete topics */

  float *s; /*!< the current state of CBA */
  int s_size, max_pts; /*!< the size of the state vector and maximum number of data points to allocate */
  bool action_complete, autonomous_action; /*!< if the action has been reported by the agent as finished and if the current action was executed autonomously */
  double dist_thresh; /*!< the distance threshold value */

  ANNpointArray data; /*!< data points */
  ANNkd_tree *kd_tree; /*!< search structure for ANN */
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
