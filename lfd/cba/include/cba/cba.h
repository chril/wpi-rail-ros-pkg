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
/*!
 * \def DIST_THRESH_MULT
 * The distance threshold multiplier
 */
#define DIST_THRESH_MULT "~dist_thresh_mult"
/*!
 * \def DEFAULT_DIST_MULT
 * The default distance threshold multiplier
 */
#define DEFAULT_DIST_MULT 1.5
/*!
 * \def ANN_EPSILON
 * The error bound on ANN
 */
#define ANN_EPSILON 0

/*!
 * \struct prediction
 * A prediction contains the classification label, confidence, and decision boundary label for a given classification.
 */
typedef struct
{
  int l;
  double c;
  int db;
} prediction;

/*!
 * \struct conf
 * A confidence threshold value for a given action label and decision boundary pair as well as a counter indicating the number of miss-classified points used in the computation of this threshold.
 */
typedef struct
{
  int l;
  int db;
  int cnt;
  double thresh;
} conf;

class cba_learner
{
public:
  cba_learner();
  virtual ~cba_learner();
  void step();

private:
  prediction *classify_state();
  double nearest_neighbor();
  double conf_thresh(int l, int db);
  void update_thresholds();
  void state_listener_callback(const lfd_common::state::ConstPtr &msg);
  void a_complete_callback(const std_msgs::Bool::ConstPtr &msg);

  ros::NodeHandle node; /*!< a handle for this ROS node */

  ros::Publisher execute, add_point; /*!< the execute and add_point topics */
  ros::Subscriber state_listener, a_complete; /*!< the state_listener and a_complete topics */
  ros::ServiceClient classify, demonstration; /*!< the classify and demonstration services */

  float *s, *sc; /*!< the current state of CBA and the last confidence state */
  int s_size, max_pts, pts, a; /*!< the length of the state vector, maximum number of data points to allocate, current number of data points, and current action for the agent to execute */
  bool action_complete, autonomous_action; /*!< if the action has been reported by the agent as finished and if the current action was executed autonomously */
  double dist_thresh, dist_mult; /*!< the distance threshold value and distance threshold multiplier */
  std::vector<conf*> conf_thresholds; /*!< confidence threshold values for action label and decision boundary pairs */

  ANNpointArray ann_data; /*!< data points for ANN */
  int *labels; /*!< labels for each entry in the data set */
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
