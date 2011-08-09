/*
 * rovio_ctrl.cpp
 *
 *  Created on: Aug 2, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <rovio_ctrl/head_ctrl.h>
#include <rovio_shared/rovio_http.h>
#include <sstream>
#include <std_msgs/String.h>
#include <stdio.h>

class head_controller
{
public:
  head_controller();
  ~head_controller();

  // published topics
  void pub_head_sensor();
private:
  // service callbacks
  bool head_ctrl_callback(rovio_ctrl::head_ctrl::Request &req, rovio_ctrl::head_ctrl::Response &resp);

  // host location of the Rovio
  std::string host;
  // communicates with the Rovio
  rovio_http *rovio;
  // a handle for the node
  ros::NodeHandle node;

  // services
  ros::ServiceServer head_ctrl;
  //published topics
  ros::Publisher head_sensor;
};

head_controller::head_controller()
{
  std::string user;
  std::string pass;

  // check for all the correct parameters
  if (!node.getParam("/rovio_ctrl/user", user))
  {
    ROS_ERROR("Parameter '/rovio_ctrl/user' not found.");
    exit(-1);
  }
  if (!node.getParam("/rovio_ctrl/pass", pass))
  {
    ROS_ERROR("Parameter '/rovio_ctrl/pass' not found.");
    exit(-1);
  }
  if (!node.getParam("/rovio_ctrl/host", host))
  {
    ROS_ERROR("Parameter '/rovio_ctrl/host' not found.");
    exit(-1);
  }

  rovio = new rovio_http(user, pass);

  // add services and published topics
  head_ctrl = node.advertiseService("head_ctrl", &head_controller::head_ctrl_callback, this);
  head_sensor = node.advertise<std_msgs::String> ("head_sensor", 1024);

  ROS_INFO("Rovio Head Controller Initialized");
}

head_controller::~head_controller()
{
  // free up the rovio_http object
  delete rovio;
}

bool head_controller::head_ctrl_callback(rovio_ctrl::head_ctrl::Request &req, rovio_ctrl::head_ctrl::Response &resp)
{

  // make sure the head position value is valid
  if (req.head_pos != rovio_ctrl::head_ctrl::Request::HEAD_UP && req.head_pos
      != rovio_ctrl::head_ctrl::Request::HEAD_DOWN && req.head_pos != rovio_ctrl::head_ctrl::Request::HEAD_MIDDLE)
  {
    ROS_ERROR("Head position 'head_pos' value of %i is not valid.", req.head_pos);
    return false;
  }

  // build the URL command and send it
  char url[URL_BUF_SIZE];
  snprintf(url, URL_BUF_SIZE, "http://%s/rev.cgi?Cmd=nav&action=18&drive=%i", host.c_str(), req.head_pos);
  std::string buf = "";
  rovio->send(url, &buf);

  // parse out the response
  int resp_code = -1;
  sscanf(strstr(buf.c_str(), "responses = "), "responses = %i", &resp_code);
  resp.response = resp_code;

  return true;
}

void head_controller::pub_head_sensor()
{
  // build the URL command and send it
  char url[URL_BUF_SIZE];
  snprintf(url, URL_BUF_SIZE, "http://%s/rev.cgi?Cmd=nav&action=1", host.c_str());
  std::string buf = "";
  rovio->send(url, &buf);

  // parse out the response
  int resp_code = -1;
  sscanf(strstr(buf.c_str(), "head_position="), "head_position=%i", &resp_code);

  // decide which head position the Rovio is in
  std_msgs::String msg;
  std::stringstream ss;
  if (resp_code > 140)
  {
    ss << "HEAD_DOWN";
  }
  else if (resp_code < 135)
  {
    ss << "HEAD_UP";
  }
  else
  {
    ss << "HEAD_MIDDLE";
  }

  msg.data = ss.str();
  head_sensor.publish(msg);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "rovio_head");

  // initialize the Rovio controller
  head_controller controller;

  // update at 10 Hz
  ros::Rate loop_rate(10);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    ros::spinOnce();
    //publish the topics
    controller.pub_head_sensor();
    loop_rate.sleep();
  }
}
