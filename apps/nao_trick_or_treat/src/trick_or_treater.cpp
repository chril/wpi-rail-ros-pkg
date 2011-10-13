/*
 * trick_or_treater.cpp
 *
 *  Created on: Oct 6, 2011
 *      Author: rctoris
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <math.h>
#include <NaoCPP/naocpp_motion.h>
#include <NaoCPP/naocpp_text_to_speech.h>
#include <NaoCPP/naocpp_video.h>
#include <nao_trick_or_treat/trick_or_treater.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

using namespace std;

trick_or_treater::trick_or_treater()
{
  // grab parameters
  string host, naoqi;
  int port;

  // check for all the correct parameters
  if (!node.getParam(HOST, host))
  {
    ROS_ERROR("Parameter %s not found.", HOST);
    exit(-1);
  }
  if (!node.getParam(PORT, port))
  {
    ROS_ERROR("Parameter %s not found.", PORT);
    exit(-1);
  }
  if (!node.getParam(NAOQI, naoqi))
  {
    ROS_ERROR("Parameter %s not found.", NAOQI);
    exit(-1);
  }

  //connect to the Nao
  motion = new naocpp_motion(host, port, naoqi);
  tts = new naocpp_text_to_speech(host, port, naoqi);
  video = new naocpp_video(host, port, naoqi);

  // turn on the motors we need
  motion->set_stiffnesses("Body", 0.75);
  tts->set_volume(1);

  // get up in the correct position
  motion->sit();
  motion->stand();

  //create the vector for head position names
  head.push_back("HeadYaw");
  head.push_back("HeadPitch");

  // start at 0, 0
  vector<float> head_angles;
  head_angles.push_back(0);
  head_angles.push_back(0);
  motion->set_angles(head, head_angles, 0.65);

  //subscribe to the joystick
  joy_sub = node.subscribe<sensor_msgs::Joy> ("joy", 10, &trick_or_treater::joy_cback, this);

  // setup the video stream
  video->subscribe(VIDEO_SUB, kQVGA, kRGB, 30);
  layers = 3;
  // use the top camera
  video->set_param(kCameraSelectID, 0);

  // create an openCV window
  cv::namedWindow(WINDOW_NAME, CV_WINDOW_AUTOSIZE);

  int image_width = 320;
  int image_height = 240;
  // pad the image a bit
  frame_padding_x = 10;
  frame_padding_y = 50;

  //setup the base image
  img = new cv::Mat(frame_padding_y + 10 + image_height, frame_padding_x + image_width + 125, CV_MAKETYPE(8, layers));
  cv::rectangle(*img, cv::Point(0, 0), cv::Point(img->cols, img->rows), cv::Scalar(127, 31, 31), -1, 8, 0);
  cv::putText(*img, "Nao's Trick-or-Treat Interface", cv::Point(10, 30), cv::FONT_HERSHEY_SCRIPT_COMPLEX, 1.0,
              cv::Scalar(0, 127, 255));

  ROS_INFO("Trick-or-Treater Initialized");
}

trick_or_treater::~trick_or_treater()
{
  // sit back down
  motion->sit();
  // reset all of the stiffnesses
  motion->set_stiffnesses("Body", 0);
  delete motion;
  delete tts;

  // close the video connection
  video->unsubscribe(VIDEO_SUB);
  delete video;
  delete img;
}

void trick_or_treater::joy_cback(const sensor_msgs::Joy::ConstPtr& joy)
{
  vector<float> head_yaw_pitch;

  // check if this is an "aim camera" action
  if (joy->buttons.at(5) == 1)
  {
    // get the head positions
    head_yaw_pitch.push_back(joy->axes.at(2));
    head_yaw_pitch.push_back(joy->axes.at(3));
  }
  else
  {
    // set the walk speed
    float t = joy->axes.at(2) / (M_PI / 2.0);
    motion->set_walk_velocity(joy->axes.at(1), joy->axes.at(0), t, (abs(joy->axes.at(0)) + abs(joy->axes.at(1))) / 2.0);

    // get the head positions from the D-pad
    head_yaw_pitch.push_back(joy->axes.at(4));
    head_yaw_pitch.push_back(joy->axes.at(5));

    // check for any TTS buttons
    if (joy->buttons.at(0) == 1)
      tts->say("Trick-or-Treat!");
    else if (joy->buttons.at(1) == 1)
      tts->say("Happy Halloween!");
    else if (joy->buttons.at(2) == 1)
      tts->say("Hey everybody, check out how adorable I am!");
  }

  //set the head position
  motion->set_angles(head, head_yaw_pitch, 0.4);
}

void trick_or_treater::update_gui()
{
  // get the top camera image
  cv::Mat *cur_top = video->get_image(VIDEO_SUB);

  // copy it onto to main image
  for (int i = 0; i < cur_top->rows; i++)
  {
    int cur_top_offset = i * (cur_top->cols * layers);
    int img_offset = (frame_padding_y + i) * (img->cols * layers);
    for (int j = 0; j < cur_top->cols; j++)
    {
      int cur_top_index = cur_top_offset + (j * layers);
      int img_index = img_offset + ((frame_padding_x + j) * layers);
      // set RGB values
      ((uchar *)(img->data))[img_index] = ((uchar *)(cur_top->data))[cur_top_index];
      ((uchar *)(img->data))[img_index + 1] = ((uchar *)(cur_top->data))[cur_top_index + 1];
      ((uchar *)(img->data))[img_index + 2] = ((uchar *)(cur_top->data))[cur_top_index + 2];
    }
  }

  // update the image
  cv::waitKey(1);
  cv::imshow(WINDOW_NAME, *img);

  // remove the cached image
  delete cur_top;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "trick_or_treater");

  // initialize the controller
  trick_or_treater tot;

  // update at 30 Hz
  ros::Rate loop_rate(30);
  // continue until a ctrl-c has occurred
  while (ros::ok())
  {
    // update the frames
    tot.update_gui();

    ros::spinOnce();
    loop_rate.sleep();
  }
}
