/*
 * The rovio_audio creates a ROS node that listens for the name of a .wav file as a string. The file is then streamed and played on the Rovio's speaker.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date October 11, 2011
 */

#ifndef ROVIO_AUDIO_H_
#define ROVIO_AUDIO_H_

using namespace std;

class audio_controller {
public:
	audio_controller();

private:
	// service callbacks
	bool wav_play_callback(rovio_av::wav_play::Request &req,
			rovio_av::wav_play::Response &resp);

	// authentication information for the Rovio
	string host;
	string user;
	string pass;

	// a handle for the node
	ros::NodeHandle node;

	// services
	ros::ServiceServer wav_play;
};

#endif
