/*
 * The rovio_audio creates a ROS node that listens for the name of a .wav file as a string. The file is then streamed and played on the Rovio's speaker.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date October 11, 2011
 */

#include <arpa/inet.h>
#include <cstdlib>
#include <ros/ros.h>
#include <rovio_av/wav_play.h>
#include <rovio_shared/rovio_http.h>
#include <rovio_av/rovio_audio.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <sys/socket.h>

using namespace std;

/*!
 * Creates a audio_controller object that can be used to stream .wav files to the Rovio. A valid username, password, and host must be set as ROS parameters.
 */
audio_controller::audio_controller() {
	// check for all the correct parameters
	if (!node.getParam(USER, user)) {
		ROS_ERROR("Parameter %s not found.", USER);
		exit(-1);
	}
	if (!node.getParam(PASS, pass)) {
		ROS_ERROR("Parameter %s not found.", PASS);
		exit(-1);
	}
	if (!node.getParam(HOST, host)) {
		ROS_ERROR("Parameter %s not found.", HOST);
		exit(-1);
	}

	// add services
	wav_play = node.advertiseService("wav_play",
			&audio_controller::wav_play_callback, this);

	ROS_INFO("Rovio Audio Controller Initialized");
}

/*!
 * Process the service request and attempt to stream the given file to the Rovio. This call will block until the file has finished being sent.
 *
 * \param req the request for the wav_play server
 * \param argv the response for the wav_play server; this does not contain any information for this service
 * \return if the file was successfully streamed to the Rovio
 */
bool audio_controller::wav_play_callback(rovio_av::wav_play::Request &req,
		rovio_av::wav_play::Response &resp) {
	// create a socket to talk to the Rovio
	int audio_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (audio_socket < 0) {
		ROS_ERROR("Audio socket could not be created.");
		return false;
	}

	// setup the address
	struct sockaddr_in host_addr;
	memset(&host_addr, 0, sizeof(host_addr));
	host_addr.sin_family = AF_INET;
	host_addr.sin_addr.s_addr = inet_addr(host.c_str());
	host_addr.sin_port = htons(80);

	// connect the socket
	if (connect(audio_socket, (struct sockaddr *) &host_addr, sizeof(host_addr))
			< 0) {
		ROS_ERROR("Audio socket could not be connected.");
		return false;
	}

	// get the file name from the request
	string f_name = req.f;

	// try and open the file
	FILE *f = fopen(f_name.c_str(), "rb");
	if (f == NULL) {
		ROS_ERROR("Could not open file '%s'.", f_name.c_str());
		return false;
	}

	// find out how big the file is
	fseek(f, 0, SEEK_END);
	long f_size = ftell(f);
	rewind(f);
	// create a buffer and read in the entire file
	char *buf = (char*) malloc(f_size);
	if ((long) fread(buf, 1, f_size, f) != f_size) {
		ROS_ERROR("Error reading file '%s'.", f_name.c_str());
		return false;
	}

	// build the header
	stringstream ss;
	ss << "POST /GetAudio.cgi HTTP/1.1\r\n" << "User-Agent: AudioAgent\r\n"
			<< "Host: " << host << "\r\n" << "Content-Length: " << f_size
			<< "\r\n" << "Cache-Control: no-cache\r\n" << "\r\n";

	// send the sound file
	send(audio_socket, ss.str().c_str(), ss.str().size(), 0);
	if (send(audio_socket, buf, f_size, 0) != f_size) {
		ROS_ERROR("Could not send entire file.");
		return false;
	}

	// close the file
	fclose(f);
	// free the buffer
	free(buf);
	// close the socket
	close(audio_socket);

	return true;
}

/*!
 * Creates and runs the rovio_audio node.
 *
 * \param argc argument count that is passed to ros::init
 * \param argv arguments that are passed to ros::init
 * \return EXIT_SUCCESS if the node runs correctly
 */
int main(int argc, char **argv) {
	// initialize ROS and the node
	ros::init(argc, argv, "rovio_audio");

	// initialize the Rovio controller
	audio_controller controller;

	// update at 5 Hz
	ros::Rate loop_rate(5);
	// continue until a ctrl-c has occurred
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}
