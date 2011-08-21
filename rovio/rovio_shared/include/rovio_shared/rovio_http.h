/*
 * The rovio_http object can be used to send HTTP commands to the Rovio and read in server responses. It is intended to be used by other Rovio packages.
 *
 * Author: Russell Toris, WPI - rctoris@wpi.edu
 * Version: August 15, 2011
 */

#ifndef ROVIO_HTTP_H_
#define ROVIO_HTTP_H_

#include <curl/curl.h>
#include <ros/ros.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

// name of the variables that should contain login information
#define USER "/rovio_shared/user"
#define PASS "/rovio_shared/pass"
#define HOST "/rovio_shared/host"

class rovio_http
{
public:
  rovio_http(std::string user, std::string pass);
  virtual ~rovio_http();

  void send(const char *url, std::string *buf = NULL);

private:
  // used to communicate with the Rovio
  CURL *curl;
  // used to ensure only one call to Curl occurs
  sem_t sem;
};

#endif
