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

// name of the variables that should contain login information
#define USER "/rovio_shared/user"
#define PASS "/rovio_shared/pass"
#define HOST "/rovio_shared/host"

typedef struct
{
  char *data;
  size_t size;
} rovio_response;

void rovio_response_clean(rovio_response *resp);

class rovio_http
{
public:
  rovio_http(std::string user, std::string pass);
  virtual ~rovio_http();

  rovio_response *send(const char *url);

private:
  // used to communicate with the Rovio
  CURL *curl;
  // used to ensure only one call to Curl occurs
  sem_t sem;
};

#endif
