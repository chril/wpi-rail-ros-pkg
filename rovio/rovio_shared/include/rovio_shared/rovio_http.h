/*
 * rovio_http.h
 *
 *  Created on: Aug 7, 2011
 *      Author: rctoris
 */

#ifndef ROVIO_HTTP_H_
#define ROVIO_HTTP_H_

#include <curl/curl.h>
#include "ros/ros.h"
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>

// maximum size of a URL
#define URL_BUF_SIZE 256

class rovio_http
{
public:
  rovio_http(std::string user, std::string pass);
  virtual ~rovio_http();

  void send(char *url, std::string *buf=NULL);

private:
  CURL *curl; // used to communicate with the Rovio
  sem_t sem; // used to ensure only one call to Curl occurs
};

#endif /* ROVIO_HTTP_H_ */
