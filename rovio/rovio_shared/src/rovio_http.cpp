/*
 * rovio_http.cpp
 *
 *  Created on: Aug 7, 2011
 *      Author: rctoris
 */

#include "rovio_shared/rovio_http.h"

size_t write_data(char *ptr, size_t size, size_t nmemb, std::string *buf)
{
  // the actual size of the data
  size_t tot_s = size * nmemb;

  //add the data to the buffer
  buf->append((char*)ptr, tot_s);

  return tot_s;
}

rovio_http::rovio_http(std::string user, std::string pass)
{
  // create the CURL handle
  curl = curl_easy_init();
  if (curl == NULL)
  {
    ROS_ERROR("Curl was unable to initialize.");
    exit(-1);
  }

  // set the username and password
  curl_easy_setopt(curl, CURLOPT_USERPWD, user.append(":").append(pass).c_str());
  // set the pointer to the function which handles the responses
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &write_data);

  // create the semaphore
  sem_init(&sem, 0, 1);
}

rovio_http::~rovio_http()
{
  // cleanup anything left by Curl
  curl_easy_cleanup(curl);
  // destroy the semaphore
  sem_destroy(&sem);
}

void rovio_http::send(const char *url, std::string *buf)
{
  // wait for the curl handle to be free
  sem_wait(&sem);
  // check if we are saving the data
  if (buf == NULL)
  {
    // to be thrown away afterwards
    std::string resp_buf = "";
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &resp_buf);
  }
  else
  {
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, buf);
  }
  //send the command to the Rovio
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_perform(curl);
  sem_post(&sem);
}
