/*
 * The rovio_http object can be used to send HTTP commands to the Rovio and read in server responses. It is intended to be used by other Rovio packages.
 *
 * Author: Russell Toris, WPI - rctoris@wpi.edu
 * Version: August 15, 2011
 */

#include "rovio_shared/rovio_http.h"

using namespace std;

/*!
 * The callback function used by Curl to store the response from the Rovio. This function should only be used by Curl internally by the rovio_http.
 */
size_t write_data(char *ptr, size_t size, size_t nmemb, std::string *buf)
{
  // the actual size of the data
  size_t tot_s = size * nmemb;

  //add the data to the buffer
  buf->append((char*)ptr, tot_s);

  return tot_s;
}

/*!
 * Create a rovio_http object that can be used to send HTTP commands to the Rovio. A valid username and password must be provide at construction.
 *
 * \param user the username used to authenticate with the Rovio
 * \param pass the password used to authenticate with the Rovio
 */
rovio_http::rovio_http(string user, string pass)
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

/*!
 * Cleanup any resources from the rovio_http object and from Curl.
 */
rovio_http::~rovio_http()
{
  // cleanup anything left by Curl
  curl_easy_cleanup(curl);
  // destroy the semaphore
  sem_destroy(&sem);
}

/*!
 * Send the given full URL command to the Rovio. An optional buffer can be given if the returned output is needed. This string buffer should be initialized to the empty string.
 *
 * \param url the full URL command to send to the Rovio
 * \param buf an optional empty string buffer for the response to be stored in
 */
void rovio_http::send(const char *url, string *buf)
{
  // wait for the curl handle to be free
  sem_wait(&sem);
  // check if we are saving the data
  if (buf == NULL)
  {
    // to be thrown away afterwards
    string resp_buf = "";
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
