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
size_t write_data(char *ptr, size_t size, size_t nmemb, rovio_response *buf)
{
  // actual size of the new data
  size_t new_data = size * nmemb;

  // see if there is any data
  if (new_data > 0)
  {

    // check if the buffer already has data
    if (buf->data)
      // resize the buffer
      buf->data = (char *)realloc(buf->data, buf->size + new_data + 1);
    else
      // allocate the initial memory
      buf->data = (char *)malloc(new_data + 1);

    // add the data to the buffer
    memcpy(&(buf->data[buf->size]), ptr, new_data);
    //update the size
    buf->size += new_data;
    // null terminate
    buf->data[buf->size] = '\0';
  }

  return new_data;
}

/*!
 * Cleanup any resources used by a rovio_response struct.
 *
 * \param resp the rovio_response struct to cleanup
 */
void rovio_response_clean(rovio_response *resp)
{
  if (resp)
  {
    if (resp->data)
      free(resp->data);
    free(resp);
  }
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
 * Send the given full URL command to the Rovio. A buffer containing the response is returned.
 *
 * \param url the full URL command to send to the Rovio
 * \return a buffer containing the response from the Rovio
 */
rovio_response *rovio_http::send(const char *url)
{
  // wait for the curl handle to be free
  sem_wait(&sem);

  // create the response for the Rovio
  rovio_response *resp = (rovio_response *)malloc(sizeof(rovio_response));
  resp->size = 0;
  resp->data = NULL;
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, resp);

  //send the command to the Rovio
  curl_easy_setopt(curl, CURLOPT_URL, url);
  curl_easy_perform(curl);

  sem_post(&sem);

  return resp;
}
