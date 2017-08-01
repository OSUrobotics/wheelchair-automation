/*
 * util.cpp
 *
 *  Created on: Nov 23, 2012
 *      Author: ross kidson
 */


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// cv::Mat --> sensor_msg::Image
sensor_msgs::ImagePtr convertCVToSensorMsg (const cv::Mat input)
{
  cv_bridge::CvImagePtr out_msg (new cv_bridge::CvImage());
  //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
  if(input.channels()==1)
    out_msg->encoding = sensor_msgs::image_encodings::MONO8;
  else
    out_msg->encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
  out_msg->image = input; // Your cv::Mat
  return out_msg->toImageMsg();

}

// sensor_msg::Image -> cv::Mat
cv::Mat convertSensorMsgToCV (const sensor_msgs::ImageConstPtr ros_image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy (*ros_image, ros_image->encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR ("cv_bridge exception: %s", e.what ());
  }
  return cv_ptr->image;
}

// sensor_msg::Image -> cv::Mat
cv::Mat convertSensorMsgToCV (const sensor_msgs::Image& ros_image)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy (ros_image, ros_image.encoding);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR ("cv_bridge exception: %s", e.what ());
  }
  return cv_ptr->image;
}
