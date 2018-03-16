#ifndef _KINECT__H
#define _KINECT__H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#endif

using namespace message_filters;

class Kinect_Device{

public:
  Kinect_Device(ros::NodeHandle n);
  ~Kinect_Device();
  std_msgs::Header header;
  std::string encoding;

  // void ImageCallback();
  void RgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb);
  void DepthCallback(const sensor_msgs::ImageConstPtr& msg_depth );
  void PointCloudCallback(const sensor_msgs::ImageConstPtr& msg);
  void DepthViewCallback(const sensor_msgs::ImageConstPtr& msg_depth );
  void Callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth );
  //void depthCall(const sensor_msgs::PointCloudPtr& msg_point);
  cv::Mat rgb, depth, depthView;

private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber kinect_sub,kinect_sub2,kinect_sub3;
};

#endif
