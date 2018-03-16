#ifndef _ASTRA_H
#define _ASTRA_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/common/common_headers.h>
#include <boost/foreach.hpp>
#include <pcl/filters/passthrough.h>
#include <stdio.h>
#include <boost/foreach.hpp>
#endif

using namespace message_filters;

class Astra_Camera{

public:
  Astra_Camera(ros::NodeHandle n);
  ~Astra_Camera();
  std_msgs::Header header;
  std::string encoding;

  void RgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb);
  void DepthCallback(const sensor_msgs::ImageConstPtr& msg_depth );
  void DepthViewCallback(const sensor_msgs::ImageConstPtr& msg_depth );
  void PointCB(const sensor_msgs::PointCloud2ConstPtr& msg);
  void Callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth );

  cv::Mat rgb, depth, depthView;
  sensor_msgs::PointCloud2* astraPoints;
  std::vector<float> depthData;

private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber astra_sub,astra_sub2,astra_sub3;
  ros::Subscriber point_sub;
};

#endif
