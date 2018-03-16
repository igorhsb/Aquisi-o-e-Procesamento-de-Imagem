#include "../include/project_pkg/Kinect_.h"

Kinect_Device::Kinect_Device(ros::NodeHandle n):
  n_(n),
  it_(n)
{
  kinect_sub = it_.subscribe("/cv_camera/image_raw", 1, &Kinect_Device::RgbCallback, this);
  kinect_sub2 = it_.subscribe("/cv_camera/image_raw", 1, &Kinect_Device::DepthCallback, this);
  kinect_sub3 = it_.subscribe("/cv_camera/image_raw", 1, &Kinect_Device::DepthViewCallback, this);
  //astra_sub = n_.subscribe("/camera/depth/points", 1, &Kinect_Device::depthCall, this);

//  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( n_ , "/cv_camera/image_raw" , 1 );
//  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( n_ , "/cv_camera/image_raw/compressed" , 1 );
//  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync( subscriber_rgb, subscriber_depth, 10);
//  sync.registerCallback(boost::bind(&Kinect_Device::Callback,this, _1, _2));

}

void Kinect_Device::RgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb)
{
  cv_bridge::CvImagePtr rgb_ptr;
  try
  {
    rgb_ptr = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  rgb = rgb_ptr->image;
}

void Kinect_Device::DepthCallback(const sensor_msgs::ImageConstPtr& msg_depth)
{
  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    depth_ptr = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depth = depth_ptr->image;
}

void Kinect_Device::DepthViewCallback(const sensor_msgs::ImageConstPtr& msg_depthV){
  cv_bridge::CvImagePtr depthV_ptr;
  try
  {
    depthV_ptr = cv_bridge::toCvCopy(msg_depthV, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depthView = depthV_ptr->image;
}

void Kinect_Device::PointCloudCallback(const sensor_msgs::ImageConstPtr& msg){

}


void Kinect_Device::Callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
{
  cv_bridge::CvImagePtr rgb_ptr;
  cv_bridge::CvImagePtr depth_ptr;
  try
  {
    rgb_ptr = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  rgb = rgb_ptr->image;

  try
  {
    depth_ptr = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depth = depth_ptr->image;
}
