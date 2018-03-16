#include "../include/project_pkg/Astra.h"

Astra_Camera::Astra_Camera(ros::NodeHandle n):
  n_(n),
  it_(n)
{

  astra_sub = it_.subscribe("/camera/rgb/image_rect_color", 1, &Astra_Camera::RgbCallback, this);
  astra_sub2 = it_.subscribe("/camera/depth/image_rect", 1, &Astra_Camera::DepthCallback, this);
  astra_sub3 = it_.subscribe("/camera/depth/image", 1, &Astra_Camera::DepthViewCallback, this);
  //point_sub = n_.subscribe("/camera/depth/points", 1, &Astra_Camera::PointCB, this);
  //astra_sub = n_.subscribe("/camera/depth/points", 1, &Astra_Camera::depthCall, this);

  //  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb( n_ , "/cv_camera/image_raw" , 1 );
  //  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth( n_ , "/cv_camera/image_raw/compressed" , 1 );
  //  TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync( subscriber_rgb, subscriber_depth, 10);
  //  sync.registerCallback(boost::bind(&Astra_Camera::Callback,this, _1, _2));

}

void Astra_Camera::RgbCallback(const sensor_msgs::ImageConstPtr& msg_rgb)
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

void Astra_Camera::DepthCallback(const sensor_msgs::ImageConstPtr& msg_depth)
{
  cv_bridge::CvImagePtr depth_ptr;
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
  float z_raw = depth.data[320*240];
  ROS_INFO("%f",z_raw);
}

void Astra_Camera::DepthViewCallback(const sensor_msgs::ImageConstPtr& msg_depthV){
  cv_bridge::CvImagePtr depthV_ptr;
  try
  {
    depthV_ptr = cv_bridge::toCvCopy(msg_depthV, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  depthView = depthV_ptr->image;
}

void Astra_Camera::PointCB(const sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::PCLPointCloud2 pointCloud;
  pcl_conversions::toPCL(*msg,pointCloud);
  float z = hypot(pointCloud.data[320*240 + 2], pointCloud.data[320*240]);
  //ROS_INFO("Distance: %f, asda: %u",z, pointCloud.data[320*240 + 2]);
}

void Astra_Camera::Callback( const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth )
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
