#ifndef Q_MOC_RUN
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/Image.h"
#endif
#include "Kinect.h"
#include "sync.h"
#include<stdio.h>
#include<fstream>
#include<iomanip>
#include<iostream> 

class LaserS{

public:

  ros::NodeHandle n_;
  ros::Subscriber scan_sub;
  sensor_msgs::PointCloud cloud;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LaserData laserData[729];

  LaserS(ros::NodeHandle n);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);

  void SaveFile();
  static void* fileThreadFunc(void*);
  pthread_t fileThread;
};
