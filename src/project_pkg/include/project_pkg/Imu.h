#ifndef Q_MOC_RUN
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#endif
#include "Kinect.h"
#include "sync.h"
#include<stdio.h>
#include<fstream>
#include<iomanip>
#include<iostream> 

typedef struct orientation_struct{
  double x,y,z,w;
}orientation_data;
typedef struct vel_struct{
  double x,y,z,covariance;
}angVel_data;
typedef struct lvel_struct{
  double x,y,z,covariance;
}linVel_data;

typedef struct Imu_struct{
  orientation_data orientation;
  angVel_data angular_vel;
  linVel_data linear_vel;
}Imu_Data;

class ImuSensor{
public:

  ros::NodeHandle n_;
  ros::Subscriber imu_sub;
  sensor_msgs::PointCloud cloud;
  Imu_Data dadosImu;
  ImuSensor(ros::NodeHandle n);
  void Callback(const sensor_msgs::Imu::ConstPtr& imu_msg);
  void SaveFile();
  static void* fileThreadFunc(void*);
  pthread_t fileThread;
};
