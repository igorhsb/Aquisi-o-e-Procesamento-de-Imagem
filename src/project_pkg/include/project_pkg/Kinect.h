#ifndef _KINECT_H
#define _KINECT_H

#include "libfreenect/libfreenect.hpp"
#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <cmath>
#include <vector>
#include "sync.h"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>              //Ros includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#endif

typedef struct Laser_struct{
  double distancia,angulo,x,y,z;
}LaserData;

class Mutex {
public:
  Mutex() {
    pthread_mutex_init( &m_mutex, NULL );
  }
  void lock() {
    pthread_mutex_lock( &m_mutex );
  }
  void unlock() {
    pthread_mutex_unlock( &m_mutex );
  }

  class ScopedLock
  {
    Mutex & _mutex;
  public:
    ScopedLock(Mutex & mutex)
      : _mutex(mutex)
    {
      _mutex.lock();
    }
    ~ScopedLock()
    {
      _mutex.unlock();
    }
  };
private:
  pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
  MyFreenectDevice(freenect_context *_ctx, int _index):
    Freenect::FreenectDevice(_ctx, _index),
    m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),
    m_gamma(2048),
    m_new_rgb_frame(false),
    m_new_depth_frame(false),
    cx(320.0), // center of projection
    cy(240.0), // center of projection float
    fx (600.0), // focal length in pixels
    fy(600.0), // focal length in pixels
    k1(1.1863),
    k2(2842.5),
    k3(0.1236)
  {
    for( unsigned int i = 0 ; i < 2048 ; i++) {
      gamma[i] = k3 * tan(i/k2 + k1);

      float v = i/2048.0;
      v = std::pow(v, 3)* 6;
      m_gamma[i] = v*6*256;
    }
  }

  void VideoCallback(void* _rgb, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_rgb_mutex);
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
  };

  void DepthCallback(void* _depth, uint32_t timestamp) {
    Mutex::ScopedLock lock(m_depth_mutex);
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    for (size_t v=0, n=0 ; v<480 ; v++)
    {
      for (size_t u=0 ; u<640 ; u++, n++)
      {
        m_buffer_depth[3*n+0] = (u - cx) * gamma[depth[n]] / fx;
        m_buffer_depth[3*n+1] = (v - cy) * gamma[depth[n]] / fy;
        m_buffer_depth[3*n+2] = gamma[depth[n]];
      }
    }
    m_new_depth_frame = true;
  }

  bool getRGB(std::vector<uint8_t> &buffer) {
    Mutex::ScopedLock lock(m_rgb_mutex);
    if (!m_new_rgb_frame)
      return false;
    buffer.swap(m_buffer_video);
    m_new_rgb_frame = false;
    return true;
  }

  bool getDepth(std::vector<float> &buffer) {
    Mutex::ScopedLock lock(m_depth_mutex);
    if (!m_new_depth_frame)
      return false;
    buffer.swap(m_buffer_depth);
    m_new_depth_frame = false;
    return true;
  }

  //  bool getDepthView(std::vector<uint8_t> &buffer) {
  //    Mutex::ScopedLock lock(m_depth_mutex);
  //    if (!m_new_depth_frame)
  //      return false;
  //    buffer.swap(m_buffer_depth2);
  //    m_new_depth_frame = false;
  //    return true;  }

  //  void getBuffer(){
  //    printf("\n\n %u",m_buffer_depth[1]);
  //    printf("\n\n %u",m_buffer_depth[1]);
  //  }

private:
  std::vector<float> m_buffer_depth;
  std::vector<uint16_t> m_buffer_depth2;
  std::vector<uint8_t> m_buffer_video;
  std::vector<uint16_t> m_gamma;
  float gamma[2048];
  uint16_t* depthData;
  Mutex m_rgb_mutex;
  Mutex m_depth_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
  float cx;// center of projection
  float cy;// center of projection float
  float fx;// focal length in pixels
  float fy;// focal length in pixels
  const float k1;
  const float k2;
  const float k3;
};

#endif  /* _KINECT_H */
