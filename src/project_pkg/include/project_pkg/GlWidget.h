#ifndef _GLWIDGET_H
#define _GLWIDGET_H

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/gl.h>

#include <QGLWidget>
#include <QObject>
#include <QPoint>
#include <QtGui/QMatrix4x4>
#include "Kinect.h"
#include <QTimer>
#include "Laser.h"
#include "Astra.h"
#include "Zed.h"
#include "Imu.h"
#include "sync.h"

#include <sl/Camera.hpp>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <cudaGL.h>

#include <ctime>
#include <thread>
#include <string.h>
#include <string>
#include <strings.h>
#include <math.h>
#include <iostream>


class GLWidget : public QGLWidget {

  Q_OBJECT

public:
  GLWidget(QWidget *parent = NULL, int stat = 0);
  ~GLWidget();
  Freenect::Freenect freenect;
  MyFreenectDevice* device;
  freenect_video_format requested_format;
  LaserS *laser;
  ImuSensor *imuDevice;
  Astra_Camera *astraDevice;
  ZedDevice zedDevice;
  cudaError_t err1, err2;
  sensor_msgs::PointCloud *kinectCloud;
  QString modoVideo;

  void SetStatus(int s);
  int GetStatus();
  void Start(float val);
  void Stop();

  uint8_t rgb_data[640][480];
  float depth_data[640*480*3];

  GLuint gl_depth_tex,gl_rgb_tex,rgbAstra,depthAstra,depthZed,rgbZed;

  double freenect_angle;
  int got_frames;

  int KinectStart();
  void KinectStop();
  int rotate;

  void startMatriz();
  uint8_t showRGB(int i, int j);
  double showDepth(int i, int j);

  void AstraInit(ros::NodeHandle nodeH);
  void ImuInit(ros::NodeHandle nodeH);
  void ZedInit();
  void LaserInit(ros::NodeHandle nodeH);
  void ZedDraw();
  void AstraDraw();
  void LaserDraw();
  void KinectDraw();
  void ChangeMod(int i);
  int zedOn,kinectOn,imuOn,laserOn;
  void saveFiles();

  void Syncronize(ros::AsyncSpinner *spinner);
  void Syncronize_OFF();

  cv::Mat rgbZ,depthZ;
  GLuint imageTex;
  GLuint depthTex;
  GLuint shaderF;
  GLuint program;
  cudaGraphicsResource* pcuImageRes;
  cudaGraphicsResource* pcuDepthRes;
  sl::Mat zedLeftImage,zedDepthImage, pointCloud;
  std::string strFragmentShad;
  std::string shader_s,fragment_s;
  GLuint PCprogram,matrix,position;
  int m_frame,size;
  float *drawPoints;
  static void* zedThreadFunc(void*);

protected:
  void initializeGL();
  void resizeGL(int Width, int Height);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void paintGL();

  void updateGL();

private:
  QTimer *timer;
  int status, viewStatus;
  pthread_t kinectThread,zedThread;
  float gamma[2048];
  uint16_t* depthData;
  
  int xRot;
  int yRot;
  int zRot;
  QPoint lastPos;
  bool rotateEnabled;
  void setXRotation(int angle);
  void setYRotation(int angle);
  void setZRotation(int angle);
};


#endif // _GLWIDGET_H
