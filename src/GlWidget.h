#ifndef _GLWIDGET_H
#define _GLWIDGET_H

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <QGLWidget>
#include <QObject>
#include <QPoint>
#include "Kinect.h"
#include <QTimer>
#include "Laser.h"
#include "Astra.h"
#include "Imu.h"
#include "sync.h"
#include <QtGui/QMatrix4x4>

class GLWidget : public QGLWidget {

  Q_OBJECT

public:
  GLWidget(QWidget *parent = NULL, int stat = 0);
  ~GLWidget();
  Freenect::Freenect freenect;
  MyFreenectDevice* device;
  freenect_video_format requested_format;
  LaserS *laser;
  Astra_Camera *astraDevice;
  ImuSensor *imuDevice;
  sensor_msgs::PointCloud *kinectCloud;
  QString modoVideo;

  void SetStatus(int s);
  int GetStatus();
  void Start(float val);
  void Stop();

  uint8_t rgb_data[640][480];
  float depth_data[640*480*3];

  GLuint gl_depth_tex;
  GLuint gl_rgb_tex,texture;


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
  void test_init();
  std::string shader_s,fragment_s;
  GLuint PCprogram,matrix,position;
  int m_frame,size;

protected:
  void initializeGL();
  void resizeGL(int Width, int Height);
  void mousePressEvent(QMouseEvent *event);
  void mouseMoveEvent(QMouseEvent *event);
  void mouseReleaseEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent *event);
  void paintGL();
  void updateGL();

private:
  QTimer *timer;
  int status;

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
