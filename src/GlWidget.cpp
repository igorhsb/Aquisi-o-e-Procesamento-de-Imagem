#include "../include/project_pkg/GlWidget.h"
#include <QtGui/QMouseEvent>
#define WIDTH 640
#define HEIGHT 48
#define Size WIDTH*HEIGHT

float *dataPC;
int *dataPCColor;

GLWidget::GLWidget(QWidget *parent, int stat) :
  QGLWidget(parent),
  requested_format(FREENECT_VIDEO_RGB),
  freenect_angle(0),
  got_frames(0),
  status(stat),
  rotate(0),
  modoVideo("RGB"),
  kinectCloud(new sensor_msgs::PointCloud),
  depthData(new uint16_t),
  position(0),
  xRot (0),
  yRot (0),
  zRot (0),
  rotateEnabled(false)
{
  setMouseTracking(true);
  timer = new QTimer();
  connect(timer, SIGNAL(timeout()), this, SLOT(updateGL()));
  timer->start(30);

  shader_s = "attribute vec3 position;"
             "varying float color;"
             "uniform mat4 matrix;"
             "void main(){"
             "   color = (position.z + 1.0) * 0.5;"
             "   gl_Position = matrix * vec4(position, 1.0);"
             "}";

  fragment_s = "varying float color;"
               "	float remap(float minval, float maxval, float curval){"
               "		return (curval - minval)/(maxval - minval);"
               "	}"
               "	vec4 hsl(float h, float s, float l)"
               "{"
               "	float v = (l <= 0.5) ? (l * (1.0 + s)) : (l + s - l * s);"
               "	float r = 1.00;"
               "	float g = 1.00;"
               "	float b = 1.00;"
               "	if(v > 0.0)"
               "	{"
               "		float m;"
               "		float sv;"
               "		int sextant;"
               "		float fract, vsf, mid1, mid2;"
               "		m = l + l - v;"
               "		sv = (v - m ) / v;"
               "		h *= 6.0;"
               "		sextant = int(h);"
               "		fract = h - float(sextant);"
               "		vsf = v * sv * fract;"
               "		mid1 = m + vsf;"
               "		mid2 = v - vsf;"
               "		if (sextant == 0)"
               "		{"
               "				r = v;"
               "				g = mid1;"
               "				b = m;"
               "		}"
               "		else if(sextant == 1)"
               "		{"
               "				r = mid2;"
               "				g = v;"
               "				b = m;"
               "		}"
               "		else if(sextant == 2)"
               "		{"
               "				r = m;"
               "				g = v;"
               "				b = mid1;"
               "		}"
               "		else if(sextant == 3)"
               "		{"
               "				r = m;"
               "				g = mid2;"
               "				b = v;"
               "		}"
               "		else if(sextant == 4)"
               "		{"
               "				r = mid1;"
               "				g = m;"
               "				b = v;"
               "		}"
               "		else if(sextant == 5)"
               "		{"
               "				r = v;"
               "				g = m;"
               "				b = mid2;"
               "		}"
               "	}"
               "	return vec4(r, g, b, 1.0);"
               "}	"
               "void main(){"
               "   gl_FragColor = hsl(remap(0.0, 1.0, color), 1.0, 0.5);"
               "}";
}

GLWidget::~GLWidget(){
  timer->stop();
  Stop();
}

GLuint bufferTest;

void GLWidget::initializeGL()
{
  GLenum err = glewInit();
  if (err != GLEW_OK)
  {
    printf("GLEW_OK");
    exit(1); // or handle the error in a nicer way
  }
  if (!GLEW_VERSION_2_1)  // check that the machine supports the 2.1 API.
  {
    printf("GLEW_VERSION_2_1");
    exit(1); // or handle the error in a nicer way
  }

  if(status!=1){
    // glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    // glClearDepth(1.0);
    // glDepthFunc(GL_LESS);
    // glDisable(GL_DEPTH_TEST);
    // glEnable(GL_BLEND);
    // glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // glShadeModel(GL_SMOOTH);
    printf("Creating GLContext");

    PCprogram = glCreateProgram();

    const char* shader_src = shader_s.c_str();
    const char* frag_src = fragment_s.c_str();

    GLuint shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shader, 1, (const char**) &shader_src, NULL);
    glCompileShader(shader);
    GLint compile_status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
    if (compile_status != GL_TRUE) std::cout << std::endl << "Shader compile error" << std::endl;
    std::cout << std::endl << "Shader: " << compile_status << std::endl << std::endl;

    GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, (const char**)&frag_src, NULL);
    glCompileShader(fragment);
    GLint compile_status2 = GL_FALSE;
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &compile_status2);
    if (compile_status2 != GL_TRUE) std::cout << std::endl << "Fragment compile error" << std::endl;
    std::cout << std::endl << "Frag:" << compile_status2 << std::endl << std::endl;

    glAttachShader(PCprogram, shader);
    glAttachShader(PCprogram, fragment);

    glLinkProgram(PCprogram);
    GLint link_status = GL_FALSE;
    glGetProgramiv(PCprogram, GL_LINK_STATUS, &link_status);
    if (link_status != GL_TRUE) std::cout << std::endl << "Error Linking!" << std::endl;
    std::cout << std::endl << "Link: " << link_status << std::endl << std::endl;

    glDeleteShader(shader);
    glDeleteShader(fragment);

    position = glGetAttribLocation(PCprogram, "position");
    matrix = glGetUniformLocation(PCprogram, "matrix");
    printf("Pos: %d, Mat: %d\n", position, matrix);

    int w = 320, h = 240;

    w *= 2;
    h *= 2;
    size = w*h;
    dataPC = new float[size*3];

    float _w = 2.0f / w;
    float _h = 2.0f / h;
    float z_inc = 2.0 / (w*h);
    float z = -1.0f;

    int i = 0;
    for(int y = 0; y < h; y++)
    {
      for(int x = 0; x < w; x++)
      {
        i = 3*(y*w + x);
        dataPC[i] = (x * _w) - 1.0f;
        dataPC[i+1] = (y * _h) - 1.0f;
        //data[i+2] = 2.0f*QRandomGenerator::global()->generateDouble()-1.0f;
        dataPC[i+2] = z;
        z += z_inc;
        //printf("%f - %f - %f\n", dataPC[i], dataPC[i+1], dataPC[i+2]);
      }
    }
    resizeGL(630,400);
  }
  else if(status == 1)
  {
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);
    glGenTextures(1, &gl_depth_tex);
    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    resizeGL(630, 400);
  }
}

void GLWidget::updateGL()
{
  glDraw();
}


void GLWidget::paintGL()
{
  if(status==0){
    GLfloat x,y,z,angle; // Storage for coordinates and angles
    GLfloat GL_PI = 3.1415f;
    // Clear the window with current clearing color
    glClear(GL_COLOR_BUFFER_BIT);
    // Save matrix state and do the rotation
    glPushMatrix();
    glRotatef(45.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(0.0f, 0.0f, 1.0f, 0.0f);
    // Call only once for all remaining points
    glBegin(GL_POINTS);
    z = 50.0f;
    for(angle = 0.0f; angle <= (2.0f*GL_PI)*3.0f; angle += 0.1f)
    {
      x = 310+50.0f*sin(angle);
      y = 180+50.0f*cos(angle);
      // Specify the point and move the Z value up a little
      glVertex3f(x, y, z);
      z -= 0.5f;
    }
    // Done drawing points
    glEnd();
    // Restore transformations
    glPopMatrix();
    // Flush drawing commands
    glFlush();
  }
  else if(status==1)
  {
    //    static std::vector<float> depth(640*480*4);
    //    static std::vector<uint8_t> rgb(640*480*4);

    //    device->updateState();
    //    fflush(stdout);

    //    device->getDepth(depth);
    //    device->getRGB(rgb);
    //    got_frames = 0;

    //    //    printf("%f",depth[(3*320*240)+2]);

    //    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //    glLoadIdentity();

    //    glEnable(GL_TEXTURE_2D);

    //    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    //    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

    //    glBegin(GL_TRIANGLE_FAN);
    //    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    //    glTexCoord2f(0, 0); glVertex3f(0,0,0);
    //    glTexCoord2f(1, 0); glVertex3f(315,0,0);
    //    glTexCoord2f(1, 1); glVertex3f(315,400,0);
    //    glTexCoord2f(0, 1); glVertex3f(0,400,0);
    //    glEnd();

    //    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    //    if (device->getVideoFormat() == FREENECT_VIDEO_RGB || device->getVideoFormat() == FREENECT_VIDEO_YUV_RGB)
    //      glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &rgb[0]);
    //    else
    //      glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, &rgb[0]);



    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(315,0,0);
    glTexCoord2f(1, 0); glVertex3f(630,0,0);
    glTexCoord2f(1, 1); glVertex3f(630,400,0);
    glTexCoord2f(0, 1); glVertex3f(315,400,0);
    glEnd();

  }
  else if(status==2)
  {
    int i;
    double x,y,z;

    glClear(GL_COLOR_BUFFER_BIT);
    glRotated(0,0,0,1);
    for(int i = 0; i < 729 ; i++){
      x = (laser->laserData[i].x*380/11.2)+(380/2)+125;
      y = (laser->laserData[i].y*380/11.2)+(380/2)+10;
      z = (laser->laserData[i].z);
      glBegin(GL_POINTS);
      glColor3f(1.0,0.0,0.0);
      glPointSize(100);
      glVertex3f(x,y,z);
      glEnd();
    }
  }
  else if(status == 3)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glGenTextures(3, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->rgb.data);
    // (Type of texture,Pyramid level - 0 is the top level,Internal colour format,width,height,Border width in pixels,Input image format,Image data type, image data)

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(0,0,0);
    glTexCoord2f(1, 0); glVertex3f(315,0,0);
    glTexCoord2f(1, 1); glVertex3f(315,400,0);
    glTexCoord2f(0, 1); glVertex3f(0,400,0);
    glEnd();

    //        glGenTextures(3, &texture);
    //        glBindTexture(GL_TEXTURE_2D, texture);
    //        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    //        glBindTexture(GL_TEXTURE_2D, texture);
    //        glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->depthView.data);

    //        glBegin(GL_TRIANGLE_FAN);
    //        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    //        glTexCoord2f(0, 0); glVertex3f(315,0,0);
    //        glTexCoord2f(1, 0); glVertex3f(630,0,0);
    //        glTexCoord2f(1, 1); glVertex3f(630,400,0);
    //        glTexCoord2f(0, 1); glVertex3f(315,400,0);
    //        glEnd();
  }
  else if(status == 4)
  {
    //    std::cout << std::endl << "Draw Working" << std::endl;

    glViewport(0, 0, width() , height() );
    //std::cout << std::endl << "W: "<< width() << "H " << height() << std::endl;
    //glViewport(0, 0, width(), height());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(PCprogram);

    QMatrix4x4 mat;
    mat.perspective(60.0f, (float)width()/(float)height(), 1.0f, 100.0f);
    mat.translate(0, 0, -3);
    mat.rotate(xRot, 0, 1, 0);
    mat.rotate(yRot, 1, 0, 0);

    float rawMat[16];
    for(int i = 0; i < 16; i++)
      rawMat[i] = (float)mat.constData()[i];

    glUniformMatrix4fv(matrix, 1, GL_FALSE, rawMat);
    glVertexAttribPointer(position, 3, GL_FLOAT, false, 0, dataPC);
    glEnableVertexAttribArray(position);
    glDrawArrays(GL_POINTS, 0, size);
    glDisableVertexAttribArray(position);
    ++m_frame;

    //    glBegin(GL_TRIANGLE_FAN);
    //    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    //    glTexCoord2f(0, 0); glVertex3f(0,0,0);
    //    glTexCoord2f(1, 0); glVertex3f(630,0,0);
    //    glTexCoord2f(1, 1); glVertex3f(630,400,0);
    //    glTexCoord2f(0, 1); glVertex3f(0,400,0);
    //    glEnd();
    //  swappBuffers();
  }
}

void GLWidget::resizeGL(int Width, int Height)
{  

  glViewport(0,0,Width,Height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho (0, Width, Height, 0, -50000.0f, 50000.0f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

}

void GLWidget::keyPressEvent(QKeyEvent* event) {
  if(status==1)
  {
    if (event->key() == Qt::Key_Escape) {
      status = 0;
      printf("\n\nEsc");
    }
    if (event->key() == Qt::Key_1) {
      device->setLed(LED_GREEN);
    }
    if (event->key() == Qt::Key_2) {
      device->setLed(LED_RED);
    }
    if (event->key() == Qt::Key_3) {
      device->setLed(LED_YELLOW);
    }
    if (event->key() == Qt::Key_4) {
      device->setLed(LED_BLINK_GREEN);
    }
    if (event->key() == Qt::Key_5) {
      device->setLed(LED_BLINK_RED_YELLOW);
    }
    if (event->key() == Qt::Key_6) {
      device->setLed(LED_OFF);
    }
    if (event->key() == Qt::Key_M) {
      modoVideo.clear();
      if (requested_format == FREENECT_VIDEO_IR_8BIT){
        requested_format = FREENECT_VIDEO_RGB;
        modoVideo.append("RGB");
      }
      else if (requested_format == FREENECT_VIDEO_RGB){
        requested_format = FREENECT_VIDEO_YUV_RGB;
        modoVideo.append("RGB_YUV");
      }
      else{
        requested_format = FREENECT_VIDEO_IR_8BIT;
        modoVideo.append("8 Bit");
      }
      device->setVideoFormat(requested_format);
    }

    if (event->key() == Qt::Key_W) {
      printf("\n\nW");
      freenect_angle++;
      if (freenect_angle > 30) {
        freenect_angle = 30;
      }
    }
    if (event->key() == Qt::Key_S) {
      freenect_angle = 0;
    }
    if (event->key() == Qt::Key_X) {
      freenect_angle--;
      if (freenect_angle < -30) {
        freenect_angle = -30;
      }
    }
    updateGL();
    device->setTiltDegrees(freenect_angle);
  }
  else if(status == 0)
  {
    if (event->key() == Qt::Key_X){
      printf("\n\nX");
    }
  }
  else if(status == 2)
  {

  }
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
  this->setFocus();
  lastPos = event->pos();
  rotateEnabled = true;
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
  rotateEnabled = false;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
  if(rotateEnabled == true){
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();
    xRot += dx;
    yRot += dy;
    xRot = xRot % 360;
    yRot = yRot % 360;
    lastPos = event->pos();
  }
  //  if (event->buttons() & Qt::LeftButton) {
  //    setXRotation(xRot + 2 * dy);
  //    setYRotation(yRot + 2 * dx);
  //  } else if (event->buttons() & Qt::RightButton) {
  //    setXRotation(xRot + 2 * dy);
  //    setZRotation(zRot + 2 * dx);
  //  }
}

void GLWidget::setXRotation(int angle)
{
  if (angle != xRot) {
    xRot = angle;
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    updateGL();
  }
}

void GLWidget::setYRotation(int angle)
{
  if (angle != yRot) {
    yRot = angle;
    glRotatef(yRot / 16.0, 1.0, 0.0, 0.0);
    updateGL();
  }
}

void GLWidget::setZRotation(int angle)
{
  if (angle != zRot) {
    zRot = angle;
    glRotatef(zRot / 16.0, 1.0, 0.0, 0.0);
    updateGL();
  }
}

int GLWidget::KinectStart(){
  int dev,create = 0;
  dev = freenect.deviceCount();
  if(dev>0){
    device = &freenect.createDevice<MyFreenectDevice>(0);
    device->startVideo();
    device->startDepth();
    if (!timer->isActive()) timer->start(30);
    updateGL();
  }
  return dev;
}

void GLWidget::AstraInit(ros::NodeHandle nodeH){
  astraDevice = new Astra_Camera(nodeH);
  updateGL();
}

void GLWidget::test_init()
{
  std::cout << std::endl << "Init Working" << std::endl;
  SetStatus(4);
}

void GLWidget::ImuInit(ros::NodeHandle nodeH){
  imuDevice = new ImuSensor(nodeH);
  updateGL();
}

void GLWidget::SetStatus(int s){
  this->status = s;
  timer->stop();
  if (!timer->isActive()) timer->start(30);
  updateGL();
}

int GLWidget::GetStatus(){
  return this->status;
}

void GLWidget::Start(float val)
{
  timer->start(val);
  updateGL();
}

void GLWidget::Stop()
{
  if(status == 1){
    device->stopDepth();
    device->stopVideo();
    this->freenect.deleteDevice(0);
  }
  SetStatus(0);
  updateGL();
}

// -------------------------------------------------------------------------------------------------------------------------
