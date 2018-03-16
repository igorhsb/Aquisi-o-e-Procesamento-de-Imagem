#include "../include/project_pkg/GlWidget.h"
#include <QtGui/QMouseEvent>

GLWidget::GLWidget(QWidget *parent, int stat) :
    QGLWidget(parent),
    requested_format(FREENECT_VIDEO_RGB),
    freenect_angle(0),
    got_frames(0),
    status(stat),
    viewStatus(0),
    rotate(0),
    modoVideo("RGB"),
    kinectCloud(new sensor_msgs::PointCloud),
    depthData(new uint16_t),
    zedOn(0),
    laserOn(0),
    imuOn(0)
{
    setMouseTracking(true);
    timer = new QTimer();
    connect(timer, SIGNAL(timeout()), this, SLOT(updateGL()));
    timer->start(100);
    strFragmentShad = ("uniform sampler2D texImage;\n"
                       " void main() {\n"
                       " vec4 color = texture2D(texImage, gl_TexCoord[0].st);\n"
                       " gl_FragColor = vec4(color.b, color.g, color.r, color.a);\n}");
                       
  
  shader_s = "attribute vec4 position;"
             "varying vec4 color;"
             "uniform mat4 matrix;"
			 "vec4 EncodeExp( in float value ){"
    		 "int exponent  = int( log2( abs( value ) ) + 1.0 );"
    		 "value        /= exp2( float( exponent ) );"
     		 "value         = (value + 1.0) * (256.0*256.0*256.0 - 1.0) / (2.0*256.0*256.0*256.0);"
    		 "vec4 encode   = fract( value * vec4(1.0, 256.0, 256.0*256.0, 256.0*256.0*256.0) );"
    		 "return vec4( encode.xyz - encode.yzw / 256.0 + 1.0/512.0, (float(exponent) + 127.5) / 256.0 );"
			 "}"
             "vec4 Encode( in float value ){"
             "value *= (256.0*256.0*256.0 - 1.0) / (256.0*256.0*256.0);"
             "vec4 encode = fract( value * vec4(1.0, 256.0, 256.0*256.0, 256.0*256.0*256.0) );"
             "return vec4( encode.xyz - encode.yzw / 256.0, encode.w ) + 1.0/512.0;"
             "}"
			 "vec4 decomposeFloat(const in float value){"
		 	 "   uint rgbaInt = floatBitsToUint(value);"
			 "	uint bIntValue = (rgbaInt / 256U / 256U) % 256U;"
			 "	uint gIntValue = (rgbaInt / 256U) % 256U;"
			 "	uint rIntValue = (rgbaInt) % 256U; "
		 	 "	return vec4(rIntValue / 255.0f, gIntValue / 255.0f, bIntValue / 255.0f, 1.0); "
			"}"
             "void main(){"
             "   vec3 pos = vec3(1.0);"
             "   pos.x = position.x*0.5;"
             "   pos.y = position.y*0.5;"
             "   pos.z = position.z*0.5;"
             // Decompose the 4th channel of the XYZRGBA buffer to retrieve the color of the point (1float to 4uint)
             "   color = decomposeFloat(position.a);"
             "   gl_Position = matrix * vec4(pos, 1.0);"
             "}";

  fragment_s = "varying vec4 color;"
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
               "   gl_FragColor = color;"
               //"   gl_FragColor = hsl(remap(0.0, 1.0, color), 1.0, 0.5);"
               "}";
  
    zedLeftImage.free();
    zedDepthImage.free();
}


GLWidget::~GLWidget(){
    timer->stop();
    Stop();
    pthread_cond_signal(&cond1);
    pthread_cond_signal(&cond2);
    pthread_cond_signal(&cond3);
}

void GLWidget::initializeGL()
{
    if(status!=1){

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
        glClear(GL_COLOR_BUFFER_BIT);
        glColor3f(1,0,0);
        //glRotated(1,0.0,0.0,1.0);
        glBegin(GL_POLYGON);
        glVertex2f(50,350);
        glVertex2f(265,50);
        glVertex2f(580,350);
        glEnd();
    }
    else if(status==1)
        KinectDraw();
    else if(status==2)
        LaserDraw();
    else if(status == 3)
        AstraDraw();
    else if(status == 4)
        ZedDraw();
}

void GLWidget::resizeGL(int Width, int Height)
{
    glViewport(0,0,Width,Height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho (0, Width, Height, 0, -5000.0f, 5000.0f);
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
void GLWidget::SetStatus(int s){
    this->status = s;
    timer->stop();
    if (!timer->isActive()) timer->start(10);
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
	std::cout << std::endl << "Stop Device.." << std::endl;
    if(status == 1){
        device->stopDepth();
        device->stopVideo();
        this->freenect.deleteDevice(0);
    }
    if(status == 4){
        zedLeftImage.free();
        zedDepthImage.free();
        glDeleteShader(shaderF);
        glDeleteProgram(program);
        glBindTexture(GL_TEXTURE_2D, 0);
        zedOn == 0;
        free(drawPoints);
    }
    SetStatus(0);
    updateGL();
}

// Init Functions ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::AstraInit(ros::NodeHandle nodeH){
//    makeCurrent();
//    astraDevice = new Astra_Camera(nodeH);
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    glDisable(GL_DEPTH_TEST);
//    glEnable(GL_TEXTURE_2D);

//    glGenTextures(1, &rgbAstra);
//    glBindTexture(GL_TEXTURE_2D, rgbAstra);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glBindTexture(GL_TEXTURE_2D, rgbAstra);
    
//    glGenTextures(1, &depthAstra);
//    glBindTexture(GL_TEXTURE_2D, depthAstra);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
//    glBindTexture(GL_TEXTURE_2D, depthAstra);

//    swapBuffers();
   // astraDevice->init();
    updateGL();
}

void GLWidget::ImuInit(ros::NodeHandle nodeH){
    if(imuOn != 1)
    {
        imuDevice = new ImuSensor(nodeH);
        imuOn = 1;
    }
    updateGL();
}

void GLWidget::LaserInit(ros::NodeHandle nodeH){
    if(laserOn != 1)
    {
        laser = new LaserS(nodeH);
        laserOn = 1;
    }
    
    SetStatus(2);
    updateGL();
}

int position = 3;

float* dataPC;

void GLWidget::ZedInit()
{
    if(zedOn == 0)
        zedDevice.Init();

    if(zedDevice.getStatus() == 1){
      //  printf("\nZED sucefull connect!");
        makeCurrent();
        
        GLenum err = glewInit();
  if (err == GLEW_OK)
  {
        glEnable(GL_TEXTURE_2D);

        // Create and Register OpenGL Texture for Image (RGBA -- 4channels)
        glGenTextures(1, &imageTex);
        glBindTexture(GL_TEXTURE_2D, imageTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1280, 720, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
        glBindTexture(GL_TEXTURE_2D, 0);
        err1 = cudaGraphicsGLRegisterImage(&pcuImageRes, imageTex, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);

        // Create and Register a OpenGL texture for Depth (RGBA- 4 Channels)
        glGenTextures(1, &depthTex);
        glGenTextures(1, &depthTex);
        glBindTexture(GL_TEXTURE_2D, depthTex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1280, 720, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        glBindTexture(GL_TEXTURE_2D, 0);
        err2 = cudaGraphicsGLRegisterImage(&pcuDepthRes, depthTex, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);


        if(err1 == 0 && err2 == 0) // 0 = Sucess
        {

            // Create GLSL fragment Shader for future processing (and here flip R/B)
            GLuint shaderF = glCreateShader(GL_FRAGMENT_SHADER); //fragment shader
            const char* pszConstString = strFragmentShad.c_str();
            glShaderSource(shaderF, 1, (const char**) &pszConstString, NULL);

            // Compile the shader source code and check
            glCompileShader(shaderF);
            GLint compile_status = GL_FALSE;
            glGetShaderiv(shaderF, GL_COMPILE_STATUS, &compile_status);
            if (compile_status != GL_TRUE) SetStatus(0);

            // Create the progam for both V and F Shader
            program = glCreateProgram();
            glAttachShader(program, shaderF);

            glLinkProgram(program);
            GLint link_status = GL_FALSE;
            glGetProgramiv(program, GL_LINK_STATUS, &link_status);
            if (link_status != GL_TRUE) SetStatus(0);

            glUniform1i(glGetUniformLocation(program, "texImage"), 0);
            
        }else
        	std::cout << "Falha ao desenhar textura RGB e Depth" << std::endl;
        
        //PointCloud Init
    
    
    int w = 1280, h = 720;
    size = w*h;
    
    
    PCprogram = glCreateProgram();
    
    /*glGenBuffers(1, &bufferGLID_);
    glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
    glBufferData(GL_ARRAY_BUFFER, w * h * 4 * sizeof(float), 0, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
*/
    // Set current Cuda context, as this function is called in a thread which doesn't handle the Cuda context
    //cuCtxSetCurrent(cuda_zed_ctx);

    //cudaGraphicsGLRegisterBuffer(&bufferCudaID_, bufferGLID_, cudaGraphicsRegisterFlagsNone);

    const char* shader_src = shader_s.c_str();
    const char* frag_src = fragment_s.c_str();

    GLuint shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shader, 1, (const char**) &shader_src, NULL);
    glCompileShader(shader);
    GLint compile_status = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compile_status);
    if (compile_status != GL_TRUE) std::cout << std::endl << "Shader compile error" << std::endl;
    std::cout << std::endl << "Shader: " << compile_status << std::endl;

    GLuint fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, (const char**)&frag_src, NULL);
    glCompileShader(fragment);
    GLint compile_status2 = GL_FALSE;
    glGetShaderiv(fragment, GL_COMPILE_STATUS, &compile_status2);
    if (compile_status2 != GL_TRUE) std::cout << std::endl << "Fragment compile error" << std::endl;
    std::cout << std::endl << "Frag:" << compile_status2 << std::endl;

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
    //printf("Pos: %d, Mat: %d\n", position, matrix);
    
    
            if(zedOn == 0){
                int k = zedDevice.StreamData();
                zedOn = 1;
            }
            else
            {
                printf("Error");
            }
        swapBuffers();
        SetStatus(4);

    }
    else
    {
        printf("\nZed não inicializada!");
        SetStatus(0);
    }
}
}

int GLWidget::KinectStart(){
    int dev,create = 0;
    dev = freenect.deviceCount();
    if(dev>0){
        device = &freenect.createDevice<MyFreenectDevice>(0);
        device->startVideo();
        device->startDepth();
        if (!timer->isActive()) timer->start(10);
        updateGL();
    }
    return dev;
}


// Draw Functions -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool first = true;


void GLWidget::ZedDraw()
{
    sl::Camera::sticktoCPUCore(2);

    if (zedDevice.zed.retrieveImage(zedLeftImage, sl::VIEW_LEFT, sl::MEM_GPU) == sl::SUCCESS) {
        cudaArray_t ArrIm;
        cudaGraphicsMapResources(1, &pcuImageRes, 0);
        cudaGraphicsSubResourceGetMappedArray(&ArrIm, pcuImageRes, 0, 0);
        cudaMemcpy2DToArray(ArrIm, 0, 0, zedLeftImage.getPtr<sl::uchar1>(sl::MEM_GPU), zedLeftImage.getStepBytes(sl::MEM_GPU), zedLeftImage.getWidth() * sizeof(sl::uchar4), zedLeftImage.getHeight(), cudaMemcpyDeviceToDevice);
        cudaGraphicsUnmapResources(1, &pcuImageRes, 0);
    }

    // Map GPU Ressource for Depth. Depth image == 8U 4channels
    if (zedDevice.zed.retrieveImage(zedDepthImage, sl::VIEW_DEPTH, sl::MEM_GPU) == sl::SUCCESS) {
        cudaArray_t ArrDe;
        cudaGraphicsMapResources(1, &pcuDepthRes, 0);
        cudaGraphicsSubResourceGetMappedArray(&ArrDe, pcuDepthRes, 0, 0);
        cudaMemcpy2DToArray(ArrDe, 0, 0, zedDepthImage.getPtr<sl::uchar1>(sl::MEM_GPU), zedDepthImage.getStepBytes(sl::MEM_GPU), zedLeftImage.getWidth() * sizeof(sl::uchar4), zedLeftImage.getHeight(), cudaMemcpyDeviceToDevice);
        cudaGraphicsUnmapResources(1, &pcuDepthRes, 0);
    }
    
    /*if (zedDevice.zed.retrieveMeasure(zedPointCloud, sl::MEASURE_XYZRGBA, sl::MEM_GPU) == sl::SUCCESS) {
		cudaGraphicsMapResources(1, &bufferCudaID_, 0);
		cudaGraphicsResourceGetMappedPointer((void**) &xyzrgbaMappedBuf, &numBytes, bufferCudaID_);
		cudaMemcpy(xyzrgbaMappedBuf, zedDevice.pointCloud.getPtr<sl::float4>(sl::MEM_GPU), numBytes, cudaMemcpyDeviceToDevice);
		cudaGraphicsUnmapResources(1, &bufferCudaID_, 0);
		zed_new_pointcloud = true;
    }*/

    glDrawBuffer(GL_BACK); // Write to both BACK_LEFT & BACK_RIGHT
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    if(viewStatus == 2)
    {
        glBindTexture(GL_TEXTURE_2D, imageTex);

        glUseProgram(program);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(315,0,0);
        glTexCoord2f(1, 1); glVertex3f(315,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();

        glUseProgram(0);

        // Draw depth texture in right part of side by side
        glBindTexture(GL_TEXTURE_2D, depthTex);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(315,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(315,400,0);
        glEnd();
    }
    else if(viewStatus == 1)
    {
        // Draw depth texture in right part of side by side
        glBindTexture(GL_TEXTURE_2D, depthTex);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();
    }
    else if(viewStatus == 0)
    {
        glBindTexture(GL_TEXTURE_2D, imageTex);

        glUseProgram(program);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();

        glUseProgram(0);
    }
    else if(viewStatus == 3)
    {
	    if(zed_new_pointcloud == true){		    
			glUseProgram(PCprogram);
		
			QMatrix4x4 mat;
	  		float rawMat[16];

			mat.perspective(60.0f, (float)width()/(float)height(), 1.0f, 100.0f);
			mat.translate(0, 0, -3);
			mat.rotate(xRot, 0, 1, 0);
			mat.rotate(yRot, 1, 0, 0);
			for(int i = 0; i < 16; i++)
			  rawMat[i] = (float)mat.constData()[i];
			
			glUniformMatrix4fv(matrix, 1, GL_FALSE, rawMat);
            //glBindBuffer(GL_ARRAY_BUFFER, bufferGLID_);
            drawPoints = (float*) malloc(sizeof(zedDevice.dataPC));
            std::memcpy(zedDevice.dataPC,drawPoints,sizeof(zedDevice.dataPC));
			glVertexAttribPointer(position, 4, GL_FLOAT, false, 0, zedDevice.dataPC);
			glEnableVertexAttribArray(position);
			glDrawArrays(GL_POINTS, 0, size);
			glDisableVertexAttribArray(position);
			zed_new_pointcloud = false;
		}
		
		glUseProgram(0);
    }
 
    // Swap
   if (viewStatus != 3)
    	swapBuffers();
}


void GLWidget::KinectDraw()
{
    static std::vector<float> depth(640*480*4);
    static std::vector<uint8_t> rgb(640*480*4);

    device->updateState();
    fflush(stdout);

    device->getDepth(depth);
    device->getRGB(rgb);
    got_frames = 0;

    //    printf("%f",depth[(3*320*240)+2]);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(0,0,0);
    glTexCoord2f(1, 0); glVertex3f(315,0,0);
    glTexCoord2f(1, 1); glVertex3f(315,400,0);
    glTexCoord2f(0, 1); glVertex3f(0,400,0);
    glEnd();

    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    if (device->getVideoFormat() == FREENECT_VIDEO_RGB || device->getVideoFormat() == FREENECT_VIDEO_YUV_RGB)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &rgb[0]);
    else
        glTexImage2D(GL_TEXTURE_2D, 0, 1, 640, 480, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, &rgb[0]);

    glBegin(GL_TRIANGLE_FAN);
    glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
    glTexCoord2f(0, 0); glVertex3f(315,0,0);
    glTexCoord2f(1, 0); glVertex3f(630,0,0);
    glTexCoord2f(1, 1); glVertex3f(630,400,0);
    glTexCoord2f(0, 1); glVertex3f(315,400,0);
    glEnd();
}

void GLWidget::AstraDraw()
{

    if(viewStatus == 2)
    {
        //glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->rgb.data);
        // (Type of texture,Pyramid level - 0 is the top level,Internal colour format,width,height,Border width in pixels,Input image format,Image data type, image data)

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(315,0,0);
        glTexCoord2f(1, 1); glVertex3f(315,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();

        //glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->depthView.data);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(315,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(315,400,0);
        glEnd();
    }
    else if(viewStatus == 1)
    {
        //glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->depthView.data);

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();
    }
    else if(viewStatus == 0)
    {
        //glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,640,480,0,GL_BGR,GL_UNSIGNED_BYTE,astraDevice->rgb.data);
        // (Type of texture,Pyramid level - 0 is the top level,Internal colour format,width,height,Border width in pixels,Input image format,Image data type, image data)

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
        glTexCoord2f(0, 0); glVertex3f(0,0,0);
        glTexCoord2f(1, 0); glVertex3f(630,0,0);
        glTexCoord2f(1, 1); glVertex3f(630,400,0);
        glTexCoord2f(0, 1); glVertex3f(0,400,0);
        glEnd();

    }
}

void GLWidget::LaserDraw()
{
    int i;
    double x,y,z;
    
    glClear(GL_COLOR_BUFFER_BIT);
    glRotated(0,0,0,1);
    for(int i = 0; i < 729 ; i++){
        x = (laser->laserData[i].x*315/11.2)+(315/2)+125;
        y = (laser->laserData[i].y*315/11.2)+(315/2)+10;
        z = (laser->laserData[i].z);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);
        glPointSize(100);
        glVertex3f(x,y,z);
        glEnd();
    }
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::ChangeMod(int i)
{
    viewStatus = i;
    updateGL();
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool syncronize_devices = false;

void GLWidget::Syncronize(ros::AsyncSpinner *spin)
{
    syncronize_devices = true;
    
    ros::NodeHandle imuNH;
    ros::NodeHandle laserNH;
    SetStatus(0);

    imuDevice = new ImuSensor(imuNH);
    spin->start();
    imuOn = 1;

    laser = new LaserS(laserNH);
    spin->start();
    laserOn = 1;


    if(zedOn == 0)
        zedDevice.Init();
    if(zedDevice.getStatus() == 1){
        int k = zedDevice.StreamData();
        zedOn = 1;
    }
}

void GLWidget::Syncronize_OFF()
{
    syncronize_devices = false;
}

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GLWidget::saveFiles()
{
    if(zedOn == 1)
        zedDevice.cv2file();
    
    if(laserOn == 1)
        laser->SaveFile();

    if(imuOn == 1)
        imuDevice->SaveFile();
}



