#include "../include/project_pkg/Zed.h"
#include <pcl/common/common_headers.h>

bool zed_new_pointcloud = false;

ZedDevice::ZedDevice():
    status(0),
    cont(0)
{

}
ZedDevice::~ZedDevice(){
    Stop();
}

void ZedDevice::Init(){
    errorCode.clear();
    sl::InitParameters init_params;
    init_params.sdk_verbose = true;
    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.camera_resolution = sl::RESOLUTION_HD720;

    sl::ERROR_CODE err = zed.open(init_params);
    errorCode.append(sl::errorCode2str(err));
    if (err == sl::SUCCESS) {
        runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;
        status = 1;
        printf("\nZED sucefull connect!");
    }
    else
    {
        std::cout << std::endl << "Unable to init the ZED: " << errorCode << std::endl;
    }
    
}

int ZedDevice::StreamData(){
    int k;
    k = pthread_create(&streamThread,NULL,streamFunc,(void*)this);
    return k;
}

void* ZedDevice::streamFunc(void* arg)
{
    bool first = true;
    ZedDevice* zedDevice;
    zedDevice = (ZedDevice*)arg;
    while(zedDevice->zed.grab(zedDevice->runtime_parameters) == 0){
        
        if(syncronize_devices == true)
        {
            pthread_mutex_lock(&mutexZ);
            if (!first){
                //    pthread_cond_wait(&cond3,&mutex);
            }
        }
        printf("\nGet ZED Data\n");
        if (zedDevice->zed.retrieveImage(zedDevice->rgb, sl::VIEW_LEFT, sl::MEM_CPU) == sl::SUCCESS) {
            zedDevice->convertSl2Cv(0,zedDevice->rgb);
        }
        if (zedDevice->zed.retrieveImage(zedDevice->depth, sl::VIEW_DEPTH, sl::MEM_CPU) == sl::SUCCESS) {
            zedDevice->convertSl2Cv(1,zedDevice->depth);
        }

        if (zedDevice->zed.retrieveMeasure(zedDevice->pointCloud,sl::MEASURE_XYZRGBA) == sl::SUCCESS)
        {
            zedDevice->convertSl2Cv(2,zedDevice->pointCloud);
            zedDevice->dataPC = zedDevice->pointCloud.getPtr<float>();
            zed_new_pointcloud = true;
        }
        if(syncronize_devices == true)
        {
            pthread_cond_signal(&cond1);
            first = false;
            pthread_mutex_unlock(&mutexZ);
        }
    }
}


void ZedDevice::Stop(){
    status = 0;
    pthread_exit(&streamThread);
    zed.close();
}

void ZedDevice::convertSl2Cv(int i,sl::Mat &input){
    int cv_type = -1;
    switch (input.getDataType()) {
    case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
    case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
    case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
    case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
    case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
    case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
    case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
    case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
    default: break;
    }
    if(i == 0)
        rgbZ = cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
    else if(i == 1)
        depthView = cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
    else
        pcd = cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU)); 

   // std::cout << std::endl << std::endl << std::endl << "Teste " << rgbZ.at<uint8_t>(640,360) << std::endl << std::endl << std::endl;
}

int ZedDevice::getStatus(){
    return status;
}

void ZedDevice::SaveFile(){
    int k;
    k = pthread_create(&fileThread,NULL,fileThreadFunc,(void*)this);
    if(k)
        std::cout << std::endl <<"Falha zed" << std::endl;
}

void* ZedDevice::fileThreadFunc(void* arg)
{
    ZedDevice* Z;
    Z = (ZedDevice*)arg;
    std::ofstream arq,arq2;
    int i,j;

    arq.open("zed_depth_data.txt");
    arq2.open("zed_rgb_data.txt");

    for (i = 0 ; i < 1280; i++) {
        for(j = 0; j < 720; j++){
            arq << Z->depthView.at<float>(i,j)<<"  ";
            arq2 << Z->rgbZ.at<float>(i,j)<<"  ";
        }
        arq << std::endl;
        arq2 << std::endl;
    }
    std::cout << std::endl << std::endl << "Zed Files Created!" << std::endl << std::endl;

}

void ZedDevice::cv2file()
{
    CvMat cm = rgbZ;
    cvSave("zed_rgb.xml",&cm);
    CvMat cm2 = depthView;
    cvSave("zed_depth.xml",&cm2);
    CvMat cm3 = pcd;
    cvSave("zed_pcd.xml",&cm3);
    std::cout << "Arquivos salvos!" << std::endl << std::endl << std::endl;
}

float convertColor(float colorIn){
    uint32_t color_uint = *(uint32_t *)&colorIn;
    unsigned char *color_uchar = (unsigned char *)&color_uint;
    color_uint = ((uint32_t)color_uchar[0] << 16 | (uint32_t)color_uchar[1] << 8 | (uint32_t)color_uchar[2]);
    return *reinterpret_cast<float *>(&color_uint);
}
