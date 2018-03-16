#include "../include/project_pkg/Imu.h"

ImuSensor::ImuSensor(ros::NodeHandle n) :
    n_(n)
{
    imu_sub = n_.subscribe("/imu/data", 1, &ImuSensor::Callback, this);
}

void ImuSensor::Callback (const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    pthread_mutex_lock(&mutexI);
    if(syncronize_devices == true)
    {
        std::cout << std::endl << "Imu waiting!" << std::endl << std::endl;
        pthread_cond_wait(&cond2,&mutexI);
    }
    dadosImu.orientation.x = imu_msg->orientation.x;
    dadosImu.orientation.y = imu_msg->orientation.y;
    dadosImu.orientation.z = imu_msg->orientation.z;
    dadosImu.orientation.w = imu_msg->orientation.w;
    dadosImu.angular_vel.x = imu_msg->angular_velocity.x;
    dadosImu.angular_vel.y = imu_msg->angular_velocity.y;
    dadosImu.angular_vel.z = imu_msg->angular_velocity.z;
    dadosImu.linear_vel.x  = imu_msg->linear_acceleration.x;
    dadosImu.linear_vel.y  = imu_msg->linear_acceleration.y;
    dadosImu.linear_vel.z  = imu_msg->linear_acceleration.z;

    printf("\n Orientação-> x:%f  y:%f  z:%f  w:%f",dadosImu.orientation.x,dadosImu.orientation.y,dadosImu.orientation.z,dadosImu.orientation.w);
    if(syncronize_devices == true)
    {
        pthread_cond_signal(&cond3);
    }
pthread_mutex_unlock(&mutexI);
}

void ImuSensor::SaveFile(){
    int k;
    k = pthread_create(&fileThread,NULL,fileThreadFunc,(void*)this);
    if(k)
	std::cout << std::endl <<"Falha imu" << std::endl; 
}

void* ImuSensor::fileThreadFunc(void* arg)
{
    ImuSensor* I;
    I = (ImuSensor*)arg;
    std::ofstream arq;
    int i;

    arq.open("imu_data.txt");				  

              			 
        arq << I->dadosImu.orientation.x << "  ";
        arq << I->dadosImu.orientation.y << "  ";
        arq << I->dadosImu.orientation.z << "  ";
        arq << I->dadosImu.orientation.w << "  ";			 
        arq << std::endl;
    
    std::cout << std::endl << std::endl << "Imu File Created!" << std::endl << std::endl;
    arq.close();
    pthread_exit(NULL);
}
