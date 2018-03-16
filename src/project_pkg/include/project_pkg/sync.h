#ifndef _SYNC_H_
#define _SYNC_H_

#include <pthread.h>
#include <iostream>

extern pthread_mutex_t mutex,mutexL,mutexI,mutexZ;
extern pthread_cond_t cond1,cond2,cond3,cond4;

extern bool syncronize_devices;
extern bool zed_new_pointcloud;

#endif
