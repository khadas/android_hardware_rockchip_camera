/******************************************************************************
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
 * Modification based on code covered by the License (the "License").
 * You may not use this software except in compliance with the License.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHIP DISCLAIMS 
 * ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH RESPECT TO SUCH SOFTWARE, 
 * WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, INCLUDING WITHOUT LIMITATION,
 * ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY
 * QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Rockchip shall not be liable to make any corrections to this software or to 
 * provide any support or assistance with respect to it.
 *
 *****************************************************************************/

/**
* @file SensorListener.h
*
* This defines API for camerahal to get sensor events
*
*/

#ifndef ANDROID_CAMERA_HARDWARE_SENSOR_LISTENER_H
#define ANDROID_CAMERA_HARDWARE_SENSOR_LISTENER_H

#include <android/sensor.h>
#include <gui/Sensor.h>
#include <gui/SensorManager.h>
#include <gui/SensorEventQueue.h>
#include <utils/Looper.h>

namespace android {

/**
 * SensorListner class - Registers with sensor manager to get sensor events
 */

typedef void (*orientation_callback_t) (uint32_t orientation, uint32_t tilt, void* cookie);

class SensorLooperThread : public Thread {
    public:
        SensorLooperThread(Looper* looper)
            : Thread(false) {
            mLooper = sp<Looper>(looper);
        }
        ~SensorLooperThread() {
            mLooper.clear();
        }

        virtual bool threadLoop() {
            int32_t ret = mLooper->pollOnce(-1);
            return true;
        }

        // force looper wake up
        void wake() {
            mLooper->wake();
        }
    private:
        sp<Looper> mLooper;
};


class SensorListener : public RefBase
{
/* public - types */
public:
    typedef enum {
        SENSOR_ACCELEROMETER  = 1 << 0,
        SENSOR_MAGNETIC_FIELD = 1 << 1,
        SENSOR_GYROSCOPE      = 1 << 2,
        SENSOR_LIGHT          = 1 << 3,
        SENSOR_PROXIMITY      = 1 << 4,
        SENSOR_ORIENTATION    = 1 << 5,
    } sensor_type_t;
/* public - functions */
public:
    SensorListener();
    ~SensorListener();
    status_t initialize();
    void setCallbacks(orientation_callback_t orientation_cb, void *cookie);
    void enableSensor(sensor_type_t type);
    void disableSensor(sensor_type_t type);
    void handleOrientation(uint32_t orientation, uint32_t tilt);
/* public - member variables */
public:
    sp<SensorEventQueue> mSensorEventQueue;
/* private - member variables */
private:
    int sensorsEnabled;
    orientation_callback_t mOrientationCb;
    void *mCbCookie;
    sp<Looper> mLooper;
    sp<SensorLooperThread> mSensorLooperThread;
    Mutex mLock;
};

}

#endif
