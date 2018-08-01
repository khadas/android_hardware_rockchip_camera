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
#ifndef ANDROID_HARDWARE_CAMERA_HARDWARE_MODULE_H
#define ANDROID_HARDWARE_CAMERA_HARDWARE_MODULE_H
#include <linux/videodev2.h>
#include <utils/threads.h>
#include "CameraHal_board_xml_parse.h"

using namespace android;

#define CONFIG_AUTO_DETECT_FRAMERATE    0
#if CONFIG_AUTO_DETECT_FRAMERATE
#define CAMERA_DEFAULT_PREVIEW_FPS_MIN    8000        //8 fps
#define CAMERA_DEFAULT_PREVIEW_FPS_MAX    15000
#endif
#define CAMERAS_SUPPORT_MAX             8
#if defined(TARGET_RK3399)
    #define CAMERAS_SUPPORTED_SIMUL_MAX     8
#else
    #define CAMERAS_SUPPORTED_SIMUL_MAX     1
#endif
#define CAMERA_DEVICE_NAME              "/dev/video"
#define CAMERA_MODULE_NAME              "RK29_ICS_CameraHal_Module"

typedef struct rk_cam_info_s {
    char device_path[30];
    char driver[16];
    unsigned int version;
    struct camera_info facing_info;
    struct v4l2_frmivalenum fival_list[10];   // default preview framerate, dc preview framerate, dv preview framerate(highe quality/low quality)   
    struct rk_cam_total_info *pcam_total_info;
}rk_cam_info_t;


typedef struct rk_camera_device {
    camera_device_t base;   
    int cameraid;
} rk_camera_device_t;

#if CONFIG_AUTO_DETECT_FRAMERATE 
int camera_famerate_detect_loop(void);

class CameraFpsDetectThread : public Thread {        
public:
    CameraFpsDetectThread()
        : Thread(false){ }

    virtual bool threadLoop() {
        camera_famerate_detect_loop();
        return false;
    }
};
#endif

#endif
