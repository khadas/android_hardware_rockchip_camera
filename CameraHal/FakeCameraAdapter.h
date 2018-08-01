/******************************************************************************
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd. All rights reserved.
 * BY DOWNLOADING, INSTALLING, COPYING, SAVING OR OTHERWISE USING THIS SOFTWARE,
 * YOU ACKNOWLEDGE THAT YOU AGREE THE SOFTWARE RECEIVED FORM ROCKCHIP IS PROVIDED
 * TO YOU ON AN "AS IS" BASIS and ROCKCHP DISCLAIMS ANY AND ALL WARRANTIES AND
 * REPRESENTATIONS WITH RESPECT TO SUCH FILE, WHETHER EXPRESS, IMPLIED, STATUTORY
 * OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE,
 * NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR
 * A PARTICULAR PURPOSE. 
 * Rockchip hereby grants to you a limited, non-exclusive, non-sublicensable and 
 * non-transferable license (a) to install, save and use the Software; (b) to copy 
 * and distribute the Software in binary code format only. 
 * Except as expressively authorized by Rockchip in writing, you may NOT: (a) distribute 
 * the Software in source code; (b) distribute on a standalone basis but you may distribute 
 * the Software in conjunction with platforms incorporating Rockchip integrated circuits;
 * (c) modify the Software in whole or part;(d) decompile, reverse-engineer, dissemble,
 * or attempt to derive any source code from the Software;(e) remove or obscure any copyright,
 * patent, or trademark statement or notices contained in the Software.
 *
 *****************************************************************************/
#ifndef ANDROID_HARDWARE_CAMERA_FAKE_HARDWARE_H
#define ANDROID_HARDWARE_CAMERA_FAKE_HARDWARE_H

//usb camera adapter
#include "CameraHal.h"

#define CAMERAHAL_FAKECAMERA_WIDTH_KEY "sys_graphic.cam_hal.fakewidth"
#define CAMERAHAL_FAKECAMERA_HEIGHT_KEY "sys_graphic.cam_hal.fakeheight"
#define CAMERAHAL_FAKECAMERA_DIR_KEY "sys_graphic.cam_hal.fakedir"
#define CAMERAHAL_FAKECAMERA_DIR_VALUE "/mnt/sdcard/Screencapture/camera"

namespace android{


class CameraFakeAdapter: public CameraAdapter
{
public:
    CameraFakeAdapter(int cameraId);
    virtual ~CameraFakeAdapter();
//    virtual status_t startPreview(int preview_w,int preview_h,int w, int h, int fmt,bool is_capture);
//    virtual status_t stopPreview();
//    virtual int getCurPreviewState(int *drv_w,int *drv_h);

    virtual int setParameters(const CameraParameters &params_set);
    virtual void initDefaultParameters(int camFd);
    virtual int getFrame(FramInfo_s** frame); 
	virtual void dump(int cameraId);
    
private:
    //talk to driver
    virtual int cameraCreate(int cameraId);
    virtual int cameraDestroy();
    virtual int cameraSetSize(int w, int h, int fmt, bool is_capture); 
    virtual int adapterReturnFrame(long index,int cmd);
    virtual int cameraStream(bool on);
    virtual int cameraStart();
    virtual int cameraStop();
    
};

}
#endif

