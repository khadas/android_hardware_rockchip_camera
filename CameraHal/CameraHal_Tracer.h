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
#ifndef __CAMERAHAL_TRACER_H__
#define __CAMERAHAL_TRACER_H__
#ifdef __cplusplus
extern "C"
{
#endif
#include <utils/Log.h>

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG                                  "CameraHal"          
#endif

#ifdef CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY
#undef CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY
#define CAMERAHAL_TRACE_LEVEL_PROPERTY_KEY       "sys_graphic.camhal.trace"
#endif

#ifdef ALOGV
#define LOGV(msg,...)                 ALOGV("%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__)
#endif

#ifdef ALOGI
#define LOGI(msg,...)                 ALOGI("%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__)
#endif

#ifdef ALOGW
#define LOGW(msg,...)                 ALOGW("%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__)
#endif

#ifdef ALOGE
#define TRACE_E(msg,...)              ALOGE("%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__)
#define LOGE(msg,...)                 ALOGE("%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__)
#endif

#ifdef ALOGD_IF
#define TRACE_D(level,msg,...)       ALOGD_IF((getTracerLevel()&level)==level, "%s(%d): " msg ,__FUNCTION__,__LINE__,##__VA_ARGS__);
#endif



#define LOG1(...)            TRACE_D(1, ##__VA_ARGS__)
#define LOG2(...)            TRACE_D(2, ##__VA_ARGS__)
#define LOGD(...)            TRACE_D(0, ##__VA_ARGS__)


#define LOG_FUNCTION_NAME           LOG1(" enter");
#define LOG_FUNCTION_NAME_EXIT      LOG1(" exit");


int getTracerLevel(void);
int setTracerLevel(int new_level);

#ifdef __cplusplus
}
#endif
#endif
