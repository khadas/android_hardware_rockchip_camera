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
#ifndef ANDROID_HARDWARE_COMMONTYPE_H
#define ANDROID_HARDWARE_COMMONTYPE_H

/*****************************************************************************/
/**
 *          Picmergestatus_t
 *
 * @brief   picture merge status
 *
 *****************************************************************************/
typedef enum Picmergestatus_e
{
    PIC_ALLOC_BUFF  = 0,
    PIC_ALLOC_RETURN = 1,
    PIC_EVEN_SAVE = 2,
    PIC_ODD_SAVE = 3,
} Picmergestatus_t;

//目前只有CameraAdapter为frame provider，display及event类消费完frame后，可通过该类
//将buffer返回给CameraAdapter,CameraAdapter实现该接口。

//描述帧信息，如width，height，bufaddr，fmt，便于帧消费类收到帧后做后续处理。
//包括zoom的信息
typedef struct FramInfo
{
    ulong_t phy_addr;
    ulong_t vir_addr;
    int frame_width;
    int frame_height;
    ulong_t frame_index;
    int frame_fmt;
    int zoom_value;
    ulong_t used_flag;
    int frame_size;
    void* res;
    bool vir_addr_valid;
    bool is_even_field;
    Picmergestatus_t merge_status;
}FramInfo_s;

typedef int (*func_displayCBForIsp)(void* frameinfo,void* cookie);

#endif
