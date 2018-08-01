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
#ifndef ANDROID_HARDWARE_CAMERA_ISPTUNNING_H
#define ANDROID_HARDWARE_CAMERA_ISPTUNNING_H
#include <utils/Vector.h>
#include "cam_api/camdevice.h"
#include "oslayer/oslayer.h"

#define RK_ISP_TUNNING_FILE_PATH "/data/capcmd.xml"

namespace android {


typedef enum EXPOSUSE_MODE_ENUM{
    EXPOSUSE_MODE_MANUAL = 1,
    EXPOSUSE_MODE_AUTO  =2,
    EXPOSUSE_MODE_INVALID = 3
}EXPOSUSE_MODE_e;

typedef enum WHITEBALANCE_MODE_ENUM{
    WHITEBALANCE_MODE_MANUAL = 1,
    WHITEBALANCE_MODE_AUTO  =2,
    WHITEBALANCE_MODE_INVALID  =3,
}WHITEBALANCE_MODE_e;

typedef struct ispTuneTaskInfo_t{
//from xml
    bool mTuneEnable;
    int mTuneWidth;
    int mTuneHeight;
    int mTuneFmt;
    int mTunePicNum;
    struct{
        EXPOSUSE_MODE_e exposuseMode;
        float    integrationTime;
        float    gain;
        float    integrationTimeStep;
        float    gainStep;
        int    minRaw;
        int    maxRaw;
        int    threshold;
        bool   aeRound;
        int    number;
    }mExpose;

    struct{
        WHITEBALANCE_MODE_e whiteBalanceMode;
        char     illumination[10];
        char     cc_matrix[15];
        char     cc_offset[10];
        char     rggb_gain[10];
        
    }mWhiteBalance;
    
    bool mWdrEnable;
    bool mCacEnable;
    bool mGammarEnable;
    bool mLscEnable;
    bool mDpccEnable;
    bool mBlsEnable;
    bool mAdpfEnable;
    bool mAvsEnable;
    bool mAfEnable;

//from ..
    bool mForceRGBOut;
    unsigned long y_addr;
    unsigned long uv_addr;
}ispTuneTaskInfo_s;


class CameraIspTunning
{
public:
    CameraIspTunning();
    ~CameraIspTunning();
    static CameraIspTunning* createInstance();
    static void StartElementHandler(void *userData, const char *name, const char **atts);
    static int ispTuneDesiredExp(long raw_ddr,int width,int height,int min_raw,int max_raw,int threshold);

    static int ispTuneStoreBufferRAW
    (
        ispTuneTaskInfo_s    *pIspTuneTaskInfo,
        FILE                *pFile,
        MediaBuffer_t       *pBuffer,
        bool              putHeader,
        bool              is16bit
    );

    static int ispTuneStoreBufferYUV422Semi
    (
        ispTuneTaskInfo_s    *pIspTuneTaskInfo,
        FILE                *pFile,
        MediaBuffer_t       *pBuffer,
        bool              putHeader
    );

    static int ispTuneStoreBuffer
    (
        ispTuneTaskInfo_s    *pIspTuneTaskInfo,
        MediaBuffer_t       *pBuffer,
        char     *szNmae,
        int      index     
    );
private:
    static void ConvertYCbCr444combToRGBcomb
    (
        uint8_t     *pYCbCr444,
        uint32_t    PlaneSizePixel
    );

    
public:
    Vector<ispTuneTaskInfo_s*> mTuneInfoVector;
    ispTuneTaskInfo_s* mCurTuneTask;
	int mTuneTaskcount;
	int mCurTunIndex;
    float mCurIntegrationTime;
    float mCurGain;
    int mCurAeRoundNum;
};

};

#endif

