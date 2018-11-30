/******************************************************************************
 *
 * The copyright in this software is owned by Rockchip and/or its licensors.
 * This software is made available subject to the conditions of the license 
 * terms to be determined and negotiated by Rockchip and you.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHP AND/OR 
 * ITS LICENSORS DISCLAIMS ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH 
 * RESPECT TO SUCH SOFTWARE, WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, 
 * INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, 
 * MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Except as expressively authorized by Rockchip and/or its licensors, you may not 
 * (a) disclose, distribute, sell, sub-license, or transfer this software to any third party, 
 * in whole or part; (b) modify this software, in whole or part; (c) decompile, reverse-engineer, 
 * dissemble, or attempt to derive any source code from the software.
 *
 *****************************************************************************/
#ifndef __AWB_V11_H__
#define __AWB_V11_H__

/**
 * @file awb_v11.h
 *
 * @brief
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup AWBM Auto white Balance Module
 * @{
 *
 */
#include <ebase/types.h>
#include <common/return_codes.h>
#include <common/cam_types.h>

#include <isi/isi_iss.h>
#include <isi/isi.h>

#include <cameric_drv/cameric_drv_api.h>
#include <cameric_drv/cameric_isp_awb_drv_api.h>
#include <cameric_drv/cameric_isp_lsc_drv_api.h>
#include <cameric_drv/cameric_isp_hist_drv_api.h>

#include <cam_calibdb/cam_calibdb_api.h>
#include "awb_common.h"
#define ABS_DIFF( a, b )     ( (a > b) ? (a-b) : (b-a) )

#ifdef __cplusplus
extern "C"
{
#endif


/*****************************************************************************/
/**
 * @brief   The function releases/frees the Auto White Balance module.
 *
 * @param   handle      AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbRelease
(
    AwbHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This function configures the Auto White Balance Module.
 *
 * @param   handle      AWB instance handle
 * @param   pConfig
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT V11_AwbConfigure
(
    AwbHandle_t handle,
    AwbConfig_t *pConfig
);



/*****************************************************************************/
/**
 * @brief   This function re-configures the Auto White Balance Module
 *          after e.g. resolution change
 *
 * @param   handle      AWB instance handle
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT V11_AwbReConfigure
(
    AwbHandle_t handle,
    AwbConfig_t *pConfig
);



/*****************************************************************************/
/**
 * @brief   The function
 *
 * @param   handle      AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbStart
(
    AwbHandle_t         handle,
    const AwbMode_t     mode,
    const uint32_t      idx
);



/*****************************************************************************/
/**
 * @brief   The function stops the auto white balance.
 *
 * @param   handle      AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbStop
(
    AwbHandle_t         handle
);



/*****************************************************************************/
/**
 * @brief   The function resets the auto white balance module.
 *
 * @param   handle      AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbReset
(
    AwbHandle_t     handle
);



/*****************************************************************************/
/**
 * @brief   The function sets/changes AWB processing flags
 *
 * @param   handle      AWB instance handle
 * @param   Flags       AWB processing flags to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbSetFlags
(
    AwbHandle_t     handle,
    const uint32_t  Flags
);



/*****************************************************************************/
/**
 * @brief   The function sets the histogram
 *
 * @param   handle      AWB instance handle
 * @param   bins        histogramm
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbSetHistogram
(
    AwbHandle_t         handle,
    CamerIcHistBins_t   *bins
);




/*****************************************************************************/
/**
 * @brief   The function returns AWB processing flags
 *
 * @param   handle      AWB instance handle
 * @param   pFlags      pointer to store current AWB processing flags
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbGetFlags
(
    AwbHandle_t handle,
    uint32_t    *pFlags
);



/*****************************************************************************/
/**
 * @brief   The function returns current status values of the AWB.
 *
 * @param   handle      AWB instance handle
 * @param   pRunning    pointer to return current run-state of AWB module
 * @param   pMode       pointer to return current operation mode of AWB module
 * @param   pIlluIdx    pointer to return current start profile index
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbStatus
(
    AwbHandle_t     handle,
    bool_t          *pRunning,      /**< BOOL_TRUE: running, BOOL_FALSE: stopped */
    AwbMode_t       *pMode,
    uint32_t        *pIlluIdx,
    AwbRgProj_t     *pRgProj
);


/******************************************************************************
 * AwbSettled()
 *****************************************************************************/
RESULT V11_AwbSettled
(
    AwbHandle_t handle,
    bool_t      *pSettled,
    uint32_t     *pDNoWhitePixel
);

/*****************************************************************************/
/**
 * @brief   The function starts AWB processing
 *
 * @param   handle      AWB instance handle
 * @param   pMeasResult pointer tu current AWB measuring data from hardware
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbProcessFrame
(
    AwbHandle_t                         handle,
    const CamerIcAwbMeasuringResult_t   *pMeasResult,
    const float                         fGain,
    const float                         fIntegrationTime
);



/*****************************************************************************/
/**
 * @brief   The function
 *
 * @param   handle  AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbTryLock
(
    AwbHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   The function
 *
 * @param   handle  AWB instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT V11_AwbUnLock
(
    AwbHandle_t handle
);

RESULT V11_AwbGetGainParam
(
    AwbHandle_t handle,
    float *f_RgProj, 
    float *f_s, 
    float *f_s_Max1, 
    float *f_s_Max2, 
    float *f_Bg1, 
    float *f_Rg1, 
    float *f_Bg2, 
    float *f_Rg2
);


RESULT V11_AwbGetIlluEstInfo
(
    AwbHandle_t handle,
    float *ExpPriorIn,
    float *ExpPriorOut,
    char (*name)[20],
    float likehood[],
    float wight[],
    int *curIdx,
    int *region,
    int *count
);


RESULT V11_AwbGetTemperature
(
    AwbHandle_t handle,    
    float *ct
);

RESULT V11_AwbCalcWBgainbyCT
(
    AwbHandle_t handle,    
    float ct,
    float * Rg,
    float *Bg
);

//----------------------------------------------------
// for uvc command
RESULT V11_AwbSetLscProfile(AwbHandle_t handle, CamLscProfile_t *pLscProfile);

RESULT V11_AwbGetLscProfile(AwbHandle_t handle, CamLscProfile_t *pLscProfile);

RESULT V11_AwbGetLscInfo
(
    AwbHandle_t handle,
    int8_t *aCCDnName,
    int8_t *aCCUpName,
    CamLscMatrix_t *lscMatrix
);


RESULT V11_AwbGetCcmInfo
(
    AwbHandle_t handle,
    int8_t *aCCDnName,
    int8_t *aCCUpName,
    Cam3x3FloatMatrix_t *ccMatrix,
    Cam1x3FloatMatrix_t *ccOffset
);

RESULT V11_AwbSetCcmInfo
(
    AwbHandle_t handle,
    int8_t *name,
    Cam3x3FloatMatrix_t *ccMatrix,
    Cam1x3FloatMatrix_t *ccOffset
);


RESULT V11_AwbSetForceIllumination
(
    AwbHandle_t handle,
    bool_t forceIlluFlag,
    char *illName
);


RESULT V11_AwbSetForceWbGain
(
    AwbHandle_t handle,
    bool_t forceWbGainFlag,
    float fRGain,
    float fGrGain,
    float fGbGain,
    float fBGain
);


RESULT V11_AwbGetForceWbGain
(
    AwbHandle_t handle,
    bool_t* forceWbGainFlag,
    float* fRGain,
    float* fGrGain,
    float* fGbGain,
    float* fBGain
);


RESULT V11_AwbGetForceIllumination
(
    AwbHandle_t handle,
    bool_t* forceIlluFlag,
    char* illName
);


RESULT V11_AwbSetWhitePoint
(
    AwbHandle_t handle,
    int8_t measMode,
    CamCalibAwb_V11_Global_t *pAwbWpSet,
    CamAwbMeasResult_t *pAwbmeas
);


RESULT V11_AwbGetWhitePoint
(
    AwbHandle_t handle,
    uint8_t *awb_mode,
    CamAwbWpGet_t *pAwbWpGet
);


RESULT V11_AwbSetCurve
(
    AwbHandle_t   handle,
    CamAwbCurve_t *pAwbCurve
);


RESULT V11_AwbGetCurve
(
    AwbHandle_t   handle,
    CamAwbCurve_t *pAwbCurve
);


RESULT V11_AwbSetRefWbGain
(
    AwbHandle_t handle,
    float fRGain,
    float fGrGain,
    float fGbGain,
    float fBGain,
    char *illName
);


RESULT V11_AwbGetRefWbGain
(
    AwbHandle_t handle,
    float* fRGain,
    float* fGrGain,
    float* fGbGain,
    float* fBGain,
    char* illName
);


#ifdef __cplusplus
}
#endif

/* @} AWBM */


#endif /* __AWB_V11_H__*/
