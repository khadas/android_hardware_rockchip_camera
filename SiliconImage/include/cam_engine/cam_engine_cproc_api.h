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
/**
 * @cond    cam_engine_cproc
 *
 * @file    cam_engine_cproc_api.h
 *
 * @brief
 *
 *   Interface description of the CamEngine Color Processing.
 *
 *****************************************************************************/
/**
 *
 * @defgroup cam_engine_cproc_api CamEngine Color Processing API
 * @{
 *
 */

#ifndef __CAM_ENGINE_C_PROC_API_H__
#define __CAM_ENGINE_C_PROC_API_H__

#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif


/******************************************************************************/
/**
 * @brief   Enumeration type to configure the chrominace output range in the 
 *          CamEngine color processing unit.
 *
 * @note    same as CamerIcCprocChrominaceRangeOut_t, but we want to make 
 *          it independently from driver here to easily port software on 
 *          OS with kernel-/user-mode
 *
 *****************************************************************************/
typedef enum CamEngineCprocChrominanceRangeOut_e
{
    CAM_ENGINE_CPROC_CHROM_RANGE_OUT_INVALID       = 0,    /**< lower border (only for an internal evaluation) */
    CAM_ENGINE_CPROC_CHROM_RANGE_OUT_BT601         = 1,    /**< CbCr_out clipping range 16..240 according to ITU-R BT.601 standard */
    CAM_ENGINE_CPROC_CHROM_RANGE_OUT_FULL_RANGE    = 2,    /**< full UV_out clipping range 0..255 */
    CAM_ENGINE_CPROC_CHROM_RANGE_OUT_MAX                   /**< upper border (only for an internal evaluation) */
} CamEngineCprocChrominaceRangeOut_t;


/******************************************************************************/
/**
 * @brief   Enumeration type to configure the luminance output range in the 
 *          CamEngine color processing unit.
 *
 * @note    same as CamerIcCprocLuminanceRangeOut_t, but we want to make 
 *          it independently from driver here to easily port software on 
 *          OS with kernel-/user-mode
 *
 *****************************************************************************/
typedef enum CamEngineCprocLuminanceRangeOut_e
{
    CAM_ENGINE_CPROC_LUM_RANGE_OUT_INVALID         = 0,    /**< lower border (only for an internal evaluation) */
    CAM_ENGINE_CPROC_LUM_RANGE_OUT_BT601           = 1,    /**< Y_out clipping range 16..235; offset of 16 is added to Y_out according to ITU-R BT.601 standard */
    CAM_ENGINE_CPROC_LUM_RANGE_OUT_FULL_RANGE      = 2,    /**< Y_out clipping range 0..255; no offset is added to Y_out */
    CAM_ENGINE_CPROC_LUM_RANGE_OUT_MAX                     /**< upper border (only for an internal evaluation) */
} CamEngineCprocLuminanceRangeOut_t;


/******************************************************************************/
/**
 * @brief   Enumeration type to configure the luminance input range in the 
 *          CamEngine color processing unit.
 *
 * @note    same as CamerIcCprocLuminanceRangeIn_t, but we want to make 
 *          it independently from driver here to easily port software on 
 *          OS with kernel-/user-mode
 *
 *****************************************************************************/
typedef enum CamEngineCprocLuminanceRangeIn_e
{
    CAM_ENGINE_CPROC_LUM_RANGE_IN_INVALID          = 0,    /**< lower border (only for an internal evaluation) */
    CAM_ENGINE_CPROC_LUM_RANGE_IN_BT601            = 1,    /**< Y_in range 64..940 according to ITU-R BT.601 standard; offset of 64 will be subtracted from Y_in */
    CAM_ENGINE_CPROC_LUM_RANGE_IN_FULL_RANGE       = 2,    /**< Y_in full range 0..1023; no offset will be subtracted from Y_in */
    CAM_ENGINE_CPROC_LUM_RANGE_IN_MAX                      /**< upper border (only for an internal evaluation) */
} CamEngineCprocLuminanceRangeIn_t;


/******************************************************************************/
/**
 * @brief   Configuration Structure to configure the color processing module
 *
 * @note    This structure needs to be converted to driver structure
 *
 *****************************************************************************/
typedef struct CamEngineCprocConfig_s
{
    CamEngineCprocChrominaceRangeOut_t  ChromaOut;      /**< configuration of color processing chrominance pixel clipping range at output */
    CamEngineCprocLuminanceRangeOut_t   LumaOut;        /**< configuration of color processing luminance input range (offset processing) */
    CamEngineCprocLuminanceRangeIn_t    LumaIn;         /**< configuration of color processing luminance output clipping range */

    float                               contrast;       /**< contrast value to initially set */
    int8_t                              brightness;     /**< brightness value to initially set */
    float                               saturation;     /**< saturation value to initially set */
    float                               hue;            /**< hue value to initially set */
} CamEngineCprocConfig_t;


/*****************************************************************************/
/**
 * @brief   This function returns RET_SUCCESS if the color processing 
 *          module available in hardware.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         CPROC available
 * @retval  RET_WRONG_STATE     invalid state (i.e. cam-eninge is not running)
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocIsAvailable
(
    CamEngineHandle_t   hCamEngine
);


/*****************************************************************************/
/**
 * @brief   This function enables the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   pConfig             configuration of color processing
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_WRONG_STATE     invalid state (i.e. cam-eninge is not running)
 * @retval  RET_INVALID_PARM    invalid parameter (i.e. a parameter is NULL)
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocEnable
(
    CamEngineHandle_t               hCamEngine,
    CamEngineCprocConfig_t * const  pConfig
);


/*****************************************************************************/
/**
 * @brief   This function disables the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_WRONG_STATE     invalid state (i.e. cam-eninge is not running)
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocDisable
(
    CamEngineHandle_t   hCamEngine
);


/*****************************************************************************/
/**
 * @brief   This function returns the color processing status.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   pRunning            BOOL_TRUE: running, BOOL_FALSE: stopped
 * @param   pConfig             configuration of color processing
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_INVALID_PARM    invalid parameter (i.e. a parameter is NULL)
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocStatus
(
    CamEngineHandle_t               hCamEngine,
    bool_t * const                  pRunning,
    CamEngineCprocConfig_t * const  pConfig
);


/*****************************************************************************/
/**
 * @brief   This function sets the contrast in the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   contrast            contrast value to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocSetContrast
(
    CamEngineHandle_t   hCamEngine,
    float const         contrast
);


/*****************************************************************************/
/**
 * @brief   This function sets the brightness in the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   brightness          brightness value to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocSetBrightness
(
    CamEngineHandle_t   hCamEngine,
    int8_t const        brightness
);


/*****************************************************************************/
/**
 * @brief   This function sets the saturation in the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   saturation          saturation value to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocSetSaturation
(
    CamEngineHandle_t   hCamEngine,
    float const         saturation
);


/*****************************************************************************/
/**
 * @brief   This function sets the hue in the color processing.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   hue                 hue value to set
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTSUPP         CPROC module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineCprocSetHue
(
    CamEngineHandle_t   hCamEngine,
    float const         hue
);


#ifdef __cplusplus
}
#endif

/* @} cam_engine_cproc_api */

#endif /* __CAM_ENGINE_C_PROC_API_H__ */

