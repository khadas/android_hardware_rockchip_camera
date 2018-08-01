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
 * @cond    cam_engine_simp
 *
 * @file    cam_engine_simp_api.h
 *
 * @brief
 *
 *   Interface description of the CamEngine.
 *
 *****************************************************************************/
/**
 * @defgroup cam_engine_simp CamEngine Super Impose definitions
 * @{
 *
 */
#ifndef __CAM_ENGINE_SIMP_API_H__
#define __CAM_ENGINE_SIMP_API_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <common/picture_buffer.h>

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/**
 * @brief   Enumeration type to configure the super impose working mode.
 *
 *****************************************************************************/
typedef enum CamEngineSimpMode_e
{
    CAM_ENGINE_SIMP_MODE_INVALID            = 0,        /**< lower border (only for an internal evaluation) */
    CAM_ENGINE_SIMP_MODE_OVERLAY            = 1,        /**< overlay mode */
    CAM_ENGINE_SIMP_MODE_KEYCOLORING        = 2,        /**< keycoloring mode */
    CAM_ENGINE_SIMP_MODE_MAX                            /**< upper border (only for an internal evaluation) */
} CamEngineSimpMode_t;


/******************************************************************************/
/**
 * @brief   Structure to configure the super impose module
 *
 * @note    This structure needs to be converted to driver structure
 *
 *****************************************************************************/
typedef struct CamEngineSimpConfig_s
{
    CamEngineSimpMode_t     Mode;

    union SimpModeConfig_u
    {
        struct Overlay_s
        {
            uint32_t OffsetX;
            uint32_t OffsetY;
        } Overlay;

        struct KeyColoring_s
        {
            uint8_t Y;
            uint8_t Cb;
            uint8_t Cr;
        } KeyColoring;
    } SimpModeConfig;

    PicBufMetaData_t      *pPicBuffer;

} CamEngineSimpConfig_t;

/*****************************************************************************/
/**
 * @brief   This function enables the super impose.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   pConfig             configuration off color processing
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_WRONG_CONFIG    image effects isn't configured
 * @retval  RET_NOTAVAILABLE    module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineEnableSimp
(
    CamEngineHandle_t       hCamEngine,
    CamEngineSimpConfig_t   *pConfig
);

/*****************************************************************************/
/**
 * @brief   This function disables the super impose.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTAVAILABLE    module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineDisableSimp
(
    CamEngineHandle_t   hCamEngine
);

#ifdef __cplusplus
}
#endif

/* @} cam_engine_simp */

#endif /* __CAM_ENGINE_SIMP_API_H__ */

