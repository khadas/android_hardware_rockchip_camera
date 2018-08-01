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
 * @cond    cam_engine_ie
 *
 * @file    cam_engine_imgeffects_api.h
 *
 * @brief
 *
 *   Interface description of the CamEngine Image Effects.
 *
 *****************************************************************************/
/**
 *
 * @defgroup cam_engine_imgeffects_api CamEngine Image Effects API
 * @{
 *
 */

#ifndef __CAM_ENGINE_IMG_EFFECTS_API_H__
#define __CAM_ENGINE_IMG_EFFECTS_API_H__

#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif

//FIXME
#include <cameric_drv/cameric_ie_drv_api.h>


/*****************************************************************************/
/**
 * @brief   This function enables the image effects.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_WRONG_CONFIG    image effects isn't configured
 * @retval  RET_NOTAVAILABLE    module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineEnableImageEffect
(
    CamEngineHandle_t hCamEngine,
    CamerIcIeConfig_t *pConfig
);


/*****************************************************************************/
/**
 * @brief   This function disables the image effects.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_NOTAVAILABLE    module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineDisableImageEffect
(
    CamEngineHandle_t hCamEngine
);


RESULT CamEngineImageEffectSetTintCb
(
    CamEngineHandle_t   hCamEngine,
    const uint8_t       tint
);


RESULT CamEngineImageEffectSetTintCr
(
    CamEngineHandle_t   hCamEngine,
    const uint8_t       tint
);


RESULT CamEngineImageEffectSetColorSelection
(
    CamEngineHandle_t               hCamEngine,
    const CamerIcIeColorSelection_t color,
    const uint8_t                   threshold
);


RESULT CamEngineImageEffectSetSharpen
(
    CamEngineHandle_t               hCamEngine,
    const uint8_t                   factor,
    const uint8_t                   threshold
);


#ifdef __cplusplus
}
#endif

/* @} cam_engine_imgeffects_api */

#endif /* __CAM_ENGINE_IMG_EFFECTS_API_H__ */

