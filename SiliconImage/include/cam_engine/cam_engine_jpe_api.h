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
 * @cond    cam_engine_jpe
 *
 * @file    cam_engine_jpe_api.h
 *
 * @brief
 *
 *   Interface description of the CamEngine JPE.
 *
 *****************************************************************************/
/**
 *
 * @defgroup cam_engine_jpe_api CamEngine JPE API
 * @{
 *
 */

#ifndef __CAM_ENGINE_JPE_API_H__
#define __CAM_ENGINE_JPE_API_H__

#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif

//FIXME
#include <cameric_drv/cameric_jpe_drv_api.h>


/*****************************************************************************/
/**
 * @brief   This function enables the jpe.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   pConfig             configuration of jpe
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_WRONG_CONFIG    image effects isn't configured
 * @retval  RET_NOTAVAILABLE    module not available by driver or hardware
 *
 *****************************************************************************/
RESULT CamEngineEnableJpe
(
    CamEngineHandle_t hCamEngine,
    CamerIcJpeConfig_t *pConfig
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
RESULT CamEngineDisableJpe
(
    CamEngineHandle_t hCamEngine
);


#ifdef __cplusplus
}
#endif

/* @} cam_engine_jpe_api */

#endif /* __CAM_ENGINE_JPE_API_H__ */

