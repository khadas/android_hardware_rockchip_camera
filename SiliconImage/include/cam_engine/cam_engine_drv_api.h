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
 * @cond    cam_engine_drv
 *
 * @file    cam_engine_drv_api.h
 *
 * @brief
 *
 *   Interface function to the CamerIc Driver.
 *
 *****************************************************************************/
/**
 *
 * @defgroup cam_engine_drv_api CamEngine DRV API
 * @{
 *
 */

#ifndef __CAM_ENGINE_ISP_DRV_H__
#define __CAM_ENGINE_ISP_DRV_H__

#include <ebase/types.h>

#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif


/*****************************************************************************/
/**
 * @brief   This function returns the CamerIc (Master) Revision ID.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   revision            reference to CamerIc Master Revision ID
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_INVALID_PARM    invalid parameter (revision is a NULL pointer)
 *
 *****************************************************************************/
RESULT CamEngineCamerIcMasterId
(
    CamEngineHandle_t   hCamEngine,
    uint32_t            *revision
);


/*****************************************************************************/
/**
 * @brief   This function returns the CamerIc (Slave) Revision ID.
 *
 * @param   hCamEngine          handle to the CamEngine instance
 * @param   revision            reference to CamerIc Slave Revision ID
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS         function succeed
 * @retval  RET_WRONG_HANDLE    invalid instance handle
 * @retval  RET_INVALID_PARM    invalid parameter (revision is a NULL pointer)
 *
 *****************************************************************************/
RESULT CamEngineCamerIcSlaveId
(
    CamEngineHandle_t   hCamEngine,
    uint32_t            *revision
);


#ifdef __cplusplus
}
#endif


/* @} cam_engine_isp_api */


#endif /* __CAM_ENGINE_ISP_DRV_H__ */

