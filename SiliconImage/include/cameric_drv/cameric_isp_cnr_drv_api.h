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
#ifndef __CAMERIC_ISP_CNR_DRV_API_H__
#define __CAMERIC_ISP_CNR_DRV_API_H__

/**
 * @cond    cameric_isp_cnr
 *
 * @file    cameric_isp_cnr_drv_api.h
 *
 * @brief   This file contains the CamerIC ISP CNR driver API definitions.
 *
 *****************************************************************************/
/**
 * @defgroup cameric_isp_cnr_drv_api CamerIC ISP CNR driver API definitions
 * @{
 *
 * @image html cameric20MP_isp_cnr.png "CamerIC ISP CNR driver" width=\textwidth
 * @image latex cameric20MP_isp_cnr.png "CamerIC ISP CNR driver" width=\textwidth
 *
 * The color noise reduction module is used 
 */
#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/**
 * @brief   This function returns RET_SUCCESS if the color noise reduction
 *          module available in hardware.
 *
 * @param   handle              CamerIc driver handle
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         CNR available
 * @retval  RET_NOTSUPP         CNR not available
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrIsAvailable
(
    CamerIcDrvHandle_t handle
);

/*****************************************************************************/
/**
 * @brief   This function enables the CamerIC ISP CNR Module.
 *
 * @param   handle              CamerIc driver handle
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrEnable
(
    CamerIcDrvHandle_t handle
);


/*****************************************************************************/
/**
 * @brief   This function disables the CamerIc ISP CNR module (bypass the
 *          color noise reduction module)
 *
 * @param   handle              CamerIc driver handle
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrDisable
(
    CamerIcDrvHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This function returns the status of the CamerIC ISP CNR module.
 *
 * @param   handle              CamerIc driver handle
 * @param   pIsEnabled          Pointer to value to store current state
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pIsEnabled is a NULL pointer
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrIsActivated
(
    CamerIcDrvHandle_t  handle,
    bool_t              *pIsEnabled
);



/*****************************************************************************/
/**
 * @brief   This function sets the line-width of the CamerIC ISP CNR module.
 *
 * @param   handle              CamerIc driver handle
 * @param   width               Line width to set 
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pIsEnabled is a NULL pointer
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrSetLineWidth
(
    CamerIcDrvHandle_t  handle,
    const uint16_t      width
);



/*****************************************************************************/
/**
 * @brief   This function returns the thresholds of the CamerIC ISP CNR module.
 *
 * @param   handle              CamerIc driver handle
 * @param   threshold1          Threshold Color Channel 1
 * @param   threshold2          Threshold Color Channel 2
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pIsEnabled is a NULL pointer
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrGetThresholds
(
    CamerIcDrvHandle_t  handle,
    uint32_t            *threshold1,
    uint32_t            *threshold2
);



/*****************************************************************************/
/**
 * @brief   This function sets the thresholds of the CamerIC ISP CNR module.
 *
 * @param   handle              CamerIc driver handle
 * @param   threshold1          Threshold Color Channel 1
 * @param   threshold2          Threshold Color Channel 2
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pIsEnabled is a NULL pointer
 *
 *****************************************************************************/
extern RESULT CamerIcIspCnrSetThresholds
(
    CamerIcDrvHandle_t  handle,
    const uint32_t      threshold1,
    const uint32_t      threshold2
);



#ifdef __cplusplus
}
#endif

/* @} cameric_isp_cnr_drv_api */

/* @endcond */

#endif /* __CAMERIC_ISP_CNR_DRV_API_H__ */

