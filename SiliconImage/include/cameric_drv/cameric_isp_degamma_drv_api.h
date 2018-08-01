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
#ifndef __CAMERIC_ISP_DEGAMMA_DRV_API_H__
#define __CAMERIC_ISP_DEGAMMA_DRV_API_H__

/**
 * @cond    cameric_isp_degamma
 *
 * @file    cameric_isp_degamma_drv_api.h
 *
 * @brief   This file contains the CamerIC ISP DEGAMMA driver API definitions.
 *
 *****************************************************************************/
/**
 * @defgroup cameric_isp_degamma_drv_api CamerIC ISP DEGAMMA driver API definitions
 * @{
 *
 * @image html cameric20MP_isp_degamma.png "CamerIC ISP DEGAMMA driver" width=\textwidth
 * @image latex cameric20MP_isp_degamma.png "CamerIC ISP DEGAMMA driver" width=\textwidth
 *
 * The sensor de-gamma correction is used to adapt the image signal processing to
 * the characteristics of the attached sensor device.
 *
 * This correction is typically a non-linear gradient, so this curve is represented
 * by piece-wise linear approximation. The whole range is divided into 16 sections
 * defined by x-interval dx1...16 and their begin and end values in y direction
 * y0...y16.
 *
 * @image html cameric20MP_isp_degamma_curve.png "Gamma Curve Definition" width=0.75\textwidth
 * @image latex cameric20MP_isp_degamma_curve.png "Gamma Curve Definition" width=0.75\textwidth
 *
 * Three gamma curves can be defined, one for each color component red, green, and
 * blue. The interval widths in x direction are to be defined in a 2value+4 notation,
 * where "value" has to be written to the register. So the steps would be 16 (24),
 * 32 (25), 64 (26), 128 (27), 256 (28), 512 (29), 1024 (210), 2048 (211).
 *
 * A typical application where de-gamma should be used is when a picture of a gray
 * step chart results in different colors of the gray patches. In this case a non-linear
 * characteristic of the sensor is present, which should be corrected by separately
 * adapted correction curves of the R, G and B channels.
 *
 */
#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CAMERIC_DEGAMMA_CURVE_SIZE      17U


/*******************************************************************************
 *
 * @brief   
 *
 */
typedef struct CamerIcIspDegammaCurve_s
{
    uint8_t     segment[CAMERIC_DEGAMMA_CURVE_SIZE-1];      /**< x_i segment size */
    uint16_t    red[CAMERIC_DEGAMMA_CURVE_SIZE];            /**< red point */
    uint16_t    green[CAMERIC_DEGAMMA_CURVE_SIZE];          /**< green point */
    uint16_t    blue[CAMERIC_DEGAMMA_CURVE_SIZE];           /**< blue point */
} CamerIcIspDegammaCurve_t;



/*****************************************************************************/
/**
 * @brief   This function enables the CamerIC ISP Degamma Module.
 *
 * @param   handle              CamerIc driver handle
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 *
 *****************************************************************************/
extern RESULT CamerIcIspDegammaEnable
(
    CamerIcDrvHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This function disables the CamerIc ISP Degamma module (bypass the
 *          degamma module)
 *
 * @param   handle              CamerIc driver handle
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 *
 *****************************************************************************/
extern RESULT CamerIcIspDegammaDisable
(
    CamerIcDrvHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   This function returns the status of the CamerIC ISP Degamma module.
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
extern RESULT CamerIcIspDegammaIsEnabled
(
    CamerIcDrvHandle_t  handle,
    bool_t              *pIsEnabled
);


/*****************************************************************************/
/**
 * @brief   This function returns the currently configured degamma curve 
 *          from the CamerIc ISP Degamma module.
 *
 * @param   handle              CamerIc driver handle
 * @param   pCurve              reference to curve
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pCurve is a NULL pointer
 *
 *****************************************************************************/
extern RESULT CamerIcIspDegammaGetCurve
(
    CamerIcDrvHandle_t          handle,
    CamerIcIspDegammaCurve_t    *pCurve
);


/*****************************************************************************/
/**
 * @brief   This function set the degamma curve to use in the CamerIc ISP 
 *          Degamma module.
 *
 * @param   handle              CamerIc driver handle
 * @param   pCurve              reference to curve
 *
 * @return                      Return the result of the function call.
 * @retval  RET_SUCCESS         operation succeded
 * @retval  RET_WRONG_HANDLE    handle is invalid
 * @retval  RET_INVALID_PARM    pCurve is a NULL pointer
 * @retval  RET_WRONG_STATE     degamma module is running
 *
 *****************************************************************************/
extern RESULT CamerIcIspDegammaSetCurve
(
    CamerIcDrvHandle_t          handle,
    CamerIcIspDegammaCurve_t    *pCurve
);



#ifdef __cplusplus
}
#endif

/* @} cameric_isp_degamma_drv_api */

/* @endcond */

#endif /* __CAMERIC_ISP_DEGAMMA_DRV_API_H__ */

