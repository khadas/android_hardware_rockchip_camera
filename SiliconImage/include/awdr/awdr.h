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
#ifndef __AWDR_H__
#define __AWDR_H__

/**
 * @file awdr.h
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
 * @defgroup AWDR Auto denoising pre-filter module
 * @{
 *
 */
#include <ebase/types.h>
#include <common/return_codes.h>
#include <cam_calibdb/cam_calibdb_api.h>
#include <cameric_drv/cameric_isp_wdr_drv_api.h>

#ifdef __cplusplus
extern "C"
{
#endif


/*****************************************************************************/
/**
 *          AwdrHandle_t
 *
 * @brief   AWDR Module instance handle
 *
 *****************************************************************************/
typedef struct AwdrContext_s* AwdrHandle_t;         /**< handle to ADPF context */


/*****************************************************************************/
/**
 *          AwdrInstanceConfig_t
 *
 * @brief   AWDR Module instance configuration structure
 *
 *****************************************************************************/
typedef struct AwdrInstanceConfig_s {
  AwdrHandle_t            hAwdr;              /**< handle returned by AdpfInit() */
} AwdrInstanceConfig_t;


/*****************************************************************************/
/**
 *          AwdrConfig_t
 *
 * @brief   AWDR Module configuration structure
 *
 *****************************************************************************/
typedef struct AwdrConfig_s {
  float  fSensorGain;        /**< initial sensor gain */
  
  CamerIcDrvHandle_t      hCamerIc;  /**< handle to cameric driver */
  CamerIcDrvHandle_t      hSubCamerIc; /**< handle to 2nd cameric drivder (3D) */

  CamCalibDbHandle_t      hCamCalibDb;  /**< calibration database handle */

  bool_t                  enabled;   /**< measuring enabled */
  CameraIcRKWdrMode_t     mode;
  uint8_t                 segment[CAMERIC_WDR_CURVE_SIZE - 1];    /**< x_i segment size */
  uint16_t                wdr_global_y[CAMERIC_WDR_CURVE_SIZE];
  uint16_t                wdr_block_y[CAMERIC_WDR_CURVE_SIZE];
  uint16_t                wdr_noiseratio;
  uint16_t                wdr_bestlight;
  uint32_t                wdr_gain_off1;
  uint16_t                wdr_pym_cc;
  uint8_t                 wdr_epsilon;
  uint8_t                 wdr_lvl_en;
  uint8_t                 wdr_flt_sel;
  uint8_t                 wdr_gain_max_clip_enable;
  uint8_t                 wdr_gain_max_value;
  uint8_t                 wdr_bavg_clip;
  uint8_t                 wdr_nonl_segm;
  uint8_t                 wdr_nonl_open;
  uint8_t                 wdr_nonl_mode1;
  uint32_t                wdr_coe0;
  uint32_t                wdr_coe1;
  uint32_t                wdr_coe2;
  uint32_t                wdr_coe_off;
} AwdrConfig_t;



/*****************************************************************************/
/**
 *          AdpfInit()
 *
 * @brief   This function initializes the Auto denoising pre-filter module
 *
 * @param   pInstConfig
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_INVALID_PARM
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
RESULT AwdrInit
(
    AwdrInstanceConfig_t *pInstConfig
) ;




/*****************************************************************************/
/**
 *          AdpfRelease()
 *
 * @brief   The function releases/frees the Auto denoising pre-filter module
 *
 * @param   handle  Handle to ADPFM
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT AwdrRelease
(
    AwdrHandle_t handle
);



/*****************************************************************************/
/**
 *          AdpfConfigure()
 *
 * @brief   This function configures the Auto denoising pre-filter module
 *
 * @param   handle  Handle to ADPFM
 * @param   pConfig
 *
 * @return  Returns the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_INVALID_PARM
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
RESULT AwdrConfigure
(
    AwdrHandle_t handle,
    AwdrConfig_t* pConfig
);

RESULT AwdrReConfigure
(
    AwdrHandle_t handle,
    AwdrConfig_t *pConfig
);

RESULT AwdrStart
(
    AwdrHandle_t handle
);

RESULT AwdrStop
(
    AwdrHandle_t handle
);

RESULT AwdrGetCurrentConfig
(
    AwdrHandle_t handle,
    AwdrConfig_t *pConfig
);

RESULT AwdrStatus
(
    AwdrHandle_t handle,
    bool_t       *pRunning
);

RESULT AwdrProcessFrame
(
    AwdrHandle_t    handle,
    const float     gain
);






#ifdef __cplusplus
}
#endif


/* @} Awdr */


#endif /* __Awdr_H__*/

