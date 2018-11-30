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
 * @cond    cameric_isp_flt
 *
 * @file cameric_isp_flt_drv_api.h
 *
 * @brief
 *  Interface description of FLT Module.
 *
 *****************************************************************************/
/**
 * @defgroup cameric_isp_flt_drv_api CamerIc ISP FLT Driver API definitions
 * @{
 *
 * @image html cameric20MP_isp_flt.png "CamerIC ISP FLT driver" width=\textwidth
 * @image latex cameric20MP_isp_flt.png "CamerIC ISP FLT driver" width=\textwidth
 *
 * The Filter block is a combined de-noising and sharpening filter, which is integrated in the
 * demosaicing module. This gate-count efficient implementation leads to restrictions for
 * usage of the filter block.
 *
 * The de-noising settings should be (automatically) adapted to the ambient lighting,
 * because CMOS sensors produce more noise in low light conditions. With low light the
 * setting should be moved to higher de-noising values.
 *
 * The level of sharpening depends on the used lens system as well as the shutter settings.
 * The longer the shutter time, the more blurring may occur, which could be masked by
 * sharpening.
 *
 */

#ifndef __CAMERIC_ISP_FLT_DRV_API_H__
#define __CAMERIC_ISP_FLT_DRV_API_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <common/cam_types.h>

#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************
 * CamerIcIspFltRBDisable()
 *****************************************************************************/
RESULT CamerIcIspFltRKRBDisable
(
    CamerIcDrvHandle_t handle
);
/******************************************************************************
 * CamerIcIspFltRBEnable()
 *****************************************************************************/

RESULT CamerIcIspFltRKRBEnable
(
     CamerIcDrvHandle_t handle
);
/******************************************************************************
 * CamerIcIspFltRBIsEnabled()
 *****************************************************************************/

RESULT CamerIcIspFltRKRBIsEnabled
(
	CamerIcDrvHandle_t handle,
	  bool_t                  *pIsEnabled
);
/******************************************************************************
 * CamerIcIspFltRBDisable()
 *****************************************************************************/
RESULT CamerIcIspFltRKhpDisable
(
    CamerIcDrvHandle_t handle
);
/******************************************************************************
 * CamerIcIspFltRBEnable()
 *****************************************************************************/

RESULT CamerIcIspFltRKhpEnable
(
     CamerIcDrvHandle_t handle
);
/******************************************************************************
 * CamerIcIspFltRBIsEnabled()
 *****************************************************************************/

RESULT CamerIcIspFltRKhpIsEnabled
(
	CamerIcDrvHandle_t handle,
	bool_t                  *pIsEnabled
);

RESULT CamerIcIspFltRKLpConfigure
(
	CamerIcDrvHandle_t  handle,
	CamDemosaicLpProfile_t  *rkDLpConf
) ;

RESULT CamerIcIspFltRKLpConfByGain
(
	CamerIcDrvHandle_t handle,
	CamDemosaicLpProfile_t *rkDLpConf,
	CamDemosaicLpThreshold_t *rkDLpTh
);

/*****************************************************************************/
/**
 * @brief   Enable FLT Module.
 *
 * @param   handle          CamerIc driver handle.
 *
 * @return                  Return the result of the function call.
 * @retval                  RET_SUCCESS
 * @retval                  RET_FAILURE
 *
 *****************************************************************************/
extern RESULT CamerIcIspFltEnable
(
    CamerIcDrvHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   Disable FLT Module.
 *
 * @param   handle          CamerIc driver handle.
 *
 * @return                  Return the result of the function call.
 * @retval                  RET_SUCCESS
 * @retval                  RET_FAILURE
 *
 *****************************************************************************/
extern RESULT CamerIcIspFltDisable
(
    CamerIcDrvHandle_t handle
);



/*****************************************************************************/
/**
 * @brief   Get FLT Module status.
 *
 * @param   handle          CamerIc driver handle.
 * @param   pIsEnabled
 *
 * @return                  Return the result of the function call.
 * @retval                  RET_SUCCESS
 * @retval                  RET_FAILURE
 *
 *****************************************************************************/
extern RESULT CamerIcIspFltIsEnabled
(
    CamerIcDrvHandle_t      handle,
    bool_t                  *pIsEnabled
);



/*****************************************************************************/
/**
 * @brief   Get ISP filter parameter.
 *
 * @param   handle      CamerIc driver handle.
 * @param   pDeNoiseLevel
 * @param   pSharpeningLevel
 *
 * @return              Return the result of the function call.
 * @retval              RET_SUCCESS
 * @retval              RET_FAILURE
 *
 *****************************************************************************/
extern RESULT CamerIcIspFltGetFilterParameter
(
    CamerIcDrvHandle_t                      handle,
    CamerIcIspFltDeNoiseLevel_t             *pDeNoiseLevel,
    CamerIcIspFltSharpeningLevel_t          *pSharpeningLevel
);



/*****************************************************************************/
/**
 * @brief   Set ISP filter parameter.
 *
 * @param   handle      CamerIc driver handle.
 * @param   DeNoiseLevel
 * @param   SharpeningLevel
 *
 * @return              Return the result of the function call.
 * @retval              RET_SUCCESS
 * @retval              RET_FAILURE
 *
 *****************************************************************************/
extern RESULT CamerIcIspFltSetFilterParameter
(
    CamerIcDrvHandle_t                      handle,
    const CamerIcIspFltDeNoiseLevel_t       DeNoiseLevel,
    const CamerIcIspFltSharpeningLevel_t    SharpeningLevel
);

extern RESULT CamerIcIspFltSetFilterParameterRegValue
(
    CamerIcDrvHandle_t                      handle,
    const CamerIcIspFltDeNoiseLevel_t       DeNoiseLevel,
    const CamerIcIspFltSharpeningLevel_t    SharpeningLevel,
    CamFilterLevelRegConf_t *pFiltLevelRegConf
    
);


#ifdef __cplusplus
}
#endif

/* @} cameric_isp_flt_drv_api */

/* @endcond */

#endif /* __CAMERIC_ISP_FLT_DRV_API_H__ */

