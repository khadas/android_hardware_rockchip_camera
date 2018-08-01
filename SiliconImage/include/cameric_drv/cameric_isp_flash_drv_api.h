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
#ifndef __CAMERIC_ISP_FLASH_DRV_API_H__
#define __CAMERIC_ISP_FLASH_DRV_API_H__


#include <ebase/types.h>
#include <common/return_codes.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef enum CamerIcIspFlashMode_e
{
    CAMERIC_ISP_FLASH_OFF = 0x00,
    CAMERIC_ISP_FLASH_AUTO = 0x01,
    CAMERIC_ISP_FLASH_ON = 0x02,
    CAMERIC_ISP_FLASH_RED_EYE = 0x03,
    CAMERIC_ISP_FLASH_TORCH = 0x05
} CamerIcIspFlashMode_t;

typedef enum CamerIcIspFlashIntEvent_e
{
    CAMERIC_ISP_FLASH_ON_EVENT = 0x00,
    CAMERIC_ISP_FLASH_OFF_EVENT = 0x01,
    CAMERIC_ISP_FLASH_CAPTURE_FRAME = 0x02,
    CAMERIC_ISP_FLASH_STOP_EVENT = 0x03
} CamerIcIspFlashIntEvent_t;

typedef enum CamerIcIspFlashTriggerPol_e
{
    CAMERIC_ISP_FLASH_LOW_ACTIVE = 0x0,
    CAMERIC_ISP_FLASH_HIGH_ACTIVE = 0x1
}CamerIcIspFlashTriggerPol_t;

typedef RESULT (*CamerIcIspFlashEventCb)
(
    CamerIcDrvHandle_t  handle,
    CamerIcIspFlashIntEvent_t event,
    uint32_t para
);


typedef struct CamerIcIspFlashCfg_s 
{
    CamerIcIspFlashMode_t mode;  
    CamerIcIspFlashTriggerPol_t active_pol;
    int32_t flashtype;
    unsigned int dev_mask;
} CamerIcIspFlashCfg_t;



extern RESULT CamerIcIspFlashConfigure
(
    CamerIcDrvHandle_t  handle,
    CamerIcIspFlashCfg_t *flash_cfg
);

extern RESULT CamerIcIspFlashStart
(
    CamerIcDrvHandle_t  handle,
    bool_t operate_now
);

extern RESULT CamerIcIspFlashStop
(
    CamerIcDrvHandle_t  handle,
    bool_t operate_now
);

#ifdef __cplusplus
}
#endif

#endif
