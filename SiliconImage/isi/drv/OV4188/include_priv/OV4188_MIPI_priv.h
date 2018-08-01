//OV4188_MIPI_priv.h
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

/*Modify by yanghua@ffcs.cn*/
/*
#ifndef _OV4188_PRIV_H
#define _OV4188_PRIV_H

#include "isi_priv.h"

#if( OV4188_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#ifndef __OV4188_PRIV_H__
#define __OV4188_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>



#ifdef __cplusplus
extern "C"
{
#endif

/*
*              SILICONIMAGE LIBISP VERSION NOTE
*
*v0.1.0 : ov4188 driver ok, can preview, no tuning yet //yanghua
*/

/*
*v0.2.0 : ov4188 add 1lane setting, 4224x3136 1lane can't preview yet  //oyyf
*v0.3.0:
*   1). limit AecMinIntegrationTime 0.0001 for aec.
*v0.4.0:
*   1). add sensor drv version in get sensor i2c info func
*v0.5.0:
*   1). support for isi v0.5.0
*v0.6.0:
*   1). af optimization.
v0.7.0
*   1). support for isi v0.6.0
*v0.8.0
*   1). support for isi v0.7.0
*v0.9.0
*	1). support mutil framerate and Afps;
*	2). skip frames when resolution change in OV4188_IsiChangeSensorResolutionIss;
*/


#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 1, 0x00) 


#define OV4188_CHIP_ID_HIGH_BYTE            (0x300a) // r - 
#define OV4188_CHIP_ID_LOW_BYTE          (0x300b) // r - 

#define OV4188_CHIP_ID_HIGH_BYTE_DEFAULT            (0x46) // r - 
#define OV4188_CHIP_ID_LOW_BYTE_DEFAULT          (0x88) // r - 

#define OV4188_MODE_SELECT  (0x0100)	//rw - Bit[7:1]not used  Bit[0] software_standby,  0-software_standby, 1-streaming

#define OV4188_SOFTWARE_RST                 (0x0103) // rw - Bit[7:1]not used  Bit[0]software_reset

typedef struct OV4188_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} OV4188_VcmInfo_t;

typedef struct OV4188_Context_s
{
    IsiSensorContext_t  IsiCtx;                 /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */

    //// modify below here ////

    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that sensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    bool_t              isAfpsRun;              /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */

    bool_t              GroupHold;

    float               VtPixClkFreq;           /**< pixel clock */
    uint16_t            LineLengthPck;          /**< line length with blanking */
    uint16_t            FrameLengthLines;       /**< frame line length */

    float               AecMaxGain;
    float               AecMinGain;
    float               AecMaxIntegrationTime;
    float               AecMinIntegrationTime;

    float               AecIntegrationTimeIncrement; /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;            /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */

    float               AecCurGain;
    float               AecCurIntegrationTime;

    uint16_t            OldGain;               /**< gain multiplier */
    uint32_t            OldCoarseIntegrationTime;
    uint32_t            OldFineIntegrationTime;

    IsiSensorMipiInfo   IsiSensorMipiInfo;
	OV4188_VcmInfo_t    VcmInfo; 
} OV4188_Context_t;

#ifdef __cplusplus
}
#endif

#endif

