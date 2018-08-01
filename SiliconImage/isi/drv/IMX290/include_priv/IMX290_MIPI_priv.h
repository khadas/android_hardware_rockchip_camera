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
#ifndef __IMX290_PRIV_H__
#define __IMX290_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 0x1, 0x00)

#define Sensor_CHIP_ID_HIGH_BYTE            	(0x3004) // r -
#define Sensor_CHIP_ID_LOW_BYTE          		(0x301F) // r -

#define Sensor_CHIP_ID_HIGH_BYTE_DEFAULT        (0x10) // r -
#define Sensor_CHIP_ID_LOW_BYTE_DEFAULT         (0x01) // r -

#define Sensor_MODE_SELECT  					(0x3000)
#define Sensor_SOFTWARE_RST                 	(0x3003) // rw - Bit[7:1]not used  Bit[0]software_reset


typedef struct Sensor_Context_s
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
	float               AecGainIncrement;            
	float               AecCurGain;
	float               OldGain;               
	
    uint32_t            AecMaxIntegrationTime;
    uint32_t            AecMinIntegrationTime;
    uint32_t            AecIntegrationTimeIncrement; 
    uint32_t            AecCurIntegrationTime;
	uint32_t            OldIntegrationTime;    

    IsiSensorMipiInfo   IsiSensorMipiInfo;
	uint32_t			preview_minimum_framerate;
} Sensor_Context_t;

#ifdef __cplusplus
}
#endif

#endif


