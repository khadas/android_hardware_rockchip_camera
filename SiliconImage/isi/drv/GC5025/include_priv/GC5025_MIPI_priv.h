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

#ifndef GC5025_PRIV_H
#define GC5025_PRIV_H
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
*v0.1.0 : create file
*v0.2.0 : add otp;
*/

#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 2, 0x00) 
#define Sensor_CHIP_ID_HIGH_BYTE            (0xf0) // r - 
#define Sensor_CHIP_ID_LOW_BYTE          (0xf1) // r - 
#define Sensor_CHIP_ID_HIGH_BYTE_DEFAULT            (0x50) // r - 
#define Sensor_CHIP_ID_LOW_BYTE_DEFAULT          (0x25) // r - 
//leepe
//#define Sensor_MODE_SELECT  (0x0100)
#define Sensor_SOFTWARE_RST                 (0xfe) // bit 7

typedef struct Sensor_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} Sensor_VcmInfo_t;

typedef struct Sensor_Context_s
{
    /**< common context of ISI and ISI driver layer; @note: MUST BE FIRST IN DRIVER CONTEXT */
    IsiSensorContext_t  IsiCtx;

    //// modify below here ////
    IsiSensorConfig_t   Config;                 /**< sensor configuration */
    bool_t              Configured;             /**< flags that config was applied to sensor */
    bool_t              Streaming;              /**< flags that sensor is streaming data */
    bool_t              TestPattern;            /**< flags that sensor is streaming test-pattern */

    /**< if true, just do anything required for Afps parameter calculation, but DON'T access SensorHW! */
    bool_t              isAfpsRun;    

    bool_t              GroupHold;

    float               VtPixClkFreq;      /**< pixel clock */
    uint16_t            LineLengthPck;     /**< line length with blanking */
    uint16_t            FrameLengthLines;  /**< frame line length */

    float               AecMaxGain;
    float               AecMinGain;
    float               AecMaxIntegrationTime;
    float               AecMinIntegrationTime;

    /**< _smallest_ increment the sensor/driver \ can handle (e.g. used for sliders in the application) */
    float               AecIntegrationTimeIncrement;
    /**< _smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application) */
    float               AecGainIncrement;   

    float               AecCurGain;
    float               AecCurIntegrationTime;

    uint16_t            OldGain;               /**< gain multiplier */
    uint32_t            OldCoarseIntegrationTime;
    uint32_t            OldFineIntegrationTime;

    IsiSensorMipiInfo   IsiSensorMipiInfo;
    Sensor_VcmInfo_t    VcmInfo; 
    uint32_t			preview_minimum_framerate;
} Sensor_Context_t;

#ifdef __cplusplus
}
#endif
#endif

