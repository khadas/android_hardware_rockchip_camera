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
#ifndef __TC358749XBG_PRIV_H__
#define __TC358749XBG_PRIV_H__

#include <ebase/types.h>
#include <common/return_codes.h>
#include <hal/hal_api.h>

/*
*v0.1.0x00 : Create file;
*v0.2.0x00 : change 749's drive capability to avoid pic size error;
*v0.3.0x00 :
*	1) add the state machine mechanism;
*	2) add supported resolution: 1080P30, 1080I60, 576P/I50, 480P/I60;
*v0.4.0x00 : Static variables are not thread-safe,fix them;
*/
#define CONFIG_SENSOR_DRV_VERSION KERNEL_VERSION(0, 3, 0)



#ifdef __cplusplus
extern "C"
{
#endif

#define TC358749XBG_DELAY_5MS                    (0x0000) //delay 5 ms
#define TC358749XBG_MODE_SELECT                  (0x0100) // rw - Bit[7:1]not used  Bit[0]Streaming set 0: software_standby  1: streaming       
#define TC358749XBG_SOFTWARE_RST                 (0x7080) // rw - Bit[7:1]not used  Bit[0]software_reset

#define TC358749XBG_CHIP_ID_HIGH_BYTE_DEFAULT         (0x0147)//  (0x0147) // r - 
#define TC358749XBG_CHIP_ID_MIDDLE_BYTE_DEFAULT          (0x0081) // r - 
#define TC358749XBG_CHIP_ID_LOW_BYTE_DEFAULT             (0x0000) // r - 

#define TC358749XBG_CHIP_ID_HIGH_BYTE            (0x0000) // r - 
#define TC358749XBG_CHIP_ID_MIDDLE_BYTE          (0x0002) // r - 
#define TC358749XBG_CHIP_ID_LOW_BYTE             (0x0004) // r - 
/*****************************************************************************
* Further defines for driver management
*****************************************************************************/
#define TC358749XBG_DRIVER_INIT              (0x00000001)

typedef enum
{
    STATUS_POWER_ON	= 0,
    STATUS_STANDBY	= 1,
    STATUS_READY	= 2,
    STATUS_VIDEO_TX	= 3
} TcStatus;

/*****************************************************************************
 *context structure
 *****************************************************************************/
typedef struct TC358749XBG_Context_s
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
    bool bHdmiinExit;
    osThread gHdmiinThreadId;
    TcStatus gStatus;
} TC358749XBG_Context_t;

#ifdef __cplusplus
}
#endif

#endif

