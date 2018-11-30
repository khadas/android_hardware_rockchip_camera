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
//OV2710_MIPI_priv.h

#ifndef __OV2710_PRIV_H__
#define __OV2710_PRIV_H__

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
*v0.1.0x00 : first version:preview && focus;   1: 2lane 3264x2448 is ok    2: 4lane can't preview now //hkw
*v0.2.0x00 : fix 2lane 3264x2448;
*v0.3.0:  tunning first version;
*v0.4.0:
*   1). limit AecMinIntegrationTime 0.0001 for aec.
*v0.5.0:
*   1). add sensor drv version in get sensor i2c info func
*v0.6.0:
*   1). support for isi v0.5.0
*v0.7.0:
*	1)fix AE difference between preview and capture;
*	2)fix MOTOR speed;
*v0.8.0:
*   1)support 1 lane;
*v0.9.0:
*   1) support for isi v0.6.0
*v0.a.0
*   1). support for isi v0.7.0
*v0.b.0
*   1). support mutil framerate and Afps;
*v0.c.0
*   1)  Skip frames when resolution change in OV2710_IsiChangeSensorResolutionIss;
*v0.d.0
*   1). support OTP;
*v0.e.0
*   1). support OTP i2c info;
*v0.f.0
*   1). support R1A&R2A OTP info;
*v1.0.0
*   1). fix some issues in v0.f.0;
*v1.1.0
*   1). fix somme issuse in r2a
*   2). support different otp rg bg typetical value
*v1.2.0
*   1). support another type of R1A.
*v1.2.1
*   1). support isi v0.0xd.0
*v1.2.2
*   1). gain step set to 1/16.
*/


#define CONFIG_SENSOR_DRV_VERSION  KERNEL_VERSION(1, 2, 2)

/*****************************************************************************
 * System control registers
 *****************************************************************************/
  
#define OV2710_MODE_SELECT                  (0x3008)       
#define OV2710_MODE_SELECT_OFF              (0x42U)
#define OV2710_MODE_SELECT_ON		    (0x02U)

#define OV2710_SOFTWARE_RST                 (0x3008)
#define OV2710_SOFTWARE_RST_VALUE	    (0x82)

#define OV2710_CHIP_ID_HIGH_BYTE            (0x300a) 
#define OV2710_CHIP_ID_LOW_BYTE             (0x300b)  
#define OV2710_CHIP_ID_HIGH_BYTE_DEFAULT    (0x27)
#define OV2710_CHIP_ID_LOW_BYTE_DEFAULT     (0x10) 

#define OV2710_AEC_AGC_ADJ_H                (0x350a)
#define OV2710_AEC_AGC_ADJ_L                (0x350b) 

#define OV2710_AEC_EXPO_H                   (0x3500)
#define OV2710_AEC_EXPO_M                   (0x3501)
#define OV2710_AEC_EXPO_L                   (0x3502)



/*****************************************************************************
 * ov14825 context structure
 *****************************************************************************/
typedef struct OV2710_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} OV2710_VcmInfo_t;
typedef struct OV2710_Context_s
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
	OV2710_VcmInfo_t    VcmInfo; 
	uint32_t			preview_minimum_framerate;
} OV2710_Context_t;

#ifdef __cplusplus
}
#endif

#endif
