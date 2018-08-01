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
#ifndef __SENSOR_PRIV_H__
#define __SENSOR_PRIV_H__

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
*v0.b.0:  fix being red in black background;
*/


#define CONFIG_SENSOR_DRV_VERSION  KERNEL_VERSION(0, 0x0b, 0) 

/*****************************************************************************
 * System control registers
 *****************************************************************************/
//hkw  
#define sensor_MODE_SELECT                  (0x0100) // rw - Bit[7:1]not used  Bit[0]Streaming set 0: software_standby  1: streaming       
#define sensor_MODE_SELECT_OFF              (0x00U)
#define sensor_MODE_SELECT_ON				(0x01U)

#define sensor_SOFTWARE_RST                 (0x0103) // rw - Bit[7:1]not used  Bit[0]software_reset
#define sensor_SOFTWARE_RST_VALUE			(0x01)

#define sensor_CHIP_ID_HIGH_BYTE            (0x2016) // r - 
#define sensor_CHIP_ID_MIDDLE_BYTE          (0x2017) // r - 
#define sensor_CHIP_ID_LOW_BYTE             (0x2016) // r -  
#define sensor_CHIP_ID_HIGH_BYTE_DEFAULT         (0x03) // r - 
#define sensor_CHIP_ID_MIDDLE_BYTE_DEFAULT       (0xbb) // r - 
#define sensor_CHIP_ID_LOW_BYTE_DEFAULT          (0x03) //(0x65) // r - 

#define sensor_A_GAIN_ADDR_H                     (0x0204) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_A_GAIN_ADDR_L                     (0x0205) // rw- Bit[2:0]gain output to sensor Gain[10:8]

#define sensor_D_GAIN_GREENR_ADDR_H              (0x020E) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_GREENR_ADDR_L              (0x020F) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_RED_ADDR_H                 (0x0210) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_RED_ADDR_L                 (0x0211) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_BLUE_ADDR_H                (0x0212) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_BLUE_ADDR_L                (0x0213) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_GREENB_ADDR_H              (0x0214) // rw- Bit[2:0]gain output to sensor Gain[10:8]
#define sensor_D_GAIN_GREENB_ADDR_L              (0x0215) // rw- Bit[2:0]gain output to sensor Gain[10:8]

#define sensor_AEC_FINE_EXPO_H                   (0x0200) // rw- Bit[3:0] exposure[19:16]
#define sensor_AEC_FINE_EXPO_L                   (0x0201) // rw- Bit[7:0] exposure[15:8]
#define sensor_AEC_COARSE_EXPO_H                 (0x0202) // rw- Bit[7:0] exposure[7:0] low 4 bits are fraction bits which are not supportted and should always be 0.
#define sensor_AEC_COARSE_EXPO_L                 (0x0203) // rw- Bit[7:0] exposure[7:0] low 4 bits are fraction bits which are not supportted and should always be 0.



/*****************************************************************************
 *****************************************************************************/
typedef struct sensor_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} sensor_VcmInfo_t;
typedef struct sensor_Context_s
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
	sensor_VcmInfo_t    VcmInfo; 
} sensor_Context_t;

typedef struct sensor_gain_table_s {
	uint16_t reg_gain;
	uint16_t gain_value;
}sensor_gain_table_t;

#ifdef __cplusplus
}
#endif

#endif
