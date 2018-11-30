//IMX258_MIPI_priv.h
/*****************************************************************************/
/*!
 *  \file        IMX258_priv.h \n
 *  \version     1.0 \n
 *  \author      CZF \n
 *  \brief       Private header file for sensor specific code of the OV8810. \n
 *
 *  \revision    $Revision: 432 $ \n
 *               $Author: neugebaa $ \n
 *               $Date: 2009-06-30 11:48:59 +0200 (Di, 30 Jun 2009) $ \n
 *               $Id: OV8810_priv.h 432 2009-06-30 09:48:59Z neugebaa $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/


#ifndef __IMX258_MIPI_PRIV_H__
#define __IMX258_MIPI_PRIV_H__

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
*v0.1.0x00 : first version;   1: 4lane 2100X1560 and 4208x3120 is ok    2: GZ otp awb lsc spc is ok, CMK not ok
*v0.2.0x00 : 1: GZ¡¢CMK AF is ok;
*/


#define CONFIG_SENSOR_DRV_VERSION  KERNEL_VERSION(0, 2, 0)

/*****************************************************************************
 * System control registers
 *****************************************************************************/

#define IMX258_MODE_SELECT                  (0x0100) // rw - Bit[7:1]not used  Bit[0]Streaming set 0: software_standby  1: streaming       
#define IMX258_MODE_SELECT_OFF              (0x00U)
#define IMX258_MODE_SELECT_ON								(0x01U)

#define IMX258_SOFTWARE_RST                 (0x0) // rw - Bit[7:1]not used  Bit[0]software_reset
#define IMX258_SOFTWARE_RST_VALUE						(0x0)

#define IMX258_CHIP_ID_HIGH_BYTE            (0x00) // r - 
#define IMX258_CHIP_ID_MIDDLE_BYTE          (0x00) // r - 
#define IMX258_CHIP_ID_LOW_BYTE             (0x00) // r -  

#define IMX258_CHIP_ID_HIGH_BYTE_DEFAULT            (0x00) // r - 
#define IMX258_CHIP_ID_LOW_BYTE_DEFAULT             (0x00) //(0x65) // r - 


#define IMX258_AEC_AGC_ADJ_H                (0x0204) // rw- Bit[0]gain output to sensor Gain[8]
#define IMX258_AEC_AGC_ADJ_L                (0x0205) // rw- Bit[7:0]gain output to sensor Gain[7:0] 

typedef struct IMX258_VcmInfo_s                 /* ddl@rock-chips.com: v0.3.0 */
{
    uint32_t StartCurrent;
    uint32_t RatedCurrent;
    uint32_t Step;
    uint32_t StepMode;
} IMX258_VcmInfo_t;
typedef struct IMX258_Context_s
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
		IMX258_VcmInfo_t    VcmInfo; 
		uint32_t			preview_minimum_framerate;
} IMX258_Context_t;

#ifdef __cplusplus
}
#endif

#endif
