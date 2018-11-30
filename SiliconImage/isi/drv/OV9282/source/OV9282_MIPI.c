//OV9282 the same with ov14825

/******************************************************************************
 *
 * Copyright 2010, Dream Chip Technologies GmbH. All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of:
 * Dream Chip Technologies GmbH, Steinriede 10, 30827 Garbsen / Berenbostel,
 * Germany
 *
 *****************************************************************************/
/**
 * @file OV9282.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>
#include <common/misc.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"

#include "OV9282_MIPI_priv.h"

#define  OV9282_NEWEST_TUNING_XML "22-May-2014_OUYANG_OV9282_FX288_v1.0"
//hkw no use;
#define CC_OFFSET_SCALING  2.0f
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( OV9282_INFO , "OV9282: ", INFO,    1U );//mtcai ,default 0
CREATE_TRACER( OV9282_WARN , "OV9282: ", WARNING, 1U );
CREATE_TRACER( OV9282_ERROR, "OV9282: ", ERROR,   1U );

CREATE_TRACER( OV9282_DEBUG, "OV9282: ", INFO,     1U );//mtcai

CREATE_TRACER( OV9282_NOTICE0 , "OV9282: ", TRACE_NOTICE0, 1);
CREATE_TRACER( OV9282_NOTICE1, "OV9282: ", TRACE_NOTICE1, 1U );


#define OV9282_SLAVE_ADDR       0x20U                           /**< i2c slave address of the OV9282 camera sensor */
#define OV9282_SLAVE_ADDR2      0xC0U
#define OV9282_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the OV9282 integrated AD5820 */

#define OV9282_MAXN_GAIN 		(16.0f)
#define OV9282_MIN_GAIN_STEP   ( 1.0f / OV9282_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define OV9282_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


/*!<
 * Focus position values:
 * 65 logical positions ( 0 - 64 )
 * where 64 is the setting for infinity and 0 for macro
 * corresponding to
 * 1024 register settings (0 - 1023)
 * where 0 is the setting for infinity and 1023 for macro
 */
#define MAX_LOG   64U
#define MAX_REG 1023U

#define MAX_VCMDRV_CURRENT      100U
#define MAX_VCMDRV_REG          1023U




/*!<
 * Lens movement is triggered every 133ms (VGA, 7.5fps processed frames
 * worst case assumed, usually even much slower, see OV5630 driver for
 * details). Thus the lens has to reach the requested position after
 * max. 133ms. Minimum mechanical ringing is expected with mode 1 ,
 * 100us per step. A movement over the full range needs max. 102.3ms
 * (see table 9 AD5820 datasheet).
 */
#define MDI_SLEW_RATE_CTRL 5U /* S3..0 for MOTOR hkw*/



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char OV9282_g_acName[] = "OV9282_MIPI";
//extern const IsiRegDescription_t OV9282_g_aRegDescription[];
extern const IsiRegDescription_t OV9282_g_aRegDescription_twolane[];
extern const IsiRegDescription_t OV9282_g_1280x720_twolane[];


const IsiSensorCaps_t OV9282_g_IsiSensorDefaultConfig;



#define OV9282_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define OV9282_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address and 2 bytes sub address
#define OV9282_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_ONE_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT OV9282_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT OV9282_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT OV9282_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT OV9282_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT OV9282_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV9282_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT OV9282_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT OV9282_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT OV9282_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT OV9282_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT OV9282_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV9282_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT OV9282_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT OV9282_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT OV9282_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV9282_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT OV9282_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT OV9282_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT OV9282_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT OV9282_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT OV9282_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT OV9282_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT OV9282_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT OV9282_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT OV9282_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT OV9282_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT OV9282_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT OV9282_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT OV9282_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT OV9282_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT OV9282_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT OV9282_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT OV9282_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT OV9282_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT OV9282_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT OV9282_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT OV9282_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT OV9282_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);
static RESULT OV9282_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate);

static int g_sensor_version;

/*****************************************************************************/
/**
 *          OV9282_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV9282 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT OV9282_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    OV9282_Context_t *pOV9282Ctx;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pOV9282Ctx = ( OV9282_Context_t * )malloc ( sizeof (OV9282_Context_t) );
    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR,  "%s: Can't allocate OV9282 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pOV9282Ctx, 0, sizeof( OV9282_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pOV9282Ctx );
        return ( result );
    }
    
    pOV9282Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pOV9282Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pOV9282Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pOV9282Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? OV9282_SLAVE_ADDR : pConfig->SlaveAddr;
    pOV9282Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pOV9282Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pOV9282Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? OV9282_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pOV9282Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pOV9282Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pOV9282Ctx->Configured             = BOOL_FALSE;
    pOV9282Ctx->Streaming              = BOOL_FALSE;
    pOV9282Ctx->TestPattern            = BOOL_FALSE;
    pOV9282Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV9282Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pOV9282Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pOV9282Ctx->VcmInfo.RatedCurrent   = pOV9282Ctx->VcmInfo.StartCurrent + MAX_LOG*pOV9282Ctx->VcmInfo.Step;
    pOV9282Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pOV9282Ctx->IsiSensorMipiInfo.sensorHalDevID = pOV9282Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( OV9282_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pOV9282Ctx;

    result = HalSetCamConfig( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an OV9282 sensor instance.
 *
 * @param   handle      OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT OV9282_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)OV9282_IsiSensorSetStreamingIss( pOV9282Ctx, BOOL_FALSE );
    (void)OV9282_IsiSensorSetPowerIss( pOV9282Ctx, BOOL_FALSE );

    (void)HalDelRef( pOV9282Ctx->IsiCtx.HalHandle );

    MEMSET( pOV9282Ctx, 0, sizeof( OV9282_Context_t ) );
    free ( pOV9282Ctx );

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCapsIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor capabilities structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV9282_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t  mipi_lanes
)
{
    RESULT result = RET_SUCCESS;
    
    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        if(mipi_lanes == SUPPORT_MIPI_ONE_LANE) {
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_TV720P15;
                    TRACE( OV9282_INFO, "ISI_RES_TV720P60\n");
                    break;
                }
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }
        }             
    
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT; //
        pIsiSensorCaps->Mode            = ISI_MODE_MIPI;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_BGBGGRGR;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS; //hsync?
        pIsiSensorCaps->VPol            = ISI_VPOL_POS; //VPolarity
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING; //?
        pIsiSensorCaps->Bls             = ISI_BLS_OFF; //close;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;//close;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;//close;<
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );//close;
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );//坏点

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL; //;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A  //光源；
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10; 
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP ); //跳帧;没用
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;//
    }
end:
    return result;
}
 
static RESULT OV9282_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = OV9282_IsiGetCapsIssInternal(pIsiSensorCaps,pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t OV9282_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_BGBGGRGR,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_TV720P15, 
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_F11,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_RAW_10,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};



/*****************************************************************************/
/**
 *          OV9282_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV9282 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 验证上面模式等；
 *****************************************************************************/
RESULT OV9282_SetupOutputFormat
(
    OV9282_Context_t       *pOV9282Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s%s (enter)\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* mode */
    switch ( pConfig->Mode )            /* only ISI_MODE_BAYER supported, no configuration needed here */
    {
        case( ISI_MODE_MIPI ):
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* field-selection */
    switch ( pConfig->FieldSelection )  /* only ISI_FIELDSEL_BOTH supported, no configuration needed */
    {
        case ISI_FIELDSEL_BOTH:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by OV9282 sensor, so the YCSequence parameter is not checked */
    switch ( pConfig->YCSequence )
    {
        default:
        {
            break;
        }
    }

    /* 422 conversion */
    switch ( pConfig->Conv422 )         /* only ISI_CONV422_NOCOSITED supported, no configuration needed */
    {
        case ISI_CONV422_NOCOSITED:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
        case ISI_BPAT_BGBGGRGR:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* horizontal polarity */
    switch ( pConfig->HPol )            /* only ISI_HPOL_REFPOS supported, no configuration needed */
    {
        case ISI_HPOL_REFPOS:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* vertical polarity */
    switch ( pConfig->VPol )            /*no configuration needed */
    {
        case ISI_VPOL_NEG:
        {
            break;
        }
        case ISI_VPOL_POS:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }


    /* edge */
    switch ( pConfig->Edge )            /* only ISI_EDGE_RISING supported, no configuration needed */
    {
        case ISI_EDGE_RISING:
        {
            break;
        }

        case ISI_EDGE_FALLING:          /*TODO for MIPI debug*/
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* gamma */
    switch ( pConfig->Gamma )           /* only ISI_GAMMA_OFF supported, no configuration needed */
    {
        case ISI_GAMMA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* color conversion */
    switch ( pConfig->CConv )           /* only ISI_CCONV_OFF supported, no configuration needed */
    {
        case ISI_CCONV_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->SmiaMode )        /* only ISI_SMIA_OFF supported, no configuration needed */
    {
        case ISI_SMIA_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->MipiMode )        /* only ISI_MIPI_MODE_RAW_12 supported, no configuration needed */
    {
        case ISI_MIPI_MODE_RAW_10:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    switch ( pConfig->AfpsResolutions ) /* no configuration needed */
    {
        case ISI_AFPS_NOTSUPP:
        {
            break;
        }
        default:
        {
            // don't care about what comes in here
            //TRACE( OV9282_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( OV9282_INFO, "%s%s (exit)\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

//2400 :real clock/10000
int OV9282_get_PCLK( OV9282_Context_t *pOV9282Ctx, int XVCLK)
{
#if 0
    // calculate PCLK
    uint32_t SCLK, temp1, temp2, temp3;
	int Pll2_prediv0, Pll2_prediv2x, Pll2_multiplier, Pll2_sys_pre_div, Pll2_sys_divider2x, Sys_pre_div, sclk_pdiv;
    int Pll2_prediv0_map[] = {1, 2};
    int Pll2_prediv2x_map[] = {2, 3, 4, 5, 6, 8, 12, 16};
    int Pll2_sys_divider2x_map[] = {2, 3, 4, 5, 6, 7, 8, 10};
    int Sys_pre_div_map[] = {1, 2, 4, 1};

    
    //temp1 = ReadSCCB(0x6c, 0x3007);
    OV9282_IsiRegReadIss(  pOV9282Ctx, 0x0312, &temp1 );
    temp2 = (temp1>>4) & 0x01;
    Pll2_prediv0 = Pll2_prediv0_map[temp2];

	OV9282_IsiRegReadIss(  pOV9282Ctx, 0x030b, &temp1 );
	temp2 = temp1 & 0x07;
	Pll2_prediv2x = Pll2_prediv2x_map[temp2];

	OV9282_IsiRegReadIss(  pOV9282Ctx, 0x030c, &temp1 );
	OV9282_IsiRegReadIss(  pOV9282Ctx, 0x030d, &temp3 );
	temp1 = temp1 & 0x03;
	temp2 = (temp1<<8) + temp3;
	if(!temp2) {
 		Pll2_multiplier = 1;
 	}
	else {
 	Pll2_multiplier = temp2;
	}
	
    OV9282_IsiRegReadIss(  pOV9282Ctx, 0x030f, &temp1 );
	temp1 = temp1 & 0x0f;
	Pll2_sys_pre_div = temp1 + 1;
	OV9282_IsiRegReadIss(  pOV9282Ctx, 0x030e, &temp1 );
	temp1 = temp1 & 0x07;
	Pll2_sys_divider2x = Pll2_sys_divider2x_map[temp1];

	OV9282_IsiRegReadIss(  pOV9282Ctx, 0x3106, &temp1 );
	temp2 = (temp1>>2) & 0x03;
	Sys_pre_div = Sys_pre_div_map[temp2];
    
    temp2 = (temp1>>4) & 0x0f;
	 if(!temp2) {
 		sclk_pdiv = 1;
 	}
	 else {
 		sclk_pdiv = temp2;
 	}
  	 SCLK = XVCLK * 4 / Pll2_prediv0 / Pll2_prediv2x * Pll2_multiplier /Pll2_sys_pre_div / Pll2_sys_divider2x / Sys_pre_div / sclk_pdiv;
	
	temp1 = SCLK>>8;
	OV9282_IsiRegWriteIss(pOV9282Ctx, 0x350b, temp1);
	temp1 = SCLK & 0xff;
	OV9282_IsiRegWriteIss(pOV9282Ctx, 0x350a, temp1);
	return SCLK*10000;
#endif
    return 80000000;//mtcai
}

/*****************************************************************************/
/**
 *          OV9282_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV9282 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT OV9282_SetupOutputWindowInternal
(
    OV9282_Context_t        *pOV9282Ctx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
	uint16_t usTimeHts;
	uint16_t usTimeVts;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 2400;
    
	TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE)
    {
    	switch ( pConfig->Resolution )
        {
            case ISI_RES_TV720P15:            
            {
                if (set2Sensor == BOOL_TRUE) 
                {                    
                    if (res_no_chg == BOOL_FALSE) 
                    {
						result = IsiRegDefaultsApply( pOV9282Ctx, OV9282_g_1280x720_twolane);
                        
                    }
        		}

    			usTimeHts = 0x02d8;
                usTimeVts = 0x0724;
                pOV9282Ctx->IsiSensorMipiInfo.ulMipiFreq = 800;
                
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }
            default:
            {
                TRACE( OV9282_ERROR, "%s: Resolution(0x%x) not supported\n", __FUNCTION__, pConfig->Resolution);
                return ( RET_NOTSUPP );
            }
    	}
    }
    
	
    /* write default values derived from datasheet and evaluation kit 
       (static setup altered by dynamic setup further below) 
    */
    
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	rVtPixClkFreq = OV9282_get_PCLK(pOV9282Ctx, xclk);
    
    // store frame timing for later use in AEC module
    pOV9282Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pOV9282Ctx->LineLengthPck    = usLineLengthPck;
    pOV9282Ctx->FrameLengthLines = usFrameLengthLines;

    TRACE( OV9282_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      OV9282 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT OV9282_SetupImageControl
(
    OV9282_Context_t        *pOV9282Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = OV9282_IsiRegReadIss(  pOV9282Ctx, OV9282_BLC_CTRL00, &RegValue );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = OV9282_IsiRegReadIss(  pOV9282Ctx, OV9282_BLC_CTRL00, &RegValue );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = OV9282_IsiRegReadIss(  pOV9282Ctx, OV9282_AEC_MANUAL, &RegValue );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = OV9282_IsiRegReadIss(  pOV9282Ctx, OV9282_ISP_CTRL01, &RegValue );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = OV9282_IsiRegReadIss(  pOV9282Ctx, OV9282_AEC_MANUAL, &RegValue );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = OV9282_IsiRegReadIss( pOV9282Ctx, OV9282_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = OV9282_IsiRegReadIss( pOV9282Ctx, OV9282_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( OV9282_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT OV9282_SetupOutputWindow
(
    OV9282_Context_t        *pOV9282Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV9282Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pOV9282Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return OV9282_SetupOutputWindowInternal(pOV9282Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          OV9282_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in OV9282-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      OV9282 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT OV9282_AecSetModeParameters
(
    OV9282_Context_t       *pOV9282Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"",
        pOV9282Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pOV9282Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( OV9282_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pOV9282Ctx->AecMaxIntegrationTime = ( ((float)(pOV9282Ctx->FrameLengthLines - 4)) * ((float)pOV9282Ctx->LineLengthPck) ) / pOV9282Ctx->VtPixClkFreq;
    pOV9282Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( OV9282_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"", pOV9282Ctx->AecMaxIntegrationTime  );

    pOV9282Ctx->AecMaxGain = OV9282_MAX_GAIN_AEC;
    pOV9282Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pOV9282Ctx->AecIntegrationTimeIncrement = ((float)pOV9282Ctx->LineLengthPck) / pOV9282Ctx->VtPixClkFreq;
    pOV9282Ctx->AecGainIncrement = OV9282_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pOV9282Ctx->AecCurGain               = pOV9282Ctx->AecMinGain;
    pOV9282Ctx->AecCurIntegrationTime    = 0.0f;
    pOV9282Ctx->OldCoarseIntegrationTime = 0;
    pOV9282Ctx->OldFineIntegrationTime   = 0;
    //pOV9282Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( OV9282_INFO, "%s%s (exit)\n", __FUNCTION__, pOV9282Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          OV9282_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      OV9282 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV9282_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pOV9282Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pOV9282Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = OV9282_IsiRegWriteIss ( pOV9282Ctx, OV9282_SOFTWARE_RST, OV9282_SOFTWARE_RST_VALUE );//宏定义 hkw；
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //test mtcai
    pOV9282Ctx->IsiCtx.SlaveAddress = 0x20;
    TRACE( OV9282_ERROR, "SlaveAddress :%x\n", pOV9282Ctx->IsiCtx.SlaveAddress);
    result = OV9282_IsiRegWriteIss ( pOV9282Ctx, OV9282_SOFTWARE_RST, OV9282_SOFTWARE_RST_VALUE );//宏定义 hkw；
    if(result != RET_SUCCESS)
    {
        TRACE( OV9282_ERROR, "i2c write error\n");
    }
    pOV9282Ctx->IsiCtx.SlaveAddress = 0xC0;
    TRACE( OV9282_INFO, "SlaveAddress :%x\n", pOV9282Ctx->IsiCtx.SlaveAddress);
    //end
    
    osSleep( 10 );

    TRACE( OV9282_ERROR, "%s: OV9282 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_MODE_SELECT, OV9282_MODE_SELECT_OFF );//OV9282_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: Can't write OV9282 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( pOV9282Ctx, OV9282_g_aRegDescription );
    if(pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE)
    {
        result = IsiRegDefaultsApply(pOV9282Ctx, OV9282_g_aRegDescription_twolane);
    }
    
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s IsiRegDefaultsApply failed.\n", __FUNCTION__ );	
        return ( result );
    }    

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pOV9282Ctx, OV9282_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = OV9282_IsiRegWriteIss( pOV9282Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = OV9282_SetupOutputFormat( pOV9282Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = OV9282_SetupOutputWindow( pOV9282Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV9282_SetupImageControl( pOV9282Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = OV9282_AecSetModeParameters( pOV9282Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pOV9282Ctx->Configured = BOOL_TRUE;
    }

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiChangeSensorResolutionIss
 *
 * @brief   Change image sensor resolution while keeping all other static settings.
 *          Dynamic settings like current gain & integration time are kept as
 *          close as possible. Sensor needs 2 frames to engage (first 2 frames
 *          are not correctly exposed!).
 *
 * @note    Re-read current & min/max values as they will probably have changed!
 *
 * @param   handle                  Sensor instance handle
 * @param   Resolution              new resolution ID
 * @param   pNumberOfFramesToSkip   reference to storage for number of frames to skip
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 * @retval  RET_OUTOFRANGE
 * 不用改
 *****************************************************************************/
static RESULT OV9282_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_ERROR, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pOV9282Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (OV9282_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pOV9282Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pOV9282Ctx->Config.Resolution)) && 
            (ISI_RES_H_GET(Resolution)==ISI_RES_H_GET(pOV9282Ctx->Config.Resolution))) ) {

            if (pOV9282Ctx->Streaming != BOOL_FALSE) {
                TRACE( OV9282_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( OV9282_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pOV9282Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = OV9282_SetupOutputWindowInternal( pOV9282Ctx, &pOV9282Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( OV9282_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pOV9282Ctx->AecCurGain;
        float OldIntegrationTime = pOV9282Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = OV9282_AecSetModeParameters( pOV9282Ctx, &pOV9282Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV9282_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = OV9282_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( OV9282_ERROR, "%s: OV9282_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( OV9282_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV9282_IsiSensorSetStreamingIss
 *
 * @brief   Enables/disables streaming of sensor data, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new streaming state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_WRONG_STATE
 *
 *****************************************************************************/
static RESULT OV9282_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pOV9282Ctx->Configured != BOOL_TRUE) || (pOV9282Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV9282_IsiRegWriteIss ( pOV9282Ctx, OV9282_MODE_SELECT, (RegValue | OV9282_MODE_SELECT_ON) );//OV9282_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV9282_IsiRegWriteIss ( pOV9282Ctx, OV9282_MODE_SELECT, (RegValue & ~OV9282_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pOV9282Ctx->Streaming = on;
    }

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      OV9282 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT OV9282_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pOV9282Ctx->Configured = BOOL_FALSE;
    pOV9282Ctx->Streaming  = BOOL_FALSE;

    TRACE( OV9282_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( OV9282_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( OV9282_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV9282_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV9282_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( OV9282_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pOV9282Ctx->IsiCtx.HalHandle, pOV9282Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 读pid;2或3个寄存器；
 *****************************************************************************/
static RESULT OV9282_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = OV9282_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (OV9282_CHIP_ID_LOW_BYTE_DEFAULT);

    result = OV9282_IsiGetSensorRevisionIss( handle, &value );
    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( OV9282_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( OV9282_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetSensorRevisionIss
 *
 * @brief   reads the sensor revision register and returns this value
 *
 * @param   handle      pointer to sensor description struct
 * @param   p_value     pointer to storage value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT OV9282_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = OV9282_IsiRegReadIss ( handle, OV9282_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8 );

    result = OV9282_IsiRegReadIss ( handle, OV9282_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( OV9282_ERROR, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiRegReadIss
 *
 * @brief   grants user read access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   p_value     pointer to value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 *****************************************************************************/
static RESULT OV9282_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, OV9282_g_aRegDescription_twolane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }

  //  TRACE( OV9282_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiRegWriteIss
 *
 * @brief   grants user write access to the camera register
 *
 * @param   handle      pointer to sensor description struct
 * @param   address     sensor register to write
 * @param   value       value to write
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * 不用改
 *****************************************************************************/
static RESULT OV9282_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, OV9282_g_aRegDescription_twolane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( OV9282_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          OV9282 instance
 *
 * @param   handle       OV9282 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得增益限制
 *****************************************************************************/
static RESULT OV9282_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( OV9282_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pOV9282Ctx->AecMinGain;
    *pMaxGain = pOV9282Ctx->AecMaxGain;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          OV9282 instance
 *
 * @param   handle       OV9282 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得曝光限制；
 *****************************************************************************/
static RESULT OV9282_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( OV9282_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pOV9282Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pOV9282Ctx->AecMaxIntegrationTime;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV9282_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN值
 *****************************************************************************/
RESULT OV9282_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

	
	result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_AGC_ADJ_H, &data);
	TRACE( OV9282_ERROR, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_AGC_ADJ_L, &data);
	TRACE( OV9282_ERROR, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / OV9282_MAXN_GAIN;
	
    //*pSetGain = pOV9282Ctx->AecCurGain;
    

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 不用改；获得GAIN最小值
 *****************************************************************************/
RESULT OV9282_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV9282Ctx->AecGainIncrement;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * 不用改；设置gain值
 *****************************************************************************/
RESULT OV9282_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
	uint32_t data= 0;
	uint32_t result_gain= 0;

    TRACE( OV9282_INFO, "%s: (enter) pOV9282Ctx->AecMaxGain(%f) \n", __FUNCTION__,pOV9282Ctx->AecMaxGain);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( OV9282_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pOV9282Ctx->AecMinGain ) NewGain = pOV9282Ctx->AecMinGain;
    if( NewGain > pOV9282Ctx->AecMaxGain ) NewGain = pOV9282Ctx->AecMaxGain;

    usGain = (uint16_t)(NewGain * OV9282_MAXN_GAIN+0.5); //大概加0.5 hkw

    // write new gain into sensor registers, do not write if nothing has changed
    if( (usGain != pOV9282Ctx->OldGain) )
    {
        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_AGC_ADJ_H, (usGain>>8)&0x07); //fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_AGC_ADJ_L, (usGain&0xff));
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        pOV9282Ctx->OldGain = usGain;

		/*osSleep(30);
		result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_AGC_ADJ_H, &data);
		TRACE( OV9282_ERROR, " -------reg35088888888:%x-------\n",data );
		result_gain = (data & 0x07) ;
		result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_AGC_ADJ_L, &data);
		TRACE( OV9282_ERROR, " -------reg35099999999:%x-------\n",data );
		result_gain = (result_gain<<8) + data;*/
		
    }

    //calculate gain actually set
    pOV9282Ctx->AecCurGain = ( (float)usGain ) / OV9282_MAXN_GAIN;

    //return current state
    *pSetGain = pOV9282Ctx->AecCurGain;
    TRACE( OV9282_ERROR, "-----------%s: psetgain=%f, NewGain=%f,result_gain=%x\n", __FUNCTION__, *pSetGain, NewGain,result_gain);

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV9282_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间 不用改
 *****************************************************************************/
RESULT OV9282_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pOV9282Ctx->AecCurIntegrationTime;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * 获得曝光时间的step 不用改
 *****************************************************************************/
RESULT OV9282_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pOV9282Ctx->AecIntegrationTimeIncrement;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   NewIntegrationTime      integration time to be set
 * @param   pSetIntegrationTime     set integration time
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 *设置曝光时间；根据应用手册修改寄存器宏
 *****************************************************************************/
RESULT OV9282_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t data= 0;
	uint32_t result_intertime= 0;
	
    //uint32_t FineIntegrationTime   = 0; //not supported by OV9282

    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( OV9282_ERROR, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pOV9282Ctx->AecMinIntegrationTime,
        pOV9282Ctx->AecMaxIntegrationTime);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( OV9282_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pOV9282Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pOV9282Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pOV9282Ctx->AecMinIntegrationTime ) NewIntegrationTime = pOV9282Ctx->AecMinIntegrationTime;

    //the actual integration time is given by
    //integration_time = ( coarse_integration_time * line_length_pck + fine_integration_time ) / vt_pix_clk_freq
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck )
    //fine_integration_time   = integration_time * vt_pix_clk_freq - coarse_integration_time * line_length_pck
    //
    //fine integration is not supported by OV9282
    //=>
    //coarse_integration_time = (int)( integration_time * vt_pix_clk_freq  / line_length_pck + 0.5 )

    ShutterWidthPck = NewIntegrationTime * ( (float)pOV9282Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pOV9282Ctx->LineLengthPck == 0 )
    {
        TRACE( OV9282_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

    //calculate the integer part of the integration time in units of line length
    //calculate the fractional part of the integration time in units of pixel clocks
    //CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV9282Ctx->LineLengthPck) );
    //FineIntegrationTime   = ( (uint32_t)ShutterWidthPck ) - ( CoarseIntegrationTime * pOV9282Ctx->LineLengthPck );
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pOV9282Ctx->LineLengthPck) + 0.5f );

    // write new integration time into sensor registers
    // do not write if nothing has changed
    if( CoarseIntegrationTime != pOV9282Ctx->OldCoarseIntegrationTime )
    {//
        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_EXPO_H, (CoarseIntegrationTime & 0x0000F000U) >> 12U );//fix by ov8858 datasheet
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_EXPO_M, (CoarseIntegrationTime & 0x00000FF0U) >> 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_AEC_EXPO_L, (CoarseIntegrationTime & 0x0000000FU) << 4U );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );


        pOV9282Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
        *pNumberOfFramesToSkip = 1U; //skip 1 frame
        
		/*osSleep(30);
		result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_EXPO_H, &data);
		TRACE( OV9282_ERROR, " -------reg3500:%x-------\n",data );
		result_intertime = (data & 0x0f) << 8;
		result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_EXPO_M, &data);
		TRACE( OV9282_ERROR, " -------reg3501:%x-------\n",data );
		result_intertime = result_intertime + data;
		result = OV9282_IsiRegReadIss ( pOV9282Ctx, OV9282_AEC_EXPO_L, &data);
		TRACE( OV9282_ERROR, " -------reg3502:%x-------\n",data );
		result_intertime = (result_intertime << 4) + (data >> 4);*/
		
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }

    //if( FineIntegrationTime != pOV9282Ctx->OldFineIntegrationTime )
    //{
    //    result = OV9282_IsiRegWriteIss( pOV9282Ctx, ... , FineIntegrationTime );
    //    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    //    pOV9282Ctx->OldFineIntegrationTime = FineIntegrationTime; //remember current integration time
    //    *pNumberOfFramesToSkip = 1U; //skip 1 frame
    //}

    //calculate integration time actually set
    //pOV9282Ctx->AecCurIntegrationTime = ( ((float)CoarseIntegrationTime) * ((float)pOV9282Ctx->LineLengthPck) + ((float)FineIntegrationTime) ) / pOV9282Ctx->VtPixClkFreq;
    pOV9282Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pOV9282Ctx->LineLengthPck) / pOV9282Ctx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pOV9282Ctx->AecCurIntegrationTime;

    TRACE( OV9282_ERROR, "%s:\n"
         "pOV9282Ctx->VtPixClkFreq:%f pOV9282Ctx->LineLengthPck:%x \n"
         "SetTi=%f    NewTi=%f  CoarseIntegrationTime=%x\n"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x\n", __FUNCTION__, 
         pOV9282Ctx->VtPixClkFreq,pOV9282Ctx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x0000F000U) >> 12U ,
         (CoarseIntegrationTime & 0x00000FF0U) >> 4U,
         (CoarseIntegrationTime & 0x0000000FU) << 4U);
    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          OV9282_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   NewGain                 newly calculated gain to be set
 * @param   NewIntegrationTime      newly calculated integration time to be set
 * @param   pNumberOfFramesToSkip   number of frames to skip until AE is
 *                                  executed again
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 * @retval  RET_DIVISION_BY_ZERO
 * 不用改，设置整个曝光；
 *****************************************************************************/
RESULT OV9282_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( OV9282_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( OV9282_ERROR, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = OV9282_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = OV9282_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( OV9282_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *不用改，获取gain和exposure 时间
 *****************************************************************************/
RESULT OV9282_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pOV9282Ctx->AecCurGain;
    *pSetIntegrationTime = pOV9282Ctx->AecCurIntegrationTime;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetResolutionIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  sensor instance handle
 * @param   pSettResolution         set resolution
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
RESULT OV9282_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pOV9282Ctx->Config.Resolution;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pOV9282Ctx             OV9282 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV9282_IsiGetAfpsInfoHelperIss(
    OV9282_Context_t   *pOV9282Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pOV9282Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pOV9282Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = OV9282_SetupOutputWindowInternal( pOV9282Ctx, &pOV9282Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = OV9282_AecSetModeParameters( pOV9282Ctx, &pOV9282Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( OV9282_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pOV9282Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pOV9282Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pOV9282Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pOV9282Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pOV9282Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV9282_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  OV9282 sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 * 不用改；没用；
 *****************************************************************************/
RESULT OV9282_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        TRACE( OV9282_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pOV9282Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pOV9282Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pOV9282Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pOV9282Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    OV9282_Context_t *pDummyCtx = (OV9282_Context_t*) malloc( sizeof(OV9282_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( OV9282_ERROR,  "%s: Can't allocate dummy OV9282 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pOV9282Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = OV9282_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
        if ( lres == RET_SUCCESS ) \
        { \
            ++idx; \
        } \
        else \
        { \
            UPDATE_RESULT( result, lres ); \
        } \
    }

    // check which AFPS series is requested and build its params list for the enabled AFPS resolutions
    switch (pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes)
    {
        case SUPPORT_MIPI_ONE_LANE:
        {
            switch(Resolution)
            {
                default:
                    TRACE( OV9282_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
                    result = RET_NOTSUPP;
                    break;
                   
                case ISI_RES_TV720P15:
					AFPSCHECKANDADD( ISI_RES_TV720P15 );
                    break;

                // check next series here...
            }
        
             break;
        }
        default:
            TRACE( OV9282_ERROR,  "%s: pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibKFactor
 *
 * @brief   Returns the OV9282 specific K-Factor
 *
 * @param   handle       OV9282 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&OV9282_KFactor;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the OV9282 specific PCA-Matrix
 *
 * @param   handle          OV9282 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&OV9282_PCAMatrix;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              OV9282 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&OV9282_SVDMeanValue;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              OV9282 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&OV9282_CenterLine;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              OV9282 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&OV9282_AwbClipParm;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              OV9282 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&OV9282_AwbGlobalFadeParm;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              OV9282 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&OV9282_AwbFade2Parm;

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          OV9282_IsiGetIlluProfile
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint16_t i;

        *ptIsiIlluProfile = NULL;

        /* check if we've a default profile */
        for ( i=0U; i<OV9282_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( OV9282_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &OV9282_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetLscMatrixTable
 *
 * @brief   Returns a pointer to illumination profile idetified by CieProfile
 *          bitmask
 *
 * @param   handle              sensor instance handle
 * @param   CieProfile
 * @param   ptIsiIlluProfile    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；return success;
 *****************************************************************************/
static RESULT OV9282_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        uint16_t i;


        switch ( CieProfile )
        {
            case ISI_CIEPROF_A:
            {
                if ( ( pOV9282Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pOV9282Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV9282_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pOV9282Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pOV9282Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV9282_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pOV9282Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pOV9282Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV9282_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pOV9282Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pOV9282Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV9282_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pOV9282Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pOV9282Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &OV9282_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( OV9282_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( OV9282_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          OV9282_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT OV9282_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          OV9282 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；
 *****************************************************************************/
static RESULT OV9282_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;
    return ( result );

    //TRACE( OV9282_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
 if ((pOV9282Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pOV9282Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pOV9282Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( OV9282_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = OV9282_IsiMdiFocusSet( handle, MAX_LOG );

    //TRACE( OV9282_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          OV9282 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT OV9282_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    return ( result );
    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    //TRACE( OV9282_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( OV9282_ERROR, "%s: pOV9282Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pOV9282Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pOV9282Ctx->VcmInfo.StartCurrent + (pOV9282Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( OV9282_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pOV9282Ctx->VcmInfo.StepMode );
	
    //TRACE( OV9282_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pOV9282Ctx->IsiCtx.HalHandle,
                             pOV9282Ctx->IsiCtx.I2cAfBusNum,
                             pOV9282Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pOV9282Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( OV9282_ERROR, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          OV9282 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 参考14825；外置马达；
 *****************************************************************************/
static RESULT OV9282_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    return ( result );
    //TRACE( OV9282_ERROR, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    result = HalReadI2CMem( pOV9282Ctx->IsiCtx.HalHandle,
                            pOV9282Ctx->IsiCtx.I2cAfBusNum,
                            pOV9282Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pOV9282Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    //TRACE( OV9282_ERROR, "%s: value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

    /*  //map 0 to 64 -> infinity 
    if( *pAbsStep == 0 )
    {
        *pAbsStep = MAX_LOG;
    }
    else
    {
        *pAbsStep = ( MAX_REG - *pAbsStep ) / 16U;
    }*/
	if( *pAbsStep <= pOV9282Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pOV9282Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pOV9282Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pOV9282Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pOV9282Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
   // TRACE( OV9282_ERROR, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改；没用；
 *****************************************************************************/
static RESULT OV9282_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          OV9282_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *不用改，没用，return；
 ******************************************************************************/
static RESULT OV9282_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = OV9282_IsiRegReadIss( pOV9282Ctx, OV9282_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = OV9282_IsiRegReadIss( pOV9282Ctx, OV9282_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = OV9282_IsiRegWriteIss( pOV9282Ctx, OV9282_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pOV9282Ctx->TestPattern = enable;
    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          OV9282_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          OV9282 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * 不用改
 ******************************************************************************/
static RESULT OV9282_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pOV9282Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pOV9282Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pOV9282Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( OV9282_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT OV9282_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
    	TRACE( OV9282_ERROR, "%s: pOV9282Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( OV9282_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT OV9282_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
    	TRACE( OV9282_ERROR, "%s: pOV9282Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( OV9282_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = OV9282_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          OV9282_IsiGetSensorIss
 *
 * @brief   fills in the correct pointers for the sensor description struct
 *
 * @param   param1      pointer to sensor description struct
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT OV9282_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = OV9282_g_acName;
        pIsiSensor->pRegisterTable                      = OV9282_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &OV9282_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= OV9282_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= OV9282_IsiGetSensorTuningXmlVersion;//oyyf

        pIsiSensor->pIsiCreateSensorIss                 = OV9282_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = OV9282_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = OV9282_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = OV9282_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = OV9282_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = OV9282_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = OV9282_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = OV9282_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = OV9282_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = OV9282_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = OV9282_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = OV9282_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = OV9282_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = OV9282_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = OV9282_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = OV9282_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = OV9282_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = OV9282_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = OV9282_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = OV9282_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = OV9282_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = OV9282_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = OV9282_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = OV9282_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = OV9282_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = OV9282_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = OV9282_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = OV9282_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = OV9282_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = OV9282_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = OV9282_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = OV9282_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = OV9282_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = OV9282_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = OV9282_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = OV9282_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = OV9282_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = OV9282_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = OV9282_IsiActivateTestPattern;

    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( OV9282_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT OV9282_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( OV9282_ERROR,  "%s: Can't allocate OV9282 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = OV9282_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = OV9282_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = OV9282_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = OV9282_SOFTWARE_RST_VALUE;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps;
        uint32_t lanes,i;        

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(OV9282_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
                    pCaps = malloc(sizeof(sensor_caps_t));
                    if (pCaps != NULL) {
                        memcpy(&pCaps->caps,&Caps,sizeof(IsiSensorCaps_t));
                        ListPrepareItem(pCaps);
                        ListAddTail(&pSensorI2cInfo->lane_res[i], pCaps);
                    }
                    Caps.Index++;
                }
            }
        }
    }
    
    ListInit(&pSensorI2cInfo->chipid_info);

    sensor_chipid_info_t* pChipIDInfo_H = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_H )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_H, 0, sizeof(*pChipIDInfo_H) );    
    pChipIDInfo_H->chipid_reg_addr = OV9282_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = OV9282_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = OV9282_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = OV9282_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
    return RET_SUCCESS;
}

static RESULT OV9282_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate)
{
    OV9282_Context_t *pOV9282Ctx = (OV9282_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( OV9282_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pOV9282Ctx == NULL )
    {
    	TRACE( OV9282_ERROR, "%s: pOV9282Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }
	
	//pOV9282Ctx->preview_minimum_framerate = minimum_framerate;
	return RET_SUCCESS;
}


/******************************************************************************
 * See header file for detailed comment.
 *****************************************************************************/


/*****************************************************************************/
/**
 */
/*****************************************************************************/
IsiCamDrvConfig_t IsiCamDrvConfig =
{
    0,
    OV9282_IsiGetSensorIss,
    {
        0,                      /**< IsiSensor_t.pszName */
        0,                      /**< IsiSensor_t.pRegisterTable */
        0,                      /**< IsiSensor_t.pIsiSensorCaps */
        0,						/**< IsiSensor_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiSensor_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiSensor_t.pIsiCheckOTPInfo>*/  //zyc 
        0,						/**< IsiSensor_t.pIsiSetSensorOTPInfo>*/  //zyl
        0,						/**< IsiSensor_t.pIsiEnableSensorOTP>*/  //zyl
        0,                      /**< IsiSensor_t.pIsiCreateSensorIss */
        0,                      /**< IsiSensor_t.pIsiReleaseSensorIss */
        0,                      /**< IsiSensor_t.pIsiGetCapsIss */
        0,                      /**< IsiSensor_t.pIsiSetupSensorIss */
        0,                      /**< IsiSensor_t.pIsiChangeSensorResolutionIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetStreamingIss */
        0,                      /**< IsiSensor_t.pIsiSensorSetPowerIss */
        0,                      /**< IsiSensor_t.pIsiCheckSensorConnectionIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorRevisionIss */
        0,                      /**< IsiSensor_t.pIsiRegisterReadIss */
        0,                      /**< IsiSensor_t.pIsiRegisterWriteIss */
        0,                      /**< IsiSensor_t.pIsiIsEvenFieldIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorModeIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorFiledStatIss */

        0,                      /**< IsiSensor_t.pIsiExposureControlIss */
        0,                      /**< IsiSensor_t.pIsiGetGainLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeLimitsIss */
        0,                      /**< IsiSensor_t.pIsiGetCurrentExposureIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetGainIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetGainIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetIntegrationTimeIncrementIss */
        0,                      /**< IsiSensor_t.pIsiSetIntegrationTimeIss */
        0,                      /**< IsiSensor_t.pIsiGetResolutionIss */
        0,                      /**< IsiSensor_t.pIsiGetAfpsInfoIss */

        0,                      /**< IsiSensor_t.pIsiGetCalibKFactor */
        0,                      /**< IsiSensor_t.pIsiGetCalibPcaMatrix */
        0,                      /**< IsiSensor_t.pIsiGetCalibSvdMeanValue */
        0,                      /**< IsiSensor_t.pIsiGetCalibCenterLine */
        0,                      /**< IsiSensor_t.pIsiGetCalibClipParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibGlobalFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetCalibFadeParam */
        0,                      /**< IsiSensor_t.pIsiGetIlluProfile */
        0,                      /**< IsiSensor_t.pIsiGetLscMatrixTable */

        0,                      /**< IsiSensor_t.pIsiMdiInitMotoDriveMds */
        0,                      /**< IsiSensor_t.pIsiMdiSetupMotoDrive */
        0,                      /**< IsiSensor_t.pIsiMdiFocusSet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusGet */
        0,                      /**< IsiSensor_t.pIsiMdiFocusCalibrate */

        0,                      /**< IsiSensor_t.pIsiGetSensorMipiInfoIss */

        0,                      /**< IsiSensor_t.pIsiActivateTestPattern */
        0,			/**< IsiSetSensorFrameRateLimitIss */
        0,			/**< IsiSensor_t.pIsiGetColorIss */
    },
    OV9282_IsiGetSensorI2cInfo,
};


