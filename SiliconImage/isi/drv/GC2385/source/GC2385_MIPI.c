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
 * @file GC2385.c
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

#include "GC2385_MIPI_priv.h"

#define  GC2385_NEWEST_TUNING_XML "GC2385_V1.0"

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( GC2385_INFO , "GC2385: ", INFO,    1U );
CREATE_TRACER( GC2385_WARN , "GC2385: ", WARNING, 1U );
CREATE_TRACER( GC2385_ERROR, "GC2385: ", ERROR,   1U );

CREATE_TRACER( GC2385_DEBUG, "GC2385: ", INFO,     1U );

CREATE_TRACER( GC2385_REG_INFO , "GC2385: ", INFO, 1);
CREATE_TRACER( GC2385_REG_DEBUG, "GC2385: ", INFO, 1U );

#define GC2385_SLAVE_ADDR       0x6EU                           /**< i2c slave address of the GC2385 camera sensor */
#define GC2385_SLAVE_ADDR2      0x6EU
#define GC2385_SLAVE_AF_ADDR    0x00U                           /**< i2c slave address of the GC2385 integrated AF */

#define GC2385_MIN_GAIN_STEP   	( 1.0f / 64.0f);		/**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */		 
#define GC2385_MAX_GAIN_AEC    	( 17.06f ) 				/**< max. gain used by the AEC (arbitrarily chosen) */


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
#define MDI_SLEW_RATE_CTRL 6U /* S3..0 */



/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char GC2385_g_acName[] = "GC2385_MIPI";

extern const IsiRegDescription_t GC2385_g_aRegDescription[];
extern const IsiRegDescription_t GC2385_g_1600x1200[];
extern const IsiRegDescription_t GC2385_g_1600x1200_30fps[];
extern const IsiRegDescription_t GC2385_g_1600x1200_20fps[];
extern const IsiRegDescription_t GC2385_g_1600x1200_15fps[];
extern const IsiRegDescription_t GC2385_g_1600x1200_10fps[];


const IsiSensorCaps_t GC2385_g_IsiSensorDefaultConfig;


#define GC2385_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define GC2385_I2C_NR_ADR_BYTES     (1U)                        // 1 byte base address and 2 bytes sub address
#define GC2385_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers


static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_ONE_LANE;//SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_ONE_LANE



/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT GC2385_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT GC2385_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT GC2385_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT GC2385_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT GC2385_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT GC2385_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT GC2385_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT GC2385_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT GC2385_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT GC2385_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT GC2385_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT GC2385_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT GC2385_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT GC2385_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT GC2385_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT GC2385_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT GC2385_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT GC2385_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT GC2385_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT GC2385_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT GC2385_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT GC2385_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT GC2385_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT GC2385_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT GC2385_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT GC2385_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT GC2385_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT GC2385_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT GC2385_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT GC2385_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT GC2385_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT GC2385_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT GC2385_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT GC2385_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT GC2385_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT GC2385_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT GC2385_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT GC2385_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);




/*****************************************************************************/
/**
 *          GC2385_IsiCreateSensorIss
 *
 * @brief   This function creates a new OV13850 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT GC2385_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    GC2385_Context_t *pSensorCtx;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pSensorCtx = ( GC2385_Context_t * )malloc ( sizeof (GC2385_Context_t) );
    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR,  "%s: Can't allocate GC2385 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorCtx, 0, sizeof( GC2385_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pSensorCtx );
        return ( result );
    }

    pSensorCtx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pSensorCtx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pSensorCtx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pSensorCtx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? GC2385_SLAVE_ADDR : pConfig->SlaveAddr;
    pSensorCtx->IsiCtx.NrOfAddressBytes       = 1U;

    pSensorCtx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pSensorCtx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? GC2385_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pSensorCtx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pSensorCtx->IsiCtx.pSensor                = pConfig->pSensor;

    pSensorCtx->Configured             = BOOL_FALSE;
    pSensorCtx->Streaming              = BOOL_FALSE;
    pSensorCtx->TestPattern            = BOOL_FALSE;
    pSensorCtx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pSensorCtx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pSensorCtx->VcmInfo.RatedCurrent   = pSensorCtx->VcmInfo.StartCurrent + MAX_LOG*pSensorCtx->VcmInfo.Step;
    pSensorCtx->VcmInfo.StepMode       = pConfig->VcmStepMode;  

    pSensorCtx->IsiSensorMipiInfo.sensorHalDevID = pSensorCtx->IsiCtx.HalDevID;
    if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( GC2385_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }

    pConfig->hSensor = ( IsiSensorHandle_t )pSensorCtx;

    result = HalSetCamConfig( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an Sensor instance.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT GC2385_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)GC2385_IsiSensorSetStreamingIss( pSensorCtx, BOOL_FALSE );
    (void)GC2385_IsiSensorSetPowerIss( pSensorCtx, BOOL_FALSE );

    (void)HalDelRef( pSensorCtx->IsiCtx.HalHandle );

    MEMSET( pSensorCtx, 0, sizeof( GC2385_Context_t ) );
    free ( pSensorCtx );

    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCapsIss
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
static RESULT GC2385_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t mipi_lanes
)
{

    RESULT result = RET_SUCCESS;
	TRACE( GC2385_INFO, "%s(%d) enter ....\n", __FUNCTION__, __LINE__);
    if ( pIsiSensorCaps == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
        if(mipi_lanes == SUPPORT_MIPI_ONE_LANE){
            switch (pIsiSensorCaps->Index) 
            {
				
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1600_1200P30;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1600_1200P20;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1600_1200P15;
                    break;
                }
				
                case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_1600_1200P10;
                    break;
                }
                
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }            
        }else if(mipi_lanes == SUPPORT_MIPI_TWO_LANE){
        	switch(pIsiSensorCaps->Index){
        		default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }
        	}
        	
        }else if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE){
        	switch(pIsiSensorCaps->Index){
        		default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }
        	}
        }

    
        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT;
        pIsiSensorCaps->Mode            = ISI_MODE_MIPI;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_BGBGGRGR;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_POS;
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );//»µµã

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10;
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP );
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;
    }
end:

    return ( result );
}

static RESULT GC2385_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    result = GC2385_IsiGetCapsIssInternal(pIsiSensorCaps, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes);
    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t GC2385_g_IsiSensorDefaultConfig =
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
    ISI_RES_1600_1200P30,            // Res
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
 *          GC2385_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_SetupOutputFormat
(
    GC2385_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s(%d) (enter)\n", __FUNCTION__, __LINE__);

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by Sensor, so the YCSequence parameter is not checked */
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
            TRACE( GC2385_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
		case ISI_BPAT_RGRGGBGB:
		case ISI_BPAT_GRGRBGBG:
		case ISI_BPAT_GBGBRGRG:
		case ISI_BPAT_BGBGGRGR:
		{
            break;
        }
        default:
        {
            TRACE( GC2385_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC2385_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( GC2385_ERROR, "%s%s (exit)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

#if 0
int GC2385_get_PCLK( GC2385_Context_t *pSensorCtx, int XVCLK)
{
	// calculate sysclk
	uint32_t sysclk,divx4,pre_div,temp1,temp2;
	RESULT result = RET_SUCCESS;

	TRACE( GC2385_INFO, "%s(%d): (enter)\n", __FUNCTION__, __LINE__);
	/*---define Sensor Pixel Clock:
	GC2385 Pixel Clock= InMclk * (0xf8[5:0] + 1) /2/(0xfa[7:4]+1)---*/
	result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x03);
	result = GC2385_IsiRegReadIss ( pSensorCtx, 0xf8, &temp1);
	result = GC2385_IsiRegReadIss ( pSensorCtx, 0xfa, &temp2);
	divx4 = (temp1&0x1f)+1;
	pre_div = (temp2&0xf0)>>4;
	sysclk = (XVCLK*divx4)/2/(pre_div+1);
	TRACE( GC2385_INFO, "%s(%d): divx4: %d;pre_div: %d Pixel Clock: %d\n",
		   __FUNCTION__, __LINE__, divx4, pre_div, sysclk);
	TRACE( GC2385_INFO, "%s(%d): (exit)\n", __FUNCTION__, __LINE__);
	return 41000000;
	//return sysclk;
 }
#endif
/*****************************************************************************/
/**
 *          GC2385_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_SetupOutputWindowInternal
(
    GC2385_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
    float    rVtPixClkFreq      = 0.0f;
    int xclk = 24000000;

    TRACE( GC2385_INFO, "%s (enter)---pConfig->Resolution:%x\n", __FUNCTION__,pConfig->Resolution);

	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	    /* resolution */
	    switch ( pConfig->Resolution )
	    {
	
	    case ISI_RES_1600_1200P30:
			case ISI_RES_1600_1200P20:
			case ISI_RES_1600_1200P15:
			case ISI_RES_1600_1200P10:
	        {
	            if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
						if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pSensorCtx, GC2385_g_1600x1200)) != RET_SUCCESS){
							result = RET_FAILURE;
							TRACE( GC2385_ERROR, "%s: failed to set one lane GC2385_g_1600x1200 \n", __FUNCTION__ );
			            }
					}
                    

          if (pConfig->Resolution == ISI_RES_1600_1200P30) {						 
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, GC2385_g_1600x1200_30fps);
					}else if (pConfig->Resolution == ISI_RES_1600_1200P20) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, GC2385_g_1600x1200_20fps);
					}else if (pConfig->Resolution == ISI_RES_1600_1200P15) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, GC2385_g_1600x1200_15fps);
					}else if (pConfig->Resolution == ISI_RES_1600_1200P10) {
						result = IsiRegDefaultsApply( (IsiSensorHandle_t)pSensorCtx, GC2385_g_1600x1200_10fps);
					}  
                }

	            usLineLengthPck = 1079;//0x438
				//have to reset mipi freq here
				pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 656; //656
						//vts=41MHZ/(fps*hts) = 41000000/(30*1079) = 1266;
				if (pConfig->Resolution == ISI_RES_1600_1200P30) {				
					TRACE( GC2385_ERROR, "%s: ISI_RES_1600_1200P30 \n", __FUNCTION__ );
                    usFrameLengthLines = 0x4f2;	//0x4f1
				}else if(pConfig->Resolution == ISI_RES_1600_1200P20) {
					TRACE( GC2385_ERROR, "%s: ISI_RES_1600_1200P20 \n", __FUNCTION__ );                    
	            	usFrameLengthLines = 0x76c; //0x76a
				}else if(pConfig->Resolution == ISI_RES_1600_1200P15) {				
					TRACE( GC2385_ERROR, "%s: ISI_RES_1600_1200P15 \n", __FUNCTION__ );
	            	usFrameLengthLines = 0x9e5; //0x9e2
				}else if(pConfig->Resolution == ISI_RES_1600_1200P10) {
					TRACE( GC2385_ERROR, "%s: ISI_RES_1600_1200P10 \n", __FUNCTION__ );
	            	usFrameLengthLines = 0xed8;       //0xed4
				}
	            break; 
	        }

	        default:
	        {
	            TRACE( GC2385_ERROR, "%s: one lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
		
	}

	rVtPixClkFreq = 41000000;
	// store frame timing for later use in AEC module
//	rVtPixClkFreq = GC2385_get_PCLK(pSensorCtx, xclk);;
    pSensorCtx->VtPixClkFreq     = rVtPixClkFreq;
	TRACE( GC2385_INFO, "%s: rVtPixClkFreq = %f++++++\n", __FUNCTION__,rVtPixClkFreq );
    pSensorCtx->LineLengthPck    = usLineLengthPck;
    pSensorCtx->FrameLengthLines = usFrameLengthLines;	
	pSensorCtx->AecMaxIntegrationTime = ( ((float)pSensorCtx->FrameLengthLines) * ((float)pSensorCtx->LineLengthPck) )/ pSensorCtx->VtPixClkFreq;
    TRACE( GC2385_DEBUG, "%s ethan AecMaxIntegrationTime:%f(****************exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
    					pSensorCtx->AecMaxIntegrationTime,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);


    return ( result );
}




/*****************************************************************************/
/**
 *          GC2385_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_SetupImageControl
(
    GC2385_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = GC2385_IsiRegReadIss(  pSensorCtx, GC2385_BLC_CTRL00, &RegValue );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = GC2385_IsiRegReadIss(  pSensorCtx, GC2385_BLC_CTRL00, &RegValue );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = GC2385_IsiRegReadIss(  pSensorCtx, GC2385_AEC_MANUAL, &RegValue );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = GC2385_IsiRegReadIss(  pSensorCtx, GC2385_ISP_CTRL01, &RegValue );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = GC2385_IsiRegReadIss(  pSensorCtx, GC2385_AEC_MANUAL, &RegValue );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = GC2385_IsiRegReadIss( pSensorCtx, GC2385_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( GC2385_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT GC2385_SetupOutputWindow
(
    GC2385_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return GC2385_SetupOutputWindowInternal(pSensorCtx,pConfig,BOOL_TRUE, BOOL_FALSE);
}


/*****************************************************************************/
/**
 *          GC2385_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in Sensor-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      Sensor context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_AecSetModeParameters
(
    GC2385_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	//return ( result );
    //TRACE( GC2385_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");
    TRACE( GC2385_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"AFPS Not run",
        pSensorCtx->Config.Resolution, pConfig->Resolution);

    if ( (pSensorCtx->VtPixClkFreq == 0.0f) )
    {
        TRACE( GC2385_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pSensorCtx->AecMaxIntegrationTime = ( ((float)(pSensorCtx->FrameLengthLines - 4)) * ((float)pSensorCtx->LineLengthPck) )/ pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecMinIntegrationTime = 0.0001f;

    pSensorCtx->AecMaxGain = GC2385_MAX_GAIN_AEC;
    pSensorCtx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pSensorCtx->AecIntegrationTimeIncrement = ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecGainIncrement = GC2385_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pSensorCtx->AecCurGain               = pSensorCtx->AecMinGain;
    pSensorCtx->AecCurIntegrationTime    = 0.0f;
    pSensorCtx->OldCoarseIntegrationTime = 0;
    pSensorCtx->OldFineIntegrationTime   = 0;
	//pSensorCtx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( GC2385_DEBUG, "%s%s (exit) ethan pSensorCtx->AecMaxIntegrationTime:%f, pSensorCtx->FrameLengthLines:%d, pSensorCtx->LineLengthPck:%d,pSensorCtx->VtPixClkFreq:%f\n",
    __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
    pSensorCtx->AecMaxIntegrationTime,
    pSensorCtx->FrameLengthLines,
    pSensorCtx->LineLengthPck,
    pSensorCtx->VtPixClkFreq
    );
	TRACE( GC2385_INFO, "%s(%d) (exit)\n", __FUNCTION__, __LINE__);
    return ( result );
}

/*****************************************************************************/
/**
 *          GC2385_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      Sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pSensorCtx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface),  Bit[7]software_reset 1:soft reset 0: normal mode */
    result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_SOFTWARE_RST, 0x80u );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00u );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    osSleep( 10 );
    result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_MODE_SELECT, GC2385_MODE_SELECT_OFF );//GC2385_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: Can't write GC2385 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
   // result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00u );
   // RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
		result = IsiRegDefaultsApply( pSensorCtx, GC2385_g_aRegDescription);
	}
    

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pSensorCtx, GC2385_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = GC2385_SetupOutputFormat( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = GC2385_SetupOutputWindow( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = GC2385_SetupImageControl( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = GC2385_AecSetModeParameters( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pSensorCtx->Configured = BOOL_TRUE;
    }

		result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00u );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_MODE_SELECT, GC2385_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
    		result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00u );
    		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        TRACE( GC2385_ERROR, "%s: Can't write GC2385 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiChangeSensorResolutionIss
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
 *
 *****************************************************************************/
static RESULT GC2385_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (GC2385_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

     if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }
    if ( Resolution == pSensorCtx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;

		bool_t res_no_chg;
        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
            (ISI_RES_H_GET(Resolution)==ISI_RES_H_GET(pSensorCtx->Config.Resolution))) ) {

            if (pSensorCtx->Streaming != BOOL_FALSE) {
                TRACE( GC2385_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
		TRACE( GC2385_ERROR, "%s(%d) enter....  \n", __FUNCTION__, __LINE__);
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( GC2385_INFO, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context
        pSensorCtx->Config.Resolution = Resolution;

        // tell sensor about that
        result = GC2385_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config, BOOL_TRUE, res_no_chg );
        if ( result != RET_SUCCESS )
        {
            TRACE( GC2385_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pSensorCtx->AecCurGain;
        float OldIntegrationTime = pSensorCtx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = GC2385_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( GC2385_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = GC2385_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( GC2385_ERROR, "%s: GC2385_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
        	*pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        //	*pNumberOfFramesToSkip = 0;
    }

    TRACE( GC2385_INFO, "%s (exit)  result: 0x%x\n", __FUNCTION__, result);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiSensorSetStreamingIss
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
static RESULT GC2385_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;
	uint32_t RegValue2 = 0;

    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);
    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) || (pSensorCtx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
		//while(1);
    result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00);
		result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_MODE_SELECT, 0x90); //0x91 double lanes
		//result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00);
		TRACE(GC2385_INFO," STREAM ON ++++++++++++++");
		//while(1);
    }
    else
    {   
        /* disable streaming */
    result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_PAGE_SELECT, 0x00);
		result = GC2385_IsiRegWriteIss ( pSensorCtx, GC2385_MODE_SELECT, 0x00);
        TRACE(GC2385_INFO," STREAM OFF ++++++++++++++");
    }

    if (result == RET_SUCCESS)
    {
        pSensorCtx->Streaming = on;
    }

    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      Sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pSensorCtx->Configured = BOOL_FALSE;
    pSensorCtx->Streaming  = BOOL_FALSE;
		TRACE( GC2385_ERROR, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
    		TRACE( GC2385_ERROR, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        osSleep( 50 );
    }

    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = GC2385_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (GC2385_CHIP_ID_LOW_BYTE_DEFAULT);

    result = GC2385_IsiGetSensorRevisionIss( handle, &value );

    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( GC2385_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }
		 TRACE( GC2385_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetSensorRevisionIss
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
static RESULT GC2385_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;
	//uint32_t vcm_pos = MAX_LOG;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = GC2385_IsiRegReadIss ( handle, GC2385_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = GC2385_IsiRegReadIss ( handle, GC2385_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF) );
    TRACE( GC2385_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiRegReadIss
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
 *
 *****************************************************************************/
static RESULT GC2385_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
	IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;
    RESULT result = RET_SUCCESS;
	uint8_t NrOfBytes;

	TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

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
    	NrOfBytes = IsiGetNrDatBytesIss( address, GC2385_g_aRegDescription);
    	if ( !NrOfBytes )
    	{
        	NrOfBytes = 1;
    	}
        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, 1, BOOL_TRUE );
    }

	TRACE( GC2385_INFO, "%s (exit: reg: 0x%x reg_value: 0x%02x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiRegWriteIss
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
 *
 *****************************************************************************/
static RESULT GC2385_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, GC2385_g_aRegDescription);
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }
    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );


    return ( result );
}

#define GC2385_ANALOG_GAIN_1 64    /*1.00x*/
#define GC2385_ANALOG_GAIN_2 92   // 1.43x
#define GC2385_ANALOG_GAIN_3 127  // 1.99x
#define GC2385_ANALOG_GAIN_4 183  // 2.86x
#define GC2385_ANALOG_GAIN_5 257  // 4.01x
#define GC2385_ANALOG_GAIN_6 369  // 5.76x
#define GC2385_ANALOG_GAIN_7 531  //8.30x
#define GC2385_ANALOG_GAIN_8 750  // 11.72x
#define GC2385_ANALOG_GAIN_9 1092 // 17.06x

#define GC2385_ANALOG_GAIN_REG          0xb6/* Bit 8 */
#define GC2385_PREGAIN_HIGHBITS_REG     0xb1
#define GC2385_PREGAIN_LOWBITS_REG      0xb2


#define GC2385_AEC_EXPO_COARSE_2ND_REG      0x03    /* Exposure Bits 8-15 */
#define GC2385_AEC_EXPO_COARSE_1ST_REG      0x04    /* Exposure Bits 0-7 */

#define GC2385_FETCH_2ND_BYTE_EXP(VAL)      ((VAL >> 8) & 0x3F) /* 4 Bits */
#define GC2385_FETCH_1ST_BYTE_EXP(VAL)      (VAL & 0xFF)	/* 8 Bits */

/*****************************************************************************/
/**
 *          GC2385_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          Sensor instance
 *
 * @param   handle       Sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( GC2385_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pSensorCtx->AecMinGain;
    *pMaxGain = pSensorCtx->AecMaxGain;

    TRACE( GC2385_INFO, "%s: pMinGain:%f,pMaxGain:%f(exit)\n", __FUNCTION__,pSensorCtx->AecMinGain,pSensorCtx->AecMaxGain);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          Sensor instance
 *
 * @param   handle       Sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC2385_INFO, "%s: (------oyyf enter) \n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( GC2385_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pSensorCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pSensorCtx->AecMaxIntegrationTime;
	TRACE( GC2385_INFO, "%s(%d): AecMinIntegrationTime: %f, AecMaxIntegrationTime:%f\n", 
		__FUNCTION__, __LINE__, pSensorCtx->AecMinIntegrationTime, pSensorCtx->AecMaxIntegrationTime);
    TRACE( GC2385_INFO, "%s: (------oyyf exit) (\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC2385_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  Sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pSensorCtx->AecCurGain;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  Sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecGainIncrement;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC2385_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  Sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 *
 *****************************************************************************/
RESULT GC2385_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
    RESULT result = RET_SUCCESS;

		uint16_t usGain = 0;
    uint32_t temp = 0;
		TRACE( GC2385_INFO, "%s(%d): (enter)\n", __FUNCTION__, __LINE__);
    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( GC2385_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }
  
    if( NewGain < pSensorCtx->AecMinGain ) NewGain = pSensorCtx->AecMinGain;
    if( NewGain > pSensorCtx->AecMaxGain ) NewGain = pSensorCtx->AecMaxGain;
	usGain = (uint16_t)(NewGain * 64.0f+ 0.5f);
	if (usGain < 0x40)
        usGain = 0x40;
	
    // write new gain into sensor registers, do not write if nothing has changed
    if(pSensorCtx->AecCurGain != NewGain)
    {
//new gain formula 
    	result = GC2385_IsiRegWriteIss(pSensorCtx, 0xfe, 0x00);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    	if ((GC2385_ANALOG_GAIN_1 <= usGain) && (usGain < GC2385_ANALOG_GAIN_2)) {
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x73);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa2);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x0);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_1;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, temp&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		} else if ((GC2385_ANALOG_GAIN_2 <= usGain) && (usGain < GC2385_ANALOG_GAIN_3)) {
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x73);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa2);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x1);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_2;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, temp&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		} else if ((GC2385_ANALOG_GAIN_3 <= usGain) && (usGain < GC2385_ANALOG_GAIN_4)) {
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x73);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa2);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x2);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_3;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		} else if((GC2385_ANALOG_GAIN_4<= usGain)&&(usGain < GC2385_ANALOG_GAIN_5)){
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x73);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa2);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x3);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_4;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}else if((GC2385_ANALOG_GAIN_5<= usGain)&&(usGain < GC2385_ANALOG_GAIN_6)){
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x73);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa3);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x4);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_5;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}else if((GC2385_ANALOG_GAIN_6<= usGain)&&(usGain < GC2385_ANALOG_GAIN_7)){
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x74);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa3);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x5);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_6;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}else if((GC2385_ANALOG_GAIN_7<= usGain)&&(usGain < GC2385_ANALOG_GAIN_8)){
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x74);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa3);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x6);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_7;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}else if((GC2385_ANALOG_GAIN_8<= usGain)&&(usGain < GC2385_ANALOG_GAIN_9)){
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x74);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa3);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x7);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_8;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}else {
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x20, 0x75);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, 0x22, 0xa4);
       		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
			result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_ANALOG_GAIN_REG, 0x8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        	temp = 256*usGain/GC2385_ANALOG_GAIN_9;
        	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_HIGHBITS_REG, temp>>8);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
       	 	result = GC2385_IsiRegWriteIss( pSensorCtx, GC2385_PREGAIN_LOWBITS_REG, (temp)&0xff);
        	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		}
		
    }
	pSensorCtx->OldGain = usGain;
    //calculate gain actually set
    pSensorCtx->AecCurGain = ( (float)usGain ) / 64.0f;
   
    //return current state
    *pSetGain = pSensorCtx->AecCurGain;
    TRACE( GC2385_INFO, "%s: setgain mubiao(%f) shiji(%f) usGain(%d)\n", __FUNCTION__, NewGain, *pSetGain,usGain);
	TRACE( GC2385_INFO, "%s(%d): (exit)\n", __FUNCTION__, __LINE__);
    return ( result );
}

     

/*****************************************************************************/
/**
 *          GC2385_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  Sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  Sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (-------oyyf)(enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = (float)pSensorCtx->AecIntegrationTimeIncrement;

    TRACE( GC2385_INFO, "%s: (------oyyf)(exit) pSensorCtx->AecIntegrationTimeIncrement(%u)\n", __FUNCTION__,pSensorCtx->AecIntegrationTimeIncrement);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  Sensor instance handle
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
 *
 *****************************************************************************/
RESULT GC2385_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t 			*pNumberOfFramesToSkip
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
    RESULT result = RET_SUCCESS;
    uint32_t exp_high = 0;
		uint32_t exp_low = 0;

	uint32_t CoarseIntegrationTime = 0;
	//uint32_t FineIntegrationTime   = 0; //not supported by Sensor
	uint32_t result_intertime= 0;
	float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

	TRACE( GC2385_INFO, "%s czf exp value :%f to be set\n", __FUNCTION__ ,NewIntegrationTime);

    TRACE( GC2385_INFO, "%s: (enter) NewIntegrationTime: %f (min: %lu   max: %lu)\n", __FUNCTION__,
        NewIntegrationTime,
        pSensorCtx->AecMinIntegrationTime,
        pSensorCtx->AecMaxIntegrationTime);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( GC2385_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    //maximum and minimum integration time is limited by the sensor, if this limit is not
    //considered, the exposure control loop needs lots of time to return to a new state
    //so limit to allowed range
    if ( NewIntegrationTime > pSensorCtx->AecMaxIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pSensorCtx->AecMinIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMinIntegrationTime;
	ShutterWidthPck = NewIntegrationTime * ( (float)pSensorCtx->VtPixClkFreq );
	// avoid division by zero
    if ( pSensorCtx->LineLengthPck == 0 )
    {
        TRACE( GC2385_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }
	CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) + 0.5f );
	#if 1
	// write new integration time into sensor registers
	// do not write if nothing has changed
	if( CoarseIntegrationTime != pSensorCtx->OldCoarseIntegrationTime )
	{
		result = GC2385_IsiRegWriteIss( pSensorCtx, 0xfe, 0x00 );
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		result = GC2385_IsiRegWriteIss( pSensorCtx, 0x03, GC2385_FETCH_2ND_BYTE_EXP(CoarseIntegrationTime) );
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		result = GC2385_IsiRegWriteIss( pSensorCtx, 0x04, GC2385_FETCH_1ST_BYTE_EXP(CoarseIntegrationTime) );
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
		pSensorCtx->OldCoarseIntegrationTime = CoarseIntegrationTime;	// remember current integration time
		*pNumberOfFramesToSkip = 1U; //skip 1 frame
	}
	else
	{
		*pNumberOfFramesToSkip = 0U; //no frame skip
	}
	#endif
	
	
	//calculate integration time actually set
    pSensorCtx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;

		TRACE( GC2385_INFO, "%s czf exp value :%f read after setting\n", __FUNCTION__ , pSensorCtx->AecCurIntegrationTime);

    //return current state
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

   // TRACE( Sensor_ERROR, "%s: SetTi=%f NewTi=%f\n", __FUNCTION__, *pSetIntegrationTime,NewIntegrationTime);
    TRACE( GC2385_INFO, "%s(%d): settime mubiao(%f) shiji(%f)\n", __FUNCTION__, __LINE__, NewIntegrationTime,*pSetIntegrationTime);
	TRACE( GC2385_DEBUG, "%s(%d):ethan "
         "pSensorCtx->VtPixClkFreq:%f pSensorCtx->LineLengthPck:%x ,"
         "*pSetIntegrationTime=%f    NewIntegrationTime=%f  CoarseIntegrationTime=%x ,"
         "result_intertime = %x\n H:%x\n M:%x\n L:%x", __FUNCTION__, __LINE__,
         pSensorCtx->VtPixClkFreq,pSensorCtx->LineLengthPck,
         *pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime,
         result_intertime,
         (CoarseIntegrationTime & 0x0000F000U) >> 12U,
         (CoarseIntegrationTime & 0x00000FF0U) >> 4U,
         (CoarseIntegrationTime & 0x0000000FU) << 4U);
    return ( result );
}

/*****************************************************************************/
/**
 *          GC2385_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  Sensor instance handle
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
 *
 *****************************************************************************/
RESULT GC2385_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
    RESULT result = RET_SUCCESS;
     TRACE( GC2385_ERROR, "%s(%d): czf test at start\n", __FUNCTION__, __LINE__);

    TRACE( GC2385_INFO, "%s(%d): (enter)\n", __FUNCTION__, __LINE__);
    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( GC2385_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );

    result = GC2385_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip);
    result = GC2385_IsiSetGainIss( handle, NewGain, pSetGain);

    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );  
    TRACE( GC2385_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip ); 
	TRACE( GC2385_INFO, "%s(%d) exit...\n", __FUNCTION__, __LINE__); 
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
RESULT GC2385_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
    RESULT result = RET_SUCCESS;
		TRACE( GC2385_ERROR, "%s(%d): czf test at start\n", __FUNCTION__, __LINE__);
    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pSensorCtx->AecCurGain;
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime * 1.0;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetResolutionIss
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
 *
 *****************************************************************************/
RESULT GC2385_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pSensorCtx->Config.Resolution;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pSensorCtx             Sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetAfpsInfoHelperIss(
    GC2385_Context_t   *pSensorCtx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (-----------ethan enter) pAfpsInfo->AecMaxIntTime(%lu) pSensorCtx->AecMaxIntegrationTime(%lu)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    DCT_ASSERT(pSensorCtx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pSensorCtx->Config.Resolution = Resolution;

    // tell sensor about that
    result = GC2385_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config,BOOL_FALSE,BOOL_FALSE);
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = GC2385_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC2385_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pSensorCtx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pSensorCtx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pSensorCtx->AecMinIntegrationTime ;
    pAfpsInfo->AecMaxIntTime        = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( GC2385_DEBUG, "%s: (-----------ethan exit) pAfpsInfo->AecMaxIntTime(%lu) pSensorCtx->AecMaxIntegrationTime(%lu)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC2385_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  Sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *
 *****************************************************************************/
RESULT GC2385_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;
		TRACE( GC2385_ERROR,  "%s: czf test afps at start",  __FUNCTION__ );
    if ( pSensorCtx == NULL )
    {
        TRACE( GC2385_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pSensorCtx->Config.Resolution;
    }
		
    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pSensorCtx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pSensorCtx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    GC2385_Context_t *pDummyCtx = (GC2385_Context_t*) malloc( sizeof(GC2385_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( GC2385_ERROR,  "%s: Can't allocate dummy GC2385 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pSensorCtx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = GC2385_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
        if ( lres == RET_SUCCESS ) \
        { \
            ++idx; \
        } \
        else \
        { \
            UPDATE_RESULT( result, lres ); \
        } \
    }

		TRACE( GC2385_ERROR,  "%s: czf test afps before switch",  __FUNCTION__ );
    // check which AFPS series is requested and build its params list for the enabled AFPS resolutions
	switch (pSensorCtx->IsiSensorMipiInfo.ucMipiLanes)
		{
			case SUPPORT_MIPI_ONE_LANE:
			{
				switch(Resolution)
				{
			    default:{
			       result = RET_NOTSUPP;
			    	break;
			    }
					case ISI_RES_1600_1200P30:
					case ISI_RES_1600_1200P20:
					case ISI_RES_1600_1200P15:
					case ISI_RES_1600_1200P10:
					{	
						TRACE( GC2385_ERROR,  "%s: czf test afps\n",  __FUNCTION__ );
						AFPSCHECKANDADD( ISI_RES_1600_1200P30);
						AFPSCHECKANDADD( ISI_RES_1600_1200P20);
						AFPSCHECKANDADD( ISI_RES_1600_1200P15);
						AFPSCHECKANDADD( ISI_RES_1600_1200P10);
						break;
					}
										
				}
	
				break;
			}
			
			default:
            TRACE( GC2385_ERROR,  "%s: pSensorCtx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;
			        
	}

    // release dummy context again
    free(pDummyCtx);

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibKFactor
 *
 * @brief   Returns the Sensor specific K-Factor
 *
 * @param   handle       Sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the Sensor specific PCA-Matrix
 *
 * @param   handle          Sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiCenterLine = (IsiLine_t*)&GC2385_CenterLine;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&GC2385_AwbClipParm;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              Sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiFadeParam = (IsiAwbFade2Parm_t *)&GC2385_AwbFade2Parm;

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC2385_IsiGetIlluProfile
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
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiIlluProfile == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetLscMatrixTable
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
 *
 *****************************************************************************/
static RESULT GC2385_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pLscMatrixTable == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    else
    {
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC2385_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          Sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          Sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
	GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    
    #if 1
    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( GC2385_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          Sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    #if 1
    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC2385_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT GC2385_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 0
    uint32_t ulRegValue = 0UL;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = GC2385_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = GC2385_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = GC2385_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = GC2385_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pSensorCtx->TestPattern = enable;
    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          GC2385_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          Sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *
 ******************************************************************************/
static RESULT GC2385_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

    ptIsiSensorMipiInfo->ucMipiLanes = pSensorCtx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pSensorCtx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pSensorCtx->IsiSensorMipiInfo.sensorHalDevID;


    TRACE( GC2385_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT GC2385_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( GC2385_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( GC2385_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT GC2385_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    GC2385_Context_t *pSensorCtx = (GC2385_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC2385_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( GC2385_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( GC2385_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = GC2385_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          GC2385_IsiGetSensorIss
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
RESULT GC2385_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC2385_INFO, "%s(%d): (enter)\n", __FUNCTION__, __LINE__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = GC2385_g_acName;
        pIsiSensor->pRegisterTable                      = GC2385_g_aRegDescription;
        pIsiSensor->pIsiSensorCaps                      = &GC2385_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= GC2385_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= GC2385_IsiGetSensorTuningXmlVersion;//oyyf
		pIsiSensor->pIsiCheckOTPInfo                    = NULL;//check_read_otp;//zyc
        pIsiSensor->pIsiCreateSensorIss                 = GC2385_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = GC2385_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = GC2385_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = GC2385_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = GC2385_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = GC2385_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = GC2385_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = GC2385_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = GC2385_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = GC2385_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = GC2385_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = GC2385_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = GC2385_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = GC2385_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = GC2385_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = GC2385_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = GC2385_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = GC2385_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = GC2385_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = GC2385_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = GC2385_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = GC2385_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = GC2385_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = GC2385_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = GC2385_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = GC2385_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = GC2385_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = GC2385_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = GC2385_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = GC2385_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = GC2385_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = GC2385_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = NULL; //GC2385_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = NULL; //GC2385_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = NULL; //GC2385_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = NULL; //GC2385_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = NULL; //GC2385_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = GC2385_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = GC2385_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( GC2385_INFO, "%s(%d) (exit)\n", __FUNCTION__, __LINE__);

    return ( result );
}

static RESULT GC2385_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;
	TRACE( GC2385_INFO, "%s(%d): (enter)\n", __FUNCTION__, __LINE__);
    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( GC2385_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = GC2385_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = GC2385_SLAVE_ADDR2;
    pSensorI2cInfo->reg_size = 1;
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
                while(GC2385_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = GC2385_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = GC2385_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = GC2385_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = GC2385_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
	TRACE( GC2385_INFO, "%s(%d): (exit)\n", __FUNCTION__, __LINE__);
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
    GC2385_IsiGetSensorIss,
    {
        0,                      /**< IsiGC2385_t.pszName */
        0,                      /**< IsiGC2385_t.pRegisterTable */
        0,                      /**< IsiGC2385_t.pIsiSensorCaps */
        0,						/**< IsiGC2385_t.pIsiGetSensorIsiVer_t>*/   //oyyf add
        0,                      /**< IsiGC2385_t.pIsiGetSensorTuningXmlVersion_t>*/   //oyyf add 
        0,                      /**< IsiGC2385_t.pIsiWhiteBalanceIlluminationChk>*/   //ddl@rock-chips.com 
        0,                      /**< IsiGC2385_t.pIsiWhiteBalanceIlluminationSet>*/   //ddl@rock-chips.com
        0,                      /**< IsiGC2385_t.pIsiCheckOTPInfo>*/  //zyc
        0,						/**< IsiGC2385_t.pIsiSetSensorOTPInfo>*/  //zyl
        0,						/**< IsiGC2385_t.pIsiEnableSensorOTP>*/  //zyl
        0,                      /**< IsiGC2385_t.pIsiCreateSensorIss */
        0,                      /**< IsiGC2385_t.pIsiReleaseSensorIss */
        0,                      /**< IsiGC2385_t.pIsiGetCapsIss */
        0,                      /**< IsiGC2385_t.pIsiSetupSensorIss */
        0,                      /**< IsiGC2385_t.pIsiChangeSensorResolutionIss */
        0,                      /**< IsiGC2385_t.pIsiSensorSetStreamingIss */
        0,                      /**< IsiGC2385_t.pIsiSensorSetPowerIss */
        0,                      /**< IsiGC2385_t.pIsiCheckSensorConnectionIss */
        0,                      /**< IsiGC2385_t.pIsiGetSensorRevisionIss */
        0,                      /**< IsiGC2385_t.pIsiRegisterReadIss */
        0,                      /**< IsiGC2385_t.pIsiRegisterWriteIss */

        0,                      /**< IsiSensor_t.pIsiIsEvenFieldIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorModeIss */
        0,                      /**< IsiSensor_t.pIsiGetSensorFiledStatIss */
        0,                      /**< IsiGC2385_t.pIsiExposureControlIss */
        0,                      /**< IsiGC2385_t.pIsiGetGainLimitsIss */
        0,                      /**< IsiGC2385_t.pIsiGetIntegrationTimeLimitsIss */
        0,                      /**< IsiGC2385_t.pIsiGetCurrentExposureIss */
        0,                      /**< IsiGC2385_t.pIsiGetGainIss */
        0,                      /**< IsiGC2385_t.pIsiGetGainIncrementIss */
        0,                      /**< IsiGC2385_t.pIsiSetGainIss */
        0,                      /**< IsiGC2385_t.pIsiGetIntegrationTimeIss */
        0,                      /**< IsiGC2385_t.pIsiGetIntegrationTimeIncrementIss */
        0,                      /**< IsiGC2385_t.pIsiSetIntegrationTimeIss */
        0,                      /**< IsiGC2385_t.pIsiGetResolutionIss */
        0,                      /**< IsiGC2385_t.pIsiGetAfpsInfoIss */

        0,                      /**< IsiGC2385_t.pIsiGetCalibKFactor */
        0,                      /**< IsiGC2385_t.pIsiGetCalibPcaMatrix */
        0,                      /**< IsiGC2385_t.pIsiGetCalibSvdMeanValue */
        0,                      /**< IsiGC2385_t.pIsiGetCalibCenterLine */
        0,                      /**< IsiGC2385_t.pIsiGetCalibClipParam */
        0,                      /**< IsiGC2385_t.pIsiGetCalibGlobalFadeParam */
        0,                      /**< IsiGC2385_t.pIsiGetCalibFadeParam */
        0,                      /**< IsiGC2385_t.pIsiGetIlluProfile */
        0,                      /**< IsiGC2385_t.pIsiGetLscMatrixTable */

        0,                      /**< IsiGC2385_t.pIsiMdiInitMotoDriveMds */
        0,                      /**< IsiGC2385_t.pIsiMdiSetupMotoDrive */
        0,                      /**< IsiGC2385_t.pIsiMdiFocusSet */
        0,                      /**< IsiGC2385_t.pIsiMdiFocusGet */
        0,                      /**< IsiGC2385_t.pIsiMdiFocusCalibrate */

        0,                      /**< IsiGC2385_t.pIsiGetSensorMipiInfoIss */

        0,                      /**< IsiGC2385_t.pIsiActivateTestPattern */
        0,			/**< IsiSetSensorFrameRateLimitIss */
        0,			/**< IsiSensor_t.pIsiGetColorIss */
    },
    GC2385_IsiGetSensorI2cInfo,
};


