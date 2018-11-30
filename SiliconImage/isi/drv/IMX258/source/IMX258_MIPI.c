/**
 * @file IMX258_MIPI.c
 *
 * @brief
 *   07/09/2018 IMX258_MIPI.c make
 *	 
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

#include "IMX258_MIPI_priv.h"

#define  IMX258_NEWEST_TUNING_XML "18-7-2014_oyyf-hkw_IMX258_CMK-CB0407-FV1_v0.1.2"


/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( IMX258_INFO , "IMX258: ", INFO,    1U );
CREATE_TRACER( IMX258_WARN , "IMX258: ", WARNING, 1U );
CREATE_TRACER( IMX258_ERROR, "IMX258: ", ERROR,   1U );

CREATE_TRACER( IMX258_DEBUG, "IMX258: ", WARNING,  1U );

CREATE_TRACER( IMX258_NOTICE0 , "IMX258: ", TRACE_NOTICE0, 1);
CREATE_TRACER( IMX258_NOTICE1, "IMX258: ", TRACE_NOTICE1, 1U );


#define IMX258_SLAVE_ADDR       0x20U                           /**< i2c slave address of the IMX258 camera sensor */
#define IMX258_SLAVE_ADDR2      0x34U
#define IMX258_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the IMX258 integrated AD5820 */



#define IMX258_MAXN_GAIN 		(512.0f)
#define IMX258_MIN_GAIN_STEP   ( 1.0f / IMX258_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define IMX258_MAX_GAIN_AEC    ( 16.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
const char IMX258_g_acName[] = "IMX258_MIPI";

extern const IsiRegDescription_t IMX258_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t IMX258_g_2096x1560_fourlane[];
extern const IsiRegDescription_t IMX258_g_2096x1560P30_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_2096x1560P25_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_2096x1560P20_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_2096x1560P15_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_2096x1560P10_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120_fourlane[];
extern const IsiRegDescription_t IMX258_g_4208x3120P30_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120P25_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120P20_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120P15_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120P10_fourlane_fpschg[];
extern const IsiRegDescription_t IMX258_g_4208x3120P7_fourlane_fpschg[];


const IsiSensorCaps_t IMX258_g_IsiSensorDefaultConfig;


#define IMX258_I2C_NR_ADR_BYTES     (2U)                        // 1 byte base address
#define IMX258_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_FOUR_LANE




/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT IMX258_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT IMX258_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT IMX258_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT IMX258_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT IMX258_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT IMX258_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT IMX258_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT IMX258_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT IMX258_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT IMX258_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT IMX258_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT IMX258_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT IMX258_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT IMX258_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT IMX258_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT IMX258_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT IMX258_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT IMX258_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT IMX258_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT IMX258_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT IMX258_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT IMX258_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT IMX258_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT IMX258_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT IMX258_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT IMX258_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT IMX258_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT IMX258_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT IMX258_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT IMX258_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT IMX258_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT IMX258_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT IMX258_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT IMX258_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT IMX258_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT IMX258_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT IMX258_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT IMX258_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);


/* OTP START*/
struct otp_struct {
		int base_info_flag;
    int awb_flag;
    int lsc_flag;
    int spc_flag;
    int module_integrator_id;
    int production_year;
    int production_month;
    int production_day;
    int current_R;
    int current_Gr;
    int current_Gb;
    int current_B;
    int golden_R;
    int golden_Gr;
    int golden_Gb;
    int golden_B;
};

static struct otp_struct g_otp_info ={0};

int  RG_Ratio_Typical=0;
int  BG_Ratio_Typical=0;

#define RG_Ratio_Typical_Default 0x281;
#define BG_Ratio_Typical_Default 0x259;

#define MODULE_GZ 1
#define MODULE_CMK 2
#define MODULE_NAME MODULE_GZ

#if MODULE_NAME == MODULE_GZ
	#define Sensor_OTP_SLAVE_ADDR   0xA0U
	#define MODULE_INFO_FLAG_REG 0X0000
	#define MODULE_ID_REG 0X0005
	#define MODULE_PRODUCT_YEAR_REG 0X000A
	#define MODULE_PRODUCT_MONTH_REG 0X000B
	#define MODULE_PRODUCT_DAY_REG 0X000C
	#define MODULE_AWB_FLAG_REG 0x001c
	#define MODULE_CUR_R_REG 0x001d
	#define MODULE_CUR_Gr_REG 0x001e
	#define MODULE_CUR_Gb_REG 0x001f
	#define MODULE_CUR_B_REG 0x0020
	#define MODULE_GOLDEN_R_REG 0x0021
	#define MODULE_GOLDEN_Gr_REG 0x0022
	#define MODULE_GOLDEN_Gb_REG 0x0023
	#define MODULE_GOLDEN_B_REG 0x0024
	#define MODULE_AWB_CHECKSUM_REG 0x0025
	#define MODULE_LSC_FLAG_REG 0X003A
	#define MODULE_LSC_DATA_START_REG 0x003B
	#define MODULE_LSC_CHECKSUM_REG 0x0233
	#define MODULE_SPC_FLAG_REG 0X0CE1
	#define MODULE_SPC_DATA_START_REG 0x0CE2
	#define MODULE_SPC_CHECKSUM_REG 0x0d60
	
#elif MODULE_NAME == MODULE_CMK
	#define Sensor_OTP_SLAVE_ADDR   0xB0U
	#define MODULE_INFO_FLAG_REG 0X0C00
	#define MODULE_ID_REG 0X0C01
	#define MODULE_PRODUCT_YEAR_REG 0X0C06
	#define MODULE_PRODUCT_MONTH_REG 0X0C07
	#define MODULE_PRODUCT_DAY_REG 0X0C08
	#define MODULE_AWB_FLAG_REG 0x0C2A
	#define MODULE_CUR_R_REG 0x0C37
	#define MODULE_CUR_Gr_REG 0x0C38
	#define MODULE_CUR_Gb_REG 0x0C39
	#define MODULE_CUR_B_REG 0x0C3A
	#define MODULE_GOLDEN_R_REG 0x0C3B
	#define MODULE_GOLDEN_Gr_REG 0x0C3C
	#define MODULE_GOLDEN_Gb_REG 0x0C3D
	#define MODULE_GOLDEN_B_REG 0x0C3E
	#define MODULE_AWB_CHECKSUM_REG 0x0C4F
	#define MODULE_LSC_FLAG_REG 0X1000
	#define MODULE_LSC_DATA_START_REG 0x1003
	#define MODULE_LSC_CHECKSUM_REG 0x174F
	#define MODULE_SPC_FLAG_REG 0X0CA0
	#define MODULE_SPC_DATA_START_REG 0x0CA1
	#define MODULE_SPC_CHECKSUM_REG 0x0D1F
#endif
	
static bool bDumpRaw_OTP_switch = false;

static uint8_t IMX258MIPI_SPC_Data[126]={0};
static uint8_t IMX258MIPI_LSC_Data[504]={0};

static RESULT load_imx258_SPC_Data(IsiSensorHandle_t   handle)
{
	int i;
	int temp = 0;
	RESULT result = RET_SUCCESS;

	for(i=0; i<63; i++)
	{
		result = IMX258_IsiRegWriteIss(handle, 0xD04C+i, IMX258MIPI_SPC_Data[i]);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "to sensor reg = 0x%04x, SPC_Data[%d] = %d\n",0xD04C+i, i, IMX258MIPI_SPC_Data[i]);
	}
	for(i=0; i<63; i++)
	{
		result = IMX258_IsiRegWriteIss(handle, 0xD08C+i, IMX258MIPI_SPC_Data[i+63]);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "to sensor reg = 0x%04x, SPC_Data[%d] = %d\n", 0xD08C+i, i+63, IMX258MIPI_SPC_Data[i+63]);
	}

	osSleep(10);
	//result = IMX258_IsiRegWriteIss(handle, 0x0B00, 0x00);
	//RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	//result = IMX258_IsiRegWriteIss(handle, 0x3051, 0x00);
	//RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	//result = IMX258_IsiRegWriteIss(handle, 0x3052, 0x00);
	//RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	//result = IMX258_IsiRegWriteIss(handle, 0x7BCA, 0x00);
	//RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	//result = IMX258_IsiRegWriteIss(handle, 0x7BCB, 0x00);
	//RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	result = IMX258_IsiRegWriteIss(handle, 0x7BC8, 0x01);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
#if 0
	osSleep(200);
	for(i=0; i<63; i++)
	{
		result = IMX258_IsiRegReadIss(handle, 0xD04C+i, &temp);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "read from sensor = 0x%04x, SPC_Data[%d] = %d\n",0xD04C+i, i, temp);
	}
	for(i=0; i<63; i++)
	{
		result = IMX258_IsiRegReadIss(handle, 0xD08C+i, &temp);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "read from sensor = 0x%04x, SPC_Data[%d] = %d\n", 0xD08C+i, i+63, temp);
	}
	result = IMX258_IsiRegReadIss(handle, 0x7BC8, &temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	TRACE( IMX258_DEBUG, "read from sensor 0x7BC8 = 0x%02x\n", temp);
	result = IMX258_IsiRegReadIss(handle, 0x7BC9, &temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	TRACE( IMX258_DEBUG, "read from sensor 0x7BC9 = 0x%02x\n", temp);
#endif /*print info*/
	return result;
}

static RESULT load_imx258_AWB_Data(IsiSensorHandle_t   handle)
{
	float RG_c = 0;
	float BG_c = 0;
	float GrGb_c = 0;
	float RG_g = 0;
	float BG_g = 0;
	float GrGb_g = 0;
	int Rgain = 0;
	int Bgain = 0;
	int Grgain = 0;
	int Gbgain = 0;
	RESULT result = RET_SUCCESS;
	float current_R = (float)g_otp_info.current_R;
	float current_Gr = (float)g_otp_info.current_Gr;
	float current_Gb = (float)g_otp_info.current_Gb;
	float current_B = (float)g_otp_info.current_B;
	float golden_R = (float)g_otp_info.golden_R;
	float golden_Gr = (float)g_otp_info.golden_Gr;
	float golden_Gb = (float)g_otp_info.golden_Gb;
	float golden_B = (float)g_otp_info.golden_B;
	
	RG_c = current_R / (current_Gr + current_Gb)*1024;
	BG_c = current_B / (current_Gr + current_Gb)*1024;
	GrGb_c = current_Gr / current_Gb *1024;
	
	RG_g = golden_R / (golden_Gr + golden_Gb)*1024;
	BG_g = golden_B / (golden_Gr + golden_Gb)*1024;
	GrGb_g = golden_Gr / golden_Gb *1024;
	
	Rgain = (int)((RG_g * (GrGb_g + 1)) / (RG_c * (GrGb_c + 1)) * 256 + 0.5);
	Bgain = (int)((BG_g * (GrGb_g + 1)) / (BG_c * (GrGb_c + 1)) * 256 + 0.5);
	Grgain = (int)(GrGb_g / GrGb_c * 256 + 0.5);
	Gbgain = 256;
	
	TRACE( IMX258_DEBUG, "otp write Rgain = %d\n",Rgain);
	result = IMX258_IsiRegWriteIss(handle, 0x0210, (Rgain >> 8) & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	result = IMX258_IsiRegWriteIss(handle, 0x0211, Rgain & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
	TRACE( IMX258_DEBUG, "otp write Bgain = %d\n",Bgain);
	result = IMX258_IsiRegWriteIss(handle, 0x0212, (Bgain >> 8) & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	result = IMX258_IsiRegWriteIss(handle, 0x0213, Bgain & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
	TRACE( IMX258_DEBUG, "otp write Grgain = %d\n",Grgain);
	result = IMX258_IsiRegWriteIss(handle, 0x020e, (Grgain >> 8) & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	result = IMX258_IsiRegWriteIss(handle, 0x020f, Grgain & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
	TRACE( IMX258_DEBUG, "otp write Gbgain = %d\n",Gbgain);
	result = IMX258_IsiRegWriteIss(handle, 0x0214, (Gbgain >> 8) & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	result = IMX258_IsiRegWriteIss(handle, 0x0215, Gbgain & 0xff);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

	return result;
}

static RESULT load_imx258_LSC_Data(IsiSensorHandle_t   handle)
{
	int i;
	uint32_t temp = 0;
	RESULT result = RET_SUCCESS;
	
	for(i=0; i<504; i++)
	{
		result = IMX258_IsiRegWriteIss(handle, 0xA300+i, IMX258MIPI_LSC_Data[i]);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "to sensor reg = 0x%04x, LSC_Data[%d] = %d\n",0xA300+i, i, IMX258MIPI_LSC_Data[i]);
	}
	
	osSleep(10);
	//choose lsc table 1
	result = IMX258_IsiRegReadIss(handle, 0x3021, &temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	temp |= 0x01;
	result = IMX258_IsiRegWriteIss(handle, 0x3021, temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
	//enable lsc
	result = IMX258_IsiRegReadIss(handle, 0x0B00, &temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	temp |= 0x01;
	result = IMX258_IsiRegWriteIss(handle, 0x0B00, temp);
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	
#if 0
	osSleep(200);
	for(i=0; i<504; i++)
	{
		result = IMX258_IsiRegReadIss(handle, 0xA300+i, &temp);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_DEBUG, "read from sensor = 0x%04x, LSC_Data[%d] = %d\n",0xA300+i, i, temp);
	}
#endif

	return result;
}


static int check_read_otp(
	sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	sensor_version_get_t* sensor_version_get_p,
	void* context,
	int camsys_fd
)
{
	int temp = 0;
	int ret=0;
	int i = 0;
	int check_sum_awb = 0;
	int check_sum_lsc = 0;
	int check_sum_spc = 0;
	int i2c_base_info[3];
	i2c_base_info[0] = Sensor_OTP_SLAVE_ADDR; //otp i2c addr
  i2c_base_info[1] = 2; //otp i2c reg size
  i2c_base_info[2] = 1; //otp i2c value size
 	temp = sensor_i2c_read_p(context,camsys_fd,MODULE_INFO_FLAG_REG,i2c_base_info);
 	g_otp_info.base_info_flag = temp;
 	if(0x01 == g_otp_info.base_info_flag){
 			TRACE( IMX258_DEBUG, "%s(%d):otp base info flag : %d, data is valid\n", __FUNCTION__, __LINE__,g_otp_info.base_info_flag);
 			temp = sensor_i2c_read_p(context,camsys_fd,MODULE_ID_REG,i2c_base_info);
 			g_otp_info.module_integrator_id = temp;
 			temp = sensor_i2c_read_p(context,camsys_fd,MODULE_PRODUCT_YEAR_REG,i2c_base_info);
 			g_otp_info.production_year = temp;
 			temp = sensor_i2c_read_p(context,camsys_fd,MODULE_PRODUCT_MONTH_REG,i2c_base_info);
 			g_otp_info.production_month = temp;
 			temp = sensor_i2c_read_p(context,camsys_fd,MODULE_PRODUCT_DAY_REG,i2c_base_info);
 			g_otp_info.production_day = temp;
 			TRACE( IMX258_DEBUG, "%s(%d):otp module_integrator_id = 0x%02x", __FUNCTION__, __LINE__,g_otp_info.module_integrator_id);
 			TRACE( IMX258_DEBUG, "%s(%d):otp production year/month/day = %04d/%02d/%02d\n", __FUNCTION__, __LINE__,g_otp_info.production_year+2000,g_otp_info.production_month,g_otp_info.production_day);
 	}else{
 			TRACE( IMX258_ERROR, "%s(%d):otp base info flag : %d, data is invalid\n", __FUNCTION__, __LINE__,g_otp_info.base_info_flag);
 	}
 	
 	temp = sensor_i2c_read_p(context,camsys_fd,MODULE_AWB_FLAG_REG,i2c_base_info);
 	g_otp_info.awb_flag = temp;
 	if(0x01 == g_otp_info.awb_flag){
 		TRACE( IMX258_DEBUG, "%s(%d):otp awb flag : %d, data is valid\n", __FUNCTION__, __LINE__,g_otp_info.awb_flag);
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_CUR_R_REG,i2c_base_info);
 		g_otp_info.current_R = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_CUR_Gr_REG,i2c_base_info);
 		g_otp_info.current_Gr = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_CUR_Gb_REG,i2c_base_info);
 		g_otp_info.current_Gb = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_CUR_B_REG,i2c_base_info);
 		g_otp_info.current_B = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_GOLDEN_R_REG,i2c_base_info);
 		g_otp_info.golden_R = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_GOLDEN_Gr_REG,i2c_base_info);
 		g_otp_info.golden_Gr = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_GOLDEN_Gb_REG,i2c_base_info);
 		g_otp_info.golden_Gb = temp;
 		check_sum_awb += temp;
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_GOLDEN_B_REG,i2c_base_info);
 		g_otp_info.golden_B = temp;
 		check_sum_awb += temp;
 			
 		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_AWB_CHECKSUM_REG,i2c_base_info);
		if((check_sum_awb%0xff) != temp){
				TRACE( IMX258_ERROR, "%s(%d):otp awb check sum error,sum add value = %d,read from otp = %d\n", __FUNCTION__, __LINE__,check_sum_awb%0xff,temp);
		}
			
 		TRACE( IMX258_DEBUG, "%s(%d):otp current_R = %d\n", __FUNCTION__, __LINE__,g_otp_info.current_R);
 		TRACE( IMX258_DEBUG, "%s(%d):otp current_Gr = %d\n", __FUNCTION__, __LINE__,g_otp_info.current_Gr);
 		TRACE( IMX258_DEBUG, "%s(%d):otp current_Gb = %d\n", __FUNCTION__, __LINE__,g_otp_info.current_Gb);
 		TRACE( IMX258_DEBUG, "%s(%d):otp current_B = %d\n", __FUNCTION__, __LINE__,g_otp_info.current_B);
 		TRACE( IMX258_DEBUG, "%s(%d):otp golden_R = %d\n", __FUNCTION__, __LINE__,g_otp_info.golden_R);
 		TRACE( IMX258_DEBUG, "%s(%d):otp golden_Gr = %d\n", __FUNCTION__, __LINE__,g_otp_info.golden_Gr);
 		TRACE( IMX258_DEBUG, "%s(%d):otp golden_Gb = %d\n", __FUNCTION__, __LINE__,g_otp_info.golden_Gb);
 		TRACE( IMX258_DEBUG, "%s(%d):otp golden_B = %d\n", __FUNCTION__, __LINE__,g_otp_info.golden_B);
    
  }else{
  	TRACE( IMX258_ERROR, "%s(%d):otp awb flag : %d, data is invalid\n", __FUNCTION__, __LINE__,g_otp_info.awb_flag);
  }
 	
 	temp = sensor_i2c_read_p(context,camsys_fd,MODULE_LSC_FLAG_REG,i2c_base_info);
 	g_otp_info.lsc_flag = temp;
 	if(0x01 == g_otp_info.lsc_flag){
 		TRACE( IMX258_DEBUG, "%s(%d):otp lsc flag : %d, data is valid\n", __FUNCTION__, __LINE__,g_otp_info.lsc_flag);
 		for (i = 0; i < 504; i++)
		{
			temp= sensor_i2c_read_p(context,camsys_fd,MODULE_LSC_DATA_START_REG+i,i2c_base_info);
			IMX258MIPI_LSC_Data[i] = temp;
			check_sum_lsc += temp;
			TRACE( IMX258_DEBUG, "read from eeprom reg = 0x%04x, IMX258MIPI_LSC_Data[%d] = %d\n",0x003B+i, i, IMX258MIPI_LSC_Data[i]);
		}
		
		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_LSC_CHECKSUM_REG,i2c_base_info);
		if((check_sum_lsc%0xff) != temp){
				TRACE( IMX258_ERROR, "%s(%d):otp lsc check sum error,sum add value = %d,read from otp = %d\n", __FUNCTION__, __LINE__,check_sum_lsc%0xff,temp);
		}
    
  }else{
  	TRACE( IMX258_ERROR, "%s(%d):otp lsc flag : %d, data is invalid\n", __FUNCTION__, __LINE__,g_otp_info.lsc_flag);
  }
  
 	temp = sensor_i2c_read_p(context,camsys_fd,MODULE_SPC_FLAG_REG,i2c_base_info);
 	g_otp_info.spc_flag = temp;
 	if(0x01 == g_otp_info.spc_flag){
 		TRACE( IMX258_DEBUG, "%s(%d):otp spc flag : %d, data is valid\n", __FUNCTION__, __LINE__,g_otp_info.spc_flag);
 		for (i = 0; i < 126; i++)
		{
			temp= sensor_i2c_read_p(context,camsys_fd,MODULE_SPC_DATA_START_REG+i,i2c_base_info);
			IMX258MIPI_SPC_Data[i] = temp;
			check_sum_spc += temp;
			TRACE( IMX258_DEBUG, "read from eeprom reg = 0x%04x, IMX258MIPI_SPC_Data[%d] = %d\n",0x0CE2+i, i, IMX258MIPI_SPC_Data[i]);
		}
		
		temp = sensor_i2c_read_p(context,camsys_fd,MODULE_SPC_CHECKSUM_REG,i2c_base_info);
		if((check_sum_spc%0xff) != temp){
				TRACE( IMX258_ERROR, "%s(%d):otp spc check sum error,sum add value = %d,read from otp = %d\n", __FUNCTION__, __LINE__,check_sum_spc%0xff,temp);
		}
		
  }else{
  	TRACE( IMX258_ERROR, "%s(%d):otp spc flag : %d, data is invalid\n", __FUNCTION__, __LINE__,g_otp_info.spc_flag);
  }
	return RET_SUCCESS;
}

static int apply_otp_data(IsiSensorHandle_t   handle,struct otp_struct *otp_ptr)
{
	if(0x01 == g_otp_info.spc_flag){
		load_imx258_SPC_Data(handle);
	}
	if (0x01 == g_otp_info.awb_flag){
		load_imx258_AWB_Data(handle);
	}
	if(0x01 == g_otp_info.lsc_flag){
		load_imx258_LSC_Data(handle);
	}
	TRACE( IMX258_INFO,  "%s: success!!!\n",  __FUNCTION__ );
	return RET_SUCCESS;
}

static RESULT IMX258_IsiSetOTPInfo
(
    IsiSensorHandle_t       handle,
    uint32_t OTPInfo
)
{
	RESULT result = RET_SUCCESS;

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }
#if 0
	RG_Ratio_Typical = OTPInfo>>16;
	BG_Ratio_Typical = OTPInfo&0xffff;
	TRACE( IMX258_NOTICE0, "%s:  --(RG,BG) in IQ file:(0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);
	if((RG_Ratio_Typical==0) && (BG_Ratio_Typical==0)){
		TRACE( IMX258_ERROR, "%s:  --OTP typical value in IQ file is zero, we will try another match rule.\n", __FUNCTION__);
        RG_Ratio_Typical = RG_Ratio_Typical_Default;
        BG_Ratio_Typical = BG_Ratio_Typical_Default;
	}
	TRACE( IMX258_NOTICE0, "%s:  --Finally, the (RG,BG) is (0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);
#endif
	return (result);
}

static RESULT IMX258_IsiEnableOTP
(
    IsiSensorHandle_t       handle,
    const bool_t enable
)
{
	RESULT result = RET_SUCCESS;

    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }
	bDumpRaw_OTP_switch = enable;
	return (result);
}

/* OTP END*/



/*****************************************************************************/
/**
 *          IMX258_IsiCreateSensorIss
 *
 * @brief   This function creates a new IMX258 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT IMX258_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    IMX258_Context_t *pIMX258Ctx;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pIMX258Ctx = ( IMX258_Context_t * )malloc ( sizeof (IMX258_Context_t) );
    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR,  "%s: Can't allocate IMX258 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pIMX258Ctx, 0, sizeof( IMX258_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pIMX258Ctx );
        return ( result );
    }
    
    pIMX258Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pIMX258Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pIMX258Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pIMX258Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? IMX258_SLAVE_ADDR : pConfig->SlaveAddr;
    pIMX258Ctx->IsiCtx.NrOfAddressBytes       = 2U;

    pIMX258Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pIMX258Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? IMX258_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pIMX258Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pIMX258Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pIMX258Ctx->Configured             = BOOL_FALSE;
    pIMX258Ctx->Streaming              = BOOL_FALSE;
    pIMX258Ctx->TestPattern            = BOOL_FALSE;
    pIMX258Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pIMX258Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pIMX258Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pIMX258Ctx->VcmInfo.RatedCurrent   = pIMX258Ctx->VcmInfo.StartCurrent + MAX_LOG*pIMX258Ctx->VcmInfo.Step;
    pIMX258Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pIMX258Ctx->IsiSensorMipiInfo.sensorHalDevID = pIMX258Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( IMX258_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pIMX258Ctx;

    result = HalSetCamConfig( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an IMX258 sensor instance.
 *
 * @param   handle      IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT IMX258_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)IMX258_IsiSensorSetStreamingIss( pIMX258Ctx, BOOL_FALSE );
    (void)IMX258_IsiSensorSetPowerIss( pIMX258Ctx, BOOL_FALSE );

    (void)HalDelRef( pIMX258Ctx->IsiCtx.HalHandle );

    MEMSET( pIMX258Ctx, 0, sizeof( IMX258_Context_t ) );
    free ( pIMX258Ctx );

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCapsIss
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
static RESULT IMX258_IsiGetCapsIssInternal
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
        if (mipi_lanes == SUPPORT_MIPI_FOUR_LANE) {            
            switch (pIsiSensorCaps->Index) 
            {    
                case 0:
                { 
                    pIsiSensorCaps->Resolution = ISI_RES_2096_1560P30;
                    break;
                }
               #if 1
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2096_1560P25;
                    break;
                }

                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2096_1560P20;
                    break;
                }

                case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2096_1560P15;
                    break;
                }
                
                case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2096_1560P10;
                    break;
                }
                 case 5:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_4208_3120P20;
                    break;
                }
                case 6:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_4208_3120P15;
                    break;
                }
                case 7:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_4208_3120P10;
                    break;
                }     
                case 8:
               {
               	 		pIsiSensorCaps->Resolution = ISI_RES_4208_3120P7;
                	  break;
               } 
               #endif
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
        pIsiSensorCaps->BPat            = ISI_BPAT_RGRGGBGB;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS; //hsync?
        pIsiSensorCaps->VPol            = ISI_VPOL_POS; //VPolarity
        pIsiSensorCaps->Edge            = ISI_EDGE_RISING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF; //close;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;//close;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;//close;<
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO | ISI_BLC_OFF);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );//close;
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_AUTO | ISI_DPCC_OFF );//»µµã

        pIsiSensorCaps->DwnSz           = ISI_DWNSZ_SUBSMPL; //;
        pIsiSensorCaps->CieProfile      = ( ISI_CIEPROF_A  //¹âÔ´£»
                                          | ISI_CIEPROF_D50
                                          | ISI_CIEPROF_D65
                                          | ISI_CIEPROF_D75
                                          | ISI_CIEPROF_F2
                                          | ISI_CIEPROF_F11 );
        pIsiSensorCaps->SmiaMode        = ISI_SMIA_OFF;
        pIsiSensorCaps->MipiMode        = ISI_MIPI_MODE_RAW_10; 
        pIsiSensorCaps->AfpsResolutions = ( ISI_AFPS_NOTSUPP ); //ÌøÖ¡;Ã»ÓÃ
		pIsiSensorCaps->SensorOutputMode = ISI_SENSOR_OUTPUT_MODE_RAW;//
    }
end:
    return result;
}
 
static RESULT IMX258_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = IMX258_IsiGetCapsIssInternal(pIsiSensorCaps,pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t IMX258_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_RGRGGBGB,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_POS,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_2096_1560P30, 
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
 *          IMX258_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      IMX258 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ÑéÖ¤ÉÏÃæÄ£Ê½µÈ£»
 *****************************************************************************/
RESULT IMX258_SetupOutputFormat
(
    IMX258_Context_t       *pIMX258Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s%s (enter)\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by IMX258 sensor, so the YCSequence parameter is not checked */
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
            TRACE( IMX258_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_RGRGGBGB supported, no configuration needed */
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
            TRACE( IMX258_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( IMX258_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( IMX258_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( IMX258_INFO, "%s%s (exit)\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

#if 0
int IMX258_get_PCLK( IMX258_Context_t *pIMX258Ctx, int XVCLK)
{
    // calculate sysclk
    
    int prediv, mipi_preDiv, mipi_preMul, mipi_Mul, mipi_Div, sys_preDiv, sys_Mul;
    uint32_t temp1, temp2;
    uint64_t sysclk;
    sysclk = XVCLK;
    IMX258_IsiRegReadIss(pIMX258Ctx, 0xf7, &temp1 );
    if(temp1 & 0x02)
    {
        sysclk/=2;

    }
    IMX258_IsiRegReadIss(pIMX258Ctx, 0xf5, &temp1 );
    temp1 &= 0x18;
    temp1 >>= 3;
    if(temp1 >=2)
    {
        IMX258_IsiRegReadIss(pIMX258Ctx, 0xf2, &temp2);
        sysclk /= (temp2 & 0x07) + 1;
        IMX258_IsiRegReadIss(pIMX258Ctx, 0xf4, &temp2);
        if(temp2 & 0x10)
        {
            sysclk *=2;
        }
        IMX258_IsiRegReadIss(pIMX258Ctx, 0xfa, &temp2);
        temp2++;
        temp2 *=2;
        sysclk *= temp2;

    }
    else
    {
        IMX258_IsiRegReadIss(pIMX258Ctx, 0xf6, &temp2);
        sysclk /= (temp2 & 0x07) + 1;
        IMX258_IsiRegReadIss(pIMX258Ctx, 0xf8, &temp2);
        temp2++;
        temp2 *=2;
        sysclk *= temp2;

    }
    switch(temp1)
    {
        case 0:
            sysclk /= 4;
            break;
        case 1:
            sysclk /= 3;
            break;
        case 2:
            IMX258_IsiRegReadIss(pIMX258Ctx, 0xf4, &temp2);
            if(temp2 & 0x10)
            {
                sysclk /= 4;
            }
            else
            {
                sysclk /= 2;
            }
            break;
        case 3:
            IMX258_IsiRegReadIss(pIMX258Ctx, 0xf4, &temp2);
            if(temp2 & 0x10)
            {
                sysclk /= 5;
            }
            else
            {
                sysclk /= 2.5;
            }
            break;
        default:
            TRACE( IMX258_ERROR, "%s: failed to get_PCLK \n", __FUNCTION__ );
            break;
    }
    IMX258_IsiRegReadIss(pIMX258Ctx, 0xfc, &temp2);
    if(temp2 & 0x10)
    {
        sysclk /=2;
    }
    return (int)sysclk; 
   
}
#endif
/*****************************************************************************/
/**
 *          IMX258_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      IMX258 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT IMX258_SetupOutputWindowInternal
(
    IMX258_Context_t        *pIMX258Ctx,
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
    int xclk = 24000000;
    
	TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE) {
    
        switch ( pConfig->Resolution )
        {
            case ISI_RES_2096_1560P30:
            case ISI_RES_2096_1560P25:
            case ISI_RES_2096_1560P20:
            case ISI_RES_2096_1560P15:
            case ISI_RES_2096_1560P10:            
            {
                if (set2Sensor == BOOL_TRUE) {                    
                    if (res_no_chg == BOOL_FALSE) {
											result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560_fourlane);
											TRACE( IMX258_DEBUG, "%s set IMX258_g_2096x1560_fourlane\n", __FUNCTION__);
                    }
                    if (pConfig->Resolution == ISI_RES_2096_1560P30) {                        
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2096_1560P25) {
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2096_1560P20) {
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2096_1560P15) {
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_2096_1560P10) {
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_2096x1560P10_fourlane_fpschg);
                    }	
        		}
        	pIMX258Ctx->IsiSensorMipiInfo.ulMipiFreq = 798;
    			usTimeHts = 5352; 
    			rVtPixClkFreq = 319200000;
                if (pConfig->Resolution == ISI_RES_2096_1560P30) {
                    usTimeVts = 1988;   // 33.34pfs
                } else if (pConfig->Resolution == ISI_RES_2096_1560P25) {
                    usTimeVts = 2385;   // 25.00 fps
                } else if (pConfig->Resolution == ISI_RES_2096_1560P20) {
                    usTimeVts = 2982;  // 20.00 fps
                } else if (pConfig->Resolution == ISI_RES_2096_1560P15) {
                    usTimeVts = 3976;  // 16.67 fps
                } else if (pConfig->Resolution == ISI_RES_2096_1560P10) {
                    usTimeVts = 5964;  // 10.00 fps
                }
                
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }
            case ISI_RES_4208_3120P7:
            case ISI_RES_4208_3120P10:
            case ISI_RES_4208_3120P15:
            case ISI_RES_4208_3120P20:
            {
                if (set2Sensor == BOOL_TRUE) {
                    if (res_no_chg == BOOL_FALSE) {
											result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_4208x3120_fourlane);
											TRACE( IMX258_ERROR, "%s set IMX258_g_4208x3120_fourlane\n", __FUNCTION__);
        		   			 }
           
                    if (pConfig->Resolution == ISI_RES_4208_3120P15) {                        
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_4208x3120P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_4208_3120P10) {                        
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_4208x3120P10_fourlane_fpschg);
                    } 
                    else if (pConfig->Resolution == ISI_RES_4208_3120P7) {
                        result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_4208x3120P7_fourlane_fpschg);
                    }
        	    
        		}
        	pIMX258Ctx->IsiSensorMipiInfo.ulMipiFreq = 996;
        	rVtPixClkFreq = 398400000;	
    			usTimeHts = 5352;                
                if (pConfig->Resolution == ISI_RES_4208_3120P20) {                        
                    usTimeVts = 3720;  // 16.67 fps
                } else if (pConfig->Resolution == ISI_RES_4208_3120P15) {                        
                    usTimeVts = 4962;  // 16.67 fps
                } else if (pConfig->Resolution == ISI_RES_4208_3120P10) {                        
                    usTimeVts = 7443;   // 10.00 fps
                } 
                else if (pConfig->Resolution == ISI_RES_4208_3120P7) {
                    usTimeVts = 10634;   // 7.69 fps
                }
    		    /* sleep a while, that sensor can take over new default values */
    		    osSleep( 10 );
    			break;
                
            }
        }        

    }
    
	
/* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    
	usLineLengthPck = usTimeHts;
    usFrameLengthLines = usTimeVts;
	
    
    // store frame timing for later use in AEC module
    pIMX258Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pIMX258Ctx->LineLengthPck    = usLineLengthPck;
    pIMX258Ctx->FrameLengthLines = usFrameLengthLines;
	pIMX258Ctx->AecMaxIntegrationTime = ( ((float)pIMX258Ctx->FrameLengthLines) * ((float)pIMX258Ctx->LineLengthPck) ) / pIMX258Ctx->VtPixClkFreq;
    TRACE( IMX258_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      IMX258 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT IMX258_SetupImageControl
(
    IMX258_Context_t        *pIMX258Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;


    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = IMX258_IsiRegReadIss(  pIMX258Ctx, IMX258_BLC_CTRL00, &RegValue );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = IMX258_IsiRegReadIss(  pIMX258Ctx, IMX258_BLC_CTRL00, &RegValue );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = IMX258_IsiRegReadIss(  pIMX258Ctx, IMX258_AEC_MANUAL, &RegValue );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = IMX258_IsiRegReadIss(  pIMX258Ctx, IMX258_ISP_CTRL01, &RegValue );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = IMX258_IsiRegReadIss(  pIMX258Ctx, IMX258_AEC_MANUAL, &RegValue );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = IMX258_IsiRegReadIss( pIMX258Ctx, IMX258_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = IMX258_IsiRegReadIss( pIMX258Ctx, IMX258_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( IMX258_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT IMX258_SetupOutputWindow
(
    IMX258_Context_t        *pIMX258Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pIMX258Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pIMX258Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return IMX258_SetupOutputWindowInternal(pIMX258Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          IMX258_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in IMX258-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      IMX258 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä
 *****************************************************************************/
static RESULT IMX258_AecSetModeParameters
(
    IMX258_Context_t       *pIMX258Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
    uint32_t temp = 0;
		
		result = IMX258_IsiRegReadIss(pIMX258Ctx, 0x0B00, &temp);
		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		TRACE( IMX258_ERROR, "read from sensor,lsc enable 222 0x%02x\n", temp);
		
    TRACE( IMX258_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"",
        pIMX258Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pIMX258Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( IMX258_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
    // (formula is usually MaxIntTime = (CoarseMax * LineLength + FineMax) / Clk
    //                     MinIntTime = (CoarseMin * LineLength + FineMin) / Clk )
    pIMX258Ctx->AecMaxIntegrationTime = ( ((float)(pIMX258Ctx->FrameLengthLines - 4)) * ((float)pIMX258Ctx->LineLengthPck) ) / pIMX258Ctx->VtPixClkFreq;
    pIMX258Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( IMX258_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"", pIMX258Ctx->AecMaxIntegrationTime  );

    pIMX258Ctx->AecMaxGain = IMX258_MAX_GAIN_AEC;
    pIMX258Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pIMX258Ctx->AecIntegrationTimeIncrement = ((float)pIMX258Ctx->LineLengthPck) / pIMX258Ctx->VtPixClkFreq;
    pIMX258Ctx->AecGainIncrement = IMX258_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pIMX258Ctx->AecCurGain               = pIMX258Ctx->AecMinGain;
    pIMX258Ctx->AecCurIntegrationTime    = 0.0f;
    pIMX258Ctx->OldCoarseIntegrationTime = 0;
    pIMX258Ctx->OldFineIntegrationTime   = 0;
    //pIMX258Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( IMX258_INFO, "%s%s (exit)\n", __FUNCTION__, pIMX258Ctx->isAfpsRun?"(AFPS)":"");
    	
    

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX258_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      IMX258 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT IMX258_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pIMX258Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pIMX258Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image IMX258 (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    //result = IMX258_IsiRegWriteIss ( pIMX258Ctx, IMX258_SOFTWARE_RST, IMX258_SOFTWARE_RST_VALUE );//ºê¶¨Òå hkw£»
   // RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( IMX258_DEBUG, "%s: IMX258 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)
    result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_MODE_SELECT, IMX258_MODE_SELECT_OFF );//IMX258_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: Can't write IMX258 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
    //result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_aRegDescription );
    
	  if(pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
	  		//result = HalSetReset( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, true );
       // RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	      result = IsiRegDefaultsApply( pIMX258Ctx, IMX258_g_aRegDescription_fourlane);
	     // result = HalSetReset( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, false );
    }
 
    /* End of SYNNEX DEBUG */
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that IMX258 can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pIMX258Ctx, IMX258_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = IMX258_IsiRegWriteIss( pIMX258Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = IMX258_SetupOutputFormat( pIMX258Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = IMX258_SetupOutputWindow( pIMX258Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = IMX258_SetupImageControl( pIMX258Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = IMX258_AecSetModeParameters( pIMX258Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pIMX258Ctx->Configured = BOOL_TRUE;
    }

    //set OTP info

    result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_MODE_SELECT, IMX258_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: Can't write IMX258 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
	//	osSleep(30);
	//	apply_otp_data(pIMX258Ctx,NULL);
    
    result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_MODE_SELECT, IMX258_MODE_SELECT_OFF );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: Can't write IMX258 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
   

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiChangeSensorResolutionIss
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
 * ²»ÓÃ¸Ä
 *****************************************************************************/
static RESULT IMX258_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pIMX258Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (IMX258_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pIMX258Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pIMX258Ctx->Config.Resolution)) && 
            (ISI_RES_H_GET(Resolution)==ISI_RES_H_GET(pIMX258Ctx->Config.Resolution))) ) {

            if (pIMX258Ctx->Streaming != BOOL_FALSE) {
                TRACE( IMX258_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( IMX258_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pIMX258Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = IMX258_SetupOutputWindowInternal( pIMX258Ctx, &pIMX258Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX258_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pIMX258Ctx->AecCurGain;
        float OldIntegrationTime = pIMX258Ctx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = IMX258_AecSetModeParameters( pIMX258Ctx, &pIMX258Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX258_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = IMX258_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( IMX258_ERROR, "%s: IMX258_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( IMX258_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX258_IsiSensorSetStreamingIss
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
static RESULT IMX258_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;
		int temp = 0;
		int i = 0;
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_ERROR, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pIMX258Ctx->Configured != BOOL_TRUE) || (pIMX258Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        
        result = IMX258_IsiRegWriteIss ( pIMX258Ctx, IMX258_MODE_SELECT, IMX258_MODE_SELECT_ON);//IMX258_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        osSleep(10);
       	result = IMX258_IsiRegReadIss ( pIMX258Ctx, IMX258_MODE_SELECT, &RegValue);
      	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
      	TRACE( IMX258_DEBUG, "%s Set Stream IMX258_MODE_SELECT_ON,read reg=0x%04x\n", __FUNCTION__);
      	    
    }
    else
    {
        result = IMX258_IsiRegWriteIss ( pIMX258Ctx, IMX258_MODE_SELECT, IMX258_MODE_SELECT_OFF);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        osSleep(10);
       	result = IMX258_IsiRegReadIss ( pIMX258Ctx, IMX258_MODE_SELECT, &RegValue);
      	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
      	TRACE( IMX258_DEBUG, "%s Set Stream IMX258_MODE_SELECT_OFF,read reg=0x%04x\n", __FUNCTION__);
    }

    if (result == RET_SUCCESS)
    {
        pIMX258Ctx->Streaming = on;
    }

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      IMX258 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä
 *****************************************************************************/
static RESULT IMX258_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pIMX258Ctx->Configured = BOOL_FALSE;
    pIMX258Ctx->Streaming  = BOOL_FALSE;

    TRACE( IMX258_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
		osSleep( 10 );
    TRACE( IMX258_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( IMX258_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 20 );

        TRACE( IMX258_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pIMX258Ctx->IsiCtx.HalHandle, pIMX258Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 50 );
    }

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ¶Ápid;2»ò3¸ö¼Ä´æÆ÷£»
 *****************************************************************************/
static RESULT IMX258_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = IMX258_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId<<8U) | IMX258_CHIP_ID_LOW_BYTE_DEFAULT;

    result = IMX258_IsiGetSensorRevisionIss( handle, &value );

    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( IMX258_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( IMX258_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetSensorRevisionIss
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
static RESULT IMX258_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = IMX258_IsiRegReadIss ( handle, IMX258_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = IMX258_IsiRegReadIss ( handle, IMX258_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));
    *p_value = 0;

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiRegReadIss
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
 * ²»ÓÃ¸Ä
 *****************************************************************************/
static RESULT IMX258_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, IMX258_g_aRegDescription_fourlane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
        
    }

    TRACE( IMX258_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiRegWriteIss
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
 * ²»ÓÃ¸Ä
 *****************************************************************************/
static RESULT IMX258_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, IMX258_g_aRegDescription_fourlane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

   TRACE( IMX258_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          IMX258 instance
 *
 * @param   handle       IMX258 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»»ñµÃÔöÒæÏÞÖÆ
 *****************************************************************************/
static RESULT IMX258_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( IMX258_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pIMX258Ctx->AecMinGain;
    *pMaxGain = pIMX258Ctx->AecMaxGain;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          IMX258 instance
 *
 * @param   handle       IMX258 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»»ñµÃÆØ¹âÏÞÖÆ£»
 *****************************************************************************/
static RESULT IMX258_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( IMX258_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pIMX258Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pIMX258Ctx->AecMaxIntegrationTime;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          IMX258_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  IMX258 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»»ñµÃGAINÖµ
 *****************************************************************************/
RESULT IMX258_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }
#if 0
	result = IMX258_IsiRegReadIss ( pIMX258Ctx, IMX258_AEC_AGC_ADJ_H, &data);
	TRACE( IMX258_INFO, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = IMX258_IsiRegReadIss ( pIMX258Ctx, IMX258_AEC_AGC_ADJ_L, &data);
	TRACE( IMX258_INFO, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / IMX258_MAXN_GAIN;
#else	
    *pSetGain = pIMX258Ctx->AecCurGain;
#endif    

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  IMX258 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»»ñµÃGAIN×îÐ¡Öµ
 *****************************************************************************/
RESULT IMX258_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pIMX258Ctx->AecGainIncrement;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  IMX258 sensor instance handle
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
RESULT IMX258_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
    uint16_t iReg = 0;

    TRACE( IMX258_INFO, "%s: (enter) pIMX258Ctx->AecMaxGain(%f) \n", __FUNCTION__,pIMX258Ctx->AecMaxGain);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( IMX258_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pIMX258Ctx->AecMinGain ) NewGain = pIMX258Ctx->AecMinGain;
    if( NewGain > pIMX258Ctx->AecMaxGain ) NewGain = pIMX258Ctx->AecMaxGain;

    usGain = (uint16_t)(512-(512.0/NewGain)+0.5); 
    iReg = usGain;
    IMX258_IsiRegWriteIss(pIMX258Ctx,0x0204, ((iReg & 0x100) >> 8));  
    IMX258_IsiRegWriteIss(pIMX258Ctx,0x0205, (iReg & 0xff));  
  	
	 	pIMX258Ctx->AecCurGain = (float)(512.0/(512-usGain));
    //return current state
    *pSetGain = pIMX258Ctx->AecCurGain;

    TRACE( IMX258_ERROR, "%s: setgain mubiao(%f) shiji(%f)\n", __FUNCTION__, NewGain, *pSetGain);
	
    return ( result );
}


/*****************************************************************************/
/**
 *          IMX258_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  IMX258 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * »ñµÃÆØ¹âÊ±¼ä ²»ÓÃ¸Ä
 *****************************************************************************/
RESULT IMX258_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pIMX258Ctx->AecCurIntegrationTime;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  IMX258 IMX258 instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * »ñµÃÆØ¹âÊ±¼äµÄstep ²»ÓÃ¸Ä
 *****************************************************************************/
RESULT IMX258_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the IMX258/driver can handle (e.g. used for sliders in the application)
    *pIncr = pIMX258Ctx->AecIntegrationTimeIncrement;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  IMX258 sensor instance handle
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
 *ÉèÖÃÆØ¹âÊ±¼ä£»¸ù¾ÝÓ¦ÓÃÊÖ²áÐÞ¸Ä¼Ä´æÆ÷ºê
 *****************************************************************************/
RESULT IMX258_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
	uint32_t result_intertime= 0;
    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( IMX258_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pIMX258Ctx->AecMinIntegrationTime,
        pIMX258Ctx->AecMaxIntegrationTime);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( IMX258_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( NewIntegrationTime > pIMX258Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pIMX258Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pIMX258Ctx->AecMinIntegrationTime ) NewIntegrationTime = pIMX258Ctx->AecMinIntegrationTime;

    ShutterWidthPck = NewIntegrationTime * ( (float)pIMX258Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pIMX258Ctx->LineLengthPck == 0 )
    {
        TRACE( IMX258_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pIMX258Ctx->LineLengthPck) + 0.5f );
    if( CoarseIntegrationTime != pIMX258Ctx->OldCoarseIntegrationTime)
    {
			IMX258_IsiRegWriteIss( pIMX258Ctx, 0x0202,(CoarseIntegrationTime>>8) & 0xFF );
			IMX258_IsiRegWriteIss( pIMX258Ctx, 0x0203, CoarseIntegrationTime & 0xFF);
		pIMX258Ctx->OldCoarseIntegrationTime = CoarseIntegrationTime;	// remember current integration time
		*pNumberOfFramesToSkip = 1U; //skip 1 frame
	}
	else
	{
		*pNumberOfFramesToSkip = 0U; //no frame skip
	}

	
       pIMX258Ctx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pIMX258Ctx->LineLengthPck) / pIMX258Ctx->VtPixClkFreq;
    
	*pSetIntegrationTime = pIMX258Ctx->AecCurIntegrationTime;

       TRACE( IMX258_DEBUG, "%s: vtPixClkFreq:%f, LineLengthPck:%x, SetTi=%f, NewTi=%f, CoarseIntegrationTime=%x\n", __FUNCTION__, 
         pIMX258Ctx->VtPixClkFreq,pIMX258Ctx->LineLengthPck,*pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime);
	   
    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}




/*****************************************************************************/
/**
 *          IMX258_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  IMX258 sensor instance handle
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
 * ²»ÓÃ¸Ä£¬ÉèÖÃÕû¸öÆØ¹â£»
 *****************************************************************************/
RESULT IMX258_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( IMX258_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( IMX258_INFO, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = IMX258_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = IMX258_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( IMX258_INFO, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *²»ÓÃ¸Ä£¬»ñÈ¡gainºÍexposure Ê±¼ä
 *****************************************************************************/
RESULT IMX258_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pIMX258Ctx->AecCurGain;
    *pSetIntegrationTime = pIMX258Ctx->AecCurIntegrationTime;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetResolutionIss
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
 * ²»ÓÃ¸Ä£»
 *****************************************************************************/
RESULT IMX258_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pIMX258Ctx->Config.Resolution;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pIMX258Ctx             IMX258 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»
 *****************************************************************************/
static RESULT IMX258_IsiGetAfpsInfoHelperIss(
    IMX258_Context_t   *pIMX258Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pIMX258Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pIMX258Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = IMX258_SetupOutputWindowInternal( pIMX258Ctx, &pIMX258Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = IMX258_AecSetModeParameters( pIMX258Ctx, &pIMX258Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( IMX258_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pIMX258Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pIMX258Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pIMX258Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pIMX258Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pIMX258Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX258_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  IMX258 sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»
 *****************************************************************************/
RESULT IMX258_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        TRACE( IMX258_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pIMX258Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pIMX258Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pIMX258Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pIMX258Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    IMX258_Context_t *pDummyCtx = (IMX258_Context_t*) malloc( sizeof(IMX258_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( IMX258_ERROR,  "%s: Can't allocate dummy IMX258 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pIMX258Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = IMX258_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes)
    {
        
        case SUPPORT_MIPI_FOUR_LANE:
        {
            switch(Resolution)
            {
                default:
                    TRACE( IMX258_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
                    result = RET_NOTSUPP;
                    break;
                   
                case ISI_RES_2096_1560P30:
                case ISI_RES_2096_1560P25:
                case ISI_RES_2096_1560P20:
                case ISI_RES_2096_1560P15:
                case ISI_RES_2096_1560P10:
                    AFPSCHECKANDADD( ISI_RES_2096_1560P30 );
                    AFPSCHECKANDADD( ISI_RES_2096_1560P25 );
                    AFPSCHECKANDADD( ISI_RES_2096_1560P20 );
                    AFPSCHECKANDADD( ISI_RES_2096_1560P15 );
                    AFPSCHECKANDADD( ISI_RES_2096_1560P10 );
                    break;
                case ISI_RES_4208_3120P20:
                case ISI_RES_4208_3120P15:
                case ISI_RES_4208_3120P10:
               // case ISI_RES_4208_3120P7:
                    AFPSCHECKANDADD( ISI_RES_4208_3120P20 );
                    AFPSCHECKANDADD( ISI_RES_4208_3120P15 );
                    AFPSCHECKANDADD( ISI_RES_4208_3120P10 );
                  //  AFPSCHECKANDADD( ISI_RES_4208_3120P7 );
                    break;
            }
        

            break;
        }

        default:
            TRACE( IMX258_DEBUG,  "%s: pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibKFactor
 *
 * @brief   Returns the IMX258 specific K-Factor
 *
 * @param   handle       IMX258 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&IMX258_KFactor;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the IMX258 specific PCA-Matrix
 *
 * @param   handle          IMX258 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&IMX258_PCAMatrix;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              IMX258 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&IMX258_SVDMeanValue;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              IMX258 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&IMX258_CenterLine;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              IMX258 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&IMX258_AwbClipParm;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              IMX258 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&IMX258_AwbGlobalFadeParm;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              IMX258 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&IMX258_AwbFade2Parm;

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          IMX258_IsiGetIlluProfile
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
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
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
        for ( i=0U; i<IMX258_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( IMX258_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &IMX258_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetLscMatrixTable
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
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»return success;
 *****************************************************************************/
static RESULT IMX258_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
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
                if ( ( pIMX258Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pIMX258Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX258_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pIMX258Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pIMX258Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX258_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pIMX258Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pIMX258Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX258_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pIMX258Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pIMX258Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX258_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pIMX258Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pIMX258Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &IMX258_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( IMX258_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( IMX258_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          IMX258_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»
 *****************************************************************************/
static RESULT IMX258_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          IMX258 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»
 *****************************************************************************/
static RESULT IMX258_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;
		uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;
#if MODULE_NAME == MODULE_CMK
	uint8_t puSendCmd2[2] = { 0x02, 0x02 };
    uint8_t puSendCmd3[2] = { 0x06, 0x61 };
    uint8_t puSendCmd4[2] = { 0x07, 0x31 };
    unsigned char vbusy = 0;
#endif
    TRACE( IMX258_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    
#if MODULE_NAME == MODULE_CMK

do{
	result = HalReadI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                            pIMX258Ctx->IsiCtx.I2cAfBusNum,
                            pIMX258Ctx->IsiCtx.SlaveAfAddress,
                            0x05,
                            pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                            &vbusy,
                            1U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    osSleep(10);
    TRACE( IMX258_ERROR, "%s:czf wait for vbusy %d\n", __FUNCTION__,vbusy);
}while((vbusy&0x01));    

	TRACE( IMX258_ERROR, "%s:czf enable vcm\n", __FUNCTION__);
	result = HalWriteI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                             pIMX258Ctx->IsiCtx.I2cAfBusNum,
                             pIMX258Ctx->IsiCtx.SlaveAfAddress,
                             puSendCmd2[0],
                             pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                             &puSendCmd2[1],
                             1U );
  TRACE( IMX258_ERROR, "%s:czf HalWriteI2CMem result %d\n", __FUNCTION__,result);  
  osSleep(5);                     
  result = HalWriteI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                             pIMX258Ctx->IsiCtx.I2cAfBusNum,
                             pIMX258Ctx->IsiCtx.SlaveAfAddress,
                             puSendCmd3[0],
                             pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                             &puSendCmd3[1],
                             1U );
  TRACE( IMX258_ERROR, "%s:czf HalWriteI2CMem result %d\n", __FUNCTION__,result);  
  osSleep(5);
 result = HalWriteI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                             pIMX258Ctx->IsiCtx.I2cAfBusNum,
                             pIMX258Ctx->IsiCtx.SlaveAfAddress,
                             puSendCmd4[0],
                             pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                             &puSendCmd4[1],
                             1U );
#endif


 if ((pIMX258Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pIMX258Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pIMX258Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( IMX258_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));
   // *pMaxStep = MAX_LOG;

    result = IMX258_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( IMX258_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          IMX258 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²Î¿¼14825£»ÍâÖÃÂí´ï£»
 *****************************************************************************/
static RESULT IMX258_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0};

#if MODULE_NAME == MODULE_CMK
 		unsigned char vbusy = 0;
#endif
    TRACE( IMX258_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
/* SYNNEX DEBUG*/
    #if 1
    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( IMX258_ERROR, "%s: pIMX258Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pIMX258Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pIMX258Ctx->VcmInfo.StartCurrent + (pIMX258Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

#if MODULE_NAME == MODULE_GZ
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pIMX258Ctx->VcmInfo.StepMode );
	
    TRACE( IMX258_DEBUG, "CZF %s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                             pIMX258Ctx->IsiCtx.I2cAfBusNum,
                             pIMX258Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
#elif MODULE_NAME == MODULE_CMK
		
		data[0] = (uint8_t)(0x00U | (( nPosition & 0x300U ) >> 8U));   
		data[1] = (uint8_t)(nPosition & 0xFFU); 
		do{
				result = HalReadI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                            pIMX258Ctx->IsiCtx.I2cAfBusNum,
                            pIMX258Ctx->IsiCtx.SlaveAfAddress,
                            0x05,
                            pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                            &vbusy,
                            1U );
    		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    		osSleep(10);
    		TRACE( IMX258_DEBUG, "%s:czf wait for vbusy %d\n", __FUNCTION__,vbusy);
		}while((vbusy&0x01));   
		result = HalWriteI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                             pIMX258Ctx->IsiCtx.I2cAfBusNum,
                             pIMX258Ctx->IsiCtx.SlaveAfAddress,
                             0X03,
                             pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    TRACE( IMX258_DEBUG, "czf %s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );
#endif

    TRACE( IMX258_DEBUG, "%s: (exit)\n", __FUNCTION__);
    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          IMX258 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²Î¿¼14825£»ÍâÖÃÂí´ï£»
 *****************************************************************************/
static RESULT IMX258_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };
#if MODULE_NAME == MODULE_CMK
 		unsigned char vbusy = 0;
#endif
    TRACE( IMX258_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    /* SYNNEX DEBUG */
#if MODULE_NAME == MODULE_GZ

    result = HalReadI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                            pIMX258Ctx->IsiCtx.I2cAfBusNum,
                            pIMX258Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( IMX258_ERROR, "%s:[SYNNEX_VAM_DEBUG] value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );
#elif MODULE_NAME == MODULE_CMK
		do{
				result = HalReadI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                            pIMX258Ctx->IsiCtx.I2cAfBusNum,
                            pIMX258Ctx->IsiCtx.SlaveAfAddress,
                            0x05,
                            pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                            &vbusy,
                            1U );
    		RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    		osSleep(10);
    		TRACE( IMX258_DEBUG, "%s:czf wait for vbusy %d\n", __FUNCTION__,vbusy);
		}while((vbusy&0x01)); 
		
		result = HalReadI2CMem( pIMX258Ctx->IsiCtx.HalHandle,
                            pIMX258Ctx->IsiCtx.I2cAfBusNum,
                            pIMX258Ctx->IsiCtx.SlaveAfAddress,
                            0x03,
                            pIMX258Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
  
    TRACE( IMX258_DEBUG, "%s:czf [SYNNEX_VAM_DEBUG] value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
    *pAbsStep = ( ((uint32_t)(data[0] & 0x03U)) << 8U ) | ( (uint32_t)data[1]);
#endif
    /*  //map 0 to 64 -> infinity 
    if( *pAbsStep == 0 )
    {
        *pAbsStep = MAX_LOG;
    }
    else
    {
        *pAbsStep = ( MAX_REG - *pAbsStep ) / 16U;
    }*/
	if( *pAbsStep <= pIMX258Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pIMX258Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pIMX258Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pIMX258Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pIMX258Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}

   TRACE( IMX258_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»Ã»ÓÃ£»
 *****************************************************************************/
static RESULT IMX258_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          IMX258_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *²»ÓÃ¸Ä£¬Ã»ÓÃ£¬return£»
 ******************************************************************************/
static RESULT IMX258_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = IMX258_IsiRegReadIss( pIMX258Ctx, IMX258_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = IMX258_IsiRegReadIss( pIMX258Ctx, IMX258_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = IMX258_IsiRegWriteIss( pIMX258Ctx, IMX258_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pIMX258Ctx->TestPattern = enable;
    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          IMX258_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          IMX258 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä
 ******************************************************************************/
static RESULT IMX258_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pIMX258Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pIMX258Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pIMX258Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( IMX258_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT IMX258_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
    	TRACE( IMX258_ERROR, "%s: pIMX258Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( IMX258_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT IMX258_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    IMX258_Context_t *pIMX258Ctx = (IMX258_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( IMX258_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIMX258Ctx == NULL )
    {
    	TRACE( IMX258_ERROR, "%s: pIMX258Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( IMX258_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = IMX258_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          IMX258_IsiGetSensorIss
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
RESULT IMX258_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( IMX258_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = IMX258_g_acName;
        pIsiSensor->pRegisterTable                      = IMX258_g_aRegDescription_fourlane;
        pIsiSensor->pIsiSensorCaps                      = &IMX258_g_IsiSensorDefaultConfig;
		pIsiSensor->pIsiGetSensorIsiVer					= IMX258_IsiGetSensorIsiVersion;//oyyf
		pIsiSensor->pIsiGetSensorTuningXmlVersion		= IMX258_IsiGetSensorTuningXmlVersion;//oyyf
#if MODULE_NAME == MODULE_CMK
	pIsiSensor->pIsiCheckOTPInfo                    =  NULL;
#else
		pIsiSensor->pIsiCheckOTPInfo                    = check_read_otp;
#endif
		pIsiSensor->pIsiSetSensorOTPInfo				= IMX258_IsiSetOTPInfo;
		pIsiSensor->pIsiEnableSensorOTP					= IMX258_IsiEnableOTP;
        pIsiSensor->pIsiCreateSensorIss                 = IMX258_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = IMX258_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = IMX258_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = IMX258_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = IMX258_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = IMX258_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = IMX258_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = IMX258_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = IMX258_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = IMX258_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = IMX258_IsiRegWriteIss;
        
        pIsiSensor->pIsiExposureControlIss              = IMX258_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = IMX258_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = IMX258_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = IMX258_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = IMX258_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = IMX258_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = IMX258_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = IMX258_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = IMX258_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = IMX258_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = IMX258_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = IMX258_IsiGetAfpsInfoIss;
        
        /* AWB specific functions */
/* END of SYNNEX DEBUG */
        pIsiSensor->pIsiGetCalibKFactor                 = IMX258_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = IMX258_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = IMX258_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = IMX258_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = IMX258_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = IMX258_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = IMX258_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = IMX258_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = IMX258_IsiGetLscMatrixTable;
/*SYNNEX DEBUG */
        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = IMX258_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = IMX258_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = IMX258_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = IMX258_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = IMX258_IsiMdiFocusCalibrate;
/* END of SYNNEX DEBUG */
        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = IMX258_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = IMX258_IsiActivateTestPattern;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( IMX258_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT IMX258_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( IMX258_ERROR,  "%s: Can't allocate IMX258 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = IMX258_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = IMX258_SLAVE_ADDR2;
    //pSensorI2cInfo->soft_reg_addr = 0;
    //pSensorI2cInfo->soft_reg_value = 0;
    pSensorI2cInfo->reg_size = 2;
    pSensorI2cInfo->value_size = 1;

    TRACE( IMX258_DEBUG,  "%s: i2c_addr: 0x%x\n", __FUNCTION__,  pSensorI2cInfo->i2c_addr);

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps;
        uint32_t lanes,i;        

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(IMX258_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = IMX258_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = IMX258_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    /*
    sensor_chipid_info_t* pChipIDInfo_M = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_M )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_M, 0, sizeof(*pChipIDInfo_M) ); 
    pChipIDInfo_M->chipid_reg_addr = IMX258_CHIP_ID_MIDDLE_BYTE;
    pChipIDInfo_M->chipid_reg_value = IMX258_CHIP_ID_MIDDLE_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_M );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_M );
    */
    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = IMX258_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = IMX258_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
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
    IMX258_IsiGetSensorIss,
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
    IMX258_IsiGetSensorI2cInfo,
};


