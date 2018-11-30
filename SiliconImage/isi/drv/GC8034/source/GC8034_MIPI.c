/**
 * @file GC8034_MIPI.c
 *
 * @brief
 *   05/08/2018 GC8034_MIPI.c make
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

#include "GC8034_MIPI_priv.h"
//need to change tunning xml name
#define  GC8034_NEWEST_TUNING_XML "18-7-2014_oyyf-hkw_GC8034_CMK-CB0407-FV1_v0.1.2"


/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( GC8034_INFO , "GC8034: ", INFO,    0U );
CREATE_TRACER( GC8034_WARN , "GC8034: ", WARNING, 0U );
CREATE_TRACER( GC8034_ERROR, "GC8034: ", ERROR,   1U );

CREATE_TRACER( GC8034_DEBUG, "GC8034: ", DEBUG,  1U );

CREATE_TRACER( GC8034_NOTICE0 , "GC8034: ", TRACE_NOTICE0, 0);
CREATE_TRACER( GC8034_NOTICE1, "GC8034: ", TRACE_NOTICE1, 0U );


#define GC8034_SLAVE_ADDR       0x6eU                           /**< i2c slave address of the GC8034 camera sensor */
#define GC8034_SLAVE_ADDR2      0x20U
#define GC8034_SLAVE_AF_ADDR    0x18U         //?                  /**< i2c slave address of the GC8034 integrated AD5820 */
#define Sensor_OTP_SLAVE_ADDR   0x6eU
#define Sensor_OTP_SLAVE_ADDR2   0x6eU

#define GC8034_MAXN_GAIN 		(128.0f)
#define GC8034_MIN_GAIN_STEP   ( 1.0f / GC8034_MAXN_GAIN); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define GC8034_MAX_GAIN_AEC    ( 16.0f )         /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */


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
const char GC8034_g_acName[] = "GC8034_MIPI";

extern const IsiRegDescription_t GC8034_g_aRegDescription_fourlane[];
extern const IsiRegDescription_t GC8034_g_3264x2448_fourlane[];
extern const IsiRegDescription_t GC8034_g_3264x2448P30_fourlane_fpschg[];
extern const IsiRegDescription_t GC8034_g_3264x2448P25_fourlane_fpschg[];
extern const IsiRegDescription_t GC8034_g_3264x2448P20_fourlane_fpschg[];
extern const IsiRegDescription_t GC8034_g_3264x2448P15_fourlane_fpschg[];
extern const IsiRegDescription_t GC8034_g_3264x2448P10_fourlane_fpschg[];
extern const IsiRegDescription_t GC8034_g_3264x2448P7_fourlane_fpschg[];


const IsiSensorCaps_t GC8034_g_IsiSensorDefaultConfig;



#define GC8034_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define GC8034_I2C_NR_ADR_BYTES     (1U)                        // 1 byte base address
#define GC8034_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_FOUR_LANE

static uint32_t Dgain_ratio = 1;



/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT GC8034_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT GC8034_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT GC8034_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT GC8034_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT GC8034_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT GC8034_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT GC8034_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT GC8034_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT GC8034_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT GC8034_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT GC8034_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT GC8034_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT GC8034_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT GC8034_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT GC8034_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT GC8034_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT GC8034_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT GC8034_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT GC8034_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT GC8034_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT GC8034_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT GC8034_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT GC8034_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT GC8034_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT GC8034_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT GC8034_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT GC8034_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT GC8034_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT GC8034_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT GC8034_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT GC8034_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT GC8034_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT GC8034_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT GC8034_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT GC8034_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT GC8034_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT GC8034_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT GC8034_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);
static RESULT GC8034_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate);



/* OTP START*/
static int GC8034_OTP_Read_i2c(sensor_i2c_write_t*  sensor_i2c_write_p,sensor_i2c_read_t*	sensor_i2c_read_p,void* context,int camsys_fd,int page,int address)
{
    uint32_t temp1;
    int i2c_base_info[3];
    int ret = 0;
    unsigned int otp_clk, otp_en;
    i2c_base_info[0] = 0x6e; //otp i2c addr
    i2c_base_info[1] = 1; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size
    ret = sensor_i2c_write_p(context,camsys_fd, 0xFE, 0x00, i2c_base_info );

    ret = sensor_i2c_write_p(context,camsys_fd, 0xD4, ((page << 2) & 0x3c) + ((address >> 5) & 0x03), i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xD5, (address << 3) & 0xff, i2c_base_info );
    // OTP Read    
    ret = sensor_i2c_write_p(context,camsys_fd, 0xF3, 0x20, i2c_base_info );
    temp1 = sensor_i2c_read_p(context,camsys_fd,0xD7,i2c_base_info);
    TRACE( GC8034_DEBUG, "%s:P%d Reg0x%x = 0x%x\n", __FUNCTION__,page,address, temp1);
    return      temp1;
}

typedef struct gc8034_dd{
	uint16_t x;
	uint16_t y;
	uint16_t t;
}gc8034_dd_t;
	
typedef struct otp_gc8034
{
    uint16_t dd_cnt;
    uint16_t dd_flag;
    gc8034_dd_t dd_param[160];
    uint16_t module_id;
    uint16_t lens_id;
    uint16_t vcm_id;
    uint16_t vcm_driver_id;
    uint16_t year;
    uint16_t month;
    uint16_t day;
    uint16_t rg_gain;
    uint16_t bg_gain;
    uint16_t wb_flag;
    uint16_t golden_flag; 
    uint16_t golden_rg;
    uint16_t golden_bg;   
    uint16_t lsc_flag;// 0:Empty 1:Success 2:Invalid
    int  lsc_param[396];
    uint16_t reg_page[5];
    uint16_t reg_addr[5]; 
    uint16_t reg_value[5];    
    uint16_t reg_flag;
    uint16_t reg_num;
}gc8034_otp;

static gc8034_otp gc8034_otp_info = {0};



int  RG_Ratio_Typical=0x0;
int  BG_Ratio_Typical=0x0;
bool bOTP_switch = true;

#define GC8034_MIRROR_NORMAL
#define DD_WIDTH 3284
#define DD_HEIGHT 2464

#define DD_PARAM_QTY        350
#define WINDOW_WIDTH        0x0cd4 //3284 max effective pixels
#define WINDOW_HEIGHT       0x09a0 //2462
#define REG_ROM_START       0x4e
#define RG_TYPICAL          0x0400
#define BG_TYPICAL          0x0400
#define INFO_ROM_START      0x70
#define INFO_WIDTH          0x08
#define WB_ROM_START        0x5f   
#define WB_WIDTH            0x04  
#define GOLDEN_ROM_START    0x67  //golden R/G ratio
#define GOLDEN_WIDTH        0x04
#define LSC_NUM             99//0x63 //(7+2)*(9+2)
static uint8_t LSC_ADDR[4]={0x0e,0x20,0x1a,0x88};


static void GC8034_OTP_readgroup( sensor_i2c_write_t*  sensor_i2c_write_p,sensor_i2c_read_t*	sensor_i2c_read_p,void* context,int camsys_fd,int page,   int addr, int* buff,int size)   
{
	int i = 0;
	int temp_page = 0;
	int temp_addr = 0;
	temp_addr = addr;
	temp_page = page;
	for (i = 0; i < size; i++){
		if ((temp_addr  % 0x80) == 0) {
			temp_page += 1;
			temp_addr = 0;
			TRACE( GC8034_DEBUG, "%s  temp_page %d,temp_addr %d\n", __FUNCTION__,temp_page,temp_addr);
		}
		buff[i] = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,temp_page,temp_addr);
		temp_addr += 1;
	}	
}

static void GC8034_OTP_readinfo(
	sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	void* context,
	int camsys_fd
) 
{
    int flagdd = 0;
    int flag_lsc = 0;
    int i=0, j=0;
    int total_number = 0,cnt =0;
    int temp = 0;
    uint32_t check = 0;
    int flag_wb = 0,index = 0,flag_Module = 0;
    int info[8]={0};
    int wb[4]={0};
    int golden[4]={0};
    int lsc_checksum = 0; 
    int ddtempbuff[4 * 80] = { 0 };
       
    flagdd = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0,0x0b);
    TRACE( GC8034_DEBUG, "%s  GC8034 OTP:flag_dd=0x%x!\n", __FUNCTION__,flagdd);

    switch(flagdd & 0x03)
    {
        case 0x00:
            TRACE( GC8034_ERROR, "%s GC8034 OTP:flag_dd is EMPTY!\n", __FUNCTION__);
            gc8034_otp_info.dd_flag = 0x00;
            break;
        case 0x01:
            total_number = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0,0x0c)+GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0,0x0d);;
		GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0,0x0e, &ddtempbuff[0], 4*total_number);
		for (i = 0; i < total_number; i++) {
			if ((ddtempbuff[4 * i + 3] & 0x80) == 0x80) {
				if ((ddtempbuff[4 * i + 3] & 0x03) == 0x03) {
					gc8034_otp_info.dd_param[cnt].x = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc8034_otp_info.dd_param[cnt].y = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
					gc8034_otp_info.dd_param[cnt++].t = 2;
					gc8034_otp_info.dd_param[cnt].x = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc8034_otp_info.dd_param[cnt].y = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4) + 1;
					gc8034_otp_info.dd_param[cnt++].t = 2;
				}
				else {
					gc8034_otp_info.dd_param[cnt].x = (((uint16_t)ddtempbuff[4 * i + 1] & 0x0f) << 8) + ddtempbuff[4 * i];
					gc8034_otp_info.dd_param[cnt].y = ((uint16_t)ddtempbuff[4 * i + 2] << 4) + ((ddtempbuff[4 * i + 1] & 0xf0) >> 4);
					gc8034_otp_info.dd_param[cnt++].t = ddtempbuff[4 * i + 3] & 0x03;
				}
			}
		}
		TRACE( GC8034_DEBUG, "%s GC8034 OTP:total_number = %d !\n", __FUNCTION__,total_number);
            gc8034_otp_info.dd_cnt = total_number;
            gc8034_otp_info.dd_flag = 0x01;
            break;
        case 0x02:
        case 0x03:  
            TRACE( GC8034_ERROR, "%s GC8034 OTP:flag_dd is INVAILD!\n", __FUNCTION__);
            gc8034_otp_info.dd_flag = 0x02;
            break;
        default :
            break;
    }
    flag_Module = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9,0x6f);
    flag_wb = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9,0x5e);
    flag_lsc = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,3,0x43);
    TRACE( GC8034_DEBUG, "%s  GC8034 OTP:flag_Module=0x%x, flag_wb=0x%x!\n", __FUNCTION__,flag_Module,flag_wb);

    /*SYNNEX DEBUG*/
    //INFO & WB
    for(index=0;index<2;index++)
    {
        switch((flag_Module<<(2*index))&0x0c)
        {
            case 0x00:
                TRACE( GC8034_ERROR, "%s GC8034_OTP_INFO group %d is Empty!\n", __FUNCTION__,index +1);
                break;
            case 0x04:
                TRACE( GC8034_DEBUG, "%s GC8034_OTP_INFO group %d is Vaild!\n", __FUNCTION__,index +1);
                check = 0;
                GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9,(INFO_ROM_START + index * INFO_WIDTH), &info[0], INFO_WIDTH);
                for (i = 0; i < INFO_WIDTH - 1; i++)
                {
                    check += info[i];
                }
                if ((check % 255 + 1) == info[INFO_WIDTH-1])
                {
                    gc8034_otp_info.module_id = info[0];
                    gc8034_otp_info.lens_id = info[1];
                    gc8034_otp_info.vcm_driver_id = info[2];
                    gc8034_otp_info.vcm_id = info[3];
                    gc8034_otp_info.year = info[4];
                    gc8034_otp_info.month = info[5];
                    gc8034_otp_info.day = info[6];
			TRACE( GC8034_DEBUG, "%s  module_id\t= %d\n", __FUNCTION__,gc8034_otp_info.module_id);
        		TRACE( GC8034_DEBUG, "%s  lens_id\t= %d\n", __FUNCTION__,gc8034_otp_info.lens_id);
        		TRACE( GC8034_DEBUG, "%s  vcm_id\t\t= %d\n", __FUNCTION__,gc8034_otp_info.vcm_id);
        		TRACE( GC8034_DEBUG, "%s  vcm_driver_id\t= %d\n", __FUNCTION__,gc8034_otp_info.vcm_driver_id);
        		TRACE( GC8034_DEBUG, "%s  date\t\t= %d-%d-%d\n", __FUNCTION__,gc8034_otp_info.year,
				gc8034_otp_info.month,gc8034_otp_info.day);
                }
                else
                {
                    TRACE( GC8034_ERROR, "%s GC8034_OTP_INFO Check sum %d Error!\n", __FUNCTION__,index +1);
                }
                break;
            case 0x08:
            case 0x0c:  
                TRACE( GC8034_ERROR, "%s GC8034_OTP_INFO group %d is Invalid !!\n", __FUNCTION__,index +1);
                break;
            default :
                break;
        }
        switch((flag_wb<<(2 * index))&0x0c)
        {
        case 0x00:
            TRACE( GC8034_ERROR, "%s GC8034_OTP_WB group %d is Empty !\n", __FUNCTION__,index +1);
            gc8034_otp_info.wb_flag = gc8034_otp_info.wb_flag|0x00;
            break;
        case 0x04:
            TRACE( GC8034_ERROR, "%s GC8034_OTP_WB group %d is Valid !!\n", __FUNCTION__,index +1);
            check = 0;
            GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9, (WB_ROM_START + index * WB_WIDTH), &wb[0], WB_WIDTH);
            for (i = 0; i < WB_WIDTH - 1; i++)
            {
                check += wb[i];
            }
            if ((check % 255 + 1) == wb[WB_WIDTH - 1])
            {
			gc8034_otp_info.rg_gain = (wb[0]|((wb[1]&0xf0)<<4)) > 0 ? (wb[0]|((wb[1]&0xf0)<<4)) : 0x400;
			gc8034_otp_info.bg_gain = (((wb[1]&0x0f)<<8)|wb[2]) > 0 ? (((wb[1]&0x0f)<<8)|wb[2]) : 0x400;
			gc8034_otp_info.wb_flag = gc8034_otp_info.wb_flag|0x01;
			TRACE( GC8034_DEBUG, "%s rg_gain = 0x%x\n", __FUNCTION__,gc8034_otp_info.rg_gain);
			TRACE( GC8034_DEBUG, "%s  bg_gain = 0x%x\n", __FUNCTION__,gc8034_otp_info.bg_gain);
            }
            else
            {
                TRACE( GC8034_ERROR, "%s GC8034_OTP_WB Check sum %d Error !!\n", __FUNCTION__,index +1);
            }
            property_set("sys_graphic.cam_otp_awb", "true");
         		property_set("sys_graphic.cam_otp_awb_enable", "true");
            break;
        case 0x08:
        case 0x0c: 
            TRACE( GC8034_ERROR, "%s GC8034_OTP_WB group %d is Invalid !!\n", __FUNCTION__,index +1);         
            gc8034_otp_info.wb_flag = gc8034_otp_info.wb_flag|0x02;
            break;
        default :
            break;
        }

        switch((flag_wb<<(2 * index))&0xc0)
        {
            case 0x00:
                TRACE( GC8034_ERROR, "%s GC8034_OTP_GOLDEN group %d is Empty !!\n", __FUNCTION__,index +1);
                gc8034_otp_info.golden_flag = gc8034_otp_info.golden_flag&0x00;                 
                break;
            case 0x40:
                TRACE( GC8034_DEBUG, "%s GC8034_OTP_GOLDEN group %d is Vaild !!\n", __FUNCTION__,index +1);
                check = 0;
                GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9,(GOLDEN_ROM_START + index * GOLDEN_WIDTH), &golden[0], GOLDEN_WIDTH);
                for (i = 0; i < GOLDEN_WIDTH - 1; i++)
                {
                    check += golden[i];
                }
                if ((check % 255 + 1) == golden[GOLDEN_WIDTH - 1])
                {
                    gc8034_otp_info.golden_rg = (golden[0]|((golden[1]&0xf0)<<4)) > 0 ? (golden[0]|((golden[1]&0xf0)<<4)) : RG_TYPICAL;
                    gc8034_otp_info.golden_bg = (((golden[1]&0x0f)<<8)|golden[2]) > 0 ? (((golden[1]&0x0f)<<8)|golden[2]) : BG_TYPICAL;
                    gc8034_otp_info.golden_flag = gc8034_otp_info.golden_flag|0x01; 
			TRACE( GC8034_DEBUG, "%s  golden_rg\t= 0x%x\n", __FUNCTION__,gc8034_otp_info.golden_rg);
        		TRACE( GC8034_DEBUG, "%s  golden_bg\t= 0x%x\n", __FUNCTION__,gc8034_otp_info.golden_bg);
                }
                else
                {
                    TRACE( GC8034_ERROR, "%s GC8034_OTP_GOLDEN Check sum %d Error !!\n", __FUNCTION__,index +1);
                }
                break;
                case 0x80:
                case 0xc0: 
                    TRACE( GC8034_ERROR, "%s GC8034_OTP_GOLDEN group %d is Invalid !!\n", __FUNCTION__,index +1);  
                    gc8034_otp_info.golden_flag = gc8034_otp_info.golden_flag|0x02;         
                    break;
                default :
                    break;
        }

	switch((flag_lsc<<(2 * index))&0x0c)
        {
            case 0x00:
                TRACE( GC8034_DEBUG, "%s GC8034_OTP_LSC group %d is Empty !\n", __FUNCTION__,index + 1);
                break;
            case 0x04:
                TRACE( GC8034_DEBUG, "%s  GC8034_OTP_LSC group %d is Valid !\n", __FUNCTION__,index + 1);
                property_set("sys_graphic.cam_otp_lsc", "true");
		   property_set("sys_graphic.cam_otp_lsc_enable", "true");
		   if(0 == index){
			GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,3,0x44, &gc8034_otp_info.lsc_param[0], 396);
			lsc_checksum = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,6,0x50);
		   }else{
			GC8034_OTP_readgroup(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,6,0x51, &gc8034_otp_info.lsc_param[0], 396);
			lsc_checksum = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,9,0x5d);
		   }
		   check = 0;
		   for (i = 0; i < 396; i++){
			check += gc8034_otp_info.lsc_param[i];
			//TRACE( GC8034_DEBUG, "%s lsc_param[%d] = 0x%02x\n", __FUNCTION__,i,gc8034_otp_info.lsc_param[i]);
			//osSleep(1);
		   }
		   if ((check % 255 + 1) == lsc_checksum){
				gc8034_otp_info.lsc_flag = 0x01;
		   		TRACE( GC8034_DEBUG, "%s GC8034_OTP_LSC check sum success\n",__FUNCTION__);
		   } else {
				gc8034_otp_info.lsc_flag = 0x02;
				TRACE( GC8034_DEBUG,"GC8034_OTP_LSC check sum error, check sum read : 0x%x, calculate:0x%x!!\n", lsc_checksum, check % 255 + 1);
		   }			
                break;
            case 0x08:
            case 0x0c:
                TRACE( GC8034_ERROR, "%s GC8034_OTP_LSC group %d is Invalid !\n", __FUNCTION__,index + 1);
                break;
            default :
                break;
        }
    }
	 /* Chip Register */
        //GC8034_OTP_pageselect(handle,2);
        gc8034_otp_info.reg_flag = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,2,0x4e);
        if(gc8034_otp_info.reg_flag==1)
        {
            for(i=0;i<5;i++)
            {
            TRACE( GC8034_DEBUG, "%s GC8034 Chip Register is Invalid !!\n", __FUNCTION__);  
            temp = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,2,(0x4f+5*i) );
            for(j=0;j<2;j++)
                {   
                if(((temp>>(4*j+3))&0x01)==0x01)
                    {
                        gc8034_otp_info.reg_page[gc8034_otp_info.reg_num] = (temp>>(4*j))&0x03;
                        gc8034_otp_info.reg_addr[gc8034_otp_info.reg_num] = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,2,(0x50+5*i+2*j));
                        gc8034_otp_info.reg_value[gc8034_otp_info.reg_num] = GC8034_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,2,(0x50+5*i+2*j+1));
                        gc8034_otp_info.reg_num++;
                    }
                }
            }
        }
}

static void GC8034_OTP_update_dd(IsiSensorHandle_t   handle)
{
	uint8_t i = 0, j = 0;
	uint8_t temp_val0 = 0,temp_val1 = 0,temp_val2 = 0;
	struct gc8034_dd dd_temp = {0, 0, 0};
	if (gc8034_otp_info.dd_flag == 0x01) {
		TRACE( GC8034_DEBUG,"GC8034_OTP_AUTO_DD start !\n");
		for(i = 0; i < gc8034_otp_info.dd_cnt; i++) {
#if defined(GC8034_MIRROR_H) || defined(GC8034_MIRROR_HV)
			switch(gc8034_otp_info.dd_param[i].t) {
			case 0:
				gc8034_otp_info.dd_param[i].x = DD_WIDTH - gc8034_otp_info.dd_param[i].x + 1;
				break;
			case 1:
				gc8034_otp_info.dd_param[i].x = DD_WIDTH - gc8034_otp_info.dd_param[i].x - 1;
				break;
			default:
				gc8034_otp_info.dd_param[i].x = DD_WIDTH - gc8034_otp_info.dd_param[i].x;
				break;
			}
#endif
#if defined(GC8034_MIRROR_V) || defined(GC8034_MIRROR_HV)
			gc8034_otp_info.dd_param[i].y = DD_HEIGHT -gc8034_otp_info. dd_param[i].y + 1;
#endif
		}
		for(i = 0; i < gc8034_otp_info.dd_cnt - 1; i++) {
			for(j = i + 1; j < gc8034_otp_info.dd_cnt; j++) {
				if(gc8034_otp_info.dd_param[i].y * DD_WIDTH + gc8034_otp_info.dd_param[i].x
				 > gc8034_otp_info.dd_param[j].y * DD_WIDTH + gc8034_otp_info.dd_param[j].x) {
					dd_temp.x = gc8034_otp_info.dd_param[i].x;
					dd_temp.y = gc8034_otp_info.dd_param[i].y;
					dd_temp.t = gc8034_otp_info.dd_param[i].t;
					gc8034_otp_info.dd_param[i].x = gc8034_otp_info.dd_param[j].x;
					gc8034_otp_info.dd_param[i].y = gc8034_otp_info.dd_param[j].y;
					gc8034_otp_info.dd_param[i].t = gc8034_otp_info.dd_param[j].t;
					gc8034_otp_info.dd_param[j].x = dd_temp.x;
					gc8034_otp_info.dd_param[j].y = dd_temp.y;
					gc8034_otp_info.dd_param[j].t = dd_temp.t;
				}
			}
		}
		GC8034_IsiRegWriteIss(handle,0xfe, 0x01);
		GC8034_IsiRegWriteIss(handle,0xbe, 0x00);
		GC8034_IsiRegWriteIss(handle,0xa9, 0x01);
		for (i = 0; i < gc8034_otp_info.dd_cnt; i++) {
			temp_val0 = gc8034_otp_info.dd_param[i].x & 0x00ff;
			temp_val1 = ((gc8034_otp_info.dd_param[i].y & 0x000f) << 4) + ((gc8034_otp_info.dd_param[i].x & 0x0f00)>>8);
			temp_val2 = (gc8034_otp_info.dd_param[i].y & 0x0ff0) >> 4;
			GC8034_IsiRegWriteIss(handle,0xaa,i);
			GC8034_IsiRegWriteIss(handle,0xac,temp_val0);
			GC8034_IsiRegWriteIss(handle,0xac,temp_val1);
			GC8034_IsiRegWriteIss(handle,0xac,temp_val2);
			GC8034_IsiRegWriteIss(handle,0xac,gc8034_otp_info.dd_param[i].t);
			
			TRACE( GC8034_DEBUG,"GC8034_OTP_GC val0 = 0x%x , val1 = 0x%x , val2 = 0x%x \n",temp_val0,temp_val1,temp_val2);
			TRACE( GC8034_DEBUG,"GC8034_OTP_GC x = %d , y = %d \n",((temp_val1&0x0f)<<8) + temp_val0,(temp_val2<<4) + ((temp_val1&0xf0)>>4));	
		}

		GC8034_IsiRegWriteIss(handle,0xbe, 0x01);
		GC8034_IsiRegWriteIss(handle,0xfe, 0x00);
	}

}
static void GC8034_OTP_update_wb(IsiSensorHandle_t   handle)
{
    uint16_t r_gain_current = 0 , g_gain_current = 0 , b_gain_current = 0 , base_gain = 0;
    uint16_t r_gain = 1024 , g_gain = 1024 , b_gain = 1024 ;
    uint16_t rg_typical,bg_typical;
    rg_typical=RG_Ratio_Typical;
    bg_typical=BG_Ratio_Typical;        
    TRACE( GC8034_DEBUG, "%s GC8034_OTP_UPDATE_AWB:rg_typical = 0x%x , bg_typical = 0x%x !\n", __FUNCTION__,rg_typical,bg_typical);  
    if(0x01==(gc8034_otp_info.wb_flag&0x01))
    {   
        r_gain_current = 2048 * rg_typical/gc8034_otp_info.rg_gain;
        b_gain_current = 2048 * bg_typical/gc8034_otp_info.bg_gain;
        g_gain_current = 2048;

        base_gain = (r_gain_current<b_gain_current) ? r_gain_current : b_gain_current;
        base_gain = (base_gain<g_gain_current) ? base_gain : g_gain_current;
        TRACE( GC8034_DEBUG, "%s GC8034_OTP_UPDATE_AWB:r_gain_current = 0x%x , b_gain_current = 0x%x , base_gain = 0x%x\n", __FUNCTION__,r_gain_current,b_gain_current,base_gain);           
        r_gain = 0x400 * r_gain_current / base_gain;
        g_gain = 0x400 * g_gain_current / base_gain;
        b_gain = 0x400 * b_gain_current / base_gain;
        TRACE( GC8034_DEBUG, "%s GC8034_OTP_UPDATE_AWB:r_gain = 0x%x , g_gain = 0x%x , b_gain = 0x%x\n", __FUNCTION__,r_gain,g_gain,b_gain);
        /*TODO*/
        GC8034_IsiRegWriteIss(handle,0xfe,0x01);
        GC8034_IsiRegWriteIss(handle,0x84,g_gain>>3);
        GC8034_IsiRegWriteIss(handle,0x85,r_gain>>3);
        GC8034_IsiRegWriteIss(handle,0x86,b_gain>>3);
        GC8034_IsiRegWriteIss(handle,0x87,g_gain>>3);
        GC8034_IsiRegWriteIss(handle,0x88,((g_gain&0x07) << 4) + (r_gain&0x07));
        GC8034_IsiRegWriteIss(handle,0x89,((b_gain&0x07) << 4) + (g_gain&0x07));
        GC8034_IsiRegWriteIss(handle,0xfe,0x00);
    }
}

static void GC8034_OTP_update_lsc(IsiSensorHandle_t   handle)
{
	int i,j;
	uint16_t base = 0;
	if (0x01 == (gc8034_otp_info.lsc_flag & 0x01)) {
		TRACE( GC8034_DEBUG, "%s GC8034  update lsc !\n", __FUNCTION__);   
		GC8034_IsiRegWriteIss(handle,0xfe,0x01);
		GC8034_IsiRegWriteIss(handle,0xcf,0x00);
		GC8034_IsiRegWriteIss(handle,0xc9,0x01);
		for (i = 0; i < 9; i++) {
			GC8034_IsiRegWriteIss(handle,0xca, i * 0x0c);
			for (j = 0; j < 11; j++) {
#if defined(GC8034_MIRROR_NORMAL)
				base = 4 * (11 * i + j);
#elif defined(GC8034_MIRROR_H)
				base = 4 * (11 * i + 10 - j);
#elif defined(GC8034_MIRROR_V)
				base = 4 * (11 * (8 - i) + j);
#elif defined(GC8034_MIRROR_HV)
				base = 4 * (11 * (8 - i) + 10 - j);
#endif
				GC8034_IsiRegWriteIss(handle,0xcc, gc8034_otp_info.lsc_param[base + 0]);
				GC8034_IsiRegWriteIss(handle,0xcc, gc8034_otp_info.lsc_param[base + 1]);
				GC8034_IsiRegWriteIss(handle,0xcc, gc8034_otp_info.lsc_param[base + 2]);
				GC8034_IsiRegWriteIss(handle,0xcc, gc8034_otp_info.lsc_param[base + 3]);
			}
		}
		GC8034_IsiRegWriteIss(handle,0xcf, 0x01);
		GC8034_IsiRegWriteIss(handle,0xa0, 0x13);
		GC8034_IsiRegWriteIss(handle,0xfe, 0x00);

 	}
}
static void GC8034_OTP_update_chipversion(IsiSensorHandle_t   handle)
{
    int i; 
    GC8034_IsiRegWriteIss(handle,0xfe,0x00);
    if(gc8034_otp_info.reg_flag)
    {
        for(i=0;i<gc8034_otp_info.reg_num;i++) 
        {
            GC8034_IsiRegWriteIss(handle,0xfe,gc8034_otp_info.reg_page[i]);
            GC8034_IsiRegWriteIss(handle,gc8034_otp_info.reg_addr[i],gc8034_otp_info.reg_value[i]);  
            TRACE( GC8034_ERROR, "%s  GC8034_OTP_UPDATE_CHIP_VERSION:{0x%x,0x%x}!!\n", __FUNCTION__,gc8034_otp_info.reg_addr[i],gc8034_otp_info.reg_value[i]);   
        }
    }
}

static void GC8034_OTP_update(IsiSensorHandle_t   handle)
{
	char prop_value[PROPERTY_VALUE_MAX];
    //GC8034_OTP_update_dd(handle);
    property_get("sys_graphic.cam_otp_awb_enable", prop_value, "true");
    if (!strcmp(prop_value,"true")) {
   	 	GC8034_OTP_update_wb(handle);
   	}
   	property_get("sys_graphic.cam_otp_lsc_enable", prop_value, "true");
    if (!strcmp(prop_value,"true")) {
    	GC8034_OTP_update_lsc(handle);
    }
    //GC8034_OTP_update_chipversion(handle);
}
static void GC8034_OTP_Enable(IsiSensorHandle_t   handle,    int OTPstate)
{
    uint32_t otp_clk,otp_en;
    GC8034_IsiRegReadIss(handle,0xf2, &otp_clk);
    GC8034_IsiRegReadIss(handle,0xf4, &otp_en);
    if(OTPstate)
    {
        otp_clk |= 0x01;
        otp_en |= 0x08;
        GC8034_IsiRegWriteIss(handle, 0xf2, otp_clk);
        GC8034_IsiRegWriteIss(handle, 0xf4, otp_en );
        TRACE( GC8034_DEBUG, "%s  GC8034 OTP ENABLE!\n", __FUNCTION__);
    }
    else
    {
        otp_clk &=0xfe;
        otp_en &=0xf7;
        GC8034_IsiRegWriteIss(handle, 0xf2, otp_clk);
        GC8034_IsiRegWriteIss(handle, 0xf4, otp_en );
        TRACE( GC8034_DEBUG, "%s GC8034 OTP DISABLE!\n", __FUNCTION__);
    }
}
static int apply_otp(IsiSensorHandle_t   handle)
{
   GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xf7, 0x97);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0xee);
   
   GC8034_OTP_Enable(handle,1);
   GC8034_OTP_update(handle);
   GC8034_OTP_Enable(handle,0);
   
    GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xf7, 0x95);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0x00);
   GC8034_IsiRegWriteIss(handle, 0xfc, 0xee);
   return RET_SUCCESS;
}

/* OTP */
static RESULT check_read_otp(sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	sensor_version_get_t* sensor_version_get_p,
	void* context,
	int camsys_fd)
{
	int i2c_base_info[3];
	uint32_t temp1;
	int ii;
	int ret = 0;
	uint32_t otp_clk, otp_en;
	i2c_base_info[0] = 0x6e; //otp i2c addr
  	i2c_base_info[1] = 1; //otp i2c reg size
  	i2c_base_info[2] = 1; //otp i2c value size
  	
	sensor_i2c_write_p(context,camsys_fd, 0xf2, 0x00, i2c_base_info );
	sensor_i2c_write_p(context,camsys_fd, 0xf4, 0x80, i2c_base_info );

    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xf7, 0x97, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0xee, i2c_base_info );
   
    otp_clk = sensor_i2c_read_p(context,camsys_fd, 0xF2, i2c_base_info);
    otp_en = sensor_i2c_read_p(context,camsys_fd, 0xF4, i2c_base_info);
    otp_clk |=0x01;
    otp_en |=0x08;
    ret = sensor_i2c_write_p(context,camsys_fd, 0xF2, otp_clk, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xF4, otp_en, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfe, 0x00, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0x30, 0x00, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xd3, 0x00, i2c_base_info );
    osSleep(20);

    GC8034_OTP_readinfo(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd);
		
    otp_clk = sensor_i2c_read_p(context,camsys_fd, 0xF2, i2c_base_info);
    otp_en = sensor_i2c_read_p(context,camsys_fd, 0xF4, i2c_base_info);  
    otp_clk &= 0xFE;
    otp_en &= 0xF7;

    ret = sensor_i2c_write_p(context,camsys_fd, 0xF4, otp_en, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xF2, otp_clk, i2c_base_info );

    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xf7, 0x95, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x00, i2c_base_info );
    sensor_i2c_write_p(context,camsys_fd, 0xfc, 0xee, i2c_base_info );
	
    return RET_SUCCESS;
}

static RESULT GC8034_IsiSetOTPInfo
(
    IsiSensorHandle_t       handle,
    uint32_t OTPInfo
)
{
	RESULT result = RET_SUCCESS;

    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

	RG_Ratio_Typical = OTPInfo>>16;
	BG_Ratio_Typical = OTPInfo&0xffff;
	TRACE( GC8034_DEBUG, "%s:  --(RG,BG) in IQ file:(0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);
	if((RG_Ratio_Typical==0) && (BG_Ratio_Typical==0)){
		if(0x01==(gc8034_otp_info.golden_flag&0x01))
    		{
        		RG_Ratio_Typical=gc8034_otp_info.golden_rg;
        		BG_Ratio_Typical=gc8034_otp_info.golden_bg;   
    		}else{
			RG_Ratio_Typical = RG_TYPICAL;
	       	BG_Ratio_Typical = BG_TYPICAL;
		}
		TRACE( GC8034_ERROR, "%s:  --OTP typical value in IQ file is zero, we will try another match rule.\n", __FUNCTION__);    
	}
	TRACE( GC8034_DEBUG, "%s:  --Finally, the (RG,BG) is (0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);

	return (result);
}

static RESULT GC8034_IsiEnableOTP
(
    IsiSensorHandle_t       handle,
    const bool_t enable
)
{
	RESULT result = RET_SUCCESS;

    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }
	bOTP_switch = enable;
	return (result);
}


/* OTP END*/


/*****************************************************************************/
/**
 *          GC8034_IsiCreateSensorIss
 *
 * @brief   This function creates a new GC8034 sensor instance handle.
 *
 * @param   pConfig     configuration structure to create the instance
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @retval  RET_OUTOFMEM
 *
 *****************************************************************************/
static RESULT GC8034_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
	int32_t current_distance;
    GC8034_Context_t *pGC8034Ctx;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pGC8034Ctx = ( GC8034_Context_t * )malloc ( sizeof (GC8034_Context_t) );
    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR,  "%s: Can't allocate GC8034 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pGC8034Ctx, 0, sizeof( GC8034_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pGC8034Ctx );
        return ( result );
    }
    
    pGC8034Ctx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pGC8034Ctx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pGC8034Ctx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pGC8034Ctx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? GC8034_SLAVE_ADDR : pConfig->SlaveAddr;
    pGC8034Ctx->IsiCtx.NrOfAddressBytes       = 1U;

    pGC8034Ctx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pGC8034Ctx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? GC8034_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
    pGC8034Ctx->IsiCtx.NrOfAfAddressBytes     = 0U;

    pGC8034Ctx->IsiCtx.pSensor                = pConfig->pSensor;

    pGC8034Ctx->Configured             = BOOL_FALSE;
    pGC8034Ctx->Streaming              = BOOL_FALSE;
    pGC8034Ctx->TestPattern            = BOOL_FALSE;
    pGC8034Ctx->isAfpsRun              = BOOL_FALSE;
    /* ddl@rock-chips.com: v0.3.0 */
    current_distance = pConfig->VcmRatedCurrent - pConfig->VcmStartCurrent;
    current_distance = current_distance*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pGC8034Ctx->VcmInfo.Step = (current_distance+(MAX_LOG-1))/MAX_LOG;
    pGC8034Ctx->VcmInfo.StartCurrent   = pConfig->VcmStartCurrent*MAX_VCMDRV_REG/MAX_VCMDRV_CURRENT;    
    pGC8034Ctx->VcmInfo.RatedCurrent   = pGC8034Ctx->VcmInfo.StartCurrent + MAX_LOG*pGC8034Ctx->VcmInfo.Step;
    pGC8034Ctx->VcmInfo.StepMode       = pConfig->VcmStepMode;    
	
	pGC8034Ctx->IsiSensorMipiInfo.sensorHalDevID = pGC8034Ctx->IsiCtx.HalDevID;
	if(pConfig->mipiLaneNum & g_suppoted_mipi_lanenum_type)
        pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes = pConfig->mipiLaneNum;
    else{
        TRACE( GC8034_ERROR, "%s don't support lane numbers :%d,set to default %d\n", __FUNCTION__,pConfig->mipiLaneNum,DEFAULT_NUM_LANES);
        pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }
	
    pConfig->hSensor = ( IsiSensorHandle_t )pGC8034Ctx;

    result = HalSetCamConfig( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, false, true, false ); //pwdn,reset active;hkw
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiReleaseSensorIss
 *
 * @brief   This function destroys/releases an GC8034 sensor instance.
 *
 * @param   handle      GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 *
 *****************************************************************************/
static RESULT GC8034_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)GC8034_IsiSensorSetStreamingIss( pGC8034Ctx, BOOL_FALSE );
    (void)GC8034_IsiSensorSetPowerIss( pGC8034Ctx, BOOL_FALSE );

    (void)HalDelRef( pGC8034Ctx->IsiCtx.HalHandle );

    MEMSET( pGC8034Ctx, 0, sizeof( GC8034_Context_t ) );
    free ( pGC8034Ctx );

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCapsIss
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
static RESULT GC8034_IsiGetCapsIssInternal
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
        if(mipi_lanes == SUPPORT_MIPI_ONE_LANE){
        	switch (pIsiSensorCaps->Index) 
            { 
        				default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }
            }
        }else if(mipi_lanes == SUPPORT_MIPI_TWO_LANE){
        		switch (pIsiSensorCaps->Index) 
            { 
        				default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }
            }
        }else if (mipi_lanes == SUPPORT_MIPI_FOUR_LANE) {            
            switch (pIsiSensorCaps->Index) 
            { 
            #if 1
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P30;
                    break;
                }
                case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P25;
                    break;
                }
                case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P20;
                    break;
                }
                case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P15;
                    break;
                }
                case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P10;
                    break;
                }
                case 5:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_3264_2448P7;
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
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING; //?
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
 
static RESULT GC8034_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
    
    result = GC8034_IsiGetCapsIssInternal(pIsiSensorCaps,pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes );
    
    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t GC8034_g_IsiSensorDefaultConfig =
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
    ISI_RES_3264_2448P15, 
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
 *          GC8034_SetupOutputFormat
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      GC8034 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
RESULT GC8034_SetupOutputFormat
(
    GC8034_Context_t       *pGC8034Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s%s (enter)\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* only Bayer mode is supported by GC8034 sensor, so the YCSequence parameter is not checked */
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
            TRACE( GC8034_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_RGRGGBGB supported, no configuration needed */
    {
        case ISI_BPAT_RGRGGBGB:
        {
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( GC8034_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( GC8034_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( GC8034_INFO, "%s%s (exit)\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

#if 0
int GC8034_get_PCLK( GC8034_Context_t *pGC8034Ctx, int XVCLK)
{
    // calculate sysclk
    
    int prediv, mipi_preDiv, mipi_preMul, mipi_Mul, mipi_Div, sys_preDiv, sys_Mul;
    uint32_t temp1, temp2;
    uint64_t sysclk;
    sysclk = XVCLK;
    GC8034_IsiRegReadIss(pGC8034Ctx, 0xf7, &temp1 );
    if(temp1 & 0x02)
    {
        sysclk/=2;

    }
    GC8034_IsiRegReadIss(pGC8034Ctx, 0xf5, &temp1 );
    temp1 &= 0x18;
    temp1 >>= 3;
    if(temp1 >=2)
    {
        GC8034_IsiRegReadIss(pGC8034Ctx, 0xf2, &temp2);
        sysclk /= (temp2 & 0x07) + 1;
        GC8034_IsiRegReadIss(pGC8034Ctx, 0xf4, &temp2);
        if(temp2 & 0x10)
        {
            sysclk *=2;
        }
        GC8034_IsiRegReadIss(pGC8034Ctx, 0xfa, &temp2);
        temp2++;
        temp2 *=2;
        sysclk *= temp2;

    }
    else
    {
        GC8034_IsiRegReadIss(pGC8034Ctx, 0xf6, &temp2);
        sysclk /= (temp2 & 0x07) + 1;
        GC8034_IsiRegReadIss(pGC8034Ctx, 0xf8, &temp2);
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
            GC8034_IsiRegReadIss(pGC8034Ctx, 0xf4, &temp2);
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
            GC8034_IsiRegReadIss(pGC8034Ctx, 0xf4, &temp2);
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
            TRACE( GC8034_ERROR, "%s: failed to get_PCLK \n", __FUNCTION__ );
            break;
    }
    GC8034_IsiRegReadIss(pGC8034Ctx, 0xfc, &temp2);
    if(temp2 & 0x10)
    {
        sysclk /=2;
    }
    return (int)sysclk; 
   
}
#endif
/*****************************************************************************/
/**
 *          GC8034_SetupOutputWindow
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      GC8034 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * hkw fix
 *****************************************************************************/

static RESULT GC8034_SetupOutputWindowInternal
(
    GC8034_Context_t        *pGC8034Ctx,
    const IsiSensorConfig_t *pConfig,
    bool_t set2Sensor,
    bool_t res_no_chg
)
{
    RESULT result     = RET_SUCCESS;
    uint16_t usFrameLengthLines = 0;
    uint16_t usLineLengthPck    = 0;
	uint16_t usTimeHts = 0;
	uint16_t usTimeVts = 0;
    float    rVtPixClkFreq      = 0.0f;
   // int xclk = 24000000;
    
	TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);
	
	if(pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE) {
    	pGC8034Ctx->IsiSensorMipiInfo.ulMipiFreq = 672;

        switch ( pConfig->Resolution )
        {
            case ISI_RES_3264_2448P7:
            case ISI_RES_3264_2448P10:
            case ISI_RES_3264_2448P15:
            case ISI_RES_3264_2448P20:
            case ISI_RES_3264_2448P25:
            case ISI_RES_3264_2448P30:
            {
                if (set2Sensor == BOOL_TRUE) {
                    if (res_no_chg == BOOL_FALSE) {
											result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448_fourlane);
        		   		  }
                    if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P30_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P25_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P20_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P15_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P10_fourlane_fpschg);
                    } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                        result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_3264x2448P7_fourlane_fpschg);
                    }   
        		}
        		
    			usTimeHts = 1068;  //1068	
                if (pConfig->Resolution == ISI_RES_3264_2448P30) {                        
                    usTimeVts = 0x9C0;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P25) {                        
                    usTimeVts = 0xbb4;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P20) {                        
                    usTimeVts = 0xea1;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P15) {                        
                    usTimeVts = 0x1381;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P10) {                        
                    usTimeVts = 0x1d42;
                } else if (pConfig->Resolution == ISI_RES_3264_2448P7) {
                    usTimeVts = 0x29CC;
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
	rVtPixClkFreq = 80000000;
    
    // store frame timing for later use in AEC module
    pGC8034Ctx->VtPixClkFreq     = rVtPixClkFreq;
    pGC8034Ctx->LineLengthPck    = usLineLengthPck;
    pGC8034Ctx->FrameLengthLines = usFrameLengthLines;
	pGC8034Ctx->AecMaxIntegrationTime = ( ((float)pGC8034Ctx->FrameLengthLines) * ((float)pGC8034Ctx->LineLengthPck) ) / pGC8034Ctx->VtPixClkFreq;
    TRACE( GC8034_INFO, "%s  (exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);
    
    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_SetupImageControl
 *
 * @brief   Sets the image control functions (BLC, AGC, AWB, AEC, DPCC ...)
 *
 * @param   handle      GC8034 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * @don't fix hkw
 *****************************************************************************/
RESULT GC8034_SetupImageControl
(
    GC8034_Context_t        *pGC8034Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

   // uint32_t RegValue = 0U;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = GC8034_IsiRegReadIss(  pGC8034Ctx, GC8034_BLC_CTRL00, &RegValue );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = GC8034_IsiRegReadIss(  pGC8034Ctx, GC8034_BLC_CTRL00, &RegValue );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = GC8034_IsiRegReadIss(  pGC8034Ctx, GC8034_AEC_MANUAL, &RegValue );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = GC8034_IsiRegReadIss(  pGC8034Ctx, GC8034_ISP_CTRL01, &RegValue );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = GC8034_IsiRegReadIss(  pGC8034Ctx, GC8034_AEC_MANUAL, &RegValue );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            // disable white and black pixel cancellation (clear bit 6 and 7)
            //result = GC8034_IsiRegReadIss( pGC8034Ctx, GC8034_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_ISP_CTRL00, (RegValue &0x7c) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        case ISI_DPCC_AUTO:
        {
            // enable white and black pixel cancellation (set bit 6 and 7)
            //result = GC8034_IsiRegReadIss( pGC8034Ctx, GC8034_ISP_CTRL00, &RegValue );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            //result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_ISP_CTRL00, (RegValue | 0x83) );
            //RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
            break;
        }

        default:
        {
            TRACE( GC8034_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT GC8034_SetupOutputWindow
(
    GC8034_Context_t        *pGC8034Ctx,
    const IsiSensorConfig_t *pConfig    
)
{
    bool_t res_no_chg;

    if ((ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pGC8034Ctx->Config.Resolution)) && 
        (ISI_RES_W_GET(pConfig->Resolution)==ISI_RES_W_GET(pGC8034Ctx->Config.Resolution))) {
        res_no_chg = BOOL_TRUE;
        
    } else {
        res_no_chg = BOOL_FALSE;
    }

    return GC8034_SetupOutputWindowInternal(pGC8034Ctx,pConfig,BOOL_TRUE, BOOL_FALSE);
}

/*****************************************************************************/
/**
 *          GC8034_AecSetModeParameters
 *
 * @brief   This function fills in the correct parameters in GC8034-Instances
 *          according to AEC mode selection in IsiSensorConfig_t.
 *
 * @note    It is assumed that IsiSetupOutputWindow has been called before
 *          to fill in correct values in instance structure.
 *
 * @param   handle      GC8034 context
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_AecSetModeParameters
(
    GC8034_Context_t       *pGC8034Ctx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"",
        pGC8034Ctx->Config.Resolution, pConfig->Resolution);

    if ( (pGC8034Ctx->VtPixClkFreq == 0.0f) )
    {
        TRACE( GC8034_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }
    pGC8034Ctx->AecMaxIntegrationTime = ( ((float)(pGC8034Ctx->FrameLengthLines - 4)) * ((float)pGC8034Ctx->LineLengthPck) ) / pGC8034Ctx->VtPixClkFreq;
    pGC8034Ctx->AecMinIntegrationTime = 0.0001f;

    TRACE( GC8034_DEBUG, "%s%s: AecMaxIntegrationTime = %f \n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"", pGC8034Ctx->AecMaxIntegrationTime  );

    pGC8034Ctx->AecMaxGain = GC8034_MAX_GAIN_AEC;
    pGC8034Ctx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pGC8034Ctx->AecIntegrationTimeIncrement = ((float)pGC8034Ctx->LineLengthPck) / pGC8034Ctx->VtPixClkFreq;
    pGC8034Ctx->AecGainIncrement = GC8034_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pGC8034Ctx->AecCurGain               = pGC8034Ctx->AecMinGain;
    pGC8034Ctx->AecCurIntegrationTime    = 0.0f;
    pGC8034Ctx->OldCoarseIntegrationTime = 0;
    pGC8034Ctx->OldFineIntegrationTime   = 0;
    //pGC8034Ctx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( GC8034_INFO, "%s%s (exit)\n", __FUNCTION__, pGC8034Ctx->isAfpsRun?"(AFPS)":"");

    return ( result );
}

/*****************************************************************************/
/**
 *          GC8034_IsiSetupSensorIss
 *
 * @brief   Setup of the image sensor considering the given configuration.
 *
 * @param   handle      GC8034 sensor instance handle
 * @param   pConfig     pointer to sensor configuration structure
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *
 *****************************************************************************/
static RESULT GC8034_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pGC8034Ctx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pGC8034Ctx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image GC8034 (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */
    result = GC8034_IsiRegWriteIss ( pGC8034Ctx, GC8034_SOFTWARE_RST, GC8034_SOFTWARE_RST_VALUE );//ºê¶¨Òå hkw£»
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );

    TRACE( GC8034_DEBUG, "%s: GC8034 System-Reset executed\n", __FUNCTION__);
    // disable streaming during sensor setup
    
    result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_MODE_SELECT, GC8034_MODE_SELECT_OFF );//GC8034_MODE_SELECT,stream off; hkw
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: Can't write GC8034 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
    
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
  if(pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_FOUR_LANE){
	       result = IsiRegDefaultsApply( pGC8034Ctx, GC8034_g_aRegDescription_fourlane);
     }
	
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that GC8034 can take over new default values */
    osSleep( 10 );


    /* 3.) verify default values to make sure everything has been written correctly as expected */
	#if 0
	result = IsiRegDefaultsVerify( pGC8034Ctx, GC8034_g_aRegDescription );
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }
	#endif
	
    #if 0
    // output of pclk for measurement (only debugging)
    result = GC8034_IsiRegWriteIss( pGC8034Ctx, 0x3009U, 0x10U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    #endif

    /* 4.) setup output format (RAW10|RAW12) */
    result = GC8034_SetupOutputFormat( pGC8034Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = GC8034_SetupOutputWindow( pGC8034Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = GC8034_SetupImageControl( pGC8034Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = GC8034_AecSetModeParameters( pGC8034Ctx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pGC8034Ctx->Configured = BOOL_TRUE;
    }

    //set OTP info

    result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_MODE_SELECT, GC8034_MODE_SELECT_ON );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: Can't write GC8034 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
	
    //set OTP
    if(bOTP_switch){
        apply_otp(pGC8034Ctx);
    }
 
    result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_MODE_SELECT, GC8034_MODE_SELECT_OFF );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: Can't write GC8034 Image System Register (disable streaming failed)\n", __FUNCTION__ );
        return ( result );
    }
   

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiChangeSensorResolutionIss
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
 *****************************************************************************/
static RESULT GC8034_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if (pNumberOfFramesToSkip == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    if ( (pGC8034Ctx->Configured != BOOL_TRUE) )
    {
        return RET_WRONG_STATE;
    }

    IsiSensorCaps_t Caps;
    
    Caps.Index = 0;
    Caps.Resolution = 0;
    while (GC8034_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

    if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }

    if ( Resolution == pGC8034Ctx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
    }
    else
    {
        // change resolution
        char *szResName = NULL;
        bool_t res_no_chg;

        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pGC8034Ctx->Config.Resolution)) && 
            (ISI_RES_H_GET(Resolution)==ISI_RES_H_GET(pGC8034Ctx->Config.Resolution))) ) {

            if (pGC8034Ctx->Streaming != BOOL_FALSE) {
                TRACE( GC8034_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
        
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( GC8034_DEBUG, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

        // update resolution in copy of config in context        
        pGC8034Ctx->Config.Resolution = Resolution;

        // tell sensor about that
        result = GC8034_SetupOutputWindowInternal( pGC8034Ctx, &pGC8034Ctx->Config, BOOL_TRUE, res_no_chg);
        if ( result != RET_SUCCESS )
        {
            TRACE( GC8034_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pGC8034Ctx->AecCurGain;
        float OldIntegrationTime = pGC8034Ctx->AecCurIntegrationTime;

	//if(pGC8034Ctx->AecCurIntegrationTime != pGC8034Ctx->afpsIntegrationTime )
	//OldIntegrationTime = pGC8034Ctx->afpsIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = GC8034_AecSetModeParameters( pGC8034Ctx, &pGC8034Ctx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( GC8034_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip;
        float   DummySetGain;
        float   DummySetIntegrationTime;
        result = GC8034_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( GC8034_ERROR, "%s: GC8034_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        if (res_no_chg == BOOL_TRUE)
            *pNumberOfFramesToSkip = 0;
        else 
            *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        
    }

    TRACE( GC8034_INFO, "%s (exit)  result: 0x%x   pNumberOfFramesToSkip: %d \n", __FUNCTION__, result,
        *pNumberOfFramesToSkip);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC8034_IsiSensorSetStreamingIss
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
static RESULT  GC8034_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;

    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pGC8034Ctx->Configured != BOOL_TRUE) || (pGC8034Ctx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (on == BOOL_TRUE)
    {
        /* enable streaming */
        result = GC8034_IsiRegReadIss ( pGC8034Ctx, GC8034_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = GC8034_IsiRegWriteIss ( pGC8034Ctx, GC8034_MODE_SELECT, (RegValue | GC8034_MODE_SELECT_ON) );//GC8034_MODE_SELECT,stream on; hkw
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable streaming */
        result = GC8034_IsiRegReadIss ( pGC8034Ctx, GC8034_MODE_SELECT, &RegValue);
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
        result = GC8034_IsiRegWriteIss ( pGC8034Ctx, GC8034_MODE_SELECT, (RegValue & ~GC8034_MODE_SELECT_ON) );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

    if (result == RET_SUCCESS)
    {
        pGC8034Ctx->Streaming = on;
    }

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiSensorSetPowerIss
 *
 * @brief   Performs the power-up/power-down sequence of the camera, if possible.
 *
 * @param   handle      GC8034 sensor instance handle
 * @param   on          new power state (BOOL_TRUE=on, BOOL_FALSE=off)
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pGC8034Ctx->Configured = BOOL_FALSE;
    pGC8034Ctx->Streaming  = BOOL_FALSE;

    TRACE( GC8034_DEBUG, "%s power off \n", __FUNCTION__);
    result = HalSetPower( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( GC8034_DEBUG, "%s reset on\n", __FUNCTION__);
    result = HalSetReset( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    { //power on seq; hkw
        TRACE( GC8034_DEBUG, "%s power on \n", __FUNCTION__);
        result = HalSetPower( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( GC8034_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( GC8034_DEBUG, "%s reset on \n", __FUNCTION__);
        result = HalSetReset( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        TRACE( GC8034_DEBUG, "%s reset off \n", __FUNCTION__);
        result = HalSetReset( pGC8034Ctx->IsiCtx.HalHandle, pGC8034Ctx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiCheckSensorConnectionIss
 *
 * @brief   Checks the I2C-Connection to sensor by reading sensor revision id.
 *
 * @param   handle      GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = GC8034_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId<<8U) | GC8034_CHIP_ID_LOW_BYTE_DEFAULT;

    result = GC8034_IsiGetSensorRevisionIss( handle, &value );

    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( GC8034_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }

    TRACE( GC8034_DEBUG, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetSensorRevisionIss
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
static RESULT GC8034_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = GC8034_IsiRegReadIss ( handle, GC8034_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = GC8034_IsiRegReadIss ( handle, GC8034_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF));

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiRegReadIss
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
static RESULT GC8034_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

  //  TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, GC8034_g_aRegDescription_fourlane );
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;

        IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;        
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
        /*
        TRACE( GC8034_DEBUG, "[SYNNEX DEBUG] GC8034_IsiRegReadIss:I2C_Bus%d SlaveAddress0x%x, addr 0x%x read result:0x%x ,Return:%d\n",SYNNEX_TEST->I2cBusNum,SYNNEX_TEST->SlaveAddress,address,*p_value,result);
        TRACE( GC8034_DEBUG, "[SYNNEX DEBUG] GC8034_IsiRegReadIss:NrOfAddressBytes:%d NrOfBytes:%d\n",SYNNEX_TEST->NrOfAddressBytes, NrOfBytes);
        */
    }

  //  TRACE( GC8034_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, *p_value);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiRegWriteIss
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
 *****************************************************************************/
static RESULT GC8034_IsiRegWriteIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    const uint32_t      value
)
{
    RESULT result = RET_SUCCESS;

    uint8_t NrOfBytes;

  //  TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    NrOfBytes = IsiGetNrDatBytesIss( address, GC8034_g_aRegDescription_fourlane );
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

//    TRACE( GC8034_INFO, "%s (exit: 0x%08x 0x%08x)\n", __FUNCTION__, address, value);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetGainLimitsIss
 *
 * @brief   Returns the exposure minimal and maximal values of an
 *          GC8034 instance
 *
 * @param   handle       GC8034 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( GC8034_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pGC8034Ctx->AecMinGain;
    *pMaxGain = pGC8034Ctx->AecMaxGain;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetIntegrationTimeLimitsIss
 *
 * @brief   Returns the minimal and maximal integration time values of an
 *          GC8034 instance
 *
 * @param   handle       GC8034 sensor instance handle
 * @param   pMinExposure Pointer to a variable receiving minimal exposure value
 * @param   pMaxExposure Pointer to a variable receiving maximal exposure value
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( GC8034_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pGC8034Ctx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pGC8034Ctx->AecMaxIntegrationTime;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC8034_IsiGetGainIss
 *
 * @brief   Reads gain values from the image sensor module.
 *
 * @param   handle                  GC8034 sensor instance handle
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 * ²»ÓÃ¸Ä£»»ñµÃGAINÖµ
 *****************************************************************************/
RESULT GC8034_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
	uint32_t data= 0;
	uint32_t result_gain= 0;
	
	GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }
#if 0
	result = GC8034_IsiRegReadIss ( pGC8034Ctx, GC8034_AEC_AGC_ADJ_H, &data);
	TRACE( GC8034_INFO, " -------reg3508:%x-------\n",data );
	result_gain = (data & 0x07) ;
	result = GC8034_IsiRegReadIss ( pGC8034Ctx, GC8034_AEC_AGC_ADJ_L, &data);
	TRACE( GC8034_INFO, " -------reg3509:%x-------\n",data );
	result_gain = (result_gain<<8) + data;
	*pSetGain = ( (float)result_gain ) / GC8034_MAXN_GAIN;
#else	
    *pSetGain = pGC8034Ctx->AecCurGain;
#endif    

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetGainIncrementIss
 *
 * @brief   Get smallest possible gain increment.
 *
 * @param   handle                  GC8034 sensor instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
RESULT GC8034_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pGC8034Ctx->AecGainIncrement;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiSetGainIss
 *
 * @brief   Writes gain values to the image sensor module.
 *          Updates current gain and exposure in sensor struct/state.
 *
 * @param   handle                  GC8034 sensor instance handle
 * @param   NewGain                 gain to be set
 * @param   pSetGain                set gain
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_INVALID_PARM
 * @retval  RET_FAILURE
 *****************************************************************************/
 #if 0
 #define ANALOG_GAIN_1 64   // 1.000x
 #define ANALOG_GAIN_2 88   // 1.38x
 #define ANALOG_GAIN_3 125  // 1.95x
 #define ANALOG_GAIN_4 173  // 2.70x
 #define ANALOG_GAIN_5 243  // 3.80x
 #define ANALOG_GAIN_6 345  // 5.40x
 #define ANALOG_GAIN_7 490  // 7.66x
 #define ANALOG_GAIN_8 684  // 10.69x
 #define ANALOG_GAIN_9 962  // 15.03x
 #endif
 #define MAX_AG_INDEX           9
 #define AGC_REG_NUM            14
 #define MEAG_INDEX             7
RESULT GC8034_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
    uint16_t iReg = 0,temp = 0;
    uint16_t percentGain = 0;
    uint16_t gain_index = 0;
    uint16_t temp_gain = 0;
    uint8_t  BorF = 1;//full size
	

    TRACE( GC8034_INFO, "%s: (enter) pGC8034Ctx->AecMaxGain(%f) \n", __FUNCTION__,pGC8034Ctx->AecMaxGain);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
    
        TRACE( GC8034_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pGC8034Ctx->AecMinGain ) NewGain = pGC8034Ctx->AecMinGain;
    if( NewGain > pGC8034Ctx->AecMaxGain ) NewGain = pGC8034Ctx->AecMaxGain;
    usGain = (uint16_t)(NewGain * 64+0.5); 
    iReg = usGain;
    uint16_t gain_level[MAX_AG_INDEX] = {
		0x0040, /* 1.000*/
		0x0058, /* 1.375*/
		0x007d, /* 1.950*/
		0x00ad, /* 2.700*/
		0x00f3, /* 3.800*/
		0x0159, /* 5.400*/
		0x01ea, /* 7.660*/
		0x02ac, /*10.688*/
		0x03c2, /*15.030*/
	};
	uint8_t agc_register[2 * MAX_AG_INDEX][AGC_REG_NUM] = {
	  /*{ 0xfe, 0x20, 0x33, 0xfe, 0xdf, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xfe },*/
		/* binning */
		{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
		{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
		{ 0x00, 0x4e, 0x84, 0x01, 0x0c, 0x2e, 0x2d, 0x15, 0x19, 0x47, 0x70, 0x9f, 0xd8, 0x00 },
		{ 0x00, 0x51, 0x80, 0x01, 0x07, 0x28, 0x32, 0x22, 0x20, 0x49, 0x70, 0x91, 0xd9, 0x00 },
		{ 0x00, 0x4d, 0x83, 0x01, 0x0f, 0x3b, 0x3b, 0x1c, 0x1f, 0x47, 0x6f, 0x9b, 0xd3, 0x00 },
		{ 0x00, 0x50, 0x83, 0x01, 0x08, 0x35, 0x46, 0x1e, 0x22, 0x4c, 0x70, 0x9a, 0xd2, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
		/* fullsize */
		{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
		{ 0x00, 0x55, 0x83, 0x01, 0x06, 0x18, 0x20, 0x16, 0x17, 0x50, 0x6c, 0x9b, 0xd8, 0x00 },
		{ 0x00, 0x4e, 0x84, 0x01, 0x0c, 0x2e, 0x2d, 0x15, 0x19, 0x47, 0x70, 0x9f, 0xd8, 0x00 },
		{ 0x00, 0x51, 0x80, 0x01, 0x07, 0x28, 0x32, 0x22, 0x20, 0x49, 0x70, 0x91, 0xd9, 0x00 },
		{ 0x00, 0x4d, 0x83, 0x01, 0x0f, 0x3b, 0x3b, 0x1c, 0x1f, 0x47, 0x6f, 0x9b, 0xd3, 0x00 },
		{ 0x00, 0x50, 0x83, 0x01, 0x08, 0x35, 0x46, 0x1e, 0x22, 0x4c, 0x70, 0x9a, 0xd2, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 },
		{ 0x00, 0x52, 0x80, 0x01, 0x0c, 0x35, 0x3a, 0x2b, 0x2d, 0x4c, 0x67, 0x8d, 0xc0, 0x00 }
	};

	for (gain_index = MEAG_INDEX - 1; gain_index >= 0; gain_index--)
		if (iReg >= gain_level[gain_index]) {
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6, gain_index);
			temp_gain = 256 * iReg / gain_level[gain_index];
			temp_gain = temp_gain * Dgain_ratio / 256;
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp_gain >> 8);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp_gain & 0xff);

			GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][0]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, agc_register[BorF * MAX_AG_INDEX + gain_index][1]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, agc_register[BorF * MAX_AG_INDEX + gain_index][2]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][3]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, agc_register[BorF * MAX_AG_INDEX + gain_index][4]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, agc_register[BorF * MAX_AG_INDEX + gain_index][5]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, agc_register[BorF * MAX_AG_INDEX + gain_index][6]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, agc_register[BorF * MAX_AG_INDEX + gain_index][7]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, agc_register[BorF * MAX_AG_INDEX + gain_index][8]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, agc_register[BorF * MAX_AG_INDEX + gain_index][9]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, agc_register[BorF * MAX_AG_INDEX + gain_index][10]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, agc_register[BorF * MAX_AG_INDEX + gain_index][11]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, agc_register[BorF * MAX_AG_INDEX + gain_index][12]);
			GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, agc_register[BorF * MAX_AG_INDEX + gain_index][13]);
			break;
		}
		
	#if 0
  	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x55);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x83);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
    		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x06);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x18);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x20);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x16);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x17);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x50);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x6c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x9b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x00);// 
		temp = iReg*256/ANALOG_GAIN_1;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);
	}
	else if((ANALOG_GAIN_2<= iReg)&&(iReg < ANALOG_GAIN_3))
	{                                    
   		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x55);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x83);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x06);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x18);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x20);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x16);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x17);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x50);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x6c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x9b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x01);// 
		temp = iReg*256/ANALOG_GAIN_2;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);
		
		}
		else if((ANALOG_GAIN_3<= iReg)&&(iReg < ANALOG_GAIN_4))
	{
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x4e);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x84);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
    		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x0c);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x2e);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x2d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x15);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x19);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x47);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x70);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x9f);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x02);// 
		temp = iReg*256/ANALOG_GAIN_3;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);		
		}
		else if((ANALOG_GAIN_4<= iReg)&&(iReg < ANALOG_GAIN_5))
	{
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x51);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x80);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x07);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x28);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x32);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x22);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x20);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x49);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x70);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x91);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd9);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x03);// 
		temp = iReg*256/ANALOG_GAIN_4;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);
		
		}
		else if((ANALOG_GAIN_5<= iReg)&&(iReg < ANALOG_GAIN_6))
	{
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x4d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x83);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x0f);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x3b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x3b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x1c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x1f);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x47);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x6f);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x9b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd3);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x04);// 
		temp = iReg*256/ANALOG_GAIN_5;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);
		
		}
		else if((ANALOG_GAIN_6<= iReg)&&(iReg < ANALOG_GAIN_7))
	{
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x50);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x83);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   		GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x08);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x35);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x46);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x1e);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x22);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x4c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x70);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x9a);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xd2);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x05);// 
		temp = iReg*256/ANALOG_GAIN_6;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);		
		}
			else if((ANALOG_GAIN_7<= iReg)&&(iReg < ANALOG_GAIN_8))
	 {
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x52);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x80);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   	       GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x0c);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x35);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x3a);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x2b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x2d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x4c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x67);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x8d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xc0);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x06);// 
		temp = iReg*256/ANALOG_GAIN_7;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);		
		}
		else if((ANALOG_GAIN_8<= iReg)&&(iReg < ANALOG_GAIN_9))
	 {
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x52);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x80);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   	       GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x0c);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x35);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x3a);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x2b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x2d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x4c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x67);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x8d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xc0);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x07);// 
		temp = iReg*256/ANALOG_GAIN_8;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);		
		}
		else if((ANALOG_GAIN_9<= iReg))
      {
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x20, 0x52);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0x33, 0x80);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x01);
   	       GC8034_IsiRegWriteIss(pGC8034Ctx,0xdf, 0x0c);          
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe7, 0x35);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe8, 0x3a);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xe9, 0x2b);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xea, 0x2d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xeb, 0x4c);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xec, 0x67);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xed, 0x8d);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xee, 0xc0);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xfe, 0x00);	
		//analog gain
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb6,  0x08);// 
		temp = iReg*256/ANALOG_GAIN_9;
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb1, temp>>8);
		GC8034_IsiRegWriteIss(pGC8034Ctx,0xb2, temp&0xff);		
		}
	#endif
	 pGC8034Ctx->AecCurGain = ( (float)usGain ) / 64.0f;
    //return current state
    *pSetGain = pGC8034Ctx->AecCurGain;

    TRACE( GC8034_ERROR, "%s: setgain mubiao(%f) shiji(%f)\n", __FUNCTION__, NewGain, *pSetGain);
	
    return ( result );
}


/*****************************************************************************/
/**
 *          GC8034_IsiGetIntegrationTimeIss
 *
 * @brief   Reads integration time values from the image sensor module.
 *
 * @param   handle                  GC8034 sensor instance handle
 * @param   pSetIntegrationTime     set integration time
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
RESULT GC8034_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pGC8034Ctx->AecCurIntegrationTime;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetIntegrationTimeIncrementIss
 *
 * @brief   Get smallest possible integration time increment.
 *
 * @param   handle                  GC8034 GC8034 instance handle
 * @param   pIncr                   increment
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
RESULT GC8034_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the GC8034/driver can handle (e.g. used for sliders in the application)
    *pIncr = pGC8034Ctx->AecIntegrationTimeIncrement;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiSetIntegrationTimeIss
 *
 * @brief   Writes gain and integration time values to the image sensor module.
 *          Updates current integration time and exposure in sensor
 *          struct/state.
 *
 * @param   handle                  GC8034 sensor instance handle
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
 * @retval  RET_DIVISION_BY_ZEROs
 *****************************************************************************/
RESULT GC8034_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint32_t CoarseIntegrationTime = 0;
	uint32_t result_intertime= 0;
	uint32_t cal_shutter = 0;
    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods

    TRACE( GC8034_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pGC8034Ctx->AecMinIntegrationTime,
        pGC8034Ctx->AecMaxIntegrationTime);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( GC8034_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( NewIntegrationTime > pGC8034Ctx->AecMaxIntegrationTime ) NewIntegrationTime = pGC8034Ctx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pGC8034Ctx->AecMinIntegrationTime ) NewIntegrationTime = pGC8034Ctx->AecMinIntegrationTime;
	
    ShutterWidthPck = NewIntegrationTime * ( (float)pGC8034Ctx->VtPixClkFreq );

    // avoid division by zero
    if ( pGC8034Ctx->LineLengthPck == 0 )
    {
        TRACE( GC8034_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pGC8034Ctx->LineLengthPck) + 0.5f );
    cal_shutter = CoarseIntegrationTime >> 1;
    cal_shutter = cal_shutter << 1;
    Dgain_ratio = 256 * CoarseIntegrationTime / cal_shutter;
    TRACE( GC8034_ERROR, "%s: cal_shutter = %d\n", __FUNCTION__,cal_shutter);
    if( cal_shutter != pGC8034Ctx->OldCoarseIntegrationTime)
    {
	GC8034_IsiRegWriteIss( pGC8034Ctx, 0xfe, 0x00 );
	
	GC8034_IsiRegWriteIss( pGC8034Ctx, 0x03,(cal_shutter>>8) & 0x7F );
		
	GC8034_IsiRegWriteIss( pGC8034Ctx, 0x04, cal_shutter & 0xFF);
	
	
		pGC8034Ctx->OldCoarseIntegrationTime = cal_shutter;	// remember current integration time
		*pNumberOfFramesToSkip = 1U; //skip 1 frame
	}
	else
	{
		*pNumberOfFramesToSkip = 0U; //no frame skip
	}

       pGC8034Ctx->AecCurIntegrationTime = ((float)cal_shutter) * ((float)pGC8034Ctx->LineLengthPck) / pGC8034Ctx->VtPixClkFreq;
	
	*pSetIntegrationTime = pGC8034Ctx->AecCurIntegrationTime;
	
       TRACE( GC8034_DEBUG, "%s: vtPixClkFreq:%f, LineLengthPck:%x, SetTi=%f, NewTi=%f, CoarseIntegrationTime=%x\n", __FUNCTION__, 
         pGC8034Ctx->VtPixClkFreq,pGC8034Ctx->LineLengthPck,*pSetIntegrationTime,NewIntegrationTime,CoarseIntegrationTime);
	   
       TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);
       TRACE( GC8034_ERROR, "%s: settime mubiao:%f, shiji=%f\n", __FUNCTION__,NewIntegrationTime,*pSetIntegrationTime);
    return ( result );
}




/*****************************************************************************/
/**
 *          GC8034_IsiExposureControlIss
 *
 * @brief   Camera hardware dependent part of the exposure control loop.
 *          Calculates appropriate register settings from the new exposure
 *          values and writes them to the image sensor module.
 *
 * @param   handle                  GC8034 sensor instance handle
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
 * @retval  RET_DIVISION_BY_ZERO�?*****************************************************************************/
RESULT GC8034_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( GC8034_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( GC8034_DEBUG, "%s: oyyf333 g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = GC8034_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = GC8034_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( GC8034_DEBUG, "%s: oyyf333 set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCurrentExposureIss
 *
 * @brief   Returns the currently adjusted AE values
 *
 * @param   handle                  GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
RESULT GC8034_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pGC8034Ctx->AecCurGain;
    *pSetIntegrationTime = pGC8034Ctx->AecCurIntegrationTime;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetResolutionIss
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
 *****************************************************************************/
RESULT GC8034_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pGC8034Ctx->Config.Resolution;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetAfpsInfoHelperIss
 *
 * @brief   Calc AFPS sub resolution settings for the given resolution
 *
 * @param   pGC8034Ctx             GC8034 sensor instance (dummy!) context
 * @param   Resolution              Any supported resolution to query AFPS params for
 * @param   pAfpsInfo               Reference of AFPS info structure to write the results to
 * @param   AfpsStageIdx            Index of current AFPS stage to use
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 *****************************************************************************/
static RESULT GC8034_IsiGetAfpsInfoHelperIss(
    GC8034_Context_t   *pGC8034Ctx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    DCT_ASSERT(pGC8034Ctx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pGC8034Ctx->Config.Resolution = Resolution;

    // tell sensor about that
    result = GC8034_SetupOutputWindowInternal( pGC8034Ctx, &pGC8034Ctx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = GC8034_AecSetModeParameters( pGC8034Ctx, &pGC8034Ctx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( GC8034_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pGC8034Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pGC8034Ctx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pGC8034Ctx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pGC8034Ctx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pGC8034Ctx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;
    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC8034_IsiGetAfpsInfoIss
 *
 * @brief   Returns the possible AFPS sub resolution settings for the given resolution series
 *
 * @param   handle                  GC8034 sensor instance handle
 * @param   Resolution              Any resolution within the AFPS group to query;
 *                                  0 (zero) to use the currently configured resolution
 * @param   pAfpsInfo               Reference of AFPS info structure to store the results
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 * @retval  RET_NOTSUPP
 *****************************************************************************/
RESULT GC8034_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        TRACE( GC8034_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pAfpsInfo == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    // use currently configured resolution?
    if (Resolution == 0)
    {
        Resolution = pGC8034Ctx->Config.Resolution;
    }

    // prepare index
    uint32_t idx = 0;

    // set current resolution data in info struct
    pAfpsInfo->CurrResolution = pGC8034Ctx->Config.Resolution;
    pAfpsInfo->CurrMinIntTime = pGC8034Ctx->AecMinIntegrationTime;
    pAfpsInfo->CurrMaxIntTime = pGC8034Ctx->AecMaxIntegrationTime;

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    GC8034_Context_t *pDummyCtx = (GC8034_Context_t*) malloc( sizeof(GC8034_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( GC8034_ERROR,  "%s: Can't allocate dummy GC8034 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pGC8034Ctx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = GC8034_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx); \
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
    switch (pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes)
    {
        case SUPPORT_MIPI_FOUR_LANE:
        {
            switch(Resolution)
            {
                default:
                    TRACE( GC8034_DEBUG,  "%s: Resolution %08x not supported by AFPS\n",  __FUNCTION__, Resolution );
                    result = RET_NOTSUPP;
                    break;
                case ISI_RES_3264_2448P30:
                case ISI_RES_3264_2448P25:
                case ISI_RES_3264_2448P20:
                case ISI_RES_3264_2448P15:
                case ISI_RES_3264_2448P10:
                case ISI_RES_3264_2448P7:
                    	AFPSCHECKANDADD( ISI_RES_3264_2448P30 );
                    	AFPSCHECKANDADD( ISI_RES_3264_2448P25 );
                    	AFPSCHECKANDADD( ISI_RES_3264_2448P20 );
                    	AFPSCHECKANDADD( ISI_RES_3264_2448P15 );
                    	AFPSCHECKANDADD( ISI_RES_3264_2448P10 );
                    	//AFPSCHECKANDADD( ISI_RES_3264_2448P7 );
                    break;

                // check next series here...
            }
        

            break;
        }

        default:
            TRACE( GC8034_ERROR,  "%s: pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;

    }

    // release dummy context again
    free(pDummyCtx);

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibKFactor
 *
 * @brief   Returns the GC8034 specific K-Factor
 *
 * @param   handle       GC8034 sensor instance handle
 * @param   pIsiKFactor  Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
	return ( RET_SUCCESS );
	GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&GC8034_KFactor;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibPcaMatrix
 *
 * @brief   Returns the GC8034 specific PCA-Matrix
 *
 * @param   handle          GC8034 sensor instance handle
 * @param   pIsiPcaMatrix   Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
	return ( RET_SUCCESS );
	GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&GC8034_PCAMatrix;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns the sensor specific SvdMean-Vector
 *
 * @param   handle              GC8034 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&GC8034_SVDMeanValue;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibSvdMeanValue
 *
 * @brief   Returns a pointer to the sensor specific centerline, a straight
 *          line in Hesse normal form in Rg/Bg colorspace
 *
 * @param   handle              GC8034 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiCenterLine = (IsiLine_t*)&GC8034_CenterLine;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibClipParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for Rg/Bg color
 *          space clipping
 *
 * @param   handle              GC8034 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&GC8034_AwbClipParm;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibGlobalFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for AWB out of
 *          range handling
 *
 * @param   handle              GC8034 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&GC8034_AwbGlobalFadeParm;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetCalibFadeParam
 *
 * @brief   Returns a pointer to the sensor specific arrays for near white
 *          pixel parameter calculations
 *
 * @param   handle              GC8034 sensor instance handle
 * @param   pIsiSvdMeanValue    Pointer to Pointer receiving the memory address
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

   // *ptIsiFadeParam = (IsiAwbFade2Parm_t *)&GC8034_AwbFade2Parm;

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          GC8034_IsiGetIlluProfile
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
 *****************************************************************************/
static RESULT GC8034_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	#if 0
    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
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
        for ( i=0U; i<GC8034_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( GC8034_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &GC8034_IlluProfileDefault[i];
                break;
            }
        }

       // result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetLscMatrixTable
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
static RESULT GC8034_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );
	
	#if 0
    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
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
                if ( ( pGC8034Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pGC8034Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( GC8034_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pGC8034Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pGC8034Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( GC8034_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pGC8034Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pGC8034Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( GC8034_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pGC8034Ctx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pGC8034Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( GC8034_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pGC8034Ctx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pGC8034Ctx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &GC8034_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( GC8034_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( GC8034_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
    }

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}


/*****************************************************************************/
/**
 *          GC8034_IsiMdiInitMotoDriveMds
 *
 * @brief   General initialisation tasks like I/O initialisation.
 *
 * @param   handle              GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiMdiSetupMotoDrive
 *
 * @brief   Setup of the MotoDrive and return possible max step.
 *
 * @param   handle          GC8034 sensor instance handle
 *          pMaxStep        pointer to variable to receive the maximum
 *                          possible focus step
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;

    TRACE( GC8034_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }

 if ((pGC8034Ctx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pGC8034Ctx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pGC8034Ctx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( GC8034_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
 
	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));

    result = GC8034_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( GC8034_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiMdiFocusSet
 *
 * @brief   Drives the lens system to a certain focus point.
 *
 * @param   handle          GC8034 sensor instance handle
 *          AbsStep         absolute focus point to apply
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( GC8034_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
/* SYNNEX DEBUG*/
    #if 1
    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( GC8034_ERROR, "%s: pGC8034Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pGC8034Ctx->VcmInfo.StartCurrent;
    else 
        nPosition = pGC8034Ctx->VcmInfo.StartCurrent + (pGC8034Ctx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( GC8034_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pGC8034Ctx->VcmInfo.StepMode );
	
    //TRACE( GC8034_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pGC8034Ctx->IsiCtx.HalHandle,
                             pGC8034Ctx->IsiCtx.I2cAfBusNum,
                             pGC8034Ctx->IsiCtx.SlaveAfAddress,
                             0,
                             pGC8034Ctx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( GC8034_DEBUG, "%s: (exit)\n", __FUNCTION__);
    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiMdiFocusGet
 *
 * @brief   Retrieves the currently applied focus point.
 *
 * @param   handle          GC8034 sensor instance handle
 *          pAbsStep        pointer to a variable to receive the current
 *                          focus point
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    TRACE( GC8034_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    /* SYNNEX DEBUG */
    #if 1
    result = HalReadI2CMem( pGC8034Ctx->IsiCtx.HalHandle,
                            pGC8034Ctx->IsiCtx.I2cAfBusNum,
                            pGC8034Ctx->IsiCtx.SlaveAfAddress,
                            0,
                            pGC8034Ctx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( GC8034_DEBUG, "%s:[SYNNEX_VAM_DEBUG] value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

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
	if( *pAbsStep <= pGC8034Ctx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pGC8034Ctx->VcmInfo.StartCurrent) && (*pAbsStep<=pGC8034Ctx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pGC8034Ctx->VcmInfo.RatedCurrent - *pAbsStep ) / pGC8034Ctx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
    #endif
   TRACE( GC8034_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiMdiFocusCalibrate
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 *****************************************************************************/
static RESULT GC8034_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          GC8034_IsiActivateTestPattern
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 ******************************************************************************/
static RESULT GC8034_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;
	return ( result );

	#if 0
    uint32_t ulRegValue = 0UL;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = GC8034_IsiRegReadIss( pGC8034Ctx, GC8034_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = GC8034_IsiRegReadIss( pGC8034Ctx, GC8034_PRE_ISP_CTRL00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = GC8034_IsiRegWriteIss( pGC8034Ctx, GC8034_PRE_ISP_CTRL00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pGC8034Ctx->TestPattern = enable;
    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
	#endif
}



/*****************************************************************************/
/**
 *          GC8034_IsiGetSensorMipiInfoIss
 *
 * @brief   Triggers a forced calibration of the focus hardware.
 *
 * @param   handle          GC8034 sensor instance handle
 *
 * @return  Return the result of the function call.
 * @retval  RET_SUCCESS
 * @retval  RET_WRONG_HANDLE
 * @retval  RET_NULL_POINTER
 ******************************************************************************/
static RESULT GC8034_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }


    if ( ptIsiSensorMipiInfo == NULL )
    {
        return ( result );
    }

	ptIsiSensorMipiInfo->ucMipiLanes = pGC8034Ctx->IsiSensorMipiInfo.ucMipiLanes;
    ptIsiSensorMipiInfo->ulMipiFreq= pGC8034Ctx->IsiSensorMipiInfo.ulMipiFreq;
    ptIsiSensorMipiInfo->sensorHalDevID = pGC8034Ctx->IsiSensorMipiInfo.sensorHalDevID;
    TRACE( GC8034_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT GC8034_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
    	TRACE( GC8034_ERROR, "%s: pGC8034Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( GC8034_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT GC8034_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
    	TRACE( GC8034_ERROR, "%s: pGC8034Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( GC8034_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = GC8034_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          GC8034_IsiGetSensorIss
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
RESULT GC8034_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = GC8034_g_acName;
        pIsiSensor->pRegisterTable                      = GC8034_g_aRegDescription_fourlane;
        pIsiSensor->pIsiSensorCaps                      = &GC8034_g_IsiSensorDefaultConfig;
			  pIsiSensor->pIsiGetSensorIsiVer									= GC8034_IsiGetSensorIsiVersion;//oyyf
				pIsiSensor->pIsiGetSensorTuningXmlVersion				= GC8034_IsiGetSensorTuningXmlVersion;//oyyf
				pIsiSensor->pIsiCheckOTPInfo                    = check_read_otp;
				pIsiSensor->pIsiSetSensorOTPInfo								= GC8034_IsiSetOTPInfo;
				pIsiSensor->pIsiEnableSensorOTP									= GC8034_IsiEnableOTP;
        pIsiSensor->pIsiCreateSensorIss                 = GC8034_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = GC8034_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = GC8034_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = GC8034_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = GC8034_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = GC8034_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = GC8034_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = GC8034_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = GC8034_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = GC8034_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = GC8034_IsiRegWriteIss;
        
        pIsiSensor->pIsiExposureControlIss              = GC8034_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = GC8034_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = GC8034_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = GC8034_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = GC8034_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = GC8034_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = GC8034_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = GC8034_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = GC8034_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = GC8034_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = GC8034_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = GC8034_IsiGetAfpsInfoIss;
        
        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = GC8034_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = GC8034_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = GC8034_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = GC8034_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = GC8034_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = GC8034_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = GC8034_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = GC8034_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = GC8034_IsiGetLscMatrixTable;
        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = GC8034_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = GC8034_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = GC8034_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = GC8034_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = GC8034_IsiMdiFocusCalibrate;
        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = GC8034_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = GC8034_IsiActivateTestPattern;
		pIsiSensor->pIsiSetSensorFrameRateLimit			= GC8034_IsiSetSensorFrameRateLimit;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( GC8034_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

//fix;hkw 14825
static RESULT GC8034_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( GC8034_ERROR,  "%s: Can't allocate GC8034 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = GC8034_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = GC8034_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = GC8034_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = GC8034_SOFTWARE_RST_VALUE;
    pSensorI2cInfo->reg_size = 1;
    pSensorI2cInfo->value_size = 1;

    TRACE( GC8034_DEBUG,  "%s: i2c_addr: 0x%x\n", __FUNCTION__,  pSensorI2cInfo->i2c_addr);

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps;
        uint32_t lanes,i;        

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(GC8034_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = GC8034_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = GC8034_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    
    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = GC8034_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = GC8034_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
    return RET_SUCCESS;
}

static RESULT GC8034_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate)
{
    GC8034_Context_t *pGC8034Ctx = (GC8034_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( GC8034_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pGC8034Ctx == NULL )
    {
    	TRACE( GC8034_ERROR, "%s: pOV8858Ctx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }
	
	pGC8034Ctx->preview_minimum_framerate = minimum_framerate;
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
    GC8034_IsiGetSensorIss,
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
    GC8034_IsiGetSensorI2cInfo,
};


