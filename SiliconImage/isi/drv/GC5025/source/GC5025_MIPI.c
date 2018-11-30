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
 * @file GC5025.c
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <math.h>
#include <common/return_codes.h>
#include <common/misc.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "GC5025_MIPI_priv.h"

#define  Sensor_NEWEST_TUNING_XML "GC5025_v1.0"
#define I2C_COMPLIANT_STARTBIT 1U

/******************************************************************************
 * local macro definitions
 *****************************************************************************/
CREATE_TRACER( Sensor_INFO , "GC5025: ", INFO,    0U );
CREATE_TRACER( Sensor_WARN , "GC5025: ", WARNING, 1U );
CREATE_TRACER( Sensor_ERROR, "GC5025: ", ERROR,   1U );
CREATE_TRACER( Sensor_DEBUG, "GC5025: ", DEBUG,     1U );
CREATE_TRACER( Sensor_REG_INFO , "GC5025: ", INFO, 0);
CREATE_TRACER( Sensor_REG_DEBUG, "GC5025: ", INFO, 0U );

#define Sensor_SLAVE_ADDR       0x6eU                           /**< i2c slave address of the GC5025 camera sensor */
#define Sensor_SLAVE_ADDR2      0x6eU
#define Sensor_SLAVE_AF_ADDR    0x18U                           /**< i2c slave address of the GC5025 integrated AD5820 */

#define Sensor_MIN_GAIN_STEP   ( 1.0f / 64.0f); /**< min gain step size used by GUI ( 32/(32-7) - 32/(32-6); min. reg value is 6 as of datasheet; depending on actual gain ) */
#define Sensor_MAX_GAIN_AEC    ( 8.0f )            /**< max. gain used by the AEC (arbitrarily chosen, recommended by Omnivision) */
//max analog gain = 8x

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

//#define VCM_ID_CN3927E
#define VCM_ID_DW9714

/*!<
 * Lens movement is triggered every 133ms (VGA, 7.5fps processed frames
 * worst case assumed, usually even much slower, see OV5630 driver for
 * details). Thus the lens has to reach the requested position after
 * max. 133ms. Minimum mechanical ringing is expected with mode 1 ,
 * 100us per step. A movement over the full range needs max. 102.3ms
 * (see table 9 AD5820 datasheet).
 */
#define MDI_SLEW_RATE_CTRL 6U /* S3..0 */
#define FLOAT_EPS 1e-6

/******************************************************************************
 * local variable declarations
 *****************************************************************************/
const char Sensor_g_acName[] = "GC5025_MIPI";
extern const IsiRegDescription_t Sensor_g_aRegDescription_twolane[];
extern const IsiRegDescription_t Sensor_g_aRegDescription_twolane_2[];
extern const IsiRegDescription_t Sensor_g_aRegDescription_twolane_3[];

extern const IsiRegDescription_t Sensor_g_twolane_resolution_2592_1944[];
extern const IsiRegDescription_t Sensor_g_2592x1944P30_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_2592x1944P25_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_2592x1944P20_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_2592x1944P15_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_2592x1944P10_twolane_fpschg[];
extern const IsiRegDescription_t Sensor_g_2592x1944P7_twolane_fpschg[];

const IsiSensorCaps_t Sensor_g_IsiSensorDefaultConfig;

#define Sensor_I2C_START_BIT        (I2C_COMPLIANT_STARTBIT)    // I2C bus start condition
#define Sensor_I2C_NR_ADR_BYTES     (1U)                        // 1 byte base address and 2 bytes sub address
#define Sensor_I2C_NR_DAT_BYTES     (1U)                        // 8 bit registers

static uint16_t g_suppoted_mipi_lanenum_type = SUPPORT_MIPI_TWO_LANE;//SUPPORT_MIPI_ONE_LANE|SUPPORT_MIPI_TWO_LANE|SUPPORT_MIPI_FOUR_LANE;
#define DEFAULT_NUM_LANES SUPPORT_MIPI_TWO_LANE

static uint32_t Dgain_ratio = 1;
static bool DR_State = true;


/******************************************************************************
 * local function prototypes
 *****************************************************************************/
static RESULT Sensor_IsiCreateSensorIss( IsiSensorInstanceConfig_t *pConfig );
static RESULT Sensor_IsiReleaseSensorIss( IsiSensorHandle_t handle );
static RESULT Sensor_IsiGetCapsIss( IsiSensorHandle_t handle, IsiSensorCaps_t *pIsiSensorCaps );
static RESULT Sensor_IsiSetupSensorIss( IsiSensorHandle_t handle, const IsiSensorConfig_t *pConfig );
static RESULT Sensor_IsiSensorSetStreamingIss( IsiSensorHandle_t handle, bool_t on );
static RESULT Sensor_IsiSensorSetPowerIss( IsiSensorHandle_t handle, bool_t on );
static RESULT Sensor_IsiCheckSensorConnectionIss( IsiSensorHandle_t handle );
static RESULT Sensor_IsiGetSensorRevisionIss( IsiSensorHandle_t handle, uint32_t *p_value);

static RESULT Sensor_IsiGetGainLimitsIss( IsiSensorHandle_t handle, float *pMinGain, float *pMaxGain);
static RESULT Sensor_IsiGetIntegrationTimeLimitsIss( IsiSensorHandle_t handle, float *pMinIntegrationTime, float *pMaxIntegrationTime );
static RESULT Sensor_IsiExposureControlIss( IsiSensorHandle_t handle, float NewGain, float NewIntegrationTime, uint8_t *pNumberOfFramesToSkip, float *pSetGain, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetCurrentExposureIss( IsiSensorHandle_t handle, float *pSetGain, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetAfpsInfoIss ( IsiSensorHandle_t handle, uint32_t Resolution, IsiAfpsInfo_t* pAfpsInfo);
static RESULT Sensor_IsiGetGainIss( IsiSensorHandle_t handle, float *pSetGain );
static RESULT Sensor_IsiGetGainIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT Sensor_IsiSetGainIss( IsiSensorHandle_t handle, float NewGain, float *pSetGain );
static RESULT Sensor_IsiGetIntegrationTimeIss( IsiSensorHandle_t handle, float *pSetIntegrationTime );
static RESULT Sensor_IsiGetIntegrationTimeIncrementIss( IsiSensorHandle_t handle, float *pIncr );
static RESULT Sensor_IsiSetIntegrationTimeIss( IsiSensorHandle_t handle, float NewIntegrationTime, float *pSetIntegrationTime, uint8_t *pNumberOfFramesToSkip );
static RESULT Sensor_IsiGetResolutionIss( IsiSensorHandle_t handle, uint32_t *pSetResolution );


static RESULT Sensor_IsiRegReadIss( IsiSensorHandle_t handle, const uint32_t address, uint32_t *p_value );
static RESULT Sensor_IsiRegWriteIss( IsiSensorHandle_t handle, const uint32_t address, const uint32_t value );

static RESULT Sensor_IsiGetCalibKFactor( IsiSensorHandle_t handle, Isi1x1FloatMatrix_t **pIsiKFactor );
static RESULT Sensor_IsiGetCalibPcaMatrix( IsiSensorHandle_t   handle, Isi3x2FloatMatrix_t **pIsiPcaMatrix );
static RESULT Sensor_IsiGetCalibSvdMeanValue( IsiSensorHandle_t   handle, Isi3x1FloatMatrix_t **pIsiSvdMeanValue );
static RESULT Sensor_IsiGetCalibCenterLine( IsiSensorHandle_t   handle, IsiLine_t  **ptIsiCenterLine);
static RESULT Sensor_IsiGetCalibClipParam( IsiSensorHandle_t   handle, IsiAwbClipParm_t    **pIsiClipParam );
static RESULT Sensor_IsiGetCalibGlobalFadeParam( IsiSensorHandle_t       handle, IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam);
static RESULT Sensor_IsiGetCalibFadeParam( IsiSensorHandle_t   handle, IsiAwbFade2Parm_t   **ptIsiFadeParam);
static RESULT Sensor_IsiGetIlluProfile( IsiSensorHandle_t   handle, const uint32_t CieProfile, IsiIlluProfile_t **ptIsiIlluProfile );

static RESULT Sensor_IsiMdiInitMotoDriveMds( IsiSensorHandle_t handle );
static RESULT Sensor_IsiMdiSetupMotoDrive( IsiSensorHandle_t handle, uint32_t *pMaxStep );
static RESULT Sensor_IsiMdiFocusSet( IsiSensorHandle_t handle, const uint32_t Position );
static RESULT Sensor_IsiMdiFocusGet( IsiSensorHandle_t handle, uint32_t *pAbsStep );
static RESULT Sensor_IsiMdiFocusCalibrate( IsiSensorHandle_t handle );

static RESULT Sensor_IsiGetSensorMipiInfoIss( IsiSensorHandle_t handle, IsiSensorMipiInfo *ptIsiSensorMipiInfo);
static RESULT Sensor_IsiGetSensorIsiVersion(  IsiSensorHandle_t   handle, unsigned int* pVersion);
static RESULT Sensor_IsiGetSensorTuningXmlVersion(  IsiSensorHandle_t   handle, char** pTuningXmlVersion);
static RESULT GC5025_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate);
/* OTP START*/

void GC5025_OTP_Select_Page(sensor_i2c_write_t*  sensor_i2c_write_p,sensor_i2c_read_t*	sensor_i2c_read_p,void* context,int camsys_fd, uint8_t page)//otp_page otp_select_page
{
    int32_t ret = 0;
    uint32_t  page_reg = 0;
    int i2c_base_info[3];
    i2c_base_info[0] = 0x6e; //otp i2c addr
    i2c_base_info[1] = 1; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size
    ret = sensor_i2c_write_p(context,camsys_fd, 0xFE, 0x00, i2c_base_info );
    page_reg = sensor_i2c_read_p(context,camsys_fd,0xD4,i2c_base_info);
    switch(page)
    {
        case 0:
        page_reg = page_reg & 0xfb;
        break;
        case 1:
        page_reg = page_reg | 0x04;
        break;
        default:
        break;
    }
    ret = sensor_i2c_write_p(context,camsys_fd, 0xD4, page_reg , i2c_base_info );
}
static int GC5025_OTP_Read_i2c(sensor_i2c_write_t*  sensor_i2c_write_p,sensor_i2c_read_t*	sensor_i2c_read_p,void* context,int camsys_fd,int address)
{
    uint32_t value = 0;
    int i2c_base_info[3];
    int ret = 0;
    uint32_t addr_high = 0;
    uint32_t addr_low = 0;
    i2c_base_info[0] = 0x6e; //otp i2c addr
    i2c_base_info[1] = 1; //otp i2c reg size
    i2c_base_info[2] = 1; //otp i2c value size
    ret = sensor_i2c_write_p(context,camsys_fd, 0xFE, 0x00, i2c_base_info );
    addr_low = address & 0xff;
    ret = sensor_i2c_write_p(context,camsys_fd, 0xD5, addr_low , i2c_base_info );
    addr_high = sensor_i2c_read_p(context,camsys_fd,0xD4,i2c_base_info);
    addr_high &= 0xfc;
    addr_high |= ( (address & 0xf00) >> 8) & 0x003;
    ret = sensor_i2c_write_p(context,camsys_fd, 0xD4, addr_high , i2c_base_info );
    // OTP Read    
    ret = sensor_i2c_write_p(context,camsys_fd, 0xF3, 0x20, i2c_base_info );
    value = sensor_i2c_read_p(context,camsys_fd,0xD7,i2c_base_info);
    TRACE( Sensor_DEBUG, "%s Reg 0x%x = 0x%x\n", __FUNCTION__,address, value);
    return      value;
}

#define DD_PARAM_QTY_5025 200
#define INFO_ROM_START_5025 0x08
#define INFO_WIDTH_5025 0x08
#define WB_ROM_START_5025 0x88
#define WB_WIDTH_5025 0x05
#define GOLDEN_ROM_START_5025 0xe0
#define GOLDEN_WIDTH_5025 0x05

#define RG_TYPICAL          0x0400
#define BG_TYPICAL          0x0400

#define WINDOW_WIDTH  		0x0a30 //2608 max effective pixels
#define WINDOW_HEIGHT 		0x079c //1948

#define IMAGE_NORMAL_MIRROR

int  RG_Ratio_Typical=0x0;
int  BG_Ratio_Typical=0x0;
bool bOTP_switch = true;

typedef struct otp_gc5025
{
    uint16_t module_id;                       
    uint16_t lens_id;                         
    uint16_t vcm_id;                          
    uint16_t vcm_driver_id;                   
    uint16_t year;                            
    uint16_t month;                           
    uint16_t day;                             
    uint16_t rg_gain;                         
    uint16_t bg_gain;                         
    uint16_t module_info_flag;                
    uint16_t wb_flag;                         
    uint16_t golden_flag;                  
    uint16_t dd_param_x[DD_PARAM_QTY_5025];   
    uint16_t dd_param_y[DD_PARAM_QTY_5025];   
    uint16_t dd_param_type[DD_PARAM_QTY_5025];
    uint16_t dd_cnt;                          
    uint16_t dd_flag;                         
    uint16_t golden_rg;                       
    uint16_t golden_bg;                      

}gc5025_otp;

static gc5025_otp gc5025_otp_info = {0};

static void GC5025_OTP_readinfo(
	sensor_i2c_write_t*  sensor_i2c_write_p,
	sensor_i2c_read_t*	sensor_i2c_read_p,
	void* context,
	int camsys_fd
) 
{
    int flagdd = 0;
    //int flag_lsc = 0;
   int i=0,j=0,cnt=0;
   int total_number = 0;
   uint8_t m_DD_Otp_Value[182];
   int sum3_c = 0;	//dd
   uint8_t sum3_r = 0;
   uint8_t check_dd_flag, type;
   uint8_t dd0 = 0, dd1 = 0, dd2 = 0;
   uint16_t x, y;
   uint16_t dd_rom_start, offset;
   int flag1;
   int flag_golden;
   int index = 0;
   uint8_t info_start_add, wb_start_add, golden_start_add;
   int sum0_c = 0;	//
   uint8_t sum0_r = 0;
   int sum1_c = 0;	//
   uint8_t sum1_r = 0;
   int sum2_c = 0;	//
   uint8_t sum2_r = 0;
   uint16_t tmpH = 0;
   uint16_t tmpL = 0;
    GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0);   
    flagdd = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x0);
    TRACE( Sensor_DEBUG, "%s  GC5025 OTP:flag_dd=0x%x!\n", __FUNCTION__,flagdd);

    switch(flagdd & 0x03)
    {
        case 0x00:
            TRACE( Sensor_ERROR, "%s GC5025 OTP:flag_dd is EMPTY!\n", __FUNCTION__);
            gc5025_otp_info.dd_flag = 0x00;
            break;
        case 0x01:
            total_number = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x08)+
                                GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x10);
		GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0);  
		for(i = 0; i < 126; i++)
		{
			m_DD_Otp_Value[i] = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,
                                                context,camsys_fd,0x08 + 8 * i);
			sum3_c += m_DD_Otp_Value[i];
		}
		GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,1);
		for(i = 0; i < 56; i++)
		{
			m_DD_Otp_Value[126 + i] = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,
                                                         context,camsys_fd,0x148 + 8 * i);
			sum3_c += m_DD_Otp_Value[126 + i];
		}
		sum3_c = sum3_c % 255 + 1;
		sum3_r = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x308);
		if (sum3_c == sum3_r)
		{
			for(i = 0; i < total_number; i++)
			{
				if(i < 31)
				{	
					GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0);
					dd_rom_start = 0x18;
					offset = 0;
				}
				else
				{
					GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,1);		
					dd_rom_start = 0x148;
					offset = 124;//31*4
				}	
				check_dd_flag = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,
                                                dd_rom_start + 8 * (4 * i - offset + 3));
				if(check_dd_flag & 0x10)
				{
					//Read OTP
					type = check_dd_flag & 0x0f;
					dd0 = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,
                                        dd_rom_start + 8 * (4 * i - offset));
					dd1 = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,
                                        dd_rom_start + 8 * (4 * i - offset + 1));   
					dd2 = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,
                                        dd_rom_start + 8 * (4 * i - offset + 2));
					x = ((dd1 & 0x0f) << 8) + dd0;
					y = (dd2 << 4) + ((dd1 & 0xf0) >> 4);			

					if(type == 3)
					{
						for(j = 0; j < 4; j++)
						{
							gc5025_otp_info.dd_param_x[cnt] = x;
							gc5025_otp_info.dd_param_y[cnt] = y + j;
							gc5025_otp_info.dd_param_type[cnt++] = 2;
						}
					}
					else if(type == 4)
					{
						for(j = 0; j < 2; j++)
						{
							gc5025_otp_info.dd_param_x[cnt] = x;
							gc5025_otp_info.dd_param_y[cnt] = y + j;
							gc5025_otp_info.dd_param_type[cnt++] = 2;
						}
					}			
					else
					{
						gc5025_otp_info.dd_param_x[cnt] = x;
						gc5025_otp_info.dd_param_y[cnt] = y;
						gc5025_otp_info.dd_param_type[cnt++] = type;
					}     
				}
				else
				{
				    TRACE( Sensor_ERROR, "%s GC5025_OTP_DD:check_id[%d] = %x,checkid error!!\n", __FUNCTION__,i,check_dd_flag);
				//	LOG_INF("GC5025_OTP_DD:check_id[%d] = %x,checkid error!!\n",i,check_dd_flag);
				}
			}
			gc5025_otp_info.dd_cnt = cnt;
			gc5025_otp_info.dd_flag = 0x01;
		}
		else
		{
			gc5025_otp_info.dd_flag = 0x02;
		}
            break;
        case 0x02:
        case 0x03:  
            TRACE( Sensor_ERROR, "%s GC5025 OTP:flag_dd is INVAILD!\n", __FUNCTION__);
            gc5025_otp_info.dd_flag = 0x02;
            break;
        default :
            break;
    }
    GC5025_OTP_Select_Page(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,1);
    flag1 = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x0);
    flag_golden = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0xd8);
    TRACE( Sensor_DEBUG, "%s  GC5025 OTP:flag1=0x%x\n", __FUNCTION__,flag1);

    /*SYNNEX DEBUG*/
    //INFO & WB
    for(index=0;index<2;index++)
    {
        switch((flag1 >> (4 + 2 * index)) & 0x03)
        {
            case 0x00:
                gc5025_otp_info.module_info_flag = 0x00;
                TRACE( Sensor_ERROR, "%s GC5025_OTP_INFO group %d is Empty!\n", __FUNCTION__,index +1);
                break;
            case 0x01:
                TRACE( Sensor_DEBUG, "%s GC5025_OTP_INFO group %d is Vaild!\n", __FUNCTION__,index +1);
                info_start_add = INFO_ROM_START_5025 + 8 * index * INFO_WIDTH_5025;
			gc5025_otp_info.module_id = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add);
			sum0_c += gc5025_otp_info.module_id;
			gc5025_otp_info.lens_id = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 1); 
			sum0_c += gc5025_otp_info.lens_id;
			gc5025_otp_info.vcm_driver_id = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 2);
			sum0_c += gc5025_otp_info.vcm_driver_id;
			gc5025_otp_info.vcm_id = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 3);
			sum0_c += gc5025_otp_info.vcm_id;
			gc5025_otp_info.year = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 4);
			sum0_c += gc5025_otp_info.year;
			gc5025_otp_info.month = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 5);
			sum0_c += gc5025_otp_info.month;
			gc5025_otp_info.day = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,info_start_add + 8 * 6);
			sum0_c += gc5025_otp_info.day;
			sum0_c = sum0_c % 255 + 1;
			sum0_r = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x40 + 8 * index * INFO_WIDTH_5025);
			if (sum0_c == sum0_r){
			    gc5025_otp_info.module_info_flag = 0x01;
                        TRACE( Sensor_DEBUG, "module_id 0x%02x,lens_id 0x%02x,vcm_driver_id 0x%02x,vcm_id 0x%02x,year 0x%02x,month 0x%02x,day 0x%02x,\n",
                            gc5025_otp_info.module_id,
                            gc5025_otp_info.lens_id,
                            gc5025_otp_info.vcm_driver_id,
                            gc5025_otp_info.vcm_id,
                            gc5025_otp_info.year,
                            gc5025_otp_info.month,
                            gc5025_otp_info.day);
			}else{
				gc5025_otp_info.module_info_flag = 0x02;
                         TRACE( Sensor_ERROR, "otp module info check sum error\n");
                    }
                break;
            case 0x02:
            case 0x03:  
                gc5025_otp_info.module_info_flag = 0x02;
                TRACE( Sensor_ERROR, "%s GC5025_OTP_INFO group %d is Invalid !!\n", __FUNCTION__,index +1);
                break;
            default :
                break;
        }
        
        switch((flag1 >> (2 * index)) & 0x03)
        {
        case 0x00:
            TRACE( Sensor_ERROR, "%s GC5025_OTP_WB group %d is Empty !\n", __FUNCTION__,index +1);
            gc5025_otp_info.wb_flag = 0x00;
            break;
        case 0x01:
            TRACE( Sensor_ERROR, "%s GC5025_OTP_WB group %d is Valid !!\n", __FUNCTION__,index +1);
            wb_start_add = WB_ROM_START_5025 + 8 * index * WB_WIDTH_5025;
		tmpH = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,wb_start_add);
		sum1_c += tmpH;
		tmpL = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,wb_start_add + 8 * 1); 
		sum1_c += tmpL;
		gc5025_otp_info.rg_gain = (tmpH << 8) | tmpL;
		tmpH = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,wb_start_add + 8 * 2); 
		sum1_c += tmpH;
		tmpL = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,wb_start_add + 8 * 3); 
		sum1_c += tmpL;
		gc5025_otp_info.bg_gain = (tmpH << 8) | tmpL;
		sum1_c = sum1_c % 255 + 1;
		sum1_r = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0xa8 + 8 * index * WB_WIDTH_5025);
		if (sum1_c == sum1_r)
		{
			gc5025_otp_info.wb_flag = 0x01;
                    TRACE( Sensor_DEBUG, "rg_gain 0x%02x, bg_gain 0x%02x,\n",
                            gc5025_otp_info.rg_gain,
                            gc5025_otp_info.bg_gain);
		}
		else
		{
			gc5025_otp_info.wb_flag = 0x02;
		}
		break;
        case 0x02:
        case 0x03: 
            TRACE( Sensor_ERROR, "%s GC5025_OTP_WB group %d is Invalid !!\n", __FUNCTION__,index +1);         
            gc5025_otp_info.wb_flag = 0x02;
            break;
        default :
            break;
        }

        switch((flag_golden >> (2 * index)) & 0x03)
        {
            case 0x00:
                TRACE( Sensor_ERROR, "%s GC5025_OTP_GOLDEN group %d is Empty !!\n", __FUNCTION__,index +1);
                gc5025_otp_info.golden_flag = 0x00;                 
                break;
            case 0x01:
                TRACE( Sensor_DEBUG, "%s GC5025_OTP_GOLDEN group %d is Vaild !!\n", __FUNCTION__,index +1);
                golden_start_add = GOLDEN_ROM_START_5025 + 8 * index * GOLDEN_WIDTH_5025;
                tmpH = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,golden_start_add);
                sum2_c += tmpH;
                tmpL = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,golden_start_add + 8 * 1); 
                sum2_c += tmpL;
                gc5025_otp_info.golden_rg = (tmpH << 8) | tmpL;
                tmpH = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,golden_start_add + 8 * 2); 
                sum2_c += tmpH;
                tmpL = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,golden_start_add + 8 * 3); 
                sum2_c += tmpL;
                gc5025_otp_info.golden_bg = (tmpH << 8) | tmpL;
                sum2_c = sum2_c % 255 + 1;
                sum2_r = GC5025_OTP_Read_i2c(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd,0x100 + 8 * index * GOLDEN_WIDTH_5025);
                if (sum2_c == sum2_r)
                {
                    gc5025_otp_info.golden_flag = 0x01;			
                }
                else
                {
                    gc5025_otp_info.golden_flag = 0x02;	
                }
                break;
                case 0x02:
                case 0x03: 
                    TRACE( Sensor_ERROR, "%s GC5025_OTP_GOLDEN group %d is Invalid !!\n", __FUNCTION__,index +1);  
                    gc5025_otp_info.golden_flag = 0x02;         
                    break;
                default :
                    break;
        }     
    }
       
}

static void GC5025_OTP_update_dd(IsiSensorHandle_t   handle)
{
    	uint16_t i = 0, j = 0;
	uint16_t temp_x = 0, temp_y = 0;
	uint8_t temp_type = 0;
	uint8_t temp_val0, temp_val1, temp_val2;
	/*TODO*/

	if(0x01 == gc5025_otp_info.dd_flag)
	{
		// #elif defined(IMAGE_H_MIRROR) 
#if defined(IMAGE_NORMAL_MIRROR)
		
#elif defined(IMAGE_H_MIRROR)
  for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{
			if(gc5025_otp_info.dd_param_type[i]==0)
			{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] + 1;	}
			else if(gc5025_otp_info.dd_param_type[i]==1)
			{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] - 1;	}
			else
			{	gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i];	}
		}
		//do nothing
#elif defined(IMAGE_V_MIRROR)
   for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{	gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;	}
		
#elif defined(IMAGE_HV_MIRROR)
		
   for(i=0; i<gc5025_otp_info.dd_cnt; i++)
		{
			if(gc5025_otp_info.dd_param_type[i]==0)
			{	
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]+1;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i]+1;
			}
			else if(gc5025_otp_info.dd_param_type[i]==1)
			{
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i]-1;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i]+1;
			}
			else
			{
				gc5025_otp_info.dd_param_x[i]= WINDOW_WIDTH - gc5025_otp_info.dd_param_x[i] ;
				gc5025_otp_info.dd_param_y[i]= WINDOW_HEIGHT - gc5025_otp_info.dd_param_y[i] + 1;
			}
		}
#endif

		//y
		for(i = 0 ; i < gc5025_otp_info.dd_cnt - 1; i++) 
		{
			for(j = 0; j < gc5025_otp_info.dd_cnt - 1 - i; j++) 
			{  
				if(gc5025_otp_info.dd_param_y[j] > gc5025_otp_info.dd_param_y[j+1])  
				{  
					temp_x = gc5025_otp_info.dd_param_x[j] ; gc5025_otp_info.dd_param_x[j] = gc5025_otp_info.dd_param_x[j+1] ;  gc5025_otp_info.dd_param_x[j+1] = temp_x;
					temp_y = gc5025_otp_info.dd_param_y[j] ; gc5025_otp_info.dd_param_y[j] = gc5025_otp_info.dd_param_y[j+1] ;  gc5025_otp_info.dd_param_y[j+1] = temp_y;
					temp_type = gc5025_otp_info.dd_param_type[j] ; gc5025_otp_info.dd_param_type[j] = gc5025_otp_info.dd_param_type[j+1]; gc5025_otp_info.dd_param_type[j+1]= temp_type;
				} 
			}
		}

		//x
		int column = 0;
		for(i = 0 ; i < gc5025_otp_info.dd_cnt - 1; ++i) 
		{ 		
			if (gc5025_otp_info.dd_param_y[i] == gc5025_otp_info.dd_param_y[i+1])
			{
				column++;
				if (gc5025_otp_info.dd_cnt - 2 != i)
				{
					continue;
				}
			}
			if (gc5025_otp_info.dd_cnt - 2 == i && gc5025_otp_info.dd_param_y[i] == gc5025_otp_info.dd_param_y[i+1])
			{
				i = gc5025_otp_info.dd_cnt - 1;
			}
			int iii = i - column;
			for(int ii = i - column; ii < i ; ++ii) 
			{  
				for(int jj = i - column; jj < i - (ii - iii); ++jj) 
				{  
					if(gc5025_otp_info.dd_param_x[jj] > gc5025_otp_info.dd_param_x[jj+1])  
					{  
						temp_x = gc5025_otp_info.dd_param_x[jj] ; gc5025_otp_info.dd_param_x[jj] = gc5025_otp_info.dd_param_x[jj+1] ;  gc5025_otp_info.dd_param_x[jj+1] = temp_x;
						temp_y = gc5025_otp_info.dd_param_y[jj] ; gc5025_otp_info.dd_param_y[jj] = gc5025_otp_info.dd_param_y[jj+1] ;  gc5025_otp_info.dd_param_y[jj+1] = temp_y;
						temp_type = gc5025_otp_info.dd_param_type[jj] ; gc5025_otp_info.dd_param_type[jj] = gc5025_otp_info.dd_param_type[jj+1]; gc5025_otp_info.dd_param_type[jj+1]= temp_type;
					}  
				}			
			}     
			column = 0;
		}

		//write SRAM
		Sensor_IsiRegWriteIss(handle,0xfe, 0x00);
		Sensor_IsiRegWriteIss(handle,0x80, 0x50);
		Sensor_IsiRegWriteIss(handle,0xfe, 0x01);
		Sensor_IsiRegWriteIss(handle,0xa8, 0x00);
		Sensor_IsiRegWriteIss(handle,0x9d, 0x04);
		Sensor_IsiRegWriteIss(handle,0xbe, 0x00);
		Sensor_IsiRegWriteIss(handle,0xa9, 0x01);

		for(i = 0; i < gc5025_otp_info.dd_cnt; i++)
		{
			temp_val0 = gc5025_otp_info.dd_param_x[i] & 0x00ff;
			temp_val1 = ((gc5025_otp_info.dd_param_y[i] << 4) & 0x00f0) + ((gc5025_otp_info.dd_param_x[i] >> 8) & 0x000f);
			temp_val2 = (gc5025_otp_info.dd_param_y[i] >> 4) & 0xff;
			Sensor_IsiRegWriteIss(handle,0xaa, i);
			Sensor_IsiRegWriteIss(handle,0xac, temp_val0);
			Sensor_IsiRegWriteIss(handle,0xac, temp_val1);
			Sensor_IsiRegWriteIss(handle,0xac, temp_val2);
			Sensor_IsiRegWriteIss(handle,0xac, gc5025_otp_info.dd_param_type[i]);
		}
		Sensor_IsiRegWriteIss(handle,0xbe,0x01);
		Sensor_IsiRegWriteIss(handle,0xfe,0x00);
	}

}
static void GC5025_OTP_update_wb(IsiSensorHandle_t   handle)
{
    uint16_t r_gain_current = 0 , g_gain_current = 0 , b_gain_current = 0 , base_gain = 0;
    uint16_t r_gain = 1024 , g_gain = 1024 , b_gain = 1024 ;
    uint16_t rg_typical,bg_typical;
    rg_typical=RG_Ratio_Typical;
    bg_typical=BG_Ratio_Typical;        
    TRACE( Sensor_DEBUG, "%s GC5025_OTP_UPDATE_AWB:rg_typical = 0x%x , bg_typical = 0x%x !\n", __FUNCTION__,rg_typical,bg_typical);  
    if(0x01==(gc5025_otp_info.wb_flag&0x01))
    {   
        r_gain_current = 2048 * rg_typical/gc5025_otp_info.rg_gain;
        b_gain_current = 2048 * bg_typical/gc5025_otp_info.bg_gain;
        g_gain_current = 2048;

        base_gain = (r_gain_current<b_gain_current) ? r_gain_current : b_gain_current;
        base_gain = (base_gain<g_gain_current) ? base_gain : g_gain_current;
        TRACE( Sensor_DEBUG, "%s GC5025_OTP_UPDATE_AWB:r_gain_current = 0x%x , b_gain_current = 0x%x , base_gain = 0x%x\n", __FUNCTION__,r_gain_current,b_gain_current,base_gain);           
        r_gain = 0x400 * r_gain_current / base_gain;
        g_gain = 0x400 * g_gain_current / base_gain;
        b_gain = 0x400 * b_gain_current / base_gain;
        TRACE( Sensor_DEBUG, "%s GC5025_OTP_UPDATE_AWB:r_gain = 0x%x , g_gain = 0x%x , b_gain = 0x%x\n", __FUNCTION__,r_gain,g_gain,b_gain);
        /*TODO*/
        Sensor_IsiRegWriteIss(handle,0xfe,0x00);
        Sensor_IsiRegWriteIss(handle,0xc6,(g_gain & 0x7f8) >> 7);
        Sensor_IsiRegWriteIss(handle,0xc7,(r_gain & 0x7f8) >> 7);
        Sensor_IsiRegWriteIss(handle,0xc8,(b_gain & 0x7f8) >> 7);
        Sensor_IsiRegWriteIss(handle,0xc9,(g_gain & 0x7f8) >> 7);
        
        Sensor_IsiRegWriteIss(handle,0xc4,((g_gain & 0x7) << 4) | (r_gain & 0x7));
        Sensor_IsiRegWriteIss(handle,0xc5,((b_gain & 0x7) << 4) | (g_gain & 0x7));
    }
}

static void GC5025_OTP_update(IsiSensorHandle_t   handle)
{
    GC5025_OTP_update_dd(handle);
    GC5025_OTP_update_wb(handle);
}
static void GC5025_OTP_Enable(IsiSensorHandle_t   handle,    int OTPstate)
{
    uint32_t otp_clk,otp_en;
    Sensor_IsiRegReadIss(handle,0xfa, &otp_clk);
    Sensor_IsiRegReadIss(handle,0xd4, &otp_en);
    if(OTPstate)
    {
        otp_clk |= 0x10;
        otp_en |= 0x80;
        Sensor_IsiRegWriteIss(handle, 0xfa, otp_clk);
        Sensor_IsiRegWriteIss(handle, 0xd4, otp_en );
        TRACE( Sensor_DEBUG, "%s  GC5025 OTP ENABLE!\n", __FUNCTION__);
    }
    else
    {
        otp_clk &=0xef;
        otp_en &=0x7f;
        Sensor_IsiRegWriteIss(handle, 0xfa, otp_clk);
        Sensor_IsiRegWriteIss(handle, 0xd4, otp_en );
        TRACE( Sensor_DEBUG, "%s GC5025 OTP DISABLE!\n", __FUNCTION__);
    }
}
static int apply_otp(IsiSensorHandle_t   handle)
{
    Sensor_IsiRegWriteIss(handle,0xfe, 0x00); 
    Sensor_IsiRegWriteIss(handle,0xf7, 0x01); 
    Sensor_IsiRegWriteIss(handle,0xf8, 0x11); 
    Sensor_IsiRegWriteIss(handle,0xf9, 0x00);
    Sensor_IsiRegWriteIss(handle,0xfa, 0xa0); 
    Sensor_IsiRegWriteIss(handle,0xfc, 0x2a); 
    Sensor_IsiRegWriteIss(handle,0xfe, 0x03); 
    Sensor_IsiRegWriteIss(handle,0x01, 0x07); 
    Sensor_IsiRegWriteIss(handle,0xfc, 0x2e); 
    Sensor_IsiRegWriteIss(handle,0xfe, 0x00);
    osSleep(1);
    Sensor_IsiRegWriteIss(handle,0x88, 0x03); 
    Sensor_IsiRegWriteIss(handle,0xe7, 0xcc); 
    Sensor_IsiRegWriteIss(handle,0xfc, 0x2e); 
    Sensor_IsiRegWriteIss(handle,0xfa, 0xb0);
   
   GC5025_OTP_Enable(handle,1);
   GC5025_OTP_update(handle);
   GC5025_OTP_Enable(handle,0);
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
  	
	//sensor_i2c_write_p(context,camsys_fd, 0xf2, 0x00, i2c_base_info );
	//sensor_i2c_write_p(context,camsys_fd, 0xf4, 0x80, i2c_base_info );

    ret = sensor_i2c_write_p(context,camsys_fd, 0xfe, 0x00, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xf7, 0x01, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xf8, 0x11, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xf9, 0x00, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfa, 0xa0, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x2a, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfe, 0x03, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0x01, 0x07, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x2e, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfe, 0x00, i2c_base_info);
    osSleep(1);
    ret = sensor_i2c_write_p(context,camsys_fd, 0x88, 0x03, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xe7, 0xcc, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfc, 0x2e, i2c_base_info);
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfa, 0xb0, i2c_base_info);
    
    otp_clk = sensor_i2c_read_p(context,camsys_fd, 0xfa, i2c_base_info);
    otp_en = sensor_i2c_read_p(context,camsys_fd, 0xd4, i2c_base_info);
    otp_clk |=0x10;
    otp_en |=0x80;
    ret = sensor_i2c_write_p(context,camsys_fd, 0xFa, otp_clk, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xd4, otp_en, i2c_base_info );
    osSleep(20);

    GC5025_OTP_readinfo(sensor_i2c_write_p,sensor_i2c_read_p,context,camsys_fd);
		
    otp_clk = sensor_i2c_read_p(context,camsys_fd, 0xfa, i2c_base_info);
    otp_en = sensor_i2c_read_p(context,camsys_fd, 0xd4, i2c_base_info);  
    otp_clk &= 0xef;
    otp_en &= 0x7f;
    ret = sensor_i2c_write_p(context,camsys_fd, 0xfa, otp_clk, i2c_base_info );
    ret = sensor_i2c_write_p(context,camsys_fd, 0xd4, otp_en, i2c_base_info );
    
    return RET_SUCCESS;
}

static RESULT GC5025_IsiSetOTPInfo
(
    IsiSensorHandle_t       handle,
    uint32_t OTPInfo
)
{
	RESULT result = RET_SUCCESS;

    Sensor_Context_t *pGC5025Ctx = (Sensor_Context_t *)handle;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC5025Ctx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

	RG_Ratio_Typical = OTPInfo>>16;
	BG_Ratio_Typical = OTPInfo&0xffff;
	TRACE( Sensor_DEBUG, "%s:  --(RG,BG) in IQ file:(0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);
	if((RG_Ratio_Typical==0) && (BG_Ratio_Typical==0)){
		if(0x01==(gc5025_otp_info.golden_flag&0x01))
    		{
        		RG_Ratio_Typical=gc5025_otp_info.golden_rg;
        		BG_Ratio_Typical=gc5025_otp_info.golden_bg;   
    		}else{
			RG_Ratio_Typical = RG_TYPICAL;
	    BG_Ratio_Typical = BG_TYPICAL;
		}
		TRACE( Sensor_ERROR, "%s:  --OTP typical value in IQ file is zero, we will try another match rule.\n", __FUNCTION__);    
	}
	TRACE( Sensor_DEBUG, "%s:  --Finally, the (RG,BG) is (0x%x, 0x%x)\n", __FUNCTION__ , RG_Ratio_Typical, BG_Ratio_Typical);

	return (result);
}

static RESULT GC5025_IsiEnableOTP
(
    IsiSensorHandle_t       handle,
    const bool_t enable
)
{
	RESULT result = RET_SUCCESS;

    Sensor_Context_t *pGC5025Ctx = (Sensor_Context_t *)handle;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pGC5025Ctx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }
	bOTP_switch = enable;
	return (result);
}


/* OTP END*/

/*****************************************************************************/
/**
 *          Sensor_IsiCreateSensorIss
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
static RESULT Sensor_IsiCreateSensorIss
(
    IsiSensorInstanceConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;
    int32_t current_distance;
    Sensor_Context_t *pSensorCtx = NULL;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( (pConfig == NULL) || (pConfig->pSensor ==NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    pSensorCtx = ( Sensor_Context_t * )malloc ( sizeof (Sensor_Context_t) );
    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR,  "%s: Can't allocate GC5025 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorCtx, 0, sizeof( Sensor_Context_t ) );

    result = HalAddRef( pConfig->HalHandle );
    if ( result != RET_SUCCESS )
    {
        free ( pSensorCtx );
        return ( result );
    }

    pSensorCtx->IsiCtx.HalHandle              = pConfig->HalHandle;
    pSensorCtx->IsiCtx.HalDevID               = pConfig->HalDevID;
    pSensorCtx->IsiCtx.I2cBusNum              = pConfig->I2cBusNum;
    pSensorCtx->IsiCtx.SlaveAddress           = ( pConfig->SlaveAddr == 0 ) ? Sensor_SLAVE_ADDR : pConfig->SlaveAddr;
    pSensorCtx->IsiCtx.NrOfAddressBytes       = 1U;

    pSensorCtx->IsiCtx.I2cAfBusNum            = pConfig->I2cAfBusNum;
    pSensorCtx->IsiCtx.SlaveAfAddress         = ( pConfig->SlaveAfAddr == 0 ) ? Sensor_SLAVE_AF_ADDR : pConfig->SlaveAfAddr;
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
        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes = DEFAULT_NUM_LANES;
    }

    pConfig->hSensor = ( IsiSensorHandle_t )pSensorCtx;

    result = HalSetCamConfig( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false, true, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetClock( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, 24000000U);
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiReleaseSensorIss
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
static RESULT Sensor_IsiReleaseSensorIss
(
    IsiSensorHandle_t handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    (void)Sensor_IsiSensorSetStreamingIss( pSensorCtx, BOOL_FALSE );
    (void)Sensor_IsiSensorSetPowerIss( pSensorCtx, BOOL_FALSE );

    (void)HalDelRef( pSensorCtx->IsiCtx.HalHandle );

    MEMSET( pSensorCtx, 0, sizeof( Sensor_Context_t ) );
    free ( pSensorCtx );

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiGetCapsIss
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
static RESULT Sensor_IsiGetCapsIssInternal
(
    IsiSensorCaps_t   *pIsiSensorCaps,
    uint32_t mipi_lanes
)
{
    RESULT result = RET_SUCCESS;

    if ( pIsiSensorCaps == NULL )
    {
    		TRACE( Sensor_ERROR, "%s (enter)null!!!!!!!!!!!!!!!\n", __FUNCTION__);
        return ( RET_NULL_POINTER );
    }
    else
    {
		 if(mipi_lanes == SUPPORT_MIPI_ONE_LANE) {
	                    result = RET_OUTOFRANGE;
	                    goto end;

	     }
		 if(mipi_lanes == SUPPORT_MIPI_TWO_LANE){
            switch (pIsiSensorCaps->Index) 
            {
                case 0:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P30;
                    break;
                }
				case 1:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P25;
                    break;
                }
				case 2:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P20;
                    break;
                }
		  		case 3:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P15;
                    break;
                }
		   		case 4:
                {
                    pIsiSensorCaps->Resolution = ISI_RES_2592_1944P10;
                    break;
                }
                default:
                {
                    result = RET_OUTOFRANGE;
                    goto end;
                }

            }   
        } 
		if(mipi_lanes == SUPPORT_MIPI_FOUR_LANE) {
                    result = RET_OUTOFRANGE;
                    goto end;

        }   

        pIsiSensorCaps->BusWidth        = ISI_BUSWIDTH_10BIT;
        pIsiSensorCaps->Mode            = ISI_MODE_MIPI;
        pIsiSensorCaps->FieldSelection  = ISI_FIELDSEL_BOTH;
        pIsiSensorCaps->YCSequence      = ISI_YCSEQ_YCBYCR;           /**< only Bayer supported, will not be evaluated */
        pIsiSensorCaps->Conv422         = ISI_CONV422_NOCOSITED;
        pIsiSensorCaps->BPat            = ISI_BPAT_RGRGGBGB;
        pIsiSensorCaps->HPol            = ISI_HPOL_REFPOS;
        pIsiSensorCaps->VPol            = ISI_VPOL_NEG;
        pIsiSensorCaps->Edge            = ISI_EDGE_FALLING;
        pIsiSensorCaps->Bls             = ISI_BLS_OFF;
        pIsiSensorCaps->Gamma           = ISI_GAMMA_OFF;
        pIsiSensorCaps->CConv           = ISI_CCONV_OFF;
        pIsiSensorCaps->BLC             = ( ISI_BLC_AUTO);
        pIsiSensorCaps->AGC             = ( ISI_AGC_OFF );
        pIsiSensorCaps->AWB             = ( ISI_AWB_OFF );
        pIsiSensorCaps->AEC             = ( ISI_AEC_OFF );
        pIsiSensorCaps->DPCC            = ( ISI_DPCC_OFF );

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

static RESULT Sensor_IsiGetCapsIss
(
    IsiSensorHandle_t handle,
    IsiSensorCaps_t   *pIsiSensorCaps
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    result = Sensor_IsiGetCapsIssInternal(pIsiSensorCaps, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes);
    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_g_IsiSensorDefaultConfig
 *
 * @brief   recommended default configuration for application use via call
 *          to IsiGetSensorIss()
 *
 *****************************************************************************/
const IsiSensorCaps_t Sensor_g_IsiSensorDefaultConfig =
{
    ISI_BUSWIDTH_10BIT,         // BusWidth
    ISI_MODE_MIPI,              // MIPI
    ISI_FIELDSEL_BOTH,          // FieldSel
    ISI_YCSEQ_YCBYCR,           // YCSeq
    ISI_CONV422_NOCOSITED,      // Conv422
    ISI_BPAT_RGRGGBGB,          // BPat
    ISI_HPOL_REFPOS,            // HPol
    ISI_VPOL_NEG,               // VPol
    ISI_EDGE_RISING,            // Edge
    ISI_BLS_OFF,                // Bls
    ISI_GAMMA_OFF,              // Gamma
    ISI_CCONV_OFF,              // CConv
    ISI_RES_2592_1944P30,          // Res
    ISI_DWNSZ_SUBSMPL,          // DwnSz
    ISI_BLC_AUTO,               // BLC
    ISI_AGC_OFF,                // AGC
    ISI_AWB_OFF,                // AWB
    ISI_AEC_OFF,                // AEC
    ISI_DPCC_OFF,               // DPCC
    ISI_CIEPROF_D65,            // CieProfile, this is also used as start profile for AWB (if not altered by menu settings)
    ISI_SMIA_OFF,               // SmiaMode
    ISI_MIPI_MODE_RAW_10,       // MipiMode
    ISI_AFPS_NOTSUPP,           // AfpsResolutions
    ISI_SENSOR_OUTPUT_MODE_RAW,
    0,
};

/*****************************************************************************/
/**
 *          Sensor_SetupOutputFormat
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
RESULT Sensor_SetupOutputFormat
(
    Sensor_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );

    /* bus-width */
    switch ( pConfig->BusWidth )        /* only ISI_BUSWIDTH_12BIT supported, no configuration needed here */
    {
        case ISI_BUSWIDTH_10BIT:
        {
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s%s: bus width not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: field selection not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: 422 conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* bayer-pattern */
    switch ( pConfig->BPat )            /* only ISI_BPAT_BGBGGRGR supported, no configuration needed */
    {
     case ISI_BPAT_BGBGGRGR:
	 case ISI_BPAT_RGRGGBGB:
	 case ISI_BPAT_GRGRBGBG: 
	 case ISI_BPAT_GBGBRGRG:     
        {
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s%s: bayer pattern not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: HPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            return ( RET_NOTSUPP );
        }
    }

    /* vertical polarity */
    switch ( pConfig->VPol )            /* only ISI_VPOL_NEG supported, no configuration needed */
    {
        case ISI_VPOL_NEG:
        {
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s%s: VPol not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s:  edge mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s:  gamma not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: color conversion not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: SMIA mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            TRACE( Sensor_ERROR, "%s%s: MIPI mode not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
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
            //TRACE( Sensor_ERROR, "%s%s: AFPS not supported\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"" );
            //return ( RET_NOTSUPP );
        }
    }

    TRACE( Sensor_INFO, "%s%s (exit)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_SetupOutputWindow
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
 //leepe
static RESULT Sensor_SetupOutputWindowInternal
(
    Sensor_Context_t        *pSensorCtx,
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

    TRACE( Sensor_INFO, "%s (enter)---pConfig->Resolution:%x\n", __FUNCTION__,pConfig->Resolution);

	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_ONE_LANE){
	    /* resolution */
	    switch ( pConfig->Resolution )
	    {
	        default:
	        {
	            TRACE( Sensor_ERROR, "%s: one lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
	}



	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
	    /* resolution */
	    switch ( pConfig->Resolution )
	    {
	    	
		  case ISI_RES_2592_1944P30:
		  case ISI_RES_2592_1944P25:
		  case ISI_RES_2592_1944P20:
		  case ISI_RES_2592_1944P15:
	        case ISI_RES_2592_1944P10:
		  case ISI_RES_2592_1944P7:
	        {
				
				if (set2Sensor == BOOL_TRUE) {
					if (res_no_chg == BOOL_FALSE) {
						
						if((result = IsiRegDefaultsApply((IsiSensorHandle_t)pSensorCtx, Sensor_g_twolane_resolution_2592_1944)) != RET_SUCCESS){
							result = RET_FAILURE;
							TRACE( Sensor_ERROR, "%s: failed to set two lane ISI_RES_2592_1944 \n", __FUNCTION__ );
			            }
					}
	
				}

	            usLineLengthPck = 1200;
				if (pConfig->Resolution == ISI_RES_2592_1944P30) {
	            				usFrameLengthLines = 1800; 
				}
				if (pConfig->Resolution == ISI_RES_2592_1944P25) {
	            				usFrameLengthLines = 2400; 
				}
				if (pConfig->Resolution == ISI_RES_2592_1944P20) {
	            				usFrameLengthLines = 3000; 
				}
				if (pConfig->Resolution == ISI_RES_2592_1944P15) {
	            				usFrameLengthLines = 3600; 
				}
				if(pConfig->Resolution == ISI_RES_2592_1944P10) {
	            			usFrameLengthLines = 6000; 
				}
				if(pConfig->Resolution == ISI_RES_2592_1944P7) {
	            			usFrameLengthLines = 7800; 
				}
				
				pSensorCtx->IsiSensorMipiInfo.ulMipiFreq = 864;
	            break;
	            
	        }

	        default:
	        {
	            TRACE( Sensor_ERROR, "%s: two lane Resolution not supported\n", __FUNCTION__ );
	            return ( RET_NOTSUPP );
	        }
	    }
		
	}


	// store frame timing for later use in AEC module
	//rVtPixClkFreq = Sensor_get_PCLK(pSensorCtx, xclk); 

    rVtPixClkFreq = 72000000;
    pSensorCtx->VtPixClkFreq     = rVtPixClkFreq;
    pSensorCtx->LineLengthPck    = usLineLengthPck;
    pSensorCtx->FrameLengthLines = usFrameLengthLines;	
	//pSensorCtx->AecMaxIntegrationTime = ( ((float)pSensorCtx->FrameLengthLines) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;
	pSensorCtx->AecMaxIntegrationTime = ( ((float)pSensorCtx->FrameLengthLines-2) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;

	//have to reset mipi freq here,zyc
    TRACE( Sensor_INFO, "%s AecMaxIntegrationTime:%f(****************exit): Resolution %dx%d@%dfps  MIPI %dlanes  res_no_chg: %d   rVtPixClkFreq: %f\n", __FUNCTION__,
    					pSensorCtx->AecMaxIntegrationTime,
                        ISI_RES_W_GET(pConfig->Resolution),ISI_RES_H_GET(pConfig->Resolution),
                        ISI_FPS_GET(pConfig->Resolution),
                        pSensorCtx->IsiSensorMipiInfo.ucMipiLanes,
                        res_no_chg,rVtPixClkFreq);


    return ( result );
}




/*****************************************************************************/
/**
 *          Sensor_SetupImageControl
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
RESULT Sensor_SetupImageControl
(
    Sensor_Context_t        *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0U;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    switch ( pConfig->Bls )      /* only ISI_BLS_OFF supported, no configuration needed */
    {
        case ISI_BLS_OFF:
        {
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: Black level not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* black level compensation */
    switch ( pConfig->BLC )
    {
        case ISI_BLC_OFF:
        {
            /* turn off black level correction (clear bit 0) */
            //result = Sensor_IsiRegReadIss(  pSensorCtx, Sensor_BLC_CTRL00, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, Sensor_BLC_CTRL00, RegValue & 0x7F);
            break;
        }

        case ISI_BLC_AUTO:
        {
            /* turn on black level correction (set bit 0)
             * (0x331E[7] is assumed to be already setup to 'auto' by static configration) */
            //result = Sensor_IsiRegReadIss(  pSensorCtx, Sensor_BLC_CTRL00, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, Sensor_BLC_CTRL00, RegValue | 0x80 );
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: BLC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic gain control */
    switch ( pConfig->AGC )
    {
        case ISI_AGC_OFF:
        {
            // manual gain (appropriate for AEC with Marvin)
            //result = Sensor_IsiRegReadIss(  pSensorCtx, Sensor_AEC_MANUAL, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, Sensor_AEC_MANUAL, RegValue | 0x02 );
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: AGC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    /* automatic white balance */
    switch( pConfig->AWB )
    {
        case ISI_AWB_OFF:
        {
            //result = Sensor_IsiRegReadIss(  pSensorCtx, Sensor_ISP_CTRL01, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, Sensor_ISP_CTRL01, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: AWB not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }

    switch( pConfig->AEC )
    {
        case ISI_AEC_OFF:
        {
            //result = Sensor_IsiRegReadIss(  pSensorCtx, Sensor_AEC_MANUAL, &RegValue );
            //result = Sensor_IsiRegWriteIss( pSensorCtx, Sensor_AEC_MANUAL, RegValue | 0x01 );
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: AEC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }


    switch( pConfig->DPCC )
    {
        case ISI_DPCC_OFF:
        {
            
            break;
        }

        default:
        {
            TRACE( Sensor_ERROR, "%s: DPCC not supported\n", __FUNCTION__ );
            return ( RET_NOTSUPP );
        }
    }// I have not update this commented part yet, as I did not find DPCC setting in the current 8810 driver of Trillian board. - SRJ

    return ( result );
}
static RESULT Sensor_SetupOutputWindow
(
    Sensor_Context_t        *pSensorCtx,
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

    return Sensor_SetupOutputWindowInternal(pSensorCtx,pConfig,BOOL_TRUE, BOOL_FALSE);
}


/*****************************************************************************/
/**
 *          Sensor_AecSetModeParameters
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
static RESULT Sensor_AecSetModeParameters
(
    Sensor_Context_t       *pSensorCtx,
    const IsiSensorConfig_t *pConfig
)
{
    RESULT result = RET_SUCCESS;

    //TRACE( Sensor_INFO, "%s%s (enter)\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"");
    TRACE( Sensor_INFO, "%s%s (enter)  Res: 0x%x  0x%x\n", __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
        pSensorCtx->Config.Resolution, pConfig->Resolution);

    if ( (fabsf(pSensorCtx->VtPixClkFreq) <= FLOAT_EPS) )
    {
        TRACE( Sensor_ERROR, "%s%s: Division by zero!\n", __FUNCTION__  );
        return ( RET_OUTOFRANGE );
    }

    //as of mail from Omnivision FAE the limit is VTS - 6 (above that we observed a frame
    //exposed way too dark from time to time)
   
    //pSensorCtx->AecMaxIntegrationTime = ( ((float)pSensorCtx->FrameLengthLines) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecMaxIntegrationTime = ( ((float)pSensorCtx->FrameLengthLines-2) * ((float)pSensorCtx->LineLengthPck) ) / pSensorCtx->VtPixClkFreq;

    pSensorCtx->AecMinIntegrationTime = 0.0001f;    

    pSensorCtx->AecMaxGain = Sensor_MAX_GAIN_AEC;
    pSensorCtx->AecMinGain = 1.0f; //as of sensor datasheet 32/(32-6)

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    pSensorCtx->AecIntegrationTimeIncrement = ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;
    pSensorCtx->AecGainIncrement = Sensor_MIN_GAIN_STEP;

    //reflects the state of the sensor registers, must equal default settings
    pSensorCtx->AecCurGain               = pSensorCtx->AecMinGain;
    pSensorCtx->AecCurIntegrationTime    = 0.0f;
    pSensorCtx->OldCoarseIntegrationTime = 0;
    pSensorCtx->OldFineIntegrationTime   = 0;
    //pSensorCtx->GroupHold                = true; //must be true (for unknown reason) to correctly set gain the first time

    TRACE( Sensor_INFO, "%s%s (exit)\n pSensorCtx->AecMaxIntegrationTime:%f\n pSensorCtx->FrameLengthLines:%d\n pSensorCtx->LineLengthPck:%d\n pSensorCtx->VtPixClkFreq:%f\n",
    __FUNCTION__, pSensorCtx->isAfpsRun?"(AFPS)":"",
    pSensorCtx->AecMaxIntegrationTime,
    pSensorCtx->FrameLengthLines,
    pSensorCtx->LineLengthPck,
    pSensorCtx->VtPixClkFreq
    );

    return ( result );
}

/**
 *          Sensor_IsiSetupSensorIss
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
static RESULT Sensor_IsiSetupSensorIss
(
    IsiSensorHandle_t       handle,
    const IsiSensorConfig_t *pConfig
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pConfig == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid configuration (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    if ( pSensorCtx->Streaming != BOOL_FALSE )
    {
        return RET_WRONG_STATE;
    }

    MEMCPY( &pSensorCtx->Config, pConfig, sizeof( IsiSensorConfig_t ) );

    /* 1.) SW reset of image sensor (via I2C register interface)  be careful, bits 6..0 are reserved, reset bit is not sticky */

		result = Sensor_IsiRegWriteIss ( pSensorCtx, Sensor_SOFTWARE_RST, 0x80U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    osSleep( 10 );


    // disable streaming during sensor setup
    // (this seems not to be necessary, however Omnivision is doing it in their
    // reference settings, simply overwrite upper bits since setup takes care
    // of 'em later on anyway)

   
    /* 2.) write default values derived from datasheet and evaluation kit (static setup altered by dynamic setup further below) */
	if(pSensorCtx->IsiSensorMipiInfo.ucMipiLanes == SUPPORT_MIPI_TWO_LANE){
        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_twolane);
	 if(DR_State)        result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_twolane_2);
        else			result = IsiRegDefaultsApply( pSensorCtx, Sensor_g_aRegDescription_twolane_3);
		
	}else{
		TRACE( Sensor_ERROR, "%s: not this lane number supported\n", __FUNCTION__ );	
		return RET_WRONG_STATE;
	}
    
    if ( result != RET_SUCCESS )
    {
        return ( result );
    }

    /* sleep a while, that sensor can take over new default values */
    osSleep( 10 );

    /* 3.) verify default values to make sure everything has been written correctly as expected */

    /* 4.) setup output format (RAW10|RAW12) */
    result = Sensor_SetupOutputFormat( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: SetupOutputFormat failed.\n", __FUNCTION__);
        return ( result );
    }

    /* 5.) setup output window */
    result = Sensor_SetupOutputWindow( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
        return ( result );
    }

    result = Sensor_SetupImageControl( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: SetupImageControl failed.\n", __FUNCTION__);
        return ( result );
    }

    result = Sensor_AecSetModeParameters( pSensorCtx, pConfig );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
        return ( result );
    }
    if (result == RET_SUCCESS)
    {
        pSensorCtx->Configured = BOOL_TRUE;
    }

    if(bOTP_switch){
        apply_otp(pSensorCtx);
    }

    TRACE( Sensor_ERROR, "%s: (exit)INIT OK!!!!!!!!!!!!!!!!!\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiChangeSensorResolutionIss
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
static RESULT Sensor_IsiChangeSensorResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    uint8_t             *pNumberOfFramesToSkip
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    //TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);
    TRACE( Sensor_ERROR, "%s (enter)  Resolution: %dx%d@%dfps\n", __FUNCTION__,
        ISI_RES_W_GET(Resolution),ISI_RES_H_GET(Resolution), ISI_FPS_GET(Resolution));

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
    while (Sensor_IsiGetCapsIss( handle, &Caps) == RET_SUCCESS) {
        if (Resolution == Caps.Resolution) {            
            break;
        }
        Caps.Index++;
    }

     if (Resolution != Caps.Resolution) {
        return RET_OUTOFRANGE;
    }
	//TRACE( Sensor_ERROR, "%s (11111111enter)  \n", __FUNCTION__);
    if ( Resolution == pSensorCtx->Config.Resolution )
    {
        // well, no need to worry
        *pNumberOfFramesToSkip = 0;
		TRACE( Sensor_ERROR, "%s (Resolution:%d;pSensorCtx->Config.Resolution:%d)  \n", __FUNCTION__,Resolution,pSensorCtx->Config.Resolution);
    }
    else
    {
        // change resolution
        char *szResName = NULL;

		bool_t res_no_chg;
		
        if (!((ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution)) && 
            (ISI_RES_W_GET(Resolution)==ISI_RES_W_GET(pSensorCtx->Config.Resolution))) ) {

            if (pSensorCtx->Streaming != BOOL_FALSE) {
                TRACE( Sensor_ERROR, "%s: Sensor is streaming, Change resolution is not allow\n",__FUNCTION__);
                return RET_WRONG_STATE;
            }
            res_no_chg = BOOL_FALSE;
        } else {
            res_no_chg = BOOL_TRUE;
        }
		
        result = IsiGetResolutionName( Resolution, &szResName );
        TRACE( Sensor_INFO, "%s: NewRes=0x%08x (%s)\n", __FUNCTION__, Resolution, szResName);

       
        pSensorCtx->Config.Resolution = Resolution;

        
        result = Sensor_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config, BOOL_TRUE, res_no_chg );
        if ( result != RET_SUCCESS )
        {
            TRACE( Sensor_ERROR, "%s: SetupOutputWindow failed.\n", __FUNCTION__);
            return ( result );
        }

        // remember old exposure values
        float OldGain = pSensorCtx->AecCurGain;
        float OldIntegrationTime = pSensorCtx->AecCurIntegrationTime;

        // update limits & stuff (reset current & old settings)
        result = Sensor_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
        if ( result != RET_SUCCESS )
        {
            TRACE( Sensor_ERROR, "%s: AecSetModeParameters failed.\n", __FUNCTION__);
            return ( result );
        }

        // restore old exposure values (at least within new exposure values' limits)
        uint8_t NumberOfFramesToSkip = 0;
        float   DummySetGain = 0.0;
        float   DummySetIntegrationTime =0.0;
        result = Sensor_IsiExposureControlIss( handle, OldGain, OldIntegrationTime, &NumberOfFramesToSkip, &DummySetGain, &DummySetIntegrationTime );
        if ( result != RET_SUCCESS )
        {
            TRACE( Sensor_ERROR, "%s: Sensor_IsiExposureControlIss failed.\n", __FUNCTION__);
            return ( result );
        }

        // return number of frames that aren't exposed correctly
        *pNumberOfFramesToSkip = NumberOfFramesToSkip + 1;
        //	*pNumberOfFramesToSkip = 0;
    }

    TRACE( Sensor_INFO, "%s (exit)  result: 0x%x\n", __FUNCTION__, result);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSensorSetStreamingIss
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
 //leepe 
static RESULT Sensor_IsiSensorSetStreamingIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    uint32_t RegValue = 0;
	uint32_t RegValue2 = 0;

    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s (enter)  on = %d\n", __FUNCTION__,on);
    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSensorCtx->Configured != BOOL_TRUE) || (pSensorCtx->Streaming == on) )
    {
        return RET_WRONG_STATE;
    }

    if (result == RET_SUCCESS)
    {
        pSensorCtx->Streaming = on;
    }

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSensorSetPowerIss
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
static RESULT Sensor_IsiSensorSetPowerIss
(
    IsiSensorHandle_t   handle,
    bool_t              on
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    pSensorCtx->Configured = BOOL_FALSE;
    pSensorCtx->Streaming  = BOOL_FALSE;

    result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    if (on == BOOL_TRUE)
    {
        result = HalSetPower( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, true );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        osSleep( 10 );

        result = HalSetReset( pSensorCtx->IsiCtx.HalHandle, pSensorCtx->IsiCtx.HalDevID, false );

        osSleep( 50 );
    }

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiCheckSensorConnectionIss
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
static RESULT Sensor_IsiCheckSensorConnectionIss
(
    IsiSensorHandle_t   handle
)
{
    uint32_t RevId;
    uint32_t value;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    RevId = Sensor_CHIP_ID_HIGH_BYTE_DEFAULT;
    RevId = (RevId << 8U) | (Sensor_CHIP_ID_LOW_BYTE_DEFAULT);

    result = Sensor_IsiGetSensorRevisionIss( handle, &value );

    if ( (result != RET_SUCCESS) || (RevId != value) )
    {
        TRACE( Sensor_ERROR, "%s RevId = 0x%08x, value = 0x%08x \n", __FUNCTION__, RevId, value );
        return ( RET_FAILURE );
    }


    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorRevisionIss
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
static RESULT Sensor_IsiGetSensorRevisionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

    uint32_t data;
	uint32_t vcm_pos = MAX_LOG;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( handle == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( p_value == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *p_value = 0U;
    result = Sensor_IsiRegReadIss ( handle, Sensor_CHIP_ID_HIGH_BYTE, &data );
    *p_value = ( (data & 0xFF) << 8U );
    result = Sensor_IsiRegReadIss ( handle, Sensor_CHIP_ID_LOW_BYTE, &data );
    *p_value |= ( (data & 0xFF) );
    uint32_t flag_doublereset = 0,flag_GC5025A = 0;

    result = Sensor_IsiRegReadIss ( handle, 0x26, &flag_doublereset );
    result = Sensor_IsiRegReadIss ( handle, 0x27, &flag_GC5025A );

	if((flag_GC5025A & 0x01) == 0x01) {
		TRACE( Sensor_ERROR, "%s GC5025A sensor!\n", __FUNCTION__);
		DR_State = false;
    }
	else { 
		if((flag_doublereset & 0x03)==0x01) {
			DR_State = false;
			TRACE( Sensor_ERROR, "%s GC5025 sensor double reset off\n", __FUNCTION__);
		} else {
			DR_State = true;
			TRACE( Sensor_ERROR, "%s GC5025 sensor double reset on\n", __FUNCTION__);
		}
    }	

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiRegReadIss
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
static RESULT Sensor_IsiRegReadIss
(
    IsiSensorHandle_t   handle,
    const uint32_t      address,
    uint32_t            *p_value
)
{
    RESULT result = RET_SUCCESS;

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
        uint8_t NrOfBytes = IsiGetNrDatBytesIss( address, Sensor_g_aRegDescription_twolane);
        if ( !NrOfBytes )
        {
            NrOfBytes = 1;
        }

        *p_value = 0;
        result = IsiI2cReadSensorRegister( handle, address, (uint8_t *)p_value, NrOfBytes, BOOL_TRUE );
    }


    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiRegWriteIss
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
static RESULT Sensor_IsiRegWriteIss
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

    NrOfBytes = IsiGetNrDatBytesIss( address, Sensor_g_aRegDescription_twolane);
    if ( !NrOfBytes )
    {
        NrOfBytes = 1;
    }

    result = IsiI2cWriteSensorRegister( handle, address, (uint8_t *)(&value), NrOfBytes, BOOL_TRUE );

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetGainLimitsIss
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
static RESULT Sensor_IsiGetGainLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinGain,
    float               *pMaxGain
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinGain == NULL) || (pMaxGain == NULL) )
    {
        TRACE( Sensor_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinGain = pSensorCtx->AecMinGain;
    *pMaxGain = pSensorCtx->AecMaxGain;

    TRACE( Sensor_INFO, "%s: pMinGain:%f,pMaxGain:%f(exit)\n", __FUNCTION__,pSensorCtx->AecMinGain,pSensorCtx->AecMaxGain);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeLimitsIss
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
static RESULT Sensor_IsiGetIntegrationTimeLimitsIss
(
    IsiSensorHandle_t   handle,
    float               *pMinIntegrationTime,
    float               *pMaxIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( Sensor_INFO, "%s: (------oyyf enter) \n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pMinIntegrationTime == NULL) || (pMaxIntegrationTime == NULL) )
    {
        TRACE( Sensor_ERROR, "%s: NULL pointer received!!\n" );
        return ( RET_NULL_POINTER );
    }

    *pMinIntegrationTime = pSensorCtx->AecMinIntegrationTime;
    *pMaxIntegrationTime = pSensorCtx->AecMaxIntegrationTime;

    TRACE( Sensor_INFO, "%s: (------oyyf exit) (\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetGainIss
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
RESULT Sensor_IsiGetGainIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain = pSensorCtx->AecCurGain;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetGainIncrementIss
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
RESULT Sensor_IsiGetGainIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL)
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecGainIncrement;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSetGainIss
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
 #define ANALOG_GAIN_1 64   // 1.000x
 #define ANALOG_GAIN_2 92   // 1.445x
 
 //leepe gain
RESULT Sensor_IsiSetGainIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               *pSetGain
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint16_t usGain = 0;
    uint16_t iReg = 0;
    uint16_t temp = 0;
    uint16_t percentGain = 0;
    uint32_t reg_h = 0;
    uint32_t reg_l = 0;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);
    
    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetGain == NULL)
    {
        TRACE( Sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

  
    if( NewGain < pSensorCtx->AecMinGain ) NewGain = pSensorCtx->AecMinGain;
    if( NewGain > pSensorCtx->AecMaxGain ) NewGain = pSensorCtx->AecMaxGain;

	
	 usGain = (uint16_t)(NewGain * 64.0f+0.5);
   iReg = usGain;
   if(iReg < 0x40)
	 		iReg = 0x40;
	if((ANALOG_GAIN_1<= iReg)&&(iReg < ANALOG_GAIN_2))
	{
		Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);          
		
		//analog gain
		Sensor_IsiRegWriteIss(pSensorCtx,0xb6,  0x00);// 
		temp = iReg;
		Sensor_IsiRegWriteIss(pSensorCtx,0xb1, temp>>6);
		Sensor_IsiRegWriteIss(pSensorCtx,0xb2, (temp<<2)&0xfc);
	}
	else 
	{                                    
		Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);
		//analog gain
		Sensor_IsiRegWriteIss(pSensorCtx,0xb6,  0x01);// 
		temp = 64*iReg/ANALOG_GAIN_2;
		Sensor_IsiRegWriteIss(pSensorCtx,0xb1, temp>>6);
		Sensor_IsiRegWriteIss(pSensorCtx,0xb2, (temp<<2)&0xfc);
	}
	osSleep(20);
	Sensor_IsiRegReadIss(pSensorCtx,0xb1, &reg_h);
	Sensor_IsiRegReadIss(pSensorCtx,0xb2, &reg_l);
	TRACE( Sensor_ERROR, "%s: write 0xb1=%d,0xb2=%d, read 0xb1=%d,0xb2=%d\n", __FUNCTION__,temp>>6,(temp<<2)&0xfc,reg_h,reg_l);
	
	          
	 pSensorCtx->AecCurGain = ( (float)usGain ) / 64.0f;
    //return current state
    *pSetGain = pSensorCtx->AecCurGain;

    TRACE( Sensor_ERROR, "lrk %s: setgain mubiao(%f) shiji(%f)\n", __FUNCTION__, NewGain, *pSetGain);

    return ( result );
}

     

/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeIss
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
RESULT Sensor_IsiGetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetIntegrationTime == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetIntegrationTimeIncrementIss
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
RESULT Sensor_IsiGetIntegrationTimeIncrementIss
(
    IsiSensorHandle_t   handle,
    float               *pIncr
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (-------oyyf)(enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pIncr == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //_smallest_ increment the sensor/driver can handle (e.g. used for sliders in the application)
    *pIncr = pSensorCtx->AecIntegrationTimeIncrement;

    TRACE( Sensor_INFO, "%s: (------oyyf)(exit) pSensorCtx->AecIntegrationTimeIncrement(%f)\n", __FUNCTION__,pSensorCtx->AecIntegrationTimeIncrement);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiSetIntegrationTimeIss
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
RESULT Sensor_IsiSetIntegrationTimeIss
(
    IsiSensorHandle_t   handle,
    float               NewIntegrationTime,
    float               *pSetIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t CoarseIntegrationTime = 0;
		uint32_t result_intertime= 0;
    float ShutterWidthPck = 0.0f; //shutter width in pixel clock periods
       uint32_t cal_Time = 0; 
    uint32_t reg_h = 0;
    uint32_t reg_l = 0;

    TRACE( Sensor_INFO, "%s: (enter) NewIntegrationTime: %f (min: %f   max: %f)\n", __FUNCTION__,
        NewIntegrationTime,
        pSensorCtx->AecMinIntegrationTime,
        pSensorCtx->AecMaxIntegrationTime);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetIntegrationTime == NULL) || (pNumberOfFramesToSkip == NULL) )
    {
        TRACE( Sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }


   if ( NewIntegrationTime > pSensorCtx->AecMaxIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMaxIntegrationTime;
    if ( NewIntegrationTime < pSensorCtx->AecMinIntegrationTime ) NewIntegrationTime = pSensorCtx->AecMinIntegrationTime;



    ShutterWidthPck = NewIntegrationTime * ( (float)pSensorCtx->VtPixClkFreq );

    // avoid division by zero
    if ( pSensorCtx->LineLengthPck == 0 )
    {
        TRACE( Sensor_ERROR, "%s: Division by zero!\n", __FUNCTION__ );
        return ( RET_DIVISION_BY_ZERO );
    }

	TRACE( Sensor_ERROR, "%s: ethan NewIntegrationTime = %f!\n", __FUNCTION__ ,NewIntegrationTime);
    CoarseIntegrationTime = (uint32_t)( ShutterWidthPck / ((float)pSensorCtx->LineLengthPck) + 0.5f );
	TRACE( Sensor_ERROR, "%s: ethan CoarseIntegrationTime = %d!\n", __FUNCTION__,CoarseIntegrationTime );
    // write new integration time into sensor registers
    // do not write if nothing has changed
    if(CoarseIntegrationTime < 4) CoarseIntegrationTime = 4;
	
   TRACE( Sensor_DEBUG, "%s:  CoarseIntegrationTime = %d!\n", __FUNCTION__,CoarseIntegrationTime );

    cal_Time = CoarseIntegrationTime/2;
    cal_Time = cal_Time*2;
    Dgain_ratio = 256 * CoarseIntegrationTime/cal_Time;

   TRACE( Sensor_DEBUG, "%s: Dgain_ratio= %d!\n", __FUNCTION__,Dgain_ratio );
    if( CoarseIntegrationTime != pSensorCtx->OldCoarseIntegrationTime )
    {
   #if 0	
			Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);
			Sensor_IsiRegWriteIss(pSensorCtx,0x03, (CoarseIntegrationTime>>8) & 0x3F);
			Sensor_IsiRegWriteIss(pSensorCtx,0x04, CoarseIntegrationTime & 0xFF);
   #else	   
	if(!DR_State)
	{

	    if(cal_Time <= 10)
  	    {
		    Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);
		    Sensor_IsiRegWriteIss(pSensorCtx,0xd9, 0xdd);
  	    }
  	    else 
	    {
		    Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);
		    Sensor_IsiRegWriteIss(pSensorCtx,0xd9, 0xaa);		
  	    }  
       }	

	Sensor_IsiRegWriteIss(pSensorCtx,0xfe, 0x00);
	Sensor_IsiRegWriteIss(pSensorCtx,0x03, (cal_Time>>8) & 0x3F);
	Sensor_IsiRegWriteIss(pSensorCtx,0x04, cal_Time & 0xFF);
   #endif	
	
      pSensorCtx->OldCoarseIntegrationTime = CoarseIntegrationTime;   // remember current integration time
      *pNumberOfFramesToSkip = 1U; //skip 1 frame
    }
    else
    {
        *pNumberOfFramesToSkip = 0U; //no frame skip
    }
		
		Sensor_IsiRegReadIss(pSensorCtx,0x03, &reg_h);
		Sensor_IsiRegReadIss(pSensorCtx,0x04, &reg_l);
		TRACE( Sensor_ERROR, "%s: CoarseIntegrationTime set to reg = %d, CoarseIntegrationTime read from reg = %d!\n", __FUNCTION__ ,CoarseIntegrationTime,(reg_h<<8)+reg_l);
		
    pSensorCtx->AecCurIntegrationTime = ((float)CoarseIntegrationTime) * ((float)pSensorCtx->LineLengthPck) / pSensorCtx->VtPixClkFreq;

    //return current state
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;


    return ( result );
}




/*****************************************************************************/
/**
 *          Sensor_IsiExposureControlIss
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
RESULT Sensor_IsiExposureControlIss
(
    IsiSensorHandle_t   handle,
    float               NewGain,
    float               NewIntegrationTime,
    uint8_t             *pNumberOfFramesToSkip,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_DEBUG, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pNumberOfFramesToSkip == NULL)
            || (pSetGain == NULL)
            || (pSetIntegrationTime == NULL) )
    {
        TRACE( Sensor_ERROR, "%s: Invalid parameter (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_NULL_POINTER );
    }

    TRACE( Sensor_DEBUG, "%s: g=%f, Ti=%f\n", __FUNCTION__, NewGain, NewIntegrationTime );


    result = Sensor_IsiSetIntegrationTimeIss( handle, NewIntegrationTime, pSetIntegrationTime, pNumberOfFramesToSkip );
    result = Sensor_IsiSetGainIss( handle, NewGain, pSetGain );

    TRACE( Sensor_DEBUG, "%s: set: g=%f, Ti=%f, skip=%d\n", __FUNCTION__, *pSetGain, *pSetIntegrationTime, *pNumberOfFramesToSkip );
    TRACE( Sensor_DEBUG, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCurrentExposureIss
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
RESULT Sensor_IsiGetCurrentExposureIss
(
    IsiSensorHandle_t   handle,
    float               *pSetGain,
    float               *pSetIntegrationTime
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( (pSetGain == NULL) || (pSetIntegrationTime == NULL) )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetGain            = pSensorCtx->AecCurGain;
    *pSetIntegrationTime = pSensorCtx->AecCurIntegrationTime;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetResolutionIss
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
RESULT Sensor_IsiGetResolutionIss
(
    IsiSensorHandle_t   handle,
    uint32_t            *pSetResolution
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
        return ( RET_WRONG_HANDLE );
    }

    if ( pSetResolution == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    *pSetResolution = pSensorCtx->Config.Resolution;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetAfpsInfoHelperIss
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
static RESULT Sensor_IsiGetAfpsInfoHelperIss(
    Sensor_Context_t   *pSensorCtx,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo,
    uint32_t            AfpsStageIdx
)
{
    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (-----------oyyf enter) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    DCT_ASSERT(pSensorCtx != NULL);
    DCT_ASSERT(pAfpsInfo != NULL);
    DCT_ASSERT(AfpsStageIdx <= ISI_NUM_AFPS_STAGES);

    // update resolution in copy of config in context
    pSensorCtx->Config.Resolution = Resolution;

    // tell sensor about that
    result = Sensor_SetupOutputWindowInternal( pSensorCtx, &pSensorCtx->Config,BOOL_FALSE,BOOL_FALSE );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: SetupOutputWindow failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // update limits & stuff (reset current & old settings)
    result = Sensor_AecSetModeParameters( pSensorCtx, &pSensorCtx->Config );
    if ( result != RET_SUCCESS )
    {
        TRACE( Sensor_ERROR, "%s: AecSetModeParameters failed for resolution ID %08x.\n", __FUNCTION__, Resolution);
        return ( result );
    }

    // take over params
    pAfpsInfo->Stage[AfpsStageIdx].Resolution = Resolution;
    pAfpsInfo->Stage[AfpsStageIdx].MaxIntTime = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecMinGain           = pSensorCtx->AecMinGain;
    pAfpsInfo->AecMaxGain           = pSensorCtx->AecMaxGain;
    pAfpsInfo->AecMinIntTime        = pSensorCtx->AecMinIntegrationTime;
    pAfpsInfo->AecMaxIntTime        = pSensorCtx->AecMaxIntegrationTime;
    pAfpsInfo->AecSlowestResolution = Resolution;

    TRACE( Sensor_INFO, "%s: (-----------oyyf exit) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiGetAfpsInfoIss
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
RESULT Sensor_IsiGetAfpsInfoIss(
    IsiSensorHandle_t   handle,
    uint32_t            Resolution,
    IsiAfpsInfo_t*      pAfpsInfo
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    uint32_t RegValue = 0;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        TRACE( Sensor_ERROR, "%s: Invalid sensor handle (NULL pointer detected)\n", __FUNCTION__ );
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
	TRACE( Sensor_INFO, "#%s: (-----------oyyf) pAfpsInfo->AecMaxIntTime(%f) pSensorCtx->AecMaxIntegrationTime(%f)\n", __FUNCTION__, pAfpsInfo->AecMaxIntTime,pSensorCtx->AecMaxIntegrationTime);

    // allocate dummy context used for Afps parameter calculation as a copy of current context
    Sensor_Context_t *pDummyCtx = (Sensor_Context_t*) malloc( sizeof(Sensor_Context_t) );
    if ( pDummyCtx == NULL )
    {
        TRACE( Sensor_ERROR,  "%s: Can't allocate dummy ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    *pDummyCtx = *pSensorCtx;

    // set AFPS mode in dummy context
    pDummyCtx->isAfpsRun = BOOL_TRUE;

#define AFPSCHECKANDADD(_res_) \
    { \
        RESULT lres = Sensor_IsiGetAfpsInfoHelperIss( pDummyCtx, _res_, pAfpsInfo, idx ); \
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
	switch (pSensorCtx->IsiSensorMipiInfo.ucMipiLanes)
		{
			case SUPPORT_MIPI_ONE_LANE:
			{
	
				break;
			}
	
			case SUPPORT_MIPI_TWO_LANE:
			{

				switch(Resolution)
			    {
			        
					case ISI_RES_2592_1944P30:
					case ISI_RES_2592_1944P25:
					case ISI_RES_2592_1944P20:
					case ISI_RES_2592_1944P15:
					case ISI_RES_2592_1944P10:
					case ISI_RES_2592_1944P7:
						TRACE( Sensor_ERROR, "%s: preview_minimum_framerate(%d)\n", __FUNCTION__,pSensorCtx->preview_minimum_framerate);
						if(ISI_FPS_GET(ISI_RES_2592_1944P30) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P30);
						if(ISI_FPS_GET(ISI_RES_2592_1944P25) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P25);
						if(ISI_FPS_GET(ISI_RES_2592_1944P20) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P20);
						if(ISI_FPS_GET(ISI_RES_2592_1944P15) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P15);
						if(ISI_FPS_GET(ISI_RES_2592_1944P10) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P10);
						if(ISI_FPS_GET(ISI_RES_2592_1944P7) >= pSensorCtx->preview_minimum_framerate)
							AFPSCHECKANDADD( ISI_RES_2592_1944P7);
						break;
					default:
						break;
				}
				
            break;
        	}
			
			case SUPPORT_MIPI_FOUR_LANE:
			{
				break;
			}
			default:
            TRACE( Sensor_ERROR,  "%s: pSensorCtx->IsiSensorMipiInfo.ucMipiLanes(0x%x) is invalidate!\n", 
                __FUNCTION__, pSensorCtx->IsiSensorMipiInfo.ucMipiLanes );
            result = RET_FAILURE;
            break;
			        
	}

    // release dummy context again
    free(pDummyCtx);

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibKFactor
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
static RESULT Sensor_IsiGetCalibKFactor
(
    IsiSensorHandle_t   handle,
    Isi1x1FloatMatrix_t **pIsiKFactor
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiKFactor == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiKFactor = (Isi1x1FloatMatrix_t *)&Sensor_KFactor;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibPcaMatrix
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
static RESULT Sensor_IsiGetCalibPcaMatrix
(
    IsiSensorHandle_t   handle,
    Isi3x2FloatMatrix_t **pIsiPcaMatrix
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiPcaMatrix == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiPcaMatrix = (Isi3x2FloatMatrix_t *)&Sensor_PCAMatrix;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibSvdMeanValue
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
static RESULT Sensor_IsiGetCalibSvdMeanValue
(
    IsiSensorHandle_t   handle,
    Isi3x1FloatMatrix_t **pIsiSvdMeanValue
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiSvdMeanValue == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiSvdMeanValue = (Isi3x1FloatMatrix_t *)&Sensor_SVDMeanValue;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibSvdMeanValue
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
static RESULT Sensor_IsiGetCalibCenterLine
(
    IsiSensorHandle_t   handle,
    IsiLine_t           **ptIsiCenterLine
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiCenterLine == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiCenterLine = (IsiLine_t*)&Sensor_CenterLine;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibClipParam
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
static RESULT Sensor_IsiGetCalibClipParam
(
    IsiSensorHandle_t   handle,
    IsiAwbClipParm_t    **pIsiClipParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pIsiClipParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*pIsiClipParam = (IsiAwbClipParm_t *)&Sensor_AwbClipParm;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibGlobalFadeParam
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
static RESULT Sensor_IsiGetCalibGlobalFadeParam
(
    IsiSensorHandle_t       handle,
    IsiAwbGlobalFadeParm_t  **ptIsiGlobalFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiGlobalFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiGlobalFadeParam = (IsiAwbGlobalFadeParm_t *)&Sensor_AwbGlobalFadeParm;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetCalibFadeParam
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
static RESULT Sensor_IsiGetCalibFadeParam
(
    IsiSensorHandle_t   handle,
    IsiAwbFade2Parm_t   **ptIsiFadeParam
)
{
    IsiSensorContext_t *pSensorCtx = (IsiSensorContext_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( ptIsiFadeParam == NULL )
    {
        return ( RET_NULL_POINTER );
    }

    //*ptIsiFadeParam = (IsiAwbFade2Parm_t *)&Sensor_AwbFade2Parm;

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

/*****************************************************************************/
/**
 *          Sensor_IsiGetIlluProfile
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
static RESULT Sensor_IsiGetIlluProfile
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiIlluProfile_t    **ptIsiIlluProfile
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

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
    	#if 0
        uint16_t i;

        *ptIsiIlluProfile = NULL;

        /* check if we've a default profile */
        for ( i=0U; i<Sensor_ISIILLUPROFILES_DEFAULT; i++ )
        {
            if ( Sensor_IlluProfileDefault[i].id == CieProfile )
            {
                *ptIsiIlluProfile = &Sensor_IlluProfileDefault[i];
                break;
            }
        }

        result = ( *ptIsiIlluProfile != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetLscMatrixTable
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
static RESULT Sensor_IsiGetLscMatrixTable
(
    IsiSensorHandle_t   handle,
    const uint32_t      CieProfile,
    IsiLscMatrixTable_t **pLscMatrixTable
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

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
    	#if 0
        uint16_t i;


        switch ( CieProfile )
        {
            case ISI_CIEPROF_A:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_A_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_A_4416x3312;
                }
                #endif
                else
                {
                    TRACE( Sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F2:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F2_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F2_4416x3312;
                }
                #endif
                else
                {
                    TRACE( Sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D50:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D50_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D50_4416x3312;
                }
                #endif
                else
                {
                    TRACE( Sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_D65:
            case ISI_CIEPROF_D75:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ) )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D65_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_D65_4416x3312;
                }
                #endif
                else
                {
                    TRACE( Sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            case ISI_CIEPROF_F11:
            {
                if ( ( pSensorCtx->Config.Resolution == ISI_RES_TV1080P30 ))
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F11_1920x1080;
                }
                #if 0
                else if ( pSensorCtx->Config.Resolution == ISI_RES_4416_3312 )
                {
                    *pLscMatrixTable = &Sensor_LscMatrixTable_CIE_F11_4416x3312;
                }
                #endif
                else
                {
                    TRACE( Sensor_ERROR, "%s: Resolution (%08x) not supported\n", __FUNCTION__, CieProfile );
                    *pLscMatrixTable = NULL;
                }

                break;
            }

            default:
            {
                TRACE( Sensor_ERROR, "%s: Illumination not supported\n", __FUNCTION__ );
                *pLscMatrixTable = NULL;
                break;
            }
        }

        result = ( *pLscMatrixTable != NULL ) ?  RET_SUCCESS : RET_NOTAVAILABLE;
		#endif
    }

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}


/*****************************************************************************/
/**
 *          Sensor_IsiMdiInitMotoDriveMds
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
static RESULT Sensor_IsiMdiInitMotoDriveMds
(
    IsiSensorHandle_t   handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiSetupMotoDrive
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
static RESULT Sensor_IsiMdiSetupMotoDrive
(
    IsiSensorHandle_t   handle,
    uint32_t            *pMaxStep
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;
	uint32_t vcm_movefull_t;
    RESULT result = RET_SUCCESS;
#if defined(VCM_ID_CN3927E) 
	uint8_t  data[2] = { 0, 0 };
#elif defined(VCM_ID_DW9714)

#endif

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pMaxStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
#if defined(VCM_ID_CN3927E) 
	//1 protection off
	data[0]=0xEC;
	data[1]=0xA3;
	result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

	//2 DLC and MCLK[1:0] setting
	data[0]=0xA1;
	data[1]=0x05;
	result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

	//3 T_SRC[4:0] setting
	data[0]=0xF2;
	data[1]=0x00;
	result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );	

	//4 Protection on
	data[0]=0xDC;
	data[1]=0x51;
	result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
	RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
	usleep(500);
	if((pSensorCtx->VcmInfo.StepMode & 0x0c)!=0){
		vcm_movefull_t = 81*(1 << (pSensorCtx->VcmInfo.StepMode & 0x03))*1024/
		((1 << (((pSensorCtx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
	}else{
		vcm_movefull_t = 81*1023/1000;
	}

#elif defined(VCM_ID_DW9714)
 if ((pSensorCtx->VcmInfo.StepMode & 0x0c) != 0) {
 	vcm_movefull_t = 64* (1<<(pSensorCtx->VcmInfo.StepMode & 0x03)) *1024/((1 << (((pSensorCtx->VcmInfo.StepMode & 0x0c)>>2)-1))*1000);
 }else{
 	vcm_movefull_t =64*1023/1000;
   TRACE( Sensor_ERROR, "%s: (---NO SRC---)\n", __FUNCTION__);
 }
#endif 

	  *pMaxStep = (MAX_LOG|(vcm_movefull_t<<16));

    result = Sensor_IsiMdiFocusSet( handle, MAX_LOG );

    TRACE( Sensor_DEBUG, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusSet
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
static RESULT Sensor_IsiMdiFocusSet
(
    IsiSensorHandle_t   handle,
    const uint32_t      Position
)
{
	Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    
    uint32_t nPosition;
    uint8_t  data[2] = { 0, 0 };

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }
/* SYNNEX DEBUG*/
    #if 1
    /* map 64 to 0 -> infinity */
    //nPosition = ( Position >= MAX_LOG ) ? 0 : ( MAX_REG - (Position * 16U) );
	if( Position > MAX_LOG ){
		TRACE( Sensor_ERROR, "%s: pGC5025Ctx Position (%d) max_position(%d)\n", __FUNCTION__,Position, MAX_LOG);
		//Position = MAX_LOG;
	}	
    /* ddl@rock-chips.com: v0.3.0 */
    if ( Position >= MAX_LOG )
        nPosition = pSensorCtx->VcmInfo.StartCurrent;
	else if ( Position <= 0 )
        nPosition = pSensorCtx->VcmInfo.RatedCurrent;
    else 
        nPosition = pSensorCtx->VcmInfo.StartCurrent + (pSensorCtx->VcmInfo.Step*(MAX_LOG-Position));
    /* ddl@rock-chips.com: v0.6.0 */
    if (nPosition > MAX_VCMDRV_REG)  
        nPosition = MAX_VCMDRV_REG;

    TRACE( Sensor_INFO, "%s: focus set position_reg_value(%d) position(%d) \n", __FUNCTION__, nPosition, Position);
    data[0] = (uint8_t)(0x00U | (( nPosition & 0x3F0U ) >> 4U));                 // PD,  1, D9..D4, see AD5820 datasheet
    //data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | MDI_SLEW_RATE_CTRL );    // D3..D0, S3..S0
	data[1] = (uint8_t)( ((nPosition & 0x0FU) << 4U) | pSensorCtx->VcmInfo.StepMode );
	
    //TRACE( Sensor_ERROR, "%s: value = %d, 0x%02x 0x%02x\n", __FUNCTION__, nPosition, data[0], data[1] );

    result = HalWriteI2CMem( pSensorCtx->IsiCtx.HalHandle,
                             pSensorCtx->IsiCtx.I2cAfBusNum,
                             pSensorCtx->IsiCtx.SlaveAfAddress,
                             0,
                             pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                             data,
                             2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusGet
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
static RESULT Sensor_IsiMdiFocusGet
(
    IsiSensorHandle_t   handle,
    uint32_t            *pAbsStep
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;
    uint8_t  data[2] = { 0, 0 };

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( pAbsStep == NULL )
    {
        return ( RET_NULL_POINTER );
    }
    /* SYNNEX DEBUG */
    #if 1
    result = HalReadI2CMem( pSensorCtx->IsiCtx.HalHandle,
                            pSensorCtx->IsiCtx.I2cAfBusNum,
                            pSensorCtx->IsiCtx.SlaveAfAddress,
                            0,
                            pSensorCtx->IsiCtx.NrOfAfAddressBytes,
                            data,
                            2U );
    RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

    TRACE( Sensor_DEBUG, "%s:[SYNNEX_VAM_DEBUG] value = 0x%02x 0x%02x\n", __FUNCTION__, data[0], data[1] );

    /* Data[0] = PD,  1, D9..D4, see VM149C datasheet */
    /* Data[1] = D3..D0, S3..S0 */
    *pAbsStep = ( ((uint32_t)(data[0] & 0x3FU)) << 4U ) | ( ((uint32_t)data[1]) >> 4U );

	if( *pAbsStep <= pSensorCtx->VcmInfo.StartCurrent)
    {
        *pAbsStep = MAX_LOG;
    }
    else if((*pAbsStep>pSensorCtx->VcmInfo.StartCurrent) && (*pAbsStep<=pSensorCtx->VcmInfo.RatedCurrent))
    {
        *pAbsStep = (pSensorCtx->VcmInfo.RatedCurrent - *pAbsStep ) / pSensorCtx->VcmInfo.Step;
    }
	else
	{
		*pAbsStep = 0;
	}
    #endif
   TRACE( Sensor_DEBUG, "%s: (exit)\n", __FUNCTION__);
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiMdiFocusCalibrate
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
static RESULT Sensor_IsiMdiFocusCalibrate
(
    IsiSensorHandle_t   handle
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 1
    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);
    #endif
    
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiActivateTestPattern
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
static RESULT Sensor_IsiActivateTestPattern
(
    IsiSensorHandle_t   handle,
    const bool_t        enable
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    #if 0
    uint32_t ulRegValue = 0UL;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
        return ( RET_WRONG_HANDLE );
    }

    if ( BOOL_TRUE == enable )
    {
        /* enable test-pattern */
        result = Sensor_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue |= ( 0x80U );

        result = Sensor_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }
    else
    {
        /* disable test-pattern */
        result = Sensor_IsiRegReadIss( pSensorCtx, 0x5e00, &ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );

        ulRegValue &= ~( 0x80 );

        result = Sensor_IsiRegWriteIss( pSensorCtx, 0x5e00, ulRegValue );
        RETURN_RESULT_IF_DIFFERENT( RET_SUCCESS, result );
    }

     pSensorCtx->TestPattern = enable;
    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    #endif
    return ( result );
}



/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorMipiInfoIss
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
static RESULT Sensor_IsiGetSensorMipiInfoIss
(
    IsiSensorHandle_t   handle,
    IsiSensorMipiInfo   *ptIsiSensorMipiInfo
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

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


    TRACE( Sensor_INFO, "%s: (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT Sensor_IsiGetSensorIsiVersion
(  IsiSensorHandle_t   handle,
   unsigned int*     pVersion
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( Sensor_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pVersion == NULL)
	{
		TRACE( Sensor_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pVersion = CONFIG_ISI_VERSION;
	return result;
}

static RESULT Sensor_IsiGetSensorTuningXmlVersion
(  IsiSensorHandle_t   handle,
   char**     pTuningXmlVersion
)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;


    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL )
    {
    	TRACE( Sensor_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

	if(pTuningXmlVersion == NULL)
	{
		TRACE( Sensor_ERROR, "%s: pVersion IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
	}

	*pTuningXmlVersion = Sensor_NEWEST_TUNING_XML;
	return result;
}


/*****************************************************************************/
/**
 *          Sensor_IsiGetSensorIss
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
RESULT Sensor_IsiGetSensorIss
(
    IsiSensor_t *pIsiSensor
)
{
    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pIsiSensor != NULL )
    {
        pIsiSensor->pszName                             = Sensor_g_acName;
        pIsiSensor->pRegisterTable                      = Sensor_g_aRegDescription_twolane;
        pIsiSensor->pIsiSensorCaps                      = &Sensor_g_IsiSensorDefaultConfig;
        pIsiSensor->pIsiGetSensorIsiVer					= Sensor_IsiGetSensorIsiVersion;//oyyf
        pIsiSensor->pIsiGetSensorTuningXmlVersion		= Sensor_IsiGetSensorTuningXmlVersion;//oyyf
        pIsiSensor->pIsiCheckOTPInfo                    =  check_read_otp;//check_read_otp;//zyc
        pIsiSensor->pIsiSetSensorOTPInfo			= GC5025_IsiSetOTPInfo;
        pIsiSensor->pIsiEnableSensorOTP			= GC5025_IsiEnableOTP;
        pIsiSensor->pIsiCreateSensorIss                 = Sensor_IsiCreateSensorIss;
        pIsiSensor->pIsiReleaseSensorIss                = Sensor_IsiReleaseSensorIss;
        pIsiSensor->pIsiGetCapsIss                      = Sensor_IsiGetCapsIss;
        pIsiSensor->pIsiSetupSensorIss                  = Sensor_IsiSetupSensorIss;
        pIsiSensor->pIsiChangeSensorResolutionIss       = Sensor_IsiChangeSensorResolutionIss;
        pIsiSensor->pIsiSensorSetStreamingIss           = Sensor_IsiSensorSetStreamingIss;
        pIsiSensor->pIsiSensorSetPowerIss               = Sensor_IsiSensorSetPowerIss;
        pIsiSensor->pIsiCheckSensorConnectionIss        = Sensor_IsiCheckSensorConnectionIss;
        pIsiSensor->pIsiGetSensorRevisionIss            = Sensor_IsiGetSensorRevisionIss;
        pIsiSensor->pIsiRegisterReadIss                 = Sensor_IsiRegReadIss;
        pIsiSensor->pIsiRegisterWriteIss                = Sensor_IsiRegWriteIss;

        /* AEC functions */
        pIsiSensor->pIsiExposureControlIss              = Sensor_IsiExposureControlIss;
        pIsiSensor->pIsiGetGainLimitsIss                = Sensor_IsiGetGainLimitsIss;
        pIsiSensor->pIsiGetIntegrationTimeLimitsIss     = Sensor_IsiGetIntegrationTimeLimitsIss;
        pIsiSensor->pIsiGetCurrentExposureIss           = Sensor_IsiGetCurrentExposureIss;
        pIsiSensor->pIsiGetGainIss                      = Sensor_IsiGetGainIss;
        pIsiSensor->pIsiGetGainIncrementIss             = Sensor_IsiGetGainIncrementIss;
        pIsiSensor->pIsiSetGainIss                      = Sensor_IsiSetGainIss;
        pIsiSensor->pIsiGetIntegrationTimeIss           = Sensor_IsiGetIntegrationTimeIss;
        pIsiSensor->pIsiGetIntegrationTimeIncrementIss  = Sensor_IsiGetIntegrationTimeIncrementIss;
        pIsiSensor->pIsiSetIntegrationTimeIss           = Sensor_IsiSetIntegrationTimeIss;
        pIsiSensor->pIsiGetResolutionIss                = Sensor_IsiGetResolutionIss;
        pIsiSensor->pIsiGetAfpsInfoIss                  = Sensor_IsiGetAfpsInfoIss;

        /* AWB specific functions */
        pIsiSensor->pIsiGetCalibKFactor                 = Sensor_IsiGetCalibKFactor;
        pIsiSensor->pIsiGetCalibPcaMatrix               = Sensor_IsiGetCalibPcaMatrix;
        pIsiSensor->pIsiGetCalibSvdMeanValue            = Sensor_IsiGetCalibSvdMeanValue;
        pIsiSensor->pIsiGetCalibCenterLine              = Sensor_IsiGetCalibCenterLine;
        pIsiSensor->pIsiGetCalibClipParam               = Sensor_IsiGetCalibClipParam;
        pIsiSensor->pIsiGetCalibGlobalFadeParam         = Sensor_IsiGetCalibGlobalFadeParam;
        pIsiSensor->pIsiGetCalibFadeParam               = Sensor_IsiGetCalibFadeParam;
        pIsiSensor->pIsiGetIlluProfile                  = Sensor_IsiGetIlluProfile;
        pIsiSensor->pIsiGetLscMatrixTable               = Sensor_IsiGetLscMatrixTable;

        /* AF functions */
        pIsiSensor->pIsiMdiInitMotoDriveMds             = Sensor_IsiMdiInitMotoDriveMds;
        pIsiSensor->pIsiMdiSetupMotoDrive               = Sensor_IsiMdiSetupMotoDrive;
        pIsiSensor->pIsiMdiFocusSet                     = Sensor_IsiMdiFocusSet;
        pIsiSensor->pIsiMdiFocusGet                     = Sensor_IsiMdiFocusGet;
        pIsiSensor->pIsiMdiFocusCalibrate               = Sensor_IsiMdiFocusCalibrate;

        /* MIPI */
        pIsiSensor->pIsiGetSensorMipiInfoIss            = Sensor_IsiGetSensorMipiInfoIss;

        /* Testpattern */
        pIsiSensor->pIsiActivateTestPattern             = Sensor_IsiActivateTestPattern;
		/* Preview minimun framerate*/
		pIsiSensor->pIsiSetSensorFrameRateLimit          = GC5025_IsiSetSensorFrameRateLimit;
    }
    else
    {
        result = RET_NULL_POINTER;
    }

    TRACE( Sensor_INFO, "%s (exit)\n", __FUNCTION__);

    return ( result );
}

static RESULT Sensor_IsiGetSensorI2cInfo(sensor_i2c_info_t** pdata)
{
    sensor_i2c_info_t* pSensorI2cInfo = NULL;

    pSensorI2cInfo = ( sensor_i2c_info_t * )malloc ( sizeof (sensor_i2c_info_t) );

    if ( pSensorI2cInfo == NULL )
    {
        TRACE( Sensor_ERROR,  "%s: Can't allocate ov14825 context\n",  __FUNCTION__ );
        return ( RET_OUTOFMEM );
    }
    MEMSET( pSensorI2cInfo, 0, sizeof( sensor_i2c_info_t ) );

    
    pSensorI2cInfo->i2c_addr = Sensor_SLAVE_ADDR;
    pSensorI2cInfo->i2c_addr2 = Sensor_SLAVE_ADDR2;
    pSensorI2cInfo->soft_reg_addr = Sensor_SOFTWARE_RST;
    pSensorI2cInfo->soft_reg_value = 0x80;
    pSensorI2cInfo->reg_size = 1;
    pSensorI2cInfo->value_size = 1;

    {
        IsiSensorCaps_t Caps;
        sensor_caps_t *pCaps = NULL;
        uint32_t lanes;        
        uint32_t i;

        for (i=0; i<3; i++) {
            lanes = (1<<i);
            ListInit(&pSensorI2cInfo->lane_res[i]);
            if (g_suppoted_mipi_lanenum_type & lanes) {
                Caps.Index = 0;            
                while(Sensor_IsiGetCapsIssInternal(&Caps,lanes)==RET_SUCCESS) {
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
    pChipIDInfo_H->chipid_reg_addr = Sensor_CHIP_ID_HIGH_BYTE;  
    pChipIDInfo_H->chipid_reg_value = Sensor_CHIP_ID_HIGH_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_H );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_H );

    sensor_chipid_info_t* pChipIDInfo_L = (sensor_chipid_info_t *) malloc( sizeof(sensor_chipid_info_t) );
    if ( !pChipIDInfo_L )
    {
        return RET_OUTOFMEM;
    }
    MEMSET( pChipIDInfo_L, 0, sizeof(*pChipIDInfo_L) ); 
    pChipIDInfo_L->chipid_reg_addr = Sensor_CHIP_ID_LOW_BYTE;
    pChipIDInfo_L->chipid_reg_value = Sensor_CHIP_ID_LOW_BYTE_DEFAULT;
    ListPrepareItem( pChipIDInfo_L );
    ListAddTail( &pSensorI2cInfo->chipid_info, pChipIDInfo_L );

	//oyyf sensor drv version
	pSensorI2cInfo->sensor_drv_version = CONFIG_SENSOR_DRV_VERSION;
	
    *pdata = pSensorI2cInfo;
    return RET_SUCCESS;
}

static RESULT GC5025_IsiSetSensorFrameRateLimit(IsiSensorHandle_t handle, uint32_t minimum_framerate)
{
    Sensor_Context_t *pSensorCtx = (Sensor_Context_t *)handle;

    RESULT result = RET_SUCCESS;

    TRACE( Sensor_INFO, "%s: (enter)\n", __FUNCTION__);

    if ( pSensorCtx == NULL ){
        TRACE( Sensor_ERROR, "%s: pSensorCtx IS NULL\n", __FUNCTION__);
        return ( RET_WRONG_HANDLE );
    }

    pSensorCtx->preview_minimum_framerate = minimum_framerate;
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
    Sensor_IsiGetSensorIss,
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
    Sensor_IsiGetSensorI2cInfo,
};


