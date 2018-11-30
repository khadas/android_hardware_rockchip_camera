//IMX274_tables.c
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

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "IMX274_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet OV13850_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.

//four lane




const IsiRegDescription_t Sensor_g_aRegDescription_fourlane[] =
{
	{0x3000   ,0x12, "0x0100", eReadWrite}, 
	{0x0000,   0x00, "eTableEnd",eTableEnd}   

};




const IsiRegDescription_t Sensor_g_twolane_resolution_3840_2160[] =
{	
	{0x3000   ,0x12, "0x0100", eReadWrite},                                           
	{0x303E   ,0x03, "0x0100", eReadWrite},                                           
	{0x3120   ,0xC0, "0x0100", eReadWrite},                                           
	{0x3121   ,0x00, "0x0100", eReadWrite},                                           
	{0x3122   ,0x02, "0x0100", eReadWrite},                                           
	{0x3123   ,0x01, "0x0100", eReadWrite},                                           
	{0x3129   ,0x9C, "0x0100", eReadWrite},                                           
	{0x312A   ,0x02, "0x0100", eReadWrite},                                           
	{0x312D   ,0x02, "0x0100", eReadWrite},                                           
	{0x3AC4   ,0x01, "0x0100", eReadWrite},                                           
	{0x310B   ,0x00, "0x0100", eReadWrite},                                           
	{0x30EE   ,0x01, "0x0100", eReadWrite},                                           
	{0x3304   ,0x32, "0x0100", eReadWrite},                                           
	{0x3306   ,0x32, "0x0100", eReadWrite},                                           
	{0x3590   ,0x32, "0x0100", eReadWrite},                                           
	{0x3686   ,0x32, "0x0100", eReadWrite},                                           
	{0x3045   ,0x32, "0x0100", eReadWrite},                                           
	{0x301A   ,0x00, "0x0100", eReadWrite},                                           
	{0x304C   ,0x00, "0x0100", eReadWrite},                                           
	{0x304D   ,0x03, "0x0100", eReadWrite},                                           
	{0x331C   ,0x1A, "0x0100", eReadWrite},                                           
	{0x3502   ,0x02, "0x0100", eReadWrite},                                           
	{0x3529   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352A   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352B   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3538   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3539   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3553   ,0x00, "0x0100", eReadWrite},                                           
	{0x357D   ,0x05, "0x0100", eReadWrite},                                           
	{0x357F   ,0x05, "0x0100", eReadWrite},                                           
	{0x3581   ,0x04, "0x0100", eReadWrite},                                           
	{0x3583   ,0x76, "0x0100", eReadWrite},                                           
	{0x3587   ,0x01, "0x0100", eReadWrite},                                           
	{0x35BB   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BC   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BD   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BE   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BF   ,0x0E, "0x0100", eReadWrite},                                           
	{0x366E   ,0x00, "0x0100", eReadWrite},                                           
	{0x366F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3670   ,0x00, "0x0100", eReadWrite},                                           
	{0x3671   ,0x00, "0x0100", eReadWrite},                                           
	{0x3004   ,0x01, "0x0100", eReadWrite},                                           
	{0x3005   ,0x01, "0x0100", eReadWrite},                                           
	{0x3006   ,0x00, "0x0100", eReadWrite},                                           
	{0x3007   ,0x02, "0x0100", eReadWrite},                                           
	{0x300E   ,0x00, "0x0100", eReadWrite},                                           
	{0x300F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3037   ,0x00, "0x0100", eReadWrite},                                           
	{0x3038   ,0x00, "0x0100", eReadWrite},                                           
	{0x3039   ,0x00, "0x0100", eReadWrite},                                           
	{0x303A   ,0x00, "0x0100", eReadWrite},                                           
	{0x303B   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DD   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DE   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DF   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E0   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E1   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E2   ,0x01, "0x0100", eReadWrite},                                           
	{0x30F6   ,0xE0, "0x0100", eReadWrite},                                           
	{0x30F7   ,0x04, "0x0100", eReadWrite},                                           
	{0x30F8   ,0xDA, "0x0100", eReadWrite},                                           
	{0x30F9   ,0x16, "0x0100", eReadWrite},                                           
	{0x30FA   ,0x00, "0x0100", eReadWrite},                                           
	{0x3130   ,0x86, "0x0100", eReadWrite},                                           
	{0x3131   ,0x08, "0x0100", eReadWrite},                                           
	{0x3132   ,0x7E, "0x0100", eReadWrite},                                           
	{0x3133   ,0x08, "0x0100", eReadWrite},                                           
	{0x3A54   ,0x18, "0x0100", eReadWrite},                                           
	{0x3A55   ,0x0F, "0x0100", eReadWrite},                                           
	{0x3342   ,0x0A, "0x0100", eReadWrite},                                           
	{0x3343   ,0x00, "0x0100", eReadWrite},                                           
	{0x3344   ,0x16, "0x0100", eReadWrite},                                           
	{0x3345   ,0x00, "0x0100", eReadWrite},                                           
	{0x3528   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3554   ,0x1F, "0x0100", eReadWrite},                                           
	{0x3555   ,0x01, "0x0100", eReadWrite},                                           
	{0x3556   ,0x01, "0x0100", eReadWrite},                                           
	{0x3557   ,0x01, "0x0100", eReadWrite},                                           
	{0x3558   ,0x01, "0x0100", eReadWrite},                                           
	{0x3559   ,0x00, "0x0100", eReadWrite},                                           
	{0x355A   ,0x00, "0x0100", eReadWrite},                                           
	{0x35BA   ,0x0E, "0x0100", eReadWrite},                                           
	{0x366A   ,0x1B, "0x0100", eReadWrite},                                           
	{0x366B   ,0x1A, "0x0100", eReadWrite},                                           
	{0x366C   ,0x19, "0x0100", eReadWrite},                                           
	{0x366D   ,0x17, "0x0100", eReadWrite},                                           
	{0x33A6   ,0x01, "0x0100", eReadWrite},                                           
	{0x306B   ,0x05, "0x0100", eReadWrite},                                           
	{0x3A41   ,0x08, "0x0100", eReadWrite},	                                           
	{0x3134   ,0x5F, "0x0100", eReadWrite},                                           
	{0x3135   ,0x00, "0x0100", eReadWrite},                                           
	{0x3136   ,0x47, "0x0100", eReadWrite},                                           
	{0x3137   ,0x00, "0x0100", eReadWrite},                                           
	{0x3138   ,0x27, "0x0100", eReadWrite},                                           
	{0x3139   ,0x00, "0x0100", eReadWrite},                                           
	{0x313A   ,0x27, "0x0100", eReadWrite},                                           
	{0x313B   ,0x00, "0x0100", eReadWrite},                                           
	{0x313C   ,0x27, "0x0100", eReadWrite},                                           
	{0x313D   ,0x00, "0x0100", eReadWrite},                                           
	{0x313E   ,0x97, "0x0100", eReadWrite},                                           
	{0x313F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3140   ,0x27, "0x0100", eReadWrite},                                                             
	{0x3141   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3142   ,0x1F, "0x0100", eReadWrite},                                                             
	{0x3143   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3144   ,0x0F, "0x0100", eReadWrite},                                                             
	{0x3145   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3A85   ,0x03, "0x0100", eReadWrite},                                                             
	{0x3A86   ,0x47, "0x0100", eReadWrite},                                                             
	{0x3A87   ,0x00, "0x0100", eReadWrite},                                                             
	{0x0      ,0x10,"0x0100",eDelay},  	                                                                 
	//{0x3000   ,0x00, "0x0100", eReadWrite},                                                           
	{0x303E   ,0x03, "0x0100", eReadWrite},                                                                 
	{0x0      ,0x07,"0x0100",eDelay},  	                                                                 
	{0x30F4	  ,0x00, "0x0100", eReadWrite},                                                              
	{0x3018   ,0xA2, "0x0100", eReadWrite},                                                                 
	{0x300C   ,0x0C, "0x0100", eReadWrite},                                                                 
	{0x300D   ,0x00, "0x0100", eReadWrite},                                                                 
	{0x3001   ,0x10, "0x0100", eReadWrite},
	{0x312e   ,0x01, "0x0100", eReadWrite},
	{0x3aa2   ,0x01, "0x0100", eReadWrite},		
	{0x0000   ,0x00,"eTableEnd",eTableEnd}
};




//720M 3840x2160 30FPS
const IsiRegDescription_t Sensor_g_fourlane_resolution_3840_2160[] =
{

    {0x3000   ,0x1f, "0x0100", eReadWrite},
	{0x303E   ,0x02, "0x0100", eReadWrite},  

	{0x3120   ,0xF0, "0x0100", eReadWrite},                                           
	{0x3121   ,0x00, "0x0100", eReadWrite},                                           
	{0x3122   ,0x02, "0x0100", eReadWrite},  
	{0x3123   ,0x01, "0x0100", eReadWrite}, 
	{0x3129   ,0x9C, "0x0100", eReadWrite},                                           
	{0x312A   ,0x02, "0x0100", eReadWrite},                                           
	{0x312D   ,0x02, "0x0100", eReadWrite},
	{0x3AC4   ,0x01, "0x0100", eReadWrite}, 
	{0x310B   ,0x00, "0x0100", eReadWrite},                                           
	{0x30EE   ,0x01, "0x0100", eReadWrite},                                           
	{0x3304   ,0x32, "0x0100", eReadWrite},                                           
	{0x3306   ,0x32, "0x0100", eReadWrite},                                           
	{0x3590   ,0x32, "0x0100", eReadWrite},                                           
	{0x3686   ,0x32, "0x0100", eReadWrite},                                           
	{0x3045   ,0x32, "0x0100", eReadWrite},                                           
	{0x301A   ,0x00, "0x0100", eReadWrite},                                           
	{0x304C   ,0x00, "0x0100", eReadWrite},                                           
	{0x304D   ,0x03, "0x0100", eReadWrite},                                           
	{0x331C   ,0x1A, "0x0100", eReadWrite},                                           
	{0x3502   ,0x02, "0x0100", eReadWrite},                                           
	{0x3529   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352A   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352B   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3538   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3539   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3553   ,0x00, "0x0100", eReadWrite},                                           
	{0x357D   ,0x05, "0x0100", eReadWrite},                                           
	{0x357F   ,0x05, "0x0100", eReadWrite},                                           
	{0x3581   ,0x04, "0x0100", eReadWrite},                                           
	{0x3583   ,0x76, "0x0100", eReadWrite},                                           
	{0x3587   ,0x01, "0x0100", eReadWrite},                                           
	{0x35BB   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BC   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BD   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BE   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BF   ,0x0E, "0x0100", eReadWrite},                                           
	{0x366E   ,0x00, "0x0100", eReadWrite},                                           
	{0x366F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3670   ,0x00, "0x0100", eReadWrite},                                           
	{0x3671   ,0x00, "0x0100", eReadWrite},                                           
	{0x3004   ,0x01, "0x0100", eReadWrite},                                           
	{0x3005   ,0x01, "0x0100", eReadWrite},                                           
	{0x3006   ,0x00, "0x0100", eReadWrite},                                           
	{0x3007   ,0x02, "0x0100", eReadWrite},                                           
	{0x300E   ,0x00, "0x0100", eReadWrite},                                           
	{0x300F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3037   ,0x00, "0x0100", eReadWrite},                                           
	{0x3038   ,0x00, "0x0100", eReadWrite},                                           
	{0x3039   ,0x00, "0x0100", eReadWrite},                                           
	{0x303A   ,0x00, "0x0100", eReadWrite},                                           
	{0x303B   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DD   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DE   ,0x00, "0x0100", eReadWrite},                                           
	{0x30DF   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E0   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E1   ,0x00, "0x0100", eReadWrite},                                           
	{0x30E2   ,0x01, "0x0100", eReadWrite}, 

    //30FPS: sony
	
	{0x30F6   ,0x10, "0x0100", eReadWrite}, //0xa0											
	{0x30F7   ,0x02, "0x0100", eReadWrite}, //0x09											
	{0x30F8   ,0xc6, "0x0100", eReadWrite}, //0xda											
	{0x30F9   ,0x11, "0x0100", eReadWrite}, //0x16	  
		
	{0x30FA   ,0x00, "0x0100", eReadWrite},                                           
	{0x3130   ,0x86, "0x0100", eReadWrite},                                           
	{0x3131   ,0x08, "0x0100", eReadWrite},                                           
	{0x3132   ,0x7E, "0x0100", eReadWrite},                                           
	{0x3133   ,0x08, "0x0100", eReadWrite},                                           
	{0x3A54   ,0x18, "0x0100", eReadWrite},                                           
	{0x3A55   ,0x0F, "0x0100", eReadWrite},                                           
	{0x3342   ,0x0A, "0x0100", eReadWrite},                                           
	{0x3343   ,0x00, "0x0100", eReadWrite},                                           
	{0x3344   ,0x16, "0x0100", eReadWrite},                                           
	{0x3345   ,0x00, "0x0100", eReadWrite},                                           
	{0x3528   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3554   ,0x1F, "0x0100", eReadWrite},                                           
	{0x3555   ,0x01, "0x0100", eReadWrite},                                           
	{0x3556   ,0x01, "0x0100", eReadWrite},                                           
	{0x3557   ,0x01, "0x0100", eReadWrite},                                           
	{0x3558   ,0x01, "0x0100", eReadWrite},                                           
	{0x3559   ,0x00, "0x0100", eReadWrite},                                           
	{0x355A   ,0x00, "0x0100", eReadWrite},                                           
	{0x35BA   ,0x0E, "0x0100", eReadWrite},                                           
	{0x366A   ,0x1B, "0x0100", eReadWrite},                                           
	{0x366B   ,0x1A, "0x0100", eReadWrite},                                           
	{0x366C   ,0x19, "0x0100", eReadWrite},                                           
	{0x366D   ,0x17, "0x0100", eReadWrite},                                           
	{0x33A6   ,0x01, "0x0100", eReadWrite},                                           
	{0x306B   ,0x05, "0x0100", eReadWrite},                                           
	{0x3A41   ,0x08, "0x0100", eReadWrite},	
	
	{0x3134   ,0x77, "0x0100", eReadWrite},                                           
	{0x3135   ,0x00, "0x0100", eReadWrite},                                           
	{0x3136   ,0x67, "0x0100", eReadWrite},                                           
	{0x3137   ,0x00, "0x0100", eReadWrite},                                           
	{0x3138   ,0x37, "0x0100", eReadWrite},                                           
	{0x3139   ,0x00, "0x0100", eReadWrite},                                           
	{0x313A   ,0x37, "0x0100", eReadWrite},                                           
	{0x313B   ,0x00, "0x0100", eReadWrite},                                           
	{0x313C   ,0x37, "0x0100", eReadWrite}, 

	{0x313D   ,0x00, "0x0100", eReadWrite},                                           
	{0x313E   ,0xDF, "0x0100", eReadWrite},                                           
	{0x313F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3140   ,0x37, "0x0100", eReadWrite},                                                             
	{0x3141   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3142   ,0x2F, "0x0100", eReadWrite},                                                             
	{0x3143   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3144   ,0x0F, "0x0100", eReadWrite},                                                             
	{0x3145   ,0x00, "0x0100", eReadWrite},                                                             
	{0x3A85   ,0x03, "0x0100", eReadWrite},                                                             
	{0x3A86   ,0x47, "0x0100", eReadWrite},                                                             
	{0x3A87   ,0x00, "0x0100", eReadWrite}, 

   {0x3A43   ,0x01, "0x0100", eReadWrite}, 	 // 0x00 continues mode  ,  0x01  gatting mode
 
	{0x0      ,0x10,"0x0100",eDelay},  	                                                                 
	//{0x3000   ,0x00, "0x0100", eReadWrite},                                                           
	{0x303E   ,0x02, "0x0100", eReadWrite},  
	{0x0      ,0x07,"0x0100",eDelay},  	                                                                 
	{0x30F4	  ,0x00, "0x0100", eReadWrite},                                                              
	{0x3018   ,0xA2, "0x0100", eReadWrite},                                                                 
	//{0x300C   ,0x0C, "0x0100", eReadWrite},                                                                 
	//{0x300D   ,0x00, "0x0100", eReadWrite},                                                                 
	{0x3001   ,0x10, "0x0100", eReadWrite}, //0x16
	
	{0x0000,   0x00, "eTableEnd",eTableEnd}     
};

//720M  1080P  30FPS
const IsiRegDescription_t Sensor_g_fourlane_resolution_1920_1080[] =
{


	{0x3000   ,0x1f, "0x0100", eReadWrite}, 
	{0x303E   ,0x02, "0x0100", eReadWrite}, 
	{0x3120   ,0xF0, "0x0100", eReadWrite},                                           
	{0x3121   ,0x00, "0x0100", eReadWrite},                                           
	{0x3122   ,0x02, "0x0100", eReadWrite}, 
	{0x3123   ,0x01, "0x0100", eReadWrite}, 
	{0x3129   ,0x9C, "0x0100", eReadWrite},                                           
	{0x312A   ,0x02, "0x0100", eReadWrite},                                           
	{0x312D   ,0x02, "0x0100", eReadWrite},
	{0x3AC4   ,0x01, "0x0100", eReadWrite},
	{0x310B   ,0x00, "0x0100", eReadWrite},                                           
	{0x30EE   ,0x01, "0x0100", eReadWrite},                                           
	{0x3304   ,0x32, "0x0100", eReadWrite},                                           
	{0x3306   ,0x32, "0x0100", eReadWrite},                                           
	{0x3590   ,0x32, "0x0100", eReadWrite},                                           
	{0x3686   ,0x32, "0x0100", eReadWrite},                                           
	{0x3045   ,0x32, "0x0100", eReadWrite},                                           
	{0x301A   ,0x00, "0x0100", eReadWrite},                                           
	{0x304C   ,0x00, "0x0100", eReadWrite},                                           
	{0x304D   ,0x03, "0x0100", eReadWrite},                                           
	{0x331C   ,0x1A, "0x0100", eReadWrite},                                           
	{0x3502   ,0x02, "0x0100", eReadWrite},                                           
	{0x3529   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352A   ,0x0E, "0x0100", eReadWrite},                                           
	{0x352B   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3538   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3539   ,0x0E, "0x0100", eReadWrite},                                           
	{0x3553   ,0x00, "0x0100", eReadWrite},                                           
	{0x357D   ,0x05, "0x0100", eReadWrite},                                           
	{0x357F   ,0x05, "0x0100", eReadWrite},                                           
	{0x3581   ,0x04, "0x0100", eReadWrite},                                           
	{0x3583   ,0x76, "0x0100", eReadWrite},                                           
	{0x3587   ,0x01, "0x0100", eReadWrite},                                           
	{0x35BB   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BC   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BD   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BE   ,0x0E, "0x0100", eReadWrite},                                           
	{0x35BF   ,0x0E, "0x0100", eReadWrite},                                           
	{0x366E   ,0x00, "0x0100", eReadWrite},                                           
	{0x366F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3670   ,0x00, "0x0100", eReadWrite},                                           
	{0x3671   ,0x00, "0x0100", eReadWrite},  

	
	{0x3004 ,0x02, "0x0100",eReadWrite},
	{0x3005 ,0x21, "0x0100",eReadWrite},
	{0x3006 ,0x00, "0x0100",eReadWrite},
	{0x3007 ,0x11, "0x0100",eReadWrite},
	//add default
	{0x300E ,0x00, "0x0100",eReadWrite},
	{0x300F ,0x00, "0x0100",eReadWrite},
	
	{0x3037 ,0x00, "0x0100",eReadWrite},
	{0x3038 ,0x00, "0x0100",eReadWrite},
	{0x3039 ,0x00, "0x0100",eReadWrite},
	{0x303A ,0x00, "0x0100",eReadWrite},
	{0x303B ,0x00, "0x0100",eReadWrite},
	
	{0x30DD ,0x00, "0x0100",eReadWrite},
	{0x30DE ,0x00, "0x0100",eReadWrite},
	{0x30DF ,0x00, "0x0100",eReadWrite},
	{0x30E0 ,0x00, "0x0100",eReadWrite},
	{0x30E1 ,0x00, "0x0100",eReadWrite},
	{0x30E2 ,0x02, "0x0100",eReadWrite},
	// more
	//{0x30EE ,0x01, "0x0100",eReadWrite},
	//

//hts vts
	{0x30F6 ,0x1e, "0x0100",eReadWrite},  //0xe0
	{0x30F7 ,0x01, "0x0100",eReadWrite},  //0x04
	{0x30F8 ,0xD0, "0x0100",eReadWrite},//0a mod
	{0x30F9 ,0x20, "0x0100",eReadWrite},//0x0f

	{0x30FA ,0x00, "0x0100",eReadWrite},//0x00
	{0x3130 ,0x4e, "0x0100",eReadWrite},
	{0x3131 ,0x04, "0x0100",eReadWrite},
	{0x3132 ,0x46, "0x0100",eReadWrite},
	{0x3133 ,0x04, "0x0100",eReadWrite},
	//add default
	{0x3a54 ,0x8c, "0x0100",eReadWrite},
	{0x3a55 ,0x07, "0x0100",eReadWrite},
	
	{0x3342 ,0x0a, "0x0100",eReadWrite},
	{0x3343 ,0x00, "0x0100",eReadWrite},
	{0x3344 ,0x1a, "0x0100",eReadWrite},
	{0x3345 ,0x00, "0x0100",eReadWrite},
	
	{0x3528 ,0x0E, "0x0100",eReadWrite},
	{0x3554 ,0x00, "0x0100",eReadWrite},
	{0x3555 ,0x01, "0x0100",eReadWrite},
	{0x3556 ,0x01, "0x0100",eReadWrite},
	{0x3557 ,0x01, "0x0100",eReadWrite},
	{0x3558 ,0x01, "0x0100",eReadWrite},
	{0x3559 ,0x00, "0x0100",eReadWrite},
	{0x355A ,0x00, "0x0100",eReadWrite},
	{0x35BA ,0x0E, "0x0100",eReadWrite},
	{0x366A ,0x1B, "0x0100",eReadWrite},
	{0x366B ,0x1A, "0x0100",eReadWrite},
	{0x366C ,0x19, "0x0100",eReadWrite},
	{0x366D ,0x17, "0x0100",eReadWrite},
	{0x33A6 ,0x01, "0x0100",eReadWrite},
	//more
	{0x306B ,0x05, "0x0100",eReadWrite},
	//
	{0x3A41 ,0x08, "0x0100",eReadWrite},


	{0x3134   ,0x77, "0x0100", eReadWrite},                                           
	{0x3135   ,0x00, "0x0100", eReadWrite},                                           
	{0x3136   ,0x67, "0x0100", eReadWrite},                                           
	{0x3137   ,0x00, "0x0100", eReadWrite},                                           
	{0x3138   ,0x37, "0x0100", eReadWrite},                                           
	{0x3139   ,0x00, "0x0100", eReadWrite},                                           
	{0x313A   ,0x37, "0x0100", eReadWrite},                                           
	{0x313B   ,0x00, "0x0100", eReadWrite},                                           
	{0x313C   ,0x37, "0x0100", eReadWrite},                                           
	{0x313D   ,0x00, "0x0100", eReadWrite},                                           
	{0x313E   ,0xDF, "0x0100", eReadWrite},                                           
	{0x313F   ,0x00, "0x0100", eReadWrite},                                           
	{0x3140   ,0x37, "0x0100", eReadWrite},                                           
	{0x3141   ,0x00, "0x0100", eReadWrite},                                           
	{0x3142   ,0x2F, "0x0100", eReadWrite},                                           
	{0x3143   ,0x00, "0x0100", eReadWrite},                                           
	{0x3144   ,0x0F, "0x0100", eReadWrite},                                           
	{0x3145   ,0x00, "0x0100", eReadWrite},                                           
	{0x3A85   ,0x03, "0x0100", eReadWrite},                                           
	{0x3A86   ,0x47, "0x0100", eReadWrite},                                           
	{0x3A87   ,0x00, "0x0100", eReadWrite},                                           
	{0x0      ,0x10,"0x0100",eDelay},  
	                                     
	{0x303E   ,0x02, "0x0100", eReadWrite},  
                                      
	{0x0      ,0x07,"0x0100",eDelay},  	                                           
	{0x30F4	  ,0x00, "0x0100", eReadWrite},                                            
	{0x3018   ,0xA2, "0x0100", eReadWrite},                                           
	
	{0x3001   ,0x10, "0x0100", eReadWrite},  //0x16

	{0x0000,   0x00, "eTableEnd",eTableEnd}   
};



