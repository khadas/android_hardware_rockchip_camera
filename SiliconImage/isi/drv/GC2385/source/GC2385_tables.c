//GC2385_tables.c
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
/*
#include "stdinc.h"

#if( GC2385_DRIVER_USAGE == USE_CAM_DRV_EN )
*/


#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "GC2385_MIPI_priv.h"


/*****************************************************************************
 * DEFINES
 *****************************************************************************/


/*****************************************************************************
 * GLOBALS
 *****************************************************************************/

// Image sensor register settings default values taken from data sheet GC2385_DS_1.1.pdf.
// The settings may be altered by the code in IsiSetupSensor.

//one lane
const IsiRegDescription_t GC2385_g_aRegDescription[] =
{
//system
	{0xfe , 0x00,"eReadWrite",eReadWrite},
	{0xfe , 0x00,"eReadWrite",eReadWrite},
	{0xfe , 0x00,"eReadWrite",eReadWrite},  
//	{0xfc , 0x06,"",eReadWrite},
	{0xf2 , 0x02,"eReadWrite",eReadWrite},
//	{0xf3 , 0x00,"",eReadWrite},
	{0xf4 , 0x03,"eReadWrite",eReadWrite},
	{0xf7 , 0x01,"eReadWrite",eReadWrite},
	{0xf8 , 0x28,"eReadWrite",eReadWrite},
	{0xf9 , 0x02,"eReadWrite",eReadWrite}, 
	{0xfa , 0x08,"eReadWrite",eReadWrite},
	{0xfc , 0x8e,"eReadWrite",eReadWrite},
	{0xe7 , 0xcc,"eReadWrite",eReadWrite}, 
	{0x88 , 0x03,"eReadWrite",eReadWrite}, 

       
	//analog       
	{0x03 , 0x04,"eReadWrite",eReadWrite},
	{0x04 , 0x80,"eReadWrite",eReadWrite},
	{0x05 , 0x02,"eReadWrite",eReadWrite},
	{0x06 , 0x86,"eReadWrite",eReadWrite},
	{0x07 , 0x00,"eReadWrite",eReadWrite},
	{0x08 , 0x10,"eReadWrite",eReadWrite},    
	{0x09 , 0x00,"eReadWrite",eReadWrite},
	{0x0a , 0x04,"eReadWrite",eReadWrite},
	{0x0b , 0x00,"eReadWrite",eReadWrite},
	{0x0c , 0x02,"eReadWrite",eReadWrite},
	{0x17 , 0xd4,"eReadWrite",eReadWrite},
	{0x18 , 0x02,"eReadWrite",eReadWrite},
	{0x19 , 0x17,"eReadWrite",eReadWrite},
	{0x1c , 0x18,"eReadWrite",eReadWrite},
	{0x20 , 0x73,"eReadWrite",eReadWrite},
	{0x21 , 0x38,"eReadWrite",eReadWrite},
	{0x22 , 0xa2,"eReadWrite",eReadWrite},
	{0x29 , 0x20,"eReadWrite",eReadWrite},
	{0x2f , 0x14,"eReadWrite",eReadWrite},
	{0x3f , 0x40,"eReadWrite",eReadWrite},
	{0xcd , 0x94,"eReadWrite",eReadWrite},
	{0xce , 0x45,"eReadWrite",eReadWrite},
	{0xd1 , 0x0c,"eReadWrite",eReadWrite},
	{0xd7 , 0x9b,"eReadWrite",eReadWrite},
	{0xd8 , 0x99,"eReadWrite",eReadWrite},
	{0xda , 0x3b,"eReadWrite",eReadWrite},
	{0xd9 , 0xb5,"eReadWrite",eReadWrite},
	{0xdb , 0x75,"eReadWrite",eReadWrite},
	{0xe3 , 0x1b,"eReadWrite",eReadWrite},
	{0xe4 , 0xf8,"eReadWrite",eReadWrite},

            
	//BLK
	{0x40 , 0x22,"eReadWrite",eReadWrite},
	{0x43 , 0x07,"eReadWrite",eReadWrite},//04 
	{0x4e , 0x3c,"eReadWrite",eReadWrite},//04 
	{0x4f , 0x00,"eReadWrite",eReadWrite},//60 
	{0x68 , 0x00,"eReadWrite",eReadWrite},//90 

              
	//gain
	{0xb0 , 0x46,"eReadWrite",eReadWrite}, //row start
	{0xb1 , 0x01,"eReadWrite",eReadWrite}, //col start
	{0xb2 , 0x00,"eReadWrite",eReadWrite},
	{0xb6 , 0x00,"eReadWrite",eReadWrite},
	

  //out crop           
	{0x90 , 0x01,"eReadWrite",eReadWrite}, 
	{0x92 , 0x03,"eReadWrite",eReadWrite}, 
	{0x94 , 0x05,"eReadWrite",eReadWrite}, 
	{0x95 , 0x04,"eReadWrite",eReadWrite}, 
	{0x96 , 0xb0,"eReadWrite",eReadWrite}, 
	{0x97 , 0x06,"eReadWrite",eReadWrite}, 
	{0x98 , 0x40,"eReadWrite",eReadWrite}, 

              
   //mipi set            
	{0xfe , 0x00,"eReadWrite",eReadWrite}, 
	{0xed , 0x00,"eReadWrite",eReadWrite},
	{0xfe , 0x03,"eReadWrite",eReadWrite}, 
	{0x01 , 0x03,"eReadWrite",eReadWrite}, 
	{0x02 , 0x82,"eReadWrite",eReadWrite}, 
	{0x03 , 0xd0,"eReadWrite",eReadWrite}, 
	{0x04 , 0x04,"eReadWrite",eReadWrite}, 
	{0x05 , 0x00,"eReadWrite",eReadWrite}, 
	{0x06 , 0x80,"eReadWrite",eReadWrite},   
	{0x11 , 0x2b,"eReadWrite",eReadWrite},
	{0x12 , 0xd0,"eReadWrite",eReadWrite},//
	{0x13 , 0x07,"eReadWrite",eReadWrite},//
	{0x15 , 0x00,"eReadWrite",eReadWrite},//
	{0x1b , 0x10,"eReadWrite",eReadWrite},//
	{0x1c , 0x10,"eReadWrite",eReadWrite},//
	{0x21 , 0x08,"eReadWrite",eReadWrite},//
	{0x22 , 0x05,"eReadWrite",eReadWrite},          
	{0x23 , 0x13,"eReadWrite",eReadWrite},
	{0x24 , 0x02,"eReadWrite",eReadWrite},
	{0x25 , 0x13,"eReadWrite",eReadWrite},//
	{0x26 , 0x06,"eReadWrite",eReadWrite},
	{0x29 , 0x06,"eReadWrite",eReadWrite},
	{0x2a , 0x08,"eReadWrite",eReadWrite},//
	{0x2b , 0x06,"eReadWrite",eReadWrite},
	{0xfe , 0x00,"eReadWrite",eReadWrite},//
//	{0xed , 0x90,"eReadWrite",eReadWrite},
  {0x00 ,0x00,"eTableEnd",eTableEnd}
};


const IsiRegDescription_t GC2385_g_1600x1200[] =
{
		{0xfe,  0x00, "eReadWrite",eReadWrite},
		{0x91 , 0x00,"eReadWrite",eReadWrite}, 
		{0x92 , 0x03,"eReadWrite",eReadWrite}, 
		{0x93 , 0x00,"eReadWrite",eReadWrite}, 
		{0x94 , 0x05,"eReadWrite",eReadWrite}, 
		{0x95 , 0x04,"eReadWrite",eReadWrite}, 
		{0x96 , 0xb0,"eReadWrite",eReadWrite}, 
		{0x97 , 0x06,"eReadWrite",eReadWrite}, 
		{0x98 , 0x40,"eReadWrite",eReadWrite}, 
    {0x0000 ,0x00,"eTableEnd",eTableEnd}

};

const IsiRegDescription_t GC2385_g_1600x1200_30fps[] =
{
  {0xfe, 0x00, "eReadWrite",eReadWrite},
	{0x07, 0x00, "eReadWrite",eReadWrite},// VB H  VB=VTS-HEIGHT-20;
	{0x08, 0x2e, "eReadWrite",eReadWrite},// VB L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t GC2385_g_1600x1200_20fps[] =
{
  {0xfe, 0x00, "eReadWrite",eReadWrite},
	{0x07, 0x02, "eReadWrite",eReadWrite},// VB H
	{0x08, 0xa8, "eReadWrite",eReadWrite},// VB L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t GC2385_g_1600x1200_15fps[] =
{
  {0xfe, 0x00, "eReadWrite",eReadWrite},
	{0x07, 0x05, "eReadWrite",eReadWrite},// VB H
	{0x08, 0x21, "eReadWrite",eReadWrite},// VB L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t GC2385_g_1600x1200_10fps[] =
{
  {0xfe, 0x00, "eReadWrite",eReadWrite},
	{0x07, 0x09, "eReadWrite",eReadWrite},// VB H
	{0x08, 0x60, "eReadWrite",eReadWrite},// VB L
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
