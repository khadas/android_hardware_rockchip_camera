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
//OV2710_tables.c

#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>

#include <common/return_codes.h>

#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "OV2710_MIPI_priv.h"

/*****************************************************************************
 * GLOBALS
 *****************************************************************************/
const IsiRegDescription_t OV2710_g_aRegDescription_onelane[] =
{

	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
//MClk=24M 1920x1080 30fps sclk=80
const IsiRegDescription_t OV2710_g_1920x1080_onelane[] =
{
	{ 0x3008, 0x82, "0x0100", eReadWrite },// 
	{ 0x3008, 0x42, "0x0100", eReadWrite },//
	{ 0x4201, 0x00, "0x0100", eReadWrite },//// frame ctrl
	{ 0x4202, 0x0f, "0x0100", eReadWrite },//// frame ctrl
	{ 0x3103, 0x93, "0x0100", eReadWrite },//

	{ 0x3017, 0x7f, "0x0100", eReadWrite },//
	{ 0x3018, 0xfc, "0x0100", eReadWrite },//
	{ 0x3706, 0x61, "0x0100", eReadWrite },//
	{ 0x3712, 0x0c, "0x0100", eReadWrite },//
	{ 0x3630, 0x6d, "0x0100", eReadWrite },//
	{ 0x3800, 0x01, "0x0100", eReadWrite },//// horizontal start
	{ 0x3801, 0xb4, "0x0100", eReadWrite },//
	{ 0x3802, 0x00, "0x0100", eReadWrite },//// vertical start
	{ 0x3803, 0x0a, "0x0100", eReadWrite },////
	{ 0x3818, 0x80, "0x0100", eReadWrite },//// mirror[6]/flip[5]
	{ 0x3804, 0x07, "0x0100", eReadWrite },//// horizontal width
	{ 0x3805, 0x80, "0x0100", eReadWrite },////
	{ 0x3806, 0x04, "0x0100", eReadWrite },//// vertical height
	{ 0x3807, 0x38, "0x0100", eReadWrite },////
	{ 0x3808, 0x07, "0x0100", eReadWrite },//// DVP H width
	{ 0x3809, 0x80, "0x0100", eReadWrite },////
	{ 0x380a, 0x04, "0x0100", eReadWrite },//// DVP V height
	{ 0x380b, 0x38, "0x0100", eReadWrite },////
	//{ 0x3810, 0x10, "0x0100", eReadWrite },//// offset
	//{ 0x3811, 0x06, "0x0100", eReadWrite },////
	//{ 0x3812, 0x00, "0x0100", eReadWrite },////
	//{ 0x3813, 0x00, "0x0100", eReadWrite },////
	{ 0x3621, 0x04, "0x0100", eReadWrite },//
	{ 0x3604, 0x60, "0x0100", eReadWrite },//
	{ 0x3603, 0xa7, "0x0100", eReadWrite },//
	{ 0x3631, 0x26, "0x0100", eReadWrite },//
	{ 0x3600, 0x04, "0x0100", eReadWrite },//
	{ 0x3620, 0x37, "0x0100", eReadWrite },//
	{ 0x3623, 0x00, "0x0100", eReadWrite },//
	{ 0x3702, 0x9e, "0x0100", eReadWrite },//
	{ 0x3703, 0x5c, "0x0100", eReadWrite },//
	{ 0x3704, 0x40, "0x0100", eReadWrite },//
	{ 0x370d, 0x0f, "0x0100", eReadWrite },//
	{ 0x3713, 0x9f, "0x0100", eReadWrite },//
	{ 0x3714, 0x4c, "0x0100", eReadWrite },//
	{ 0x3710, 0x9e, "0x0100", eReadWrite },//
	{ 0x3801, 0xc4, "0x0100", eReadWrite },//
	{ 0x3605, 0x05, "0x0100", eReadWrite },//
	{ 0x3606, 0x3f, "0x0100", eReadWrite },//
	{ 0x302d, 0x90, "0x0100", eReadWrite },//
	{ 0x370b, 0x40, "0x0100", eReadWrite },//
	{ 0x3716, 0x31, "0x0100", eReadWrite },//
	{ 0x3707, 0x52, "0x0100", eReadWrite },//
	{ 0x380d, 0x74, "0x0100", eReadWrite },//HTS L
	{ 0x5181, 0x20, "0x0100", eReadWrite },//
	{ 0x518f, 0x00, "0x0100", eReadWrite },//
	{ 0x4301, 0xff, "0x0100", eReadWrite },//
	{ 0x4303, 0x00, "0x0100", eReadWrite },//
	{ 0x3a00, 0x78, "0x0100", eReadWrite },//
	{ 0x300f, 0x88, "0x0100", eReadWrite },//
	{ 0x3011, 0x28, "0x0100", eReadWrite },//
	{ 0x3a1a, 0x06, "0x0100", eReadWrite },//
	{ 0x3a18, 0x00, "0x0100", eReadWrite },//
	{ 0x3a19, 0x7a, "0x0100", eReadWrite },//
	{ 0x3a13, 0x54, "0x0100", eReadWrite },//
	{ 0x382e, 0x0f, "0x0100", eReadWrite },//
	{ 0x381a, 0x1a, "0x0100", eReadWrite },//
	{ 0x401d, 0x02, "0x0100", eReadWrite },//
	{ 0x5688, 0x03, "0x0100", eReadWrite },//
	{ 0x5684, 0x07, "0x0100", eReadWrite },//
	{ 0x5685, 0xa0, "0x0100", eReadWrite },//
	{ 0x5686, 0x04, "0x0100", eReadWrite },//
	{ 0x5687, 0x43, "0x0100", eReadWrite },//
	{ 0x3011, 0x0a, "0x0100", eReadWrite },//
	{ 0x300f, 0x8a, "0x0100", eReadWrite },//
	{ 0x3017, 0x00, "0x0100", eReadWrite },//
	{ 0x3018, 0x00, "0x0100", eReadWrite },//
	{ 0x300e, 0x04, "0x0100", eReadWrite },//
	{ 0x4801, 0x0f, "0x0100", eReadWrite },//
	{ 0x300f, 0xc3, "0x0100", eReadWrite },////
	{ 0x3a0f, 0x40, "0x0100", eReadWrite },//
	{ 0x3a10, 0x38, "0x0100", eReadWrite },//
	{ 0x3a1b, 0x48, "0x0100", eReadWrite },//
	{ 0x3a1e, 0x30, "0x0100", eReadWrite },//
	{ 0x3a11, 0x90, "0x0100", eReadWrite },//
	{ 0x3a1f, 0x10, "0x0100", eReadWrite },//
	//{ 0x380c, 0x09, "0x0100", eReadWrite },////HTS def:0x09
	//{ 0x380d, 0x74, "0x0100", eReadWrite },////    def:0x48
	//{ 0x380e, 0x04, "0x0100", eReadWrite },////VTS def:0x06
	//{ 0x380f, 0x38, "0x0100", eReadWrite },////    def:0x18
	//{ 0x4837, 0x19, "0x0100", eReadWrite },//
	//{ 0x4819, 0xc6, "0x0100", eReadWrite },//
	//{ 0x4823, 0x5c, "0x0100", eReadWrite },//
	{ 0x3500, 0x00, "0x0100", eReadWrite },////
	{ 0x3501, 0x28, "0x0100", eReadWrite },//
	{ 0x3502, 0x90, "0x0100", eReadWrite },//
	{ 0x3503, 0x07, "0x0100", eReadWrite },//
	{ 0x350a, 0x00, "0x0100", eReadWrite },//
	{ 0x350b, 0x1f, "0x0100", eReadWrite },//
	{ 0x5000, 0x5f, "0x0100", eReadWrite },//
	{ 0x5001, 0x4e, "0x0100", eReadWrite },//
	{ 0x3406, 0x01, "0x0100", eReadWrite },//// awb manual control
	{ 0x3400, 0x04, "0x0100", eReadWrite },//// awb R gain
	{ 0x3401, 0x00, "0x0100", eReadWrite },//// 
	{ 0x3402, 0x04, "0x0100", eReadWrite },//// awb G gain
	{ 0x3403, 0x00, "0x0100", eReadWrite },////
	{ 0x3404, 0x04, "0x0100", eReadWrite },//// awb b gain
	{ 0x3405, 0x00, "0x0100", eReadWrite },////
	{ 0x4800, 0x24, "0x0100", eReadWrite },//// MIPI ctrl
	//{0x0100 01,"0x0100",eReadWrite},//
	{ 0x0000, 0x00, "eTableEnd", eTableEnd }
};
