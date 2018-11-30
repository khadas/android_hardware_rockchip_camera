//GC5025_tables.c
/*****************************************************************************/
/*!
 *  \file        GC5025_tables.c \n
 *  \version     1.0 \n
 *  \author      Meinicke \n
 *  \brief       Image-sensor-specific tables and other
 *               constant values/structures for OV13850. \n
 *
 *  \revision    $Revision: 803 $ \n
 *               $Author: $ \n
 *               $Date: \n
 *               $Id:  $ \n
 */
/*  This is an unpublished work, the copyright in which vests in Silicon Image
 *  GmbH. The information contained herein is the property of Silicon Image GmbH
 *  and is supplied without liability for errors or omissions. No part may be
 *  reproduced or used expect as authorized by contract or other written
 *  permission. Copyright(c) Silicon Image GmbH, 2009, all rights reserved.
 */
/*****************************************************************************/
/*
#include "stdinc.h"
#if( GC5025_DRIVER_USAGE == USE_CAM_DRV_EN )
*/
#include <ebase/types.h>
#include <ebase/trace.h>
#include <ebase/builtins.h>
#include <common/return_codes.h>
#include "isi.h"
#include "isi_iss.h"
#include "isi_priv.h"
#include "GC5025_MIPI_priv.h"

/*****************************************************************************
 * DEFINES
 *****************************************************************************/
/*****************************************************************************
 * GLOBALS
 *****************************************************************************/
// Image sensor register settings default values taken from data sheet OV13850_DS_1.1_SiliconImage.pdf.
// The settings may be altered by the code in IsiSetupSensor.
//two lane
const IsiRegDescription_t Sensor_g_aRegDescription_twolane[] =
{
	{0xfe, 0x00,"0x0100",eReadWrite},
	{0xfe, 0x00,"0x0100",eReadWrite},
	{0xfe, 0x00,"0x0100",eReadWrite},
	{0xf7, 0x01,"0x0100",eReadWrite},
	{0xf8, 0x11,"0x0100",eReadWrite},
	{0xf9, 0x00,"0x0100",eReadWrite},
	{0xfa, 0xa0,"0x0100",eReadWrite},
	{0xfc, 0x2a,"0x0100",eReadWrite},
	{0xfe, 0x03,"0x0100",eReadWrite},
	{0x01, 0x07,"0x0100",eReadWrite},
	{0xfc, 0x2e,"0x0100",eReadWrite},
	{0xfe, 0x00,"0x0100",eReadWrite},
	{0x88, 0x03,"0x0100",eReadWrite},

	/*Analog*/
	{0x03, 0x09,"0x0100",eReadWrite},                                                                                  
	{0x04, 0x60,"0x0100",eReadWrite},                                                                                  
	{0x05, 0x02,"0x0100",eReadWrite},                                                                                  
	{0x06, 0x58,"0x0100",eReadWrite},                                                                                  
	//{0x07, 0x00,"0x0100",eReadWrite},                                                                                  
	{0x08, 0x20,"0x0100",eReadWrite},                                                                                  
	//{0x09, 0x00,"0x0100",eReadWrite},                                                                                  
	{0x0a, 0x1c,"0x0100",eReadWrite},                                                                                                
	//{0x0b, 0x00,"0x0100",eReadWrite},                                                                                  
	{0x0c, 0x04,"0x0100",eReadWrite},
	{0x0d, 0x07,"0x0100",eReadWrite},                                                                                  
	{0x0e, 0x9c,"0x0100",eReadWrite},                                                                                  
	{0x0f, 0x0a,"0x0100",eReadWrite},                                                                                  
	{0x10, 0x30,"0x0100",eReadWrite},
	{0x17, 0xc0,"0x0100",eReadWrite},                                       
	{0x18, 0x02,"0x0100",eReadWrite},                                                                                  
	{0x19, 0x17,"0x0100",eReadWrite},
	{0x1a, 0x1a,"0x0100",eReadWrite},                                                                                  
	//{0x1c, 0x0c,"0x0100",eReadWrite},                                                                                  
	{0x1e, 0x90,"0x0100",eReadWrite},  
	{0x1f, 0xb0,"0x0100",eReadWrite},
	{0x20, 0x2b,"0x0100",eReadWrite},                                                                                
	{0x21, 0x2b,"0x0100",eReadWrite},
	{0x26, 0x2b,"0x0100",eReadWrite},                                                                                  
	{0x25, 0xc1,"0x0100",eReadWrite},                                                                                  
	{0x27, 0x64,"0x0100",eReadWrite}, 
	{0x28, 0x00,"0x0100",eReadWrite},                                                                                   
	{0x29, 0x3f,"0x0100",eReadWrite},
	{0x2b, 0x80,"0x0100",eReadWrite},    
	{0x30, 0x11,"0x0100",eReadWrite},                                                                                
	{0x31, 0x20,"0x0100",eReadWrite},
	{0x32, 0xa0,"0x0100",eReadWrite},  
	{0x33, 0x00,"0x0100",eReadWrite}, 
	{0x34, 0x55,"0x0100",eReadWrite},   
	{0x3a, 0x00,"0x0100",eReadWrite}, 
	{0x3b, 0x00,"0x0100",eReadWrite}, 
	{0x81, 0x60,"0x0100",eReadWrite},      
	{0xcb, 0x02,"0x0100",eReadWrite},                                                                           
	{0xcd, 0x2d,"0x0100",eReadWrite},                                                                                  
	{0xcf, 0x50,"0x0100",eReadWrite},  
	{0xd0, 0xb3,"0x0100",eReadWrite},
    {0xd1, 0x18,"0x0100",eReadWrite},//lrk add 
	{0xd9, 0xaa,"0x0100",eReadWrite},                                                                                                        
	{0xdc, 0x03,"0x0100",eReadWrite},
	{0xdd, 0xaa,"0x0100",eReadWrite},
	{0xe0, 0x00,"0x0100",eReadWrite},                                                                                  
	{0xe1, 0x0a,"0x0100",eReadWrite},
	{0xe3, 0x2a,"0x0100",eReadWrite},
	{0xe4, 0xa0,"0x0100",eReadWrite},   
	{0xe5, 0x06,"0x0100",eReadWrite},  //add by tommy                                                                              
	{0xe6, 0x10,"0x0100",eReadWrite},                                                                                  
	{0xe7, 0xc2,"0x0100",eReadWrite}, 
	{0xfe, 0x10,"0x0100",eReadWrite}, 
	{0xfe, 0x00,"0x0100",eReadWrite}, 
	{0xfe, 0x10,"0x0100",eReadWrite}, 
	{0xfe, 0x00,"0x0100",eReadWrite}, 
/*
	{0x1c, 0x1c,"0x0100",eReadWrite}, 
	{0x2f, 0x4a,"0x0100",eReadWrite},     
	{0x38, 0x02,"0x0100",eReadWrite}, 
	{0x39, 0x00,"0x0100",eReadWrite}, 
	{0x3c, 0x02,"0x0100",eReadWrite}, 
	{0x3d, 0x02,"0x0100",eReadWrite}, 
	{0xd3, 0xcc,"0x0100",eReadWrite}, 
	{0x43, 0x03,"0x0100",eReadWrite}, 
	{0x1d, 0x13,"0x0100",eReadWrite}, 
*/
	{0xfe, 0x00,"0x0100",eReadWrite}, 
	{0x1c, 0x2c,"0x0100",eReadWrite}, 
	{0x2f, 0x4d,"0x0100",eReadWrite},     
	{0x38, 0x04,"0x0100",eReadWrite}, 
	{0x39, 0x02,"0x0100",eReadWrite}, 
	{0x3c, 0x08,"0x0100",eReadWrite}, 
	{0x3d, 0x0f,"0x0100",eReadWrite}, 
	{0xd3, 0xc4,"0x0100",eReadWrite}, 
	{0x43, 0x08,"0x0100",eReadWrite}, 
	{0x1d, 0x00,"0x0100",eReadWrite}, 

	/*ISP*/
	{0x80, 0x10,"0x0100",eReadWrite},                                                                    
	{0x89, 0x03,"0x0100",eReadWrite},                                                                    
	{0xfe, 0x01,"0x0100",eReadWrite},                                 
	{0x88, 0xf7,"0x0100",eReadWrite},                   
	{0x8a, 0x03,"0x0100",eReadWrite},                                                                    
	{0x8e, 0xc7,"0x0100",eReadWrite},                                                                  
                                                                                                                                         

	/*BLK*/
	{0xfe, 0x00,"0x0100",eReadWrite},
	{0x40, 0x22,"0x0100",eReadWrite},
	{0x41, 0x28,"0x0100",eReadWrite},
	{0x42, 0x04,"0x0100",eReadWrite},
	//{0x43, 0x03,"0x0100",eReadWrite},
	{0x4e, 0x0f,"0x0100",eReadWrite},
	{0x4f, 0xf0,"0x0100",eReadWrite},
	{0x67, 0x0c,"0x0100",eReadWrite},
	{0xae, 0x40,"0x0100",eReadWrite},
	{0xaf, 0x04,"0x0100",eReadWrite},
	{0x60, 0x00,"0x0100",eReadWrite},
	{0x61, 0x80,"0x0100",eReadWrite},
	
	/*Gain*/
	{0xb0, 0x58,"0x0100",eReadWrite},
	{0xb1, 0x01,"0x0100",eReadWrite},
	{0xb2, 0x00,"0x0100",eReadWrite},
	{0xb6, 0x00,"0x0100",eReadWrite},
/*	{0x9d, 0x19,"0x0100",eReadWrite},
	{0x9e, 0x1a,"0x0100",eReadWrite},
	{0x9f, 0x1b,"0x0100",eReadWrite},
	{0xa0, 0x1c,"0x0100",eReadWrite},
	{0xb0, 0x50,"0x0100",eReadWrite},
	{0xb1, 0x01,"0x0100",eReadWrite},
	{0xb2, 0x00,"0x0100",eReadWrite},
	{0xb6, 0x00,"0x0100",eReadWrite},*/

	/*DD*/
	/*{0xfe, 0x01,"0x0100",eReadWrite},
	{0xc2, 0x02,"0x0100",eReadWrite},
	{0xc3, 0xe0,"0x0100",eReadWrite},
	{0xc4, 0xd9,"0x0100",eReadWrite},
	{0xc5, 0x00,"0x0100",eReadWrite},
	{0xfe, 0x00,"0x0100",eReadWrite},*/
	
	/*Crop window*/
	{0x91, 0x00,"0x0100",eReadWrite},
	{0x92, 0x02,"0x0100",eReadWrite},
	{0x94, 0x03,"0x0100",eReadWrite},
	
	/*MIPI*/
	{0xfe, 0x03,"0x0100",eReadWrite},
	{0x02, 0x03,"0x0100",eReadWrite},
	{0x03, 0x8e,"0x0100",eReadWrite},
	{0x06, 0x80,"0x0100",eReadWrite},
	{0x15, 0x00,"0x0100",eReadWrite},
	{0x16, 0x09,"0x0100",eReadWrite},
	{0x18, 0x0a,"0x0100",eReadWrite},
	{0x21, 0x10,"0x0100",eReadWrite},
	{0x22, 0x05,"0x0100",eReadWrite},
	{0x23, 0x20,"0x0100",eReadWrite},
	{0x24, 0x02,"0x0100",eReadWrite},
	{0x25, 0x20,"0x0100",eReadWrite},
	{0x26, 0x08,"0x0100",eReadWrite},
	{0x29, 0x06,"0x0100",eReadWrite},
	{0x2a, 0x0a,"0x0100",eReadWrite},
	{0x2b, 0x08,"0x0100",eReadWrite},
 	{0xfe, 0x00,"0x0100",eReadWrite},
 	{0x3f ,0x91,"0x0100",eReadWrite},
	{0x00 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_aRegDescription_twolane_2[] =
{
	
	{0xfe, 0x00,"0x0100",eReadWrite}, 
	{0x1c, 0x1c,"0x0100",eReadWrite}, 
	{0x2f, 0x4a,"0x0100",eReadWrite},     
	{0x38, 0x02,"0x0100",eReadWrite}, 
	{0x39, 0x00,"0x0100",eReadWrite}, 
	{0x3c, 0x02,"0x0100",eReadWrite}, 
	{0x3d, 0x02,"0x0100",eReadWrite}, 
	{0xd3, 0xcc,"0x0100",eReadWrite}, 
	{0x43, 0x03,"0x0100",eReadWrite}, 
	{0x1d, 0x13,"0x0100",eReadWrite}, 
	
	{0x00 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_aRegDescription_twolane_3[] =
{
	
	{0xfe, 0x00,"0x0100",eReadWrite}, 
	{0x1c, 0x2c,"0x0100",eReadWrite}, 
	{0x2f, 0x4d,"0x0100",eReadWrite},     
	{0x38, 0x04,"0x0100",eReadWrite}, 
	{0x39, 0x02,"0x0100",eReadWrite}, 
	{0x3c, 0x08,"0x0100",eReadWrite}, 
	{0x3d, 0x0f,"0x0100",eReadWrite}, 
	{0xd3, 0xc4,"0x0100",eReadWrite}, 
	{0x43, 0x08,"0x0100",eReadWrite}, 
	{0x1d, 0x00,"0x0100",eReadWrite}, 
	
	{0x00 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_twolane_resolution_2592_1944[] =
{
	{0xfe,0x03,"0x0100",eReadWrite},
	{0x10,0x91,"0x0100",eReadWrite},
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}

};

const IsiRegDescription_t Sensor_g_2592x1944P30_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x07,"0x0100",eReadWrite},
	{0x08,0xc0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t Sensor_g_2592x1944P25_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x09,"0x0100",eReadWrite},
	{0x08,0x42,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P20_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x0b,"0x0100",eReadWrite},
	{0x08,0xa0,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t Sensor_g_2592x1944P15_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x0f,"0x0100",eReadWrite},
	{0x08,0x80,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};
const IsiRegDescription_t Sensor_g_2592x1944P10_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x17,"0x0100",eReadWrite},
	{0x08,0x40,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};

const IsiRegDescription_t Sensor_g_2592x1944P7_twolane_fpschg[] =
{
	{0xfe,0x00,"0x0100",eReadWrite},
	{0x07,0x21,"0x0100",eReadWrite},
	{0x08,0x36,"0x0100",eReadWrite},
	{0x0000 ,0x00,"eTableEnd",eTableEnd}
};


