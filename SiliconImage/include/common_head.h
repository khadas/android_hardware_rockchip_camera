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
#ifndef __COMMON_HEAD_H__
#define __COMMON_HEAD_H__

//Sensor power up sequence define
//type: bit0-bit4
#define SENSOR_PWRSEQ_BEGIN         0x00
#define SENSOR_PWRSEQ_AVDD          0x01
#define SENSOR_PWRSEQ_DOVDD         0x02
#define SENSOR_PWRSEQ_DVDD          0x03
#define SENSOR_PWRSEQ_CLKIN         0x04
#define SENSOR_PWRSEQ_PWR           0x05
#define SENSOR_PWRSEQ_RST         	0x06
#define SENSOR_PWRSEQ_PWRDN         0x07
#define SENSOR_PWRSEQ_END           0x0F
#define SENSOR_PWRSEQ_CNT           0x07

#define VCM_PWRSEQ_BEGIN         	0x00
#define VCM_PWRSEQ_VDD         		0x01
#define VCM_PWRSEQ_PWR         		0x02
#define VCM_PWRSEQ_PWRDN       		0x03
#define VCM_PWRSEQ_END         		0x0F
#define VCM_PWRSEQ_CNT           	0x03


#define POWERSEQ_SET(type,idx) 		(type<<(idx*4))
#define POWERSEQ_GET(seq,idx)     	((seq>>(idx*4))&0x0f)


#endif
