/*****************************************************************************/
/*!
 *  @file        utl_fixfloat.h
 *  @version     1.0
 *  @author      Neugebauer
 *  @brief       Floatingpoint to Fixpoint and vice versa conversion
 *               routines.
 */
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

#ifndef __UTL_FIXFLOAT_H__
#define __UTL_FIXFLOAT_H__

/*****************************************************************************
 * Prototypes
 *****************************************************************************/

uint32_t UtlFloatToFix_U0107( float fFloat );
float  UtlFixToFloat_U0107( uint32_t ulFix );

uint32_t UtlFloatToFix_U0208( float fFloat );
float  UtlFixToFloat_U0208( uint32_t ulFix );

uint32_t UtlFloatToFix_U0408( float fFloat );
float  UtlFixToFloat_U0408( uint32_t ulFix );

uint32_t UtlFloatToFix_U0800( float fFloat );
float  UtlFixToFloat_U0800( uint32_t ulFix );

uint32_t UtlFloatToFix_U1000( float fFloat );
float  UtlFixToFloat_U1000( uint32_t ulFix );

uint32_t UtlFloatToFix_U1200( float fFloat );
float  UtlFixToFloat_U1200( uint32_t ulFix );

uint32_t UtlFloatToFix_U0010( float fFloat );
float  UtlFixToFloat_U0010( uint32_t ulFix );

uint32_t UtlFloatToFix_S0207( float fFloat );
float  UtlFixToFloat_S0207( uint32_t ulFix );

uint32_t UtlFloatToFix_S0307( float fFloat );
float  UtlFixToFloat_S0307( uint32_t ulFix );

uint32_t UtlFloatToFix_S0407( float fFloat );
float  UtlFixToFloat_S0407( uint32_t ulFix );

uint32_t UtlFloatToFix_S0504( float fFloat );
float  UtlFixToFloat_S0504( uint32_t ulFix );

uint32_t UtlFloatToFix_S0808( float fFloat );
float  UtlFixToFloat_S0808( uint32_t ulFix );

uint32_t UtlFloatToFix_S0800( float fFloat );
float  UtlFixToFloat_S0800( uint32_t ulFix );

uint32_t UtlFloatToFix_S0900( float fFloat );
float  UtlFixToFloat_S0900( uint32_t ulFix );

uint32_t UtlFloatToFix_S1200( float fFloat );
float  UtlFixToFloat_S1200( uint32_t ulFix );

uint32_t UtlFloatToFix_S0109( float fFloat );
float UtlFixToFloat_S0109( uint32_t ulFix );

uint32_t UtlFloatToFix_S0408( float fFloat );
float UtlFixToFloat_S0408( uint32_t ulFix );

uint32_t UtlFloatToFix_S0108( float fFloat );
float UtlFixToFloat_S0108( uint32_t ulFix );

uint32_t UtlFloatToFix_S0110( float fFloat );
float UtlFixToFloat_S0110( uint32_t ulFix );

#endif /* __UTL_FIXfloat_H__ */

