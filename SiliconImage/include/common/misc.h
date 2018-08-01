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
 * @file misc.h
 *
 * @brief   Some often used little helpers; mainly implemented as macros.
 *
 *****************************************************************************/
#ifndef __MISC_H__
#define __MISC_H__

/* beware of macro side effects! */

#ifndef __FLT_EPSILON__
#define __FLT_EPSILON__     0.000000119209289550781250000000
#endif /* __FLT_EPSILON__ */

#ifndef FLT_EPSILON
#define FLT_EPSILON         __FLT_EPSILON__
#endif /* FLT_EPSILON */

#ifndef FLT_MAX
#define FLT_MAX     ((float)3.40282346638528860e+38)
#endif /* FLT_MAX */

#ifndef MIN
#define MIN(a, b)   ( ((a)<(b)) ? (a) : (b) )
#endif /* MIN */

#ifndef MAX
#define MAX(a, b)   ( ((a)>(b)) ? (a) : (b) )
#endif /* MAX */

#ifndef ABS
#define ABS(a)      ( ((a)<0) ? -(a) : (a) )
#endif /*ABS */

#ifndef SIGN
#define SIGN(a)     ( ((a)<0) ? -1 : ((a)>0) ? 1 : 0 )
#endif /* SIGN */

#endif /* __MISC_H__ */
