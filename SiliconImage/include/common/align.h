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
 * @file align.h
 *
 * @brief
 *
 *****************************************************************************/
#ifndef __ALIGN_H__
#define __ALIGN_H__

#define ALIGN_UP(addr, align)		( ((addr) + ((align)-1)) & ~((align)-1) ) //!< Aligns addr to next higher aligned addr; align must be a power of two.
#define ALIGN_DOWN(addr, align)		( ((addr)              ) & ~((align)-1) ) //!< Aligns addr to next lower aligned addr; align must be a power of two.

#define ALIGN_SIZE_1K               ( 0x400 )
#define ALIGN_UP_1K(addr)			( ALIGN_UP(addr, ALIGN_SIZE_1K) )

#define MAX_ALIGNED_SIZE(size, align) ( ALIGN_UP(size, align) + align ) //!< Calcs max size of memory required to be able to hold a block of size bytes with a start address aligned to align.

#endif /* __ALIGN_H__ */
