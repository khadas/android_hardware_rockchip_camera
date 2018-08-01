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
 *   @file builtins.h
 *
 *	This file defines some macros for standard library functions. Usually we
 *	dont link against glibc, so we use the builtins. Standard library function
 *	calls are only permitted in debug mode.
 *
 *****************************************************************************/
#ifndef BUILTINS_H_
#define BUILTINS_H_

#include "types.h"

#if defined(__GNUC__)
	#include <stddef.h>

	void* __builtin_memset( void* s, int32_t c, size_t n );
	#define MEMSET(	TARGET, C, LEN)	__builtin_memset(TARGET, C, LEN)

	void* __builtin_memcpy( void* s1, const void* s2, size_t n);
	#define MEMCPY( DST, SRC, LEN)	__builtin_memcpy(DST,SRC,LEN)
#else
	#include <string.h>
	#define MEMSET(	TARGET, C, LEN)	memset(TARGET,C,LEN)
	#define MEMCPY( DST, SRC, LEN)	memcpy(DST,SRC,LEN)
#endif

#define WIPEOBJ( TARGET ) MEMSET( &TARGET, 0, sizeof( TARGET ) )

#endif /*BUILTINS_H_*/
