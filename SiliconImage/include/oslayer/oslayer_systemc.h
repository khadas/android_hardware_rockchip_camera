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
 * @file oslayer_systemc.h
 *
 * Encapsulates and abstracts services of SystemC
 *
 *****************************************************************************/
#ifdef LINUX

#ifdef MSVD_COSIM

#include <ebase/types.h>
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef int32_t (*osThreadFunc)(void *);
  typedef int32_t (*osIsrFunc)(void *);
  typedef int32_t (*osDpcFunc)(void *);

  typedef struct _osScEvent osScEvent;
  typedef struct _osScMutex osScMutex;
  typedef struct _osScSemaphore osScSemaphore;
  typedef struct _osScThreadHandle osScThreadHandle;


#ifdef OSLAYER_EVENT
/*****************************************************************************/
/*  @brief  Event object (Linux Version) of OS Abstraction Layer */
typedef struct _osEvent
{
  osScEvent* p_event;
} osEvent;
#endif /* OSLAYER_EVENT */


#ifdef OSLAYER_MUTEX
/*****************************************************************************/
/*  @brief  Mutex object (Linux Version) of OS Abstraction Layer */
typedef struct _osMutex
{
  osScMutex* p_mutex;
} osMutex;
#endif /* OSLAYER_MUTEX */


#ifdef OSLAYER_SEMAPHORE
/*****************************************************************************/
/*  @brief  Semaphore object (Linux Version) of OS Abstraction Layer */
typedef struct _osSemaphore
{
  osScSemaphore* p_semaphore;
} osSemaphore;
#endif /* OSLAYER_SEMAPHORE */


#ifdef OSLAYER_THREAD
/*****************************************************************************/
/*  @brief  Thread object (Linux Version) of OS Abstraction Layer */

typedef struct _osThread
{
  osScThreadHandle* p_handle;
  int32_t (*pThreadFunc)(void *);
  void *p_arg;
} osThread;
#endif /* OSLAYER_THREAD */

#ifdef __cplusplus
}
#endif


#endif /*! MSVD_COSIM */

#endif /* LINUX */
