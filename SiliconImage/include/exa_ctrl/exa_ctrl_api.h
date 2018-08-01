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
 * @file exa_ctrl_api.h
 *
 * @brief
 *   Definition of exa ctrl API.
 *
 *****************************************************************************/
/**
 * @page exa_ctrl_page EXA Ctrl
 * The External Algorithm Module captures image and calls external algorithm.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref exa_ctrl_api
 * - @ref exa_ctrl_common
 * - @ref exa_ctrl
 *
 * @defgroup exa_ctrl_api EXA Ctrl API
 * @{
 *
 */

#ifndef __EXA_CTRL_API_H__
#define __EXA_CTRL_API_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <ebase/types.h>
#include <common/return_codes.h>
#include <common/cea_861.h>
#include <common/picture_buffer.h>

#include <bufferpool/media_buffer.h>

#include <hal/hal_api.h>

#include "exa_ctrl_common.h"

typedef struct exaCtrlConfig_s
{
    uint32_t                MaxPendingCommands; //!< Number of commands that can be queued and thus be pending at a time.
    uint32_t                MaxBuffers;
    exaCtrlCompletionCb_t   exaCbCompletion;    //!< Callback function for command completion.
    void                    *pUserContext;      //!< User context passed on to completion callback.
    HalHandle_t             HalHandle;          //!< HAL session to use for HW access

    exaCtrlHandle_t         exaCtrlHandle;      //!< Handle to created exa context, set by @ref exaCtrlInit if successfull, undefined otherwise.
} exaCtrlConfig_t;

extern RESULT exaCtrlInit
(
    exaCtrlConfig_t *pConfig            //!< Reference to configuration structure.
);

extern RESULT exaCtrlShutDown
(
    exaCtrlHandle_t exaCtrlHandle       //!< Handle to exa context as returned by @ref exaCtrlInit.
);

extern RESULT exaCtrlStart
(
    exaCtrlHandle_t     exaCtrlHandle,  //!< Handle to exa context as returned by @ref exaCtrlInit.
    exaCtrlSampleCb_t   exaCbSample,    //!< External algorithm callback
    void                *pSampleContext,//!< Sample context passed on to sample callback.
    uint8_t             SampleSkip      //!< Skip consecutive samples
);

extern RESULT exaCtrlStop
(
    exaCtrlHandle_t exaCtrlHandle       //!< Handle to exa context as returned by @ref exaCtrlInit.
);

extern RESULT exaCtrlPause
(
    exaCtrlHandle_t exaCtrlHandle       //!< Handle to exa context as returned by @ref exaCtrlInit.
);

extern RESULT exaCtrlResume
(
    exaCtrlHandle_t exaCtrlHandle       //!< Handle to exa context as returned by @ref exaCtrlInit.
);

extern RESULT  exaCtrlShowBuffer
(
    exaCtrlHandle_t         exaCtrlHandle,
    MediaBuffer_t           *pBuffer
);

/* @} exa_ctrl_api */

#ifdef __cplusplus
}
#endif

#endif /* __EXA_CTRL_API_H__ */
