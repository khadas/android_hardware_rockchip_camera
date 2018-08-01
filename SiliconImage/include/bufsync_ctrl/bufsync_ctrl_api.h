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
 * @file bufsync_ctrl_api.h
 *
 * @brief
 *   ADD_DESCRIPTION_HERE
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup module_name Module Name
 * @{
 *
 */
#ifndef __BUFSYNC_CTRL_API_H__
#define __BUFSYNC_CTRL_API_H__

#include <ebase/types.h>
#include <oslayer/oslayer.h>
#include <common/return_codes.h>

#include <bufferpool/media_buffer.h>
#include <bufferpool/media_buffer_queue_ex.h>

#include "bufsync_ctrl_common.h"

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * @brief   Configuration structure of the bufsync-ctrl
 *
 * @note
 *
 */
typedef struct BufSyncCtrlConfig_s
{
    uint32_t                    MaxPendingCommands;     /**< Number of commands that can be queued and thus be pending at a time. */

    osQueue                     *pPicBufQueue1;         /**< Reference to output queue to connect to. */
    osQueue                     *pPicBufQueue2;         /**< Reference to output queue to connect to. */

    BufSyncCtrlCompletionCb_t   bufsyncCbCompletion;    /**< Callback function for command completion. */
    void                        *pUserContext;          /**< User context passed on to completion callback. */

    BufSyncCtrlHandle_t         hBufSyncCtrl;           /**< Handle to bufsync control context, set by @ref BufSyncCtrlInit if successfull, undefined otherwise. */
} BufSyncCtrlConfig_t;



/*****************************************************************************/
/**
 * @brief   Initialize the BufSync-Control
 *
 * This functions initializes the Buffer Synchronisation Module .
 *
 * Some detailed description goes here ...
 *
 * @param   param1      Describe the parameter 1.
 * @param   param2      Describe the parameter 2
 *
 * @return              Return the result of the function call.
 * @retval              RET_VAL1
 * @retval              RET_VAL2
 *
 *****************************************************************************/
RESULT BufSyncCtrlInit
(
    BufSyncCtrlConfig_t *pConfig
);



/*****************************************************************************/
/**
 * @brief   Shutdown the BufSync-Control
 *
 * This functions releases and shutdowns the Buffer Synchronisation Module.
 *
 * Some detailed description goes here ...
 *
 * @param   param1      Describe the parameter 1.
 * @param   param2      Describe the parameter 2
 *
 * @return              Return the result of the function call.
 * @retval              RET_VAL1
 * @retval              RET_VAL2
 *
 *****************************************************************************/
RESULT BufSyncCtrlShutDown
(
    BufSyncCtrlHandle_t hBufSyncCtrl
);



/*****************************************************************************/
/**
 * @brief   Start the BufSync-Control 
 *
 * This functions starts the Buffer Synchronisation Module.
 *
 * Some detailed description goes here ...
 *
 * @param   param1      Describe the parameter 1.
 * @param   param2      Describe the parameter 2
 *
 * @return              Return the result of the function call.
 * @retval              RET_VAL1
 * @retval              RET_VAL2
 *
 *****************************************************************************/
RESULT BufSyncCtrlStart
(
    BufSyncCtrlHandle_t hBufSyncCtrl
);



/*****************************************************************************/
/**
 * @brief   Stop the BufSync-Control 
 *
 * This functions stops the Buffer Synchronisation Module.
 *
 * Some detailed description goes here ...
 *
 * @param   param1      Describe the parameter 1.
 * @param   param2      Describe the parameter 2
 *
 * @return              Return the result of the function call.
 * @retval              RET_VAL1
 * @retval              RET_VAL2
 *
 *****************************************************************************/
RESULT BufSyncCtrlStop
(
    BufSyncCtrlHandle_t hBufSyncCtrl
);



/*****************************************************************************/
/**
 * @brief   TODO
 *
 *****************************************************************************/
RESULT  BufSyncCtrlRegisterBufferCb
(
    BufSyncCtrlHandle_t     hBufSyncCtrl,
    BufSyncCtrlBufferCb_t   fpCallback,
    void                    *pBufferCbCtx
);



/*****************************************************************************/
/**
 * @brief   TODO
 *
 *****************************************************************************/
RESULT  BufSyncCtrlDeRegisterBufferCb
(
    BufSyncCtrlHandle_t  hBufSyncCtrl
);



#ifdef __cplusplus
}
#endif


/* @} module_name*/

#endif /* __BUFSYNC_CTRL_API_H__ */

