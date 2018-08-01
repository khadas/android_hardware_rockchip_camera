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
 * @file mom_ctrl_common.h
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
#ifndef __MOM_CTRL_COMMON_H__
#define __MOM_CTRL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Handle to mom ctrl process context.
 *
 * @note
 *
 */
typedef struct MomCtrlContext_s *MomCtrlContextHandle_t;



/**
 * @brief
 *
 * @note
 *
 */
enum MomCtrlPath_e
{
    MOM_CTRL_PATH_INVALID                   = 0,
    MOM_CTRL_PATH_MAINPATH                  = 1,
    MOM_CTRL_PATH_SELFPATH                  = 2,
    MOM_CTRL_PATH_MAX                       = 3,
};



/**
 * @brief
 *
 * @note
 *
 */
enum MomCtrlCommand_e
{
    MOM_CTRL_CMD_INVALID                    = 0,
    MOM_CTRL_CMD_START                      = 1,
    MOM_CTRL_CMD_STOP                       = 2,
    MOM_CTRL_CMD_SHUTDOWN                   = 3,
    MOM_CTRL_CMD_PROC_FULL_BUFFER_MAINPATH  = 4,
    MOM_CTRL_CMD_PROC_FULL_BUFFER_SELFPATH  = 5,
};



/**
 * @brief
 *
 * @note
 *
 */
typedef int32_t MomCtrlCmdId_t;



/**
 *  @brief Event signalling callback
 *
 *  Callback for signalling some event which could require application interaction.
 *  The eventId (see @ref momEventId_t) identifies the event and the content of
 *  pParam depends on the event ID.
 *
 */
typedef void (* MomCtrlCompletionCb_t)
(
    MomCtrlCmdId_t      CmdId,          /**< The Commad Id of the notifying event */
    RESULT              result,         /**< Result of the executed command */
    const void          *pUserContext   /**< User data pointer that was passed on chain creation */
);



typedef struct MomCtrlDrawContext_s *MomCtrlDrawHandle_t;


typedef struct MomCtrlDrawConfig_s
{
    MomCtrlDrawHandle_t     hDrawContext;
} MomCtrlDrawConfig_t;


typedef void (*MomCtrlBufferCb_t)
(
    int32_t             path,
    MediaBuffer_t       *pMediaBuffer,
    void                *pBufferCbCtx
);


typedef struct MomCtrlBuffer_s
{
    MomCtrlBufferCb_t   fpCallback;      /**< Buffer callback */
    void                *pBufferCbCtx;   /**< Pointer to user context to pass to callback */
} MomCtrlBuffer_t;


typedef struct ibdCmd_t     MomCtrlDrawCmd_t;

#ifdef __cplusplus
}
#endif

/* @} module_name*/

#endif /* __MOM_CTRL_COMMON_H__ */

