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
#ifndef __MIM_CTRL_COMMON_H__
#define __MIM_CTRL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Handle to mom ctrl process context.
 *
 */
typedef struct MimCtrlContext_s *MimCtrlContextHandle_t;


/**
 * @brief
 *
 * @note
 *
 */
enum MimCtrl_command_e
{
	MIM_CTRL_CMD_INVALID        = 0,
	MIM_CTRL_CMD_START          = 1,
	MIM_CTRL_CMD_STOP           = 2,
	MIM_CTRL_CMD_SHUTDOWN       = 3,
	MIM_CTRL_CMD_DMA_TRANSFER   = 4,
};

typedef int32_t MimCtrlCmdId_t;

/**
 *  @brief Event signalling callback
 *
 *  Callback for signalling some event which could require application interaction.
 *  The eventId (see @ref MimEventId_t) identifies the event and the content of 
 *  pParam depends on the event ID.
 *
 */
typedef void (* MimCtrlCompletionCb_t)
(
    MimCtrlCmdId_t  CmdId,          /**< The Commad Id of the notifying event */
    RESULT          result,         /**< Result of the executed command */
    const void      *pUserContext   /**< User data pointer that was passed on chain creation */
);


/* @} module_name*/

#ifdef __cplusplus
}
#endif


#endif /* __MIM_CTRL_COMMON_H__ */
