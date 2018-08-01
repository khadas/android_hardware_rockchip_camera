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
 * @file vom_ctrl_common.h
 *
 * @brief
 *   Common stuff used by vom ctrl API & implementation.
 *
 *****************************************************************************/
/**
 * @page vom_ctrl_page VOM Ctrl
 * The Video Output Module displays image buffers handed in via QuadMVDU_FX on
 * a connected HDMI display device.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref vom_ctrl
 *
 * @defgroup vom_ctrl_common VOM Ctrl Common
 * @{
 *
 */

#ifndef __VOM_CTRL_COMMON_H__
#define __VOM_CTRL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Handle to vom ctrl process context.
 *
 */
typedef struct vomCtrlContext_s *vomCtrlHandle_t;

/**
 * @brief IDs of supported commands.
 *
 */
enum vomCtrl_command_e
{
    VOM_CTRL_CMD_START          = 0,
    VOM_CTRL_CMD_STOP           = 1,
    VOM_CTRL_CMD_SHUTDOWN       = 2,
    VOM_CTRL_CMD_PROCESS_BUFFER = 3
};

/**
 * @brief Data type used for commands (@ref vomCtrl_command_e).
 *
 */
typedef enum vomCtrl_command_e vomCtrlCmdId_t;

/**
 *  @brief Command completion signalling callback
 *
 *  Callback for signalling completion of commands which could require further application interaction.
 *
 */
typedef void (* vomCtrlCompletionCb_t)
(
    vomCtrlCmdId_t  CmdId,          /**< The type of command which was completed (see @ref vomCtrl_command_e). */
    RESULT          result,         /**< Result of the executed command. */
    const void      *pUserContext   /**< Opaque user data pointer that was passed in on vom control creation. */
);

/* @} vom_ctrl_common */

#ifdef __cplusplus
}
#endif

#endif /* __VOM_CTRL_COMMON_H__ */
