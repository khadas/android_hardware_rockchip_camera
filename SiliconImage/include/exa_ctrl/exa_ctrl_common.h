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
 * @file exa_ctrl_common.h
 *
 * @brief
 *   Common stuff used by exa ctrl API & implementation.
 *
 *****************************************************************************/
/**
 * @page exa_ctrl_page EXA Ctrl
 * The External Algorithm Module captures image and calls external algorithm.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref exa_ctrl
 *
 * @defgroup exa_ctrl_common EXA Ctrl Common
 * @{
 *
 */

#ifndef __EXA_CTRL_COMMON_H__
#define __EXA_CTRL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  @brief External algorithm callback
 *
 *  Callback for external algorithm. Called when new sample available.
 *
 */
typedef RESULT (* exaCtrlSampleCb_t)
(
    PicBufMetaData_t    *pSampleBuffer,
    void                *pSampleContext   //!< Opaque sample data pointer that was passed in on exa control creation.
);

/**
 * @brief Handle to exa ctrl process context.
 *
 */
typedef struct exaCtrlContext_s *exaCtrlHandle_t;

/**
 * @brief IDs of supported commands.
 *
 */
typedef enum exaCtrl_command_e
{
    EXA_CTRL_CMD_START          = 0,
    EXA_CTRL_CMD_STOP           = 1,
    EXA_CTRL_CMD_PAUSE          = 2,
    EXA_CTRL_CMD_RESUME         = 3,
    EXA_CTRL_CMD_SHUTDOWN       = 4,
    EXA_CTRL_CMD_PROCESS_BUFFER = 5
} exaCtrlCmdID_t;

/**
 * @brief Data type used for commands (@ref exaCtrl_command_e).
 *
 */
typedef struct exaCtrlCmd_s
{
    exaCtrlCmdID_t CmdID;   //!< The command to execute.
    union
    {
        struct
        {
            exaCtrlSampleCb_t   exaCbSample;    //!< External algorithm callback
            void                *pSampleContext;//!< Sample context passed on to sample callback.
            uint8_t             SampleSkip;     //!< Skip consecutive samples
        } Start;                //!< Params structure for @ref EXA_CTRL_CMD_START.
    } Params;               //!< Params of the command to execute.
} exaCtrlCmd_t;

/**
 *  @brief Command completion signalling callback
 *
 *  Callback for signalling completion of commands which could require further application interaction.
 *
 */
typedef void (* exaCtrlCompletionCb_t)
(
    exaCtrlCmdID_t  CmdId,          //!< The type of command which was completed (see @ref exaCtrl_command_e).
    RESULT          result,         //!< Result of the executed command.
    void            *pUserContext   //!< Opaque user data pointer that was passed in on exa control creation.
);

/* @} exa_ctrl_common */

#ifdef __cplusplus
}
#endif

#endif /* __EXA_CTRL_COMMON_H__ */
