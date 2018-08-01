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
 * @file som_ctrl_common.h
 *
 * @brief
 *   Common stuff used by som ctrl API & implementation.
 *
 *****************************************************************************/
/**
 * @page som_ctrl_page SOM Ctrl
 * The Snapshot Output Module captures image buffers handed in on disk.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref som_ctrl
 *
 * @defgroup som_ctrl_common SOM Ctrl Common
 * @{
 *
 */

#ifndef __SOM_CTRL_COMMON_H__
#define __SOM_CTRL_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Handle to som ctrl process context.
 *
 */
typedef struct somCtrlContext_s *somCtrlHandle_t;

/**
 * @brief IDs of supported commands.
 *
 */
typedef enum somCtrlCmdID_e
{
    SOM_CTRL_CMD_START          = 0,
    SOM_CTRL_CMD_STOP           = 1,
    SOM_CTRL_CMD_SHUTDOWN       = 2,
    SOM_CTRL_CMD_PROCESS_BUFFER = 3
} somCtrlCmdID_t;

/**
 * @brief Parameter structure for supported commands.
 *
 */
typedef union somCtrlCmdParams_u
{
    struct
    {
        const char  *szBaseFileName;    //!< Base filename; buffer type and size information will be added to name as well as mumber of image in sequence if applicable.
        uint32_t    NumOfFrames;        //!< Number of frames/buffers to capture; 0 (zero) for continuous capture until @ref SOM_CTRL_CMD_STOP is issued.
        uint32_t    NumSkipFrames;      //!< Number of frames/buffers to skip before capture starts.
        uint32_t    AverageFrames;      //!< Calculate and store per pixel average data; Note: only for RAW type images; 1 < @ref NumOfFrames < 65535 must hold true.
        bool_t      ForceRGBOut;        //!< Converts YCbCr & RGB type image buffers to standard RGB for viewing with any image viewer;
                                        //!< @note: YCbCr is upscaled to 4:4:4 by simple pixel/line doubling as necessary & sub 8bit resolution per component is scaled to 8bit per component by simple left shift operations
        bool_t      ExtendName;         //!< Automatically extend the base filename with format, size, date/time, sequence and other information
        bool_t      Exif;               //!< write exif data
    } Start;                //!< Params structure for @ref SOM_CTRL_CMD_START.
} somCtrlCmdParams_t;


/**
 * @brief Data type used for commands (@ref somCtrlCmdID_e).
 *
 */
typedef struct somCtrlCmd_s
{
    somCtrlCmdID_t      CmdID;      //!< The command to execute.
    somCtrlCmdParams_t  *pParams;   //!< A reference to the params structure for this command. @note: The referenced data must be valid until the command is completed!
} somCtrlCmd_t;

/**
 * @brief Data type used to convey command individual information on completion.
 *
 */
typedef union somCtrlCompletionInfo_u
{
    struct
    {
        uint32_t    FramesCaptured; //!< Number of frames/buffers captured.
        uint32_t    FramesLost;     //!< Number of frames/buffers lost during capture due to high watermark concealment.
    } Start;                //!< Info structure for @ref SOM_CTRL_CMD_START.
} somCtrlCompletionInfo_t;
/**
 *  @brief Command completion signalling callback
 *
 *  Callback for signalling completion of commands which could require further application interaction.
 *
 */
typedef void (* somCtrlCompletionCb_t)
(
    somCtrlCmdID_t          CmdId,          //!< The type of command which was completed (see @ref somCtrlCmdID_e).
    RESULT                  result,         //!< Result of the executed command.
    somCtrlCmdParams_t      *pParams,       //!< The Reference to the params structure for this command. The structure is no longer referenced and may e.g. be free'd or whatever is appropriate.
    somCtrlCompletionInfo_t *pInfo,         //!< Further information for the command completed (e.g num of frames captured etc.); may be NULL if not applicable or no info available.
    void                    *pUserContext   //!< Opaque user data pointer that was passed in on som control creation.
);

/* @} som_ctrl_common */

#ifdef __cplusplus
}
#endif

#endif /* __SOM_CTRL_COMMON_H__ */
