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
 * @file exa_ctrl_interface.h
 *
 * @brief
 *   EXA (External Algorithm Module) C++ API.
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

#ifndef __EXA_CTRL_ITF_H__
#define __EXA_CTRL_ITF_H__

#include <hal/hal_api.h>
#include <bufferpool/media_buffer.h>
#include <exa_ctrl/exa_ctrl_api.h>

#include "cam_api/camdevice.h"

/**
 * @brief ExaCtrlItf class declaration.
 */
class ExaCtrlItf : public BufferCb
{
public:
    /**
     * @brief Standard constructor for the ExaCtrlItf object.
     */
    ExaCtrlItf( HalHandle_t hHal );
    ~ExaCtrlItf();

public:
    enum State
    {
        Invalid = 0,
        Idle,
        Running
    };

private:
    ExaCtrlItf (const ExaCtrlItf& other);
    ExaCtrlItf& operator = (const ExaCtrlItf& other);

public:
    State state() const;

    virtual void  bufferCb( MediaBuffer_t *pBuffer );

    bool  start( exaCtrlSampleCb_t sampleCb, void *pSampleContext, uint8_t sampleSkip = 10 );
    bool  stop();

private:
    class    ExaCtrlHolder;
    ExaCtrlHolder *m_pExaCtrl;
};


/* @} module_name_api*/

#endif /*__EXA_CTRL_ITF_H__*/
