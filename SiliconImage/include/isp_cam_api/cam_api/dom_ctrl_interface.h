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
 * @file dom_ctrl_interface.h
 *
 * @brief
 *   DOM (Display Output Module) C++ API.
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

#ifndef __DOM_CTRL_ITF_H__
#define __DOM_CTRL_ITF_H__

#include <hal/hal_api.h>
#include <bufferpool/media_buffer.h>

#include "cam_api/camdevice.h"

/**
 * @brief DomCtrlItf class declaration.
 */
class DomCtrlItf : public BufferCb
{
public:
    /**
     * @brief Standard constructor for the DomCtrlItf object.
     */
    DomCtrlItf( HalHandle_t hHal, void *hParent = NULL );
    ~DomCtrlItf();

public:
    enum State
    {
        Invalid = 0,
        Idle,
        Running
    };

private:
    DomCtrlItf (const DomCtrlItf& other);
    DomCtrlItf& operator = (const DomCtrlItf& other);

public:
    State state() const;
    void* handle() const;

    virtual void  bufferCb( MediaBuffer_t *pBuffer );

    bool  start();
    bool  stop();

private:
    class    DomCtrlHolder;
    DomCtrlHolder *m_pDomCtrl;
};


/* @} module_name_api*/

#endif /*__DOM_CTRL_ITF_H__*/
