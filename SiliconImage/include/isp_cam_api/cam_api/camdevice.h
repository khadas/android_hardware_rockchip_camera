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
 * @file camdevice.h
 *
 * @brief
 *   Cam Device C++ API.
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

#ifndef __CAMDEVICE_H__
#define __CAMDEVICE_H__

#include "cam_api/cam_engine_interface.h"
#include <exa_ctrl/exa_ctrl_api.h>

class DomCtrlItf;
class VomCtrlItf;
class SomCtrlItf;
class ExaCtrlItf;
struct BufferCbContext;

/**
 * @brief BufferCb class declaration.
 */
class BufferCb
{
public:
    virtual void bufferCb( MediaBuffer_t* pMediaBuffer ) = 0;
    virtual ~BufferCb() = 0;
};

inline BufferCb::~BufferCb() { }

/**
 * @brief CamDevice class declaration.
 */
class CamDevice : public CamEngineItf
{
public:
    /**
     * @brief Standard constructor for the CamDevice object.
     */
    CamDevice( HalHandle_t hHal, AfpsResChangeCb_t *pcbResChange = NULL, void *ctxCbResChange = NULL,void* hParent=NULL, int mipiLaneNum=1 );
    ~CamDevice();

private:
    CamDevice (const CamDevice& other);
    CamDevice& operator = (const CamDevice& other);

public:
    //zyc add
    bool setSensorResConfig(uint32_t mask);
    
    
    void enableDisplayOutput( bool enable = true );
    void enableVideoOutput( bool enable = true );
    void enableExternalAlgorithm( bool enable = true );

    bool connectCamera( bool preview = true, BufferCb *bufferCb = NULL );
    void disconnectCamera();
    void resetCamera();

    bool startPreview();
    bool pausePreview();
    bool stopPreview ();

    bool captureSnapshot( const char* fileName, int type, uint32_t resolution, CamEngineLockType_t locks = CAM_ENGINE_LOCK_ALL );

    void registerExternalAlgorithmCallback( exaCtrlSampleCb_t sampleCb, void *pSampleContext, uint8_t sampleSkip );

private:
    friend class PfidItf;

    bool            m_enableDom;
    bool            m_enableVom;
    bool            m_enableExa;

    DomCtrlItf      *m_domCtrl;
    VomCtrlItf      *m_vomCtrl;
    SomCtrlItf      *m_somCtrl;
    ExaCtrlItf      *m_exaCtrl;

    BufferCbContext *m_bufferCbCtx;

    exaCtrlSampleCb_t m_sampleCb;
    void            *m_sampleCbCtx;
    uint8_t         m_sampleSkip;
};


/* @} module_name_api*/

#endif /*__CAMDEVICE_H__ */
