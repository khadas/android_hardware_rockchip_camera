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
 * @file    calibtreewidget.h
 *
 *
 *****************************************************************************/
#ifndef __CALIBDB_H__
#define __CALIBDB_H__

//#include <QDomDocument>
//#include <QtXml>
#include <tinyxml2.h>
#include <ebase/builtins.h>
#include <ebase/dct_assert.h>

#include <common/return_codes.h>
#include <common/cam_types.h>

#include <cam_calibdb/cam_calibdb_api.h>
using namespace tinyxml2;

struct sensor_calib_info{
    CamCalibDbMetaData_t meta_data;
    CamResolution_t resolution;
    CamCalibAecGlobal_t aec_data;
    CamEcmProfile_t EcmProfile;
    CamEcmScheme_t EcmScheme;
    CamCalibAwbGlobal_t awb_data;
    CamIlluProfile_t illu;
    CamLscProfile_t lsc_profile;
    CamCcProfile_t cc_profile;
    CamBlsProfile_t bls_profile;
    CamCacProfile_t cac_profile;
    CamDpfProfile_t dpf_profile;
    CamDpccProfile_t dpcc_profile;
    CamCalibSystemData_t system_data;
};


class CalibDb
{
public:
    CalibDb( );
    ~CalibDb( );

    CamCalibDbHandle_t GetCalibDbHandle( void )
    {
        return ( m_CalibDbHandle );
    }
    
    bool SetCalibDbHandle( CamCalibDbHandle_t phandle )
    {
        if(phandle){
            m_CalibDbHandle = phandle;
            return true;
        }
       
       return false;
    }
    
    bool CreateCalibDb( const XMLElement* );
    bool CreateCalibDb( const char *device );
    struct sensor_calib_info * GetCalibDbInfo(){
        return &(m_CalibInfo);
    }
    bool SetCalibDbInfo( struct sensor_calib_info *pCalibInfo ){
        MEMCPY(&m_CalibInfo, pCalibInfo, sizeof(struct sensor_calib_info));
        return true;
    }
	bool GetCalibXMLVersion(char *XMLVerBuf, int buflen);
private:

    typedef bool (CalibDb::*parseCellContent)(const XMLElement*, void *param);

    // parse helper
    bool parseEntryCell( const XMLElement*, int, parseCellContent, void *param = NULL );

    // parse Header
    bool parseEntryHeader( const XMLElement*, void *param = NULL );
    bool parseEntryResolution( const XMLElement*, void *param = NULL );
	bool parseEntryOTPInfo( const XMLElement*, void *param = NULL );    
    bool parseEntryFramerates( const XMLElement*, void *param = NULL );

    // parse Sensor
    bool parseEntrySensor( const XMLElement*, void *param = NULL );

    // parse Sensor-AWB
    bool parseEntryAwb( const XMLElement*, void *param = NULL );
    bool parseEntryAwbGlobals( const XMLElement*, void *param = NULL );
    bool parseEntryAwbIllumination( const XMLElement*, void *param = NULL );
    bool parseEntryAwbIlluminationAlsc( const XMLElement*, void *param = NULL );
    bool parseEntryAwbIlluminationAcc( const XMLElement*, void *param = NULL );

    // parse Sensor-AEC
    bool parseEntryAec( const XMLElement*, void *param = NULL );
    bool parseEntryAecEcm( const XMLElement*, void *param = NULL );
    bool parseEntryAecEcmPriorityScheme( const XMLElement*, void *param = NULL );
	// parse Sensor-GammaOut
	bool parseEntryGammaOut( const XMLElement*, void *param = NULL );
    // parse Sensor-LSC
    bool parseEntryLsc( const XMLElement*, void *param = NULL );

    // parse Sensor-CC
    bool parseEntryCc( const XMLElement*, void *param = NULL );

    // parse Sensor-BLS
    bool parseEntryBls( const XMLElement*, void *param = NULL );

    // parse Sensor-CAC
    bool parseEntryCac( const XMLElement*, void *param = NULL );

    // parse Sensor-DPF
    bool parseEntryDpf( const XMLElement*, void *param = NULL );

    // parse Sensor-DPCC
    bool parseEntryDpcc( const XMLElement*, void *param = NULL );
    bool parseEntryDpccRegisters( const XMLElement*, void *param = NULL );

    // parse System
    bool parseEntrySystem( const XMLElement*, void *param = NULL );


void characterDataHandler(void *userData,const char *s,int len);

void startElementHandler(void *userData, const char *name, const char **atts);

private:

    CamCalibDbHandle_t  m_CalibDbHandle;
    struct sensor_calib_info m_CalibInfo;
};


#endif /* __CALIBDB_H__ */
