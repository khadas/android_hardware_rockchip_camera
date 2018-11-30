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
#ifndef __AWB_COMMON_H__
#define __AWB_COMMON_H__

/**
 * @file awb_common.h
 *
 * @brief
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup AWBM Auto white Balance Module
 * @{
 *
 */
#include <ebase/types.h>
#include <common/return_codes.h>
#include <common/cam_types.h>

#include <isi/isi_iss.h>
#include <isi/isi.h>

#include <cameric_drv/cameric_drv_api.h>
#include <cameric_drv/cameric_isp_awb_drv_api.h>
#include <cameric_drv/cameric_isp_lsc_drv_api.h>
#include <cameric_drv/cameric_isp_hist_drv_api.h>

#include <cam_calibdb/cam_calibdb_api.h>

#define ABS_DIFF( a, b )     ( (a > b) ? (a-b) : (b-a) )

#ifdef __cplusplus
extern "C"
{
#endif



/*****************************************************************************/
/**
 * @brief
 */
/*****************************************************************************/
typedef struct AwbContext_s *AwbHandle_t;           /**< handle to AWB context */



/*****************************************************************************/
/**
 * @brief
 */
/*****************************************************************************/
typedef enum AwbWorkingFlags_e
{
    AWB_WORKING_FLAG_USE_DAMPING        = 0x01,
    AWB_WORKING_FLAG_USE_CC_OFFSET      = 0x02
} AwbWorkingFlags_t;



/*****************************************************************************/
/**
 * @brief
 *
 */
/*****************************************************************************/
typedef enum AwbRunMode_e
{
    AWB_MODE_INVALID                    = 0,        /**< initialization value */
    AWB_MODE_MANUAL                     = 1,        /**< run manual white balance */
    AWB_MODE_AUTO                       = 2,        /**< run auto white balance */
    AWB_MODE_MANUAL_CT                     = 3,        /**< run manual white balance */
    AWB_MODE_MAX
    
} AwbMode_t;



/*****************************************************************************/
/**
 * @brief   type for evaluatiing number of white pixel
 */
/*****************************************************************************/
typedef enum AwbNumWhitePixelEval_e
{
    AWB_NUM_WHITE_PIXEL_INVALID         = 0,        /**< initialization value */
    AWB_NUM_WHITE_PIXEL_LTMIN           = 1,        /**< less than configured minimum */
    AWB_NUM_WHITE_PIXEL_GTMAX           = 2,        /**< greater than defined maximum */
    AWB_NUM_WHITE_PIXEL_TARGET_RANGE    = 3,        /**< in min max range */
    AWB_NUM_WHITE_PIXEL_MAX
} AwbNumWhitePixelEval_t;



/*****************************************************************************/
/**
 * @brief
 *
 */
/*****************************************************************************/
typedef struct AwbComponent_s
{
    float   fRed;
    float   fGreen;
    float   fBlue;
} AwbComponent_t;



/*****************************************************************************/
/**
 * @brief   A structure/tupple to represent gain values for four (R,Gr,Gb,B)
 *          channels.
 *
 * @note    The gain values are represented as float numbers.
 */
/*****************************************************************************/
typedef struct AwbGains_s
{
    float fRed;         /**< gain value for the red channel */
    float fGreenR;      /**< gain value for the green channel in red lines */
    float fGreenB;      /**< gain value for the green channel in blue lines */
    float fBlue;        /**< gain value for the blue channel */
} AwbGains_t;



/*****************************************************************************/
/**
 * @brief   A structure/tupple to represent gain values for four (R,Gr,Gb,B)
 *          channels.
 *
 * @note    The gain values are represented as signed numbers.
 */
/*****************************************************************************/
typedef struct AwbXTalkOffset_s
{
    float fRed;         /**< value for the red channel */
    float fGreen;       /**< value for the green channel in red lines */
    float fBlue;        /**< value for the blue channel */
} AwbXTalkOffset_t;



/*****************************************************************************/
/**
 *          AwbInstanceConfig_t
 *
 * @brief   AWB Module instance configuration structure
 *
 *****************************************************************************/
typedef struct AwbInstanceConfig_s
{
    AwbHandle_t                     hAwb;               /**< handle returns by AwbInit() */
} AwbInstanceConfig_t;



/*****************************************************************************/
/**
 *          AwbConfig_t
 *
 * @brief   AWB Module configuration structure
 *
 *****************************************************************************/
typedef struct AwbConfig_s
{
    AwbMode_t                       Mode;               /**< White Balance working mode (MANUAL | AUTO) */
    
    CamerIcDrvHandle_t              hCamerIc;           /**< cameric driver handle */
    CamerIcDrvHandle_t              hSubCamerIc;        /**< cameric driver handle */

    uint16_t                        width;              /**< picture width */
    uint16_t                        height;             /**< picture height */
    float                           framerate;          /**< frame rate */

    uint32_t                        Flags;              /**< working flags (@see AwbWorkingFlags_e) */

    CamCalibDbHandle_t              hCamCalibDb;        /**< calibration database handle */

    CamerIcIspAwbMeasuringMode_t    MeasMode;           /**< specifies the means measuring mode (YCbCr or RGB) */
    CamerIcAwbMeasuringConfig_t     MeasConfig;         /**< measuring config */

    float                           fStableDeviation;   /**< min deviation in percent to enter stable state */
    float                           fRestartDeviation;  /**< max tolerated deviation in precent for staying in stable state */
} AwbConfig_t;



/*****************************************************************************/
/**
 *          AwbRgProj_t
 *
 * @brief   AWB Projection Borders in R/G Layer
 *
 *****************************************************************************/
typedef struct AwbRgProj_s
{
    float                           fRgProjIndoorMin;
    float                           fRgProjOutdoorMin;
    float                           fRgProjMax;
    float                           fRgProjMaxSky;

    float                           fRgProjALimit;    //oyyf
    float                           fRgProjAWeight;        //oyyf
    float                           fRgProjYellowLimit;        //oyyf
    float                           fRgProjIllToCwf;        //oyyf
    float                           fRgProjIllToCwfWeight;    //oyyf
} AwbRgProj_t;



/*****************************************************************************/
/**
 * @brief   This function converts float based gains into CamerIC 2.8 fixpoint
 *          format.
 *
 * @param   pAwbGains           gains in float based format
 * @param   pCamerIcGains       gains in fix point format (2.8)
 *
 * @return                      Returns the result of the function call.
 * @retval  RET_SUCCESS         gains sucessfully converted
 * @retval  RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT AwbGains2CamerIcGains
(
    AwbGains_t      *pAwbGains,
    CamerIcGains_t  *pCamerIcGains
);



/*****************************************************************************/
/**
 * @brief   This function converts CamerIC 2.8 fixpoint format into float
 *          based gains.
 *
 * @param   pCamerIcGains       gains in fix point format (2.8)
 * @param   pAwbGains           gains in float based format
 *
 * @return                      Returns the result of the function call.
 * @retval  RET_SUCCESS         gains sucessfully converted
 * @retval  RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT CamerIcGains2AwbGains
(
    CamerIcGains_t  *pCamerIcGains,
    AwbGains_t      *pAwbGains
);



/*****************************************************************************/
/**
 * @brief       This function converts float based Color correction matrix
 *              values into CamerIC 4.7 fixpoint format.
 *
 * @param[in]   pAwbXTalkOffset     offset as float values
 * @param[out]  pCamerIcXTalkOffset offsets as 2's complement integer
 *
 * @return                          Returns the result of the function call.
 * @retval      RET_SUCCESS         offsets sucessfully converted
 * @retval      RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT AwbXtalk2CamerIcXtalk
(
    Cam3x3FloatMatrix_t *pAwbXTalkMatrix,
    CamerIc3x3Matrix_t  *pXTalkMatrix
);



/*****************************************************************************/
/**
 * @brief       This function converts CamerIC 4.7 fixpoint format based Color
 *              correction matrix into float based values.
 *
 * @param[in]   pCamerIcXTalkOffset offsets as 2's complement integer
 * @param[out]  pAwbXTalkOffset     offset as float values
 *
 * @return                          Returns the result of the function call.
 * @retval      RET_SUCCESS         offsets sucessfully converted
 * @retval      RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT CamerIcXtalk2AwbXtalk
(
    CamerIc3x3Matrix_t  *pXTalkMatrix,
    Cam3x3FloatMatrix_t *pAwbXTalkMatrix
);



/*****************************************************************************/
/**
 * @brief   This function converts float based offset values into CamerIC 12.0
 *          fix point format based offset values.
 *
 * @param   pAwbXTalkOffset     offset as float values
 * @param   pCamerIcXTalkOffset offsets as 2's complement integer
 *
 * @return                      Returns the result of the function call.
 * @retval  RET_SUCCESS         offsets sucessfully converted
 * @retval  RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT AwbXTalkOffset2CamerIcXTalkOffset
(
    Cam1x3FloatMatrix_t     *pAwbXTalkOffset,
    CamerIcXTalkOffset_t    *pCamerIcXTalkOffset
);



/*****************************************************************************/
/**
 * @brief   This function converts CamerIC 12.0 fix point format based offset
 *          values into float based offset value.
 *
 * @param   pCamerIcXTalkOffset offsets as 2's complement integer
 * @param   pAwbXTalkOffset     offset as float values
 *
 * @return                      Returns the result of the function call.
 * @retval  RET_SUCCESS         offsets sucessfully converted
 * @retval  RET_NULL_POINTER    null pointer parameter
 *
 *****************************************************************************/
RESULT CamerIcXTalkOffset2AwbXTalkOffset
(
    CamerIcXTalkOffset_t    *pCamerIcXTalkOffset,
    AwbXTalkOffset_t        *pAwbXTalkOffset
);


void LineFitLeastSquares(float *data_x, float *data_y, int data_n,float *a,float *b);



#ifdef __cplusplus
}
#endif

/* @} AWBM */


#endif /* __AWB_H__*/
