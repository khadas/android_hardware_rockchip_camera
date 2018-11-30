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
 * @file cam_types.h
 *
 * @brief Interface description for image sensor specific implementation (iss).
 *
 *****************************************************************************/
/**
 * @page module_name_page Module Name
 * Describe here what this module does.
 *
 * For a detailed list of functions and implementation detail refer to:
 * - @ref module_name
 *
 * @defgroup cam_types   Common Camera Type Definitions
 * @{
 *
 */
#ifndef __CAM_TYPES_H__
#define __CAM_TYPES_H__

#include <common/list.h>

#ifdef __cplusplus
extern "C"
{
#endif



/*****************************************************************************/
/**
 * @brief   doortype of an illumination profile
 */
/*****************************************************************************/
typedef enum  CamDoorType_e
{
    CAM_DOOR_TYPE_OUTDOOR = 0,
    CAM_DOOR_TYPE_INDOOR  = ( !CAM_DOOR_TYPE_OUTDOOR )
} CamDoorType_t;



/*****************************************************************************/
/**
 * @brief   doortype of an illumination profile
 */
/*****************************************************************************/
typedef enum CamAwbType_e
{
    CAM_AWB_TYPE_MANUAL  = 0,
    CAM_AWB_TYPE_AUTO    = ( !CAM_AWB_TYPE_MANUAL )
} CamAwbType_t;



/*****************************************************************************/
/**
 * @brief   3 channel color components
 */
/*****************************************************************************/
typedef enum Cam3ChColorComponent_e
{
    CAM_3CH_COLOR_COMPONENT_RED     = 0,
    CAM_3CH_COLOR_COMPONENT_GREEN   = 1,
    CAM_3CH_COLOR_COMPONENT_BLUE    = 2,
    CAM_3CH_COLOR_COMPONENT_MAX
} Cam3ChColorComponent_t;



/*****************************************************************************/
/**
 * @brief   4 channel color components
 */
/*****************************************************************************/
typedef enum Cam4ChColorComponent_e
{
    CAM_4CH_COLOR_COMPONENT_RED     = 0,
    CAM_4CH_COLOR_COMPONENT_GREENR  = 1,
    CAM_4CH_COLOR_COMPONENT_GREENB  = 2,
    CAM_4CH_COLOR_COMPONENT_BLUE    = 3,
    CAM_4CH_COLOR_COMPONENT_MAX
} Cam4ChColorComponent_t;

/******************************************************************************/
/**
 * @brief   A structure to represent a 5x5 matrix.
 *
 *          The 25 values are laid out as follows (zero based index):
 *
 *               | 00 01 02 03 04 | \n
 *               | 05 06 07 08 09 | \n
 *               | 10 11 12 13 14 | \n
 *               | 15 16 17 18 19 | \n
 *               | 20 21 22 23 24 | \n
 *         
 * @note    The 25 values are represented as unsigned char numbers.
 *
 *****************************************************************************/
typedef struct Cam5x5UCharMatrix_s
{
	uint8_t uCoeff[5*5];			  /**< array of 5x5 unsigned char values */
} Cam5x5UCharMatrix_t;


/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam1x1FloatMatrix_s
{
    float fCoeff[1];
} Cam1x1FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | 2 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam1x3FloatMatrix_s
{
    float fCoeff[3];
} Cam1x3FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | ... | 4 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam1x4FloatMatrix_s
{
    float fCoeff[4];
} Cam1x4FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | ... | 6 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam1x6FloatMatrix_s
{
    float fCoeff[6];
} Cam1x6FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | ... | 15 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam1x16FloatMatrix_s
{
    float fCoeff[16];
} Cam1x16FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam2x1FloatMatrix
{
    float fCoeff[2];
} Cam2x1FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |
 *          | 2 | 3 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam2x2FloatMatrix
{
    float fCoeff[4];
} Cam2x2FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |  2 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam3x1FloatMatrix
{
    float fCoeff[3];
} Cam3x1FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |  2 |
 *          | 3 | 4 |  5 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam3x2FloatMatrix_s
{
    float fCoeff[6];
} Cam3x2FloatMatrix_t;

/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |  2 |
 *          | 3 | 4 |  5 |
 *          | 6 | 7 |  8 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam3x3FloatMatrix_s
{
    float fCoeff[9];
} Cam3x3FloatMatrix_t;

/******************************************************************************/
/**
 * @brief   A structure to represent a 5x5 matrix.
 *
 *          The 25 values are laid out as follows (zero based index):
 *
 *               | 00 01 02 03 04 | \n
 *               | 05 06 07 08 09 | \n
 *               | 10 11 12 13 14 | \n
 *               | 15 16 17 18 19 | \n
 *               | 20 21 22 23 24 | \n
 *         
 * @note    The 25 values are represented as float numbers.
 *
 *****************************************************************************/
typedef struct Cam5x5FloatMatrix_s
{
    float fCoeff[25U];              /**< array of 5x5 float values */
} Cam5x5FloatMatrix_t;


/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          |   0 |   1 |   2 |   3 |   4 |   5 |   6 |   7 | ....
 *          |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 | ....
 *          |  34 |  35 |  36 |  37 |  38 |  39 |  40 |  41 | ....
 *          ...
 *          ...
 *          ...
 *          | 271 | 272 | 273 | 274 | 275 | 276 | 277 | 278 | .... | 288 |
 *
 * @note    Coefficients are represented as short numbers
 */
/*****************************************************************************/
typedef struct Cam17x17FloatMatrix_s
{
    float fCoeff[17 * 17];
} Cam17x17FloatMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | 2 |
 *
 * @note    Coefficients are represented as short numbers
 */
/*****************************************************************************/
typedef struct Cam1x3ShortMatrix_s
{
    int16_t Coeff[3];
} Cam1x3ShortMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | 2 | ... | 4 |
 *
 * @note    Coefficients are represented as short numbers
 */
/*****************************************************************************/
typedef struct Cam1x4UShortMatrix_s
{
    uint16_t uCoeff[4];
} Cam1x4UShortMatrix_t;

/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 | 2 | ... | 16 |
 *
 * @note    Coefficients are represented as short numbers
 */
/*****************************************************************************/
typedef struct Cam1x17UShortMatrix_s
{
    uint16_t uCoeff[17];
} Cam1x17UShortMatrix_t;



/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          |   0 |   1 |   2 |   3 |   4 |   5 |   6 |   7 | ....
 *          |  17 |  18 |  19 |  20 |  21 |  22 |  23 |  24 | ....
 *          |  34 |  35 |  36 |  37 |  38 |  39 |  40 |  41 | ....
 *          ...
 *          ...
 *          ...
 *          | 271 | 272 | 273 | 274 | 275 | 276 | 277 | 278 | .... | 288 |
 *
 * @note    Coefficients are represented as short numbers
 */
/*****************************************************************************/
typedef struct Cam17x17UShortMatrix_s
{
    uint16_t uCoeff[17 * 17];
} Cam17x17UShortMatrix_t;



/*****************************************************************************/
/**
 * @brief   name/identifier of a resolution
 */
/*****************************************************************************/
#define CAM_RESOLUTION_NAME         ( 15U )
typedef char                        CamResolutionName_t[CAM_RESOLUTION_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a resolution
 */
/*****************************************************************************/
#define CAM_FRAMERATE_NAME          ( 25U )
typedef char                        CamFramerateName_t[CAM_FRAMERATE_NAME];



/*****************************************************************************/
/**
 * @brief   name/identifier of an illumination
 */
/*****************************************************************************/
#define CAM_BLS_PROFILE_NAME        ( 10U )
typedef char                        CamBlsProfileName_t[CAM_BLS_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of an illumination
 */
/*****************************************************************************/
#define CAM_ILLUMINATION_NAME       ( 20U )
typedef char                        CamIlluminationName_t[CAM_ILLUMINATION_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a lense shade correction profile (LscProfile)
 */
/*****************************************************************************/
#define CAM_LSC_PROFILE_NAME        ( 25U )
typedef char                        CamLscProfileName_t[CAM_LSC_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a color correction profile (CcProfile)
 */
/*****************************************************************************/
#define CAM_CC_PROFILE_NAME         ( 20U )
typedef char                        CamCcProfileName_t[CAM_CC_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a auto white balance profile (AwbProfile)
 */
/*****************************************************************************/
#define CAM_AWB_PROFILE_NAME        ( 20U )
typedef char                        CamAwbProfileName_t[CAM_AWB_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of an exposure scheme (EcmScheme)
 */
/*****************************************************************************/
#define CAM_ECM_SCHEME_NAME         ( 20U )
typedef char                        CamEcmSchemeName_t[CAM_ECM_SCHEME_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of an exposure profile (EcmProfile)
 */
/*****************************************************************************/
#define CAM_ECM_PROFILE_NAME        ( 20U )
typedef char                        CamEcmProfileName_t[CAM_ECM_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a chromatic abberation correction profile
 *          (CacProfile)
 */
/*****************************************************************************/
#define CAM_CAC_PROFILE_NAME        ( 20U )
typedef char                        CamCacProfileName_t[CAM_CAC_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   name/identifier of a denoising prefilter profile (DpfProfile)
 */
/*****************************************************************************/
#define CAM_DPF_PROFILE_NAME        ( 20U )
typedef char                        CamDpfProfileName_t[CAM_DPF_PROFILE_NAME];

//werring.wu 2018 1.17
/*****************************************************************************/
/**
 * @brief   name/identifier of a issharpen prefilter profile (Profile)
 */
/*****************************************************************************/
#define CAM_IESHARPEN_PROFILE_NAME        ( 20U )
typedef char  CamIesharpenProfileName_t[CAM_IESHARPEN_PROFILE_NAME];



/*****************************************************************************/
/**
 * @brief   name/identifier of a denoising prefilter profile (DpccProfile)
 */
/*****************************************************************************/
#define CAM_DPCC_PROFILE_NAME       ( 20U )
typedef char                        CamDpccProfileName_t[CAM_DPCC_PROFILE_NAME];


/*****************************************************************************/
/**
 * @brief   framerate profile
 */
/*****************************************************************************/
typedef struct CamFrameRate_s
{
    void                *p_next;            /**< for adding to a list */

    CamFramerateName_t  name;               /**< name of framerate scheme */
    float               fps;                /**< framerate */
} CamFrameRate_t;


/*****************************************************************************/
/**
 * @brief   resolution profile
 */
/*****************************************************************************/
typedef struct CamResolution_t
{
    void                *p_next;            /**< for adding to a list */

    CamResolutionName_t name;               /**< desctriptor */
    uint16_t            width;              /**< resolution width */
    uint16_t            height;             /**< resolution height */
    uint32_t            id;                 /**< bitmask identifier */

    List                framerates;         /**< list of framerates */
} CamResolution_t;



/*****************************************************************************/
/**
 * @brief   parameters for a sensorgain to saturation interpolation
 */
/*****************************************************************************/
typedef struct CamSaturationCurve_s
{
    uint16_t    ArraySize;
    float       *pSensorGain;
    float       *pSaturation;
} CamSaturationCurve_t;



/*****************************************************************************/
/**
 * @brief   parameters for a sensorgain to vignetting (compensation)
 *          interpolation
 */
/*****************************************************************************/
typedef struct CamVignettingCurve_s
{
    uint16_t    ArraySize;
    float       *pSensorGain;
    float       *pVignetting;
} CamVignettingCurve_t;



/*****************************************************************************/
/**
 * @brief   parameters for a sensorgain to vignetting (compensation)
 *          interpolation
 *
 */
/*****************************************************************************/
typedef struct CamLscMatrix_s
{
    Cam17x17UShortMatrix_t  LscMatrix[CAM_4CH_COLOR_COMPONENT_MAX];
} CamLscMatrix_t;



/*****************************************************************************/
/**
 * @brief   BLS calibration structure
 */
/*****************************************************************************/
typedef struct CamBlsProfile_s
{
    void                        *p_next;                /**< for adding to a list */

    CamBlsProfileName_t         name;                   /**< profile name */
    CamResolutionName_t         resolution;             /**< resolution link */

    Cam1x4UShortMatrix_t        level;                  /**< black level for all 4 color components */
} CamBlsProfile_t;



/*****************************************************************************/
/**
 * @brief   Awb Illumination specific structure.
 */
/*****************************************************************************/
typedef struct CamIlluProfile_s {
  void*                        p_next;                /**< for adding to a list */

  CamIlluminationName_t       name;                   /**< name of the illumination profile (i.e. "D65", "A", ... )*/
  uint32_t                    id;                     /**< unique id */

  CamDoorType_t               DoorType;               /**< indoor or outdoor profile */
  CamAwbType_t                AwbType;                /**< manual or auto profile */

  /* for manual white balance data */
  Cam3x3FloatMatrix_t         CrossTalkCoeff;         /**< CrossTalk matrix coefficients */
  Cam1x3FloatMatrix_t         CrossTalkOffset;        /**< CrossTalk offsets */
  Cam1x4FloatMatrix_t         ComponentGain;          /**< White Balance Gains*/


  /* gaussian mixture modell */
  // for v10
  Cam2x1FloatMatrix_t         GaussMeanValue;         /**< */
  Cam2x2FloatMatrix_t         CovarianceMatrix;       /**< */
  Cam1x1FloatMatrix_t         GaussFactor;            /**< */
  Cam2x1FloatMatrix_t         Threshold;              /**< */
  
  ///* from awb calibration data */
  // for v11
  Cam1x4FloatMatrix_t         referenceWBgain;        /**< */

  /* adaptive color correctio */
  CamSaturationCurve_t        SaturationCurve;        /**< stauration over gain curve */

  /* adative lense shade correction */
  CamVignettingCurve_t        VignettingCurve;        /**< vignetting over gain curve */

  #define CAM_NO_CC_PROFILES  ( 10 )                  /**< max number of cc-profiles per illumination */
  int32_t                     cc_no;
  CamCcProfileName_t          cc_profiles[CAM_NO_CC_PROFILES];

  #define CAM_NO_LSC_PROFILES ( 5 )
  #define CAM_NO_RESOLUTIONS  ( 4 )
  int32_t                     lsc_res_no;
  int32_t                     lsc_no[CAM_NO_RESOLUTIONS];
  CamLscProfileName_t         lsc_profiles[CAM_NO_RESOLUTIONS][CAM_NO_LSC_PROFILES];
} CamIlluProfile_t;


/*****************************************************************************/
/**
 * @brief   LSC profile
 */
/*****************************************************************************/
typedef struct CamLscProfile_s
{
    void                    *p_next;                                /**< for adding to a list */

    CamLscProfileName_t     name;                                   /**< profile name */
    CamResolutionName_t     resolution;                             /**< resolution link */
    CamIlluminationName_t   illumination;                           /**< illumination link */
    float                   vignetting;                             /**< vignetting value */

    uint16_t                LscSectors;
    uint16_t                LscNo;
    uint16_t                LscXo;
    uint16_t                LscYo;

    uint16_t                LscXGradTbl[8];
    uint16_t                LscYGradTbl[8];
    uint16_t                LscXSizeTbl[8];
    uint16_t                LscYSizeTbl[8];

    Cam17x17UShortMatrix_t  LscMatrix[CAM_4CH_COLOR_COMPONENT_MAX];     /**< matrix for different color channels */
} CamLscProfile_t;



/*****************************************************************************/
/**
 * @brief   CC profile
 *
 */
/*****************************************************************************/
typedef struct CamCcProfile_s
{
    void                    *p_next;                /**< for adding to a list */

    CamCcProfileName_t      name;                   /**< profile name */
    float                   saturation;             /**< saturation value */

    Cam3x3FloatMatrix_t     CrossTalkCoeff;         /**< CrossTalk matrix coefficients */
    Cam1x3FloatMatrix_t     CrossTalkOffset;        /**< CrossTalk offsets */
    Cam1x4FloatMatrix_t     ComponentGain;          /**< White Balance Gains*/
} CamCcProfile_t;



/*****************************************************************************/
/**
 * @brief   CAC calibration structure
 */
/*****************************************************************************/
typedef struct CamCacProfile_s
{
    void                    *p_next;            /**< for adding to a list */

    CamCacProfileName_t     name;               /**< profile name */
    CamResolutionName_t     resolution;         /**< resolution link */

    uint8_t                 x_ns;               /**< horizontal normalization shift */
    uint8_t                 x_nf;               /**< horizontal normalization factor */
    uint8_t                 y_ns;               /**< vertical normalization shift */
    uint8_t                 y_nf;               /**< vertical normalization factor */

    Cam1x3FloatMatrix_t     Red;                /**< coeffciencts A, B and C for red */
    Cam1x3FloatMatrix_t     Blue;               /**< coeffciencts A, B and C for blue */

    int16_t                 hCenterOffset;
    int16_t                 vCenterOffset;
} CamCacProfile_t;

/**
 * @brief   Enumeration type to configure de-noising level.
 *
 */
typedef enum CamerIcIspFltDeNoiseLevel_e
{
    CAMERIC_ISP_FLT_DENOISE_LEVEL_INVALID   = 0,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_0         = 1,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_1         = 2,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_2         = 3,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_3         = 4,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_4         = 5,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_5         = 6,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_6         = 7,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_7         = 8,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_8         = 9,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_9         = 10,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_10        = 11,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_TEST      = 12,
    CAMERIC_ISP_FLT_DENOISE_LEVEL_MAX
} CamerIcIspFltDeNoiseLevel_t;



/**
 * @brief   Enumeration type to configure sharpening level.
 *
 */
typedef enum CamerIcIspFltSharpeningLevel_e
{
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_INVALID   = 0,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_0         = 1,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_1         = 2,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_2         = 3,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_3         = 4,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_4         = 5,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_5         = 6,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_6         = 7,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_7         = 8,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_8         = 9,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_9         = 10,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_10        = 11,
    CAMERIC_ISP_FLT_SHARPENING_LEVEL_MAX
} CamerIcIspFltSharpeningLevel_t;


/*****************************************************************************/
/**
 * @brief   parameters for a sensorgain to saturation interpolation
 */
/*****************************************************************************/
typedef struct CamDenoiseLevelCurve_s
{
    uint16_t    ArraySize;
    float       *pSensorGain;
    CamerIcIspFltDeNoiseLevel_t        *pDlevel;
} CamDenoiseLevelCurve_t;

/*****************************************************************************/
/**
 * @brief   parameters for a sensorgain to saturation interpolation
 */
/*****************************************************************************/
typedef struct CamSharpeningLevelCurve_s
{
    uint16_t    ArraySize;
    float       *pSensorGain;
    CamerIcIspFltSharpeningLevel_t      *pSlevel;
} CamSharpeningLevelCurve_t;

/*****************************************************************************/
/**
 * @brief   MFD calibration structure
 */
/*****************************************************************************/
typedef struct CamMfdProfile_s
{
	uint8_t					enable;			/**< mfd enable*/
	float 					gain[3];		/**< mfd gain */
	float 					frames[3];		/**< mfd frames */
} CamMfdProfile_t;

/*****************************************************************************/
/**
 * @brief   UVNR calibration structure
 */
/*****************************************************************************/
typedef struct CamUvnrProfile_s
{
	uint8_t					enable;			/**< uvnr enable*/
	float 					gain[3];		/**< uvnr gain */
	float 					ratio[3];		/**< uvnr ratio */
	float 					distances[3];	/**< uvnr distances */
} CamUvnrProfile_t;

/*****************************************************************************/
typedef struct CamFilterLevelRegConf_s {
  uint8_t FiltLevelRegConfEnable;
  uint8_t* p_FiltLevel;
  uint8_t  ArraySize;
  uint8_t* p_grn_stage1;    /* ISP_FILT_MODE register fields*/
  uint8_t  grn_stage1_ArraySize;
  uint8_t* p_chr_h_mode;    /* ISP_FILT_MODE register fields*/
  uint8_t  chr_h_mode_ArraySize;
  uint8_t* p_chr_v_mode;    /* ISP_FILT_MODE register fields*/
  uint8_t  chr_v_mode_ArraySize;
  uint32_t*  p_thresh_bl0;
  uint8_t  thresh_bl0_ArraySize;
  uint32_t*  p_thresh_bl1;
  uint8_t  thresh_bl1_ArraySize;
  uint32_t*  p_thresh_sh0;
  uint8_t  thresh_sh0_ArraySize;
  uint32_t*  p_thresh_sh1;
  uint8_t  thresh_sh1_ArraySize;
  uint32_t*  p_fac_sh1;
  uint8_t  fac_sh1_ArraySize;
  uint32_t*  p_fac_sh0;
  uint8_t  fac_sh0_ArraySize;
  uint32_t*  p_fac_mid;
  uint8_t  fac_mid_ArraySize;
  uint32_t*  p_fac_bl0;
  uint8_t  fac_bl0_ArraySize;
  uint32_t*  p_fac_bl1;
  uint8_t  fac_bl1_ArraySize;
} CamFilterLevelRegConf_t;

typedef struct CamFltLevelRegConf_s {
	uint8_t  grn_stage1;
	uint8_t  chr_h_mode;
	uint8_t  chr_v_mode;
	uint32_t thresh_bl0;
	uint32_t thresh_bl1;
	uint32_t thresh_sh0;
	uint32_t thresh_sh1;
	uint32_t fac_sh1;
	uint32_t fac_sh0;
	uint32_t fac_mid;
	uint32_t fac_bl0;
	uint32_t fac_bl1;
	bool_t   enable;
	bool_t   regConfEnable;
	uint8_t  level;
	char     resolution[20];
	uint8_t denoise_gain[5];
    uint8_t denoise_level[5];
    uint8_t sharpen_gain[5];
    uint8_t sharpen_level[5];
}CamFltLevelRegConf_t;


//werring.wu 2018 1 17
/*****************************************************************************/
/**
 * @brief   IESHARPEN calibration structure
 */
/*****************************************************************************/
typedef struct CamIesharpenGridConf_s
{
    uint32_t* p_grad;            //lowlight grad segment points
    uint8_t p_grad_ArraySize;
    uint32_t* sharp_factor;    // lowlight sharpen factor 0-64
    uint8_t sharp_factor_ArraySize;
    uint32_t* line1_filter_coe; //prefilter the data 2x3 
    uint8_t line1_filter_coe_ArraySize;
    uint32_t* line2_filter_coe;//                           3x3
    uint8_t line2_filter_coe_ArraySize;
    uint32_t* line3_filter_coe;//                           2x3
    uint8_t line3_filter_coe_ArraySize;
}CamIesharpenGridConf_t;

typedef struct CamIesharpenProfile_s
{
    void                        *p_next;                /**< for adding to a list */
    CamIesharpenProfileName_t   name;//
    CamResolutionName_t         resolution;             /**< resolution link */
    uint8_t iesharpen_en;     // iesharpen_en 0 off, 1 on
    uint8_t coring_thr;         // iesharpen coring_thr is default 0
    uint8_t full_range;         // iesharpen full range(yuv data) 1:full_range(0-255),2:range(16-24?)
    uint8_t switch_avg;       //iesharpen whether is compare center pixel with edge pixel
    uint32_t* yavg_thr;// Y channel is set five segments by the Matrix
    uint8_t yavg_thr_ArraySize;
    uint32_t* P_delta1; 
    uint8_t P_delta1_ArraySize;
    uint32_t* P_delta2;
    uint8_t P_delta2_ArraySize;
    uint32_t* pmaxnumber;
    uint8_t pmaxnumber_ArraySize;
    uint32_t* pminnumber;
    uint8_t pminnumber_ArraySize;
    uint32_t* gauss_flat_coe;    // filter mask for flat 
    uint8_t gauss_flat_coe_ArraySize;
    uint32_t* gauss_noise_coe;//filter mask2 for noise
    uint8_t gauss_noise_coe_ArraySize;
    uint32_t* gauss_other_coe; //filter mask for other
    uint8_t gauss_other_coe_ArraySize;
    CamIesharpenGridConf_t lgridconf;
    CamIesharpenGridConf_t hgridconf;
    uint32_t* uv_gauss_flat_coe; //uv channel faussian filter 3x5 in flat point
    uint8_t uv_gauss_flat_coe_ArraySize;
    uint32_t* uv_gauss_noise_coe;//uv channel faussian filter 3x5 in noise point
    uint8_t uv_gauss_noise_coe_ArraySize;
    uint32_t* uv_gauss_other_coe;//uv channel faussian filter 3x5 in other point
    uint8_t uv_gauss_other_coe_ArraySize;
} CamIesharpenProfile_t;

typedef struct CamDemosaicLpThreshold_s
{
	uint8_t  sw_thgrad_divided[5];
	uint8_t  sw_thCSC_divided[5];
	uint8_t  sw_thdiff_divided[5];
	uint16_t sw_thVar_divided[5];
}CamDemosaicLpThreshold_t;

//werring.wu add 2018 1 17
typedef struct CamDemosaicLpProfile_s
{
	uint8_t  lp_en;
	uint8_t  rb_filter_en;
	uint8_t  hp_filter_en;
	uint8_t  use_old_lp;// use old version
	uint8_t* lu_divided;//
	uint8_t  lu_divided_ArraySize;
	float* gainsArray;
	uint8_t  gainsArray_ArraySize;
	float* thH_divided0;
	uint8_t  thH_divided0_ArraySize;
	float* thH_divided1;
	uint8_t  thH_divided1_ArraySize;
	float* thH_divided2;
	uint8_t  thH_divided2_ArraySize;
	float* thH_divided3;
	uint8_t  thH_divided3_ArraySize;
	float* thH_divided4;
	uint8_t  thH_divided4_ArraySize;
	float* thCSC_divided0;
	uint8_t  thCSC_divided0_ArraySize;
	float* thCSC_divided1;
	uint8_t  thCSC_divided1_ArraySize;
	float* thCSC_divided2;
	uint8_t  thCSC_divided2_ArraySize;
	float* thCSC_divided3;
	uint8_t  thCSC_divided3_ArraySize;
	float* thCSC_divided4;
	uint8_t  thCSC_divided4_ArraySize;
	float* diff_divided0;
	uint8_t  diff_divided0_ArraySize;
	float* diff_divided1;
	uint8_t  diff_divided1_ArraySize;
	float* diff_divided2;
	uint8_t  diff_divided2_ArraySize;
	float* diff_divided3;
	uint8_t  diff_divided3_ArraySize;
	float* diff_divided4;
	uint8_t  diff_divided4_ArraySize;
	float* varTh_divided0;
	uint8_t  varTh_divided0_ArraySize;
	float* varTh_divided1;
	uint8_t  varTh_divided1_ArraySize;
	float* varTh_divided2;
	uint8_t  varTh_divided2_ArraySize;
	float* varTh_divided3;
	uint8_t  varTh_divided3_ArraySize;
	float* varTh_divided4;
	uint8_t  varTh_divided4_ArraySize;
	uint8_t  thgrad_r_fct;
	uint8_t  thdiff_r_fct;
	uint8_t  thvar_r_fct;
	uint8_t  thgrad_b_fct;
	uint8_t  thdiff_b_fct;
	uint8_t  thvar_b_fct;
	uint8_t  similarity_th;
	uint8_t  th_var_en;
	uint8_t  th_csc_en;
	uint8_t  th_diff_en;
	uint8_t  th_grad_en;
	uint16_t  th_var;
	uint8_t  th_csc;
	uint8_t  th_diff;
	uint8_t  th_grad;
	uint8_t  flat_level_sel;
	uint8_t  pattern_level_sel;
	uint8_t  edge_level_sel;
}CamDemosaicLpProfile_t;

/*****************************************************************************/
/**
 * @brief   DPF calibration structure
 */
/*****************************************************************************/
typedef struct CamDpfProfile_s
{
    void                    *p_next;            /**< for adding to a list */

    CamDpfProfileName_t     name;               /**< profile name */
    CamResolutionName_t     resolution;         /**< resolution link */
    uint16_t                adpf_enable;
    uint16_t                nll_segmentation;
    Cam1x17UShortMatrix_t   nll_coeff;

    uint16_t                SigmaGreen;         /**< */
    uint16_t                SigmaRedBlue;       /**< */
    float                   fGradient;          /**< */
    float                   fOffset;            /**< */
    Cam1x4FloatMatrix_t     NfGains;            /**< */

    CamMfdProfile_t         Mfd;                /**< mfd struct*/
    CamUvnrProfile_t        Uvnr;               /**< uvnr struct*/
    CamDenoiseLevelCurve_t        DenoiseLevelCurve;
    CamSharpeningLevelCurve_t     SharpeningLevelCurve;
    float FilterEnable;
    CamFilterLevelRegConf_t FiltLevelRegConf;
    CamDemosaicLpProfile_t DemosaicLpConf;
} CamDpfProfile_t;



/*****************************************************************************/
/**
 * @brief   DPCC calibration structure
 */
/*****************************************************************************/
typedef struct CamDpccProfile_s
{
    void                    *p_next;            /**< for adding to a list */

    CamDpccProfileName_t    name;               /**< profile name */
    CamResolutionName_t     resolution;         /**< resolution link */

    uint32_t                isp_dpcc_mode;
    uint32_t                isp_dpcc_output_mode;
    uint32_t                isp_dpcc_set_use;
    uint32_t                isp_dpcc_methods_set_1;
    uint32_t                isp_dpcc_methods_set_2;
    uint32_t                isp_dpcc_methods_set_3;
    uint32_t                isp_dpcc_line_thresh_1;
    uint32_t                isp_dpcc_line_mad_fac_1;
    uint32_t                isp_dpcc_pg_fac_1;
    uint32_t                isp_dpcc_rnd_thresh_1;
    uint32_t                isp_dpcc_rg_fac_1;
    uint32_t                isp_dpcc_line_thresh_2;
    uint32_t                isp_dpcc_line_mad_fac_2;
    uint32_t                isp_dpcc_pg_fac_2;
    uint32_t                isp_dpcc_rnd_thresh_2;
    uint32_t                isp_dpcc_rg_fac_2;
    uint32_t                isp_dpcc_line_thresh_3;
    uint32_t                isp_dpcc_line_mad_fac_3;
    uint32_t                isp_dpcc_pg_fac_3;
    uint32_t                isp_dpcc_rnd_thresh_3;
    uint32_t                isp_dpcc_rg_fac_3;
    uint32_t                isp_dpcc_ro_limits;
    uint32_t                isp_dpcc_rnd_offs;
} CamDpccProfile_t;



/*****************************************************************************/
/**
 * @brief   Contains pointers to parameter arrays for Rg/Bg color space
 *          clipping
 */
/*****************************************************************************/
typedef struct CamAwbClipParm_s
{
    float       *pRg1;
    float       *pMaxDist1;
    float       *pRg2;
    float       *pMaxDist2;
    uint16_t    ArraySize1;
    uint16_t    ArraySize2;
} CamAwbClipParm_t;



/*****************************************************************************/
/**
 * @brief   Contains pointers to parameter arrays for AWB out of range
 *          handling
 *
 */
/*****************************************************************************/
typedef struct CamAwbGlobalFadeParm_s
{
    float       *pGlobalFade1;
    float       *pGlobalGainDistance1;
    float       *pGlobalFade2;
    float       *pGlobalGainDistance2;
    uint16_t    ArraySize1;
    uint16_t    ArraySize2;
} CamAwbGlobalFadeParm_t;



/*****************************************************************************/
/**
 * @brief   Contains pointers to parameter arrays for near white pixel
 *          parameter calculations
 */
/*****************************************************************************/
typedef struct CamAwb_V11_Fade2Parm_s {
  float*      pFade;

  float*      pMaxCSum_br;
  float*      pMaxCSum_sr;
  float*      pMinC_br;
  float*      pMinC_sr;
  float*      pMaxY_br;
  float*      pMaxY_sr;
  float*      pMinY_br;
  float*      pMinY_sr;
  float*      pRefCb;
  float*      pRefCr;
  uint16_t    ArraySize;
} CamAwb_V11_Fade2Parm_t;


/*****************************************************************************/
/**
 * @brief   Contains pointers to parameter arrays for near white pixel
 *          parameter calculations
 */
/*****************************************************************************/
typedef struct CamAwb_V10_Fade2Parm_s {
  float*      pFade;
  float*      pCbMinRegionMax;
  float*      pCrMinRegionMax;
  float*      pMaxCSumRegionMax;
  float*      pCbMinRegionMin;
  float*      pCrMinRegionMin;
  float*      pMaxCSumRegionMin;
  float*      pMinCRegionMax;
  float*      pMinCRegionMin;
  float*      pMaxYRegionMax;
  float*      pMaxYRegionMin;
  float*      pMinYMaxGRegionMax;
  float*      pMinYMaxGRegionMin;
  float*      pRefCb;
  float*      pRefCr;
  uint16_t    ArraySize;
} CamAwb_V10_Fade2Parm_t;


typedef struct CamAwbMeasResult_s
{
    uint32_t    NoWhitePixel;           /**< number of white pixel */
    uint8_t     MeanY_G;               /**< Y/G  value in YCbCr/RGB Mode */
    uint8_t     MeanCb_B;              /**< Cb/B value in YCbCr/RGB Mode */
    uint8_t     MeanCr_R;              /**< Cr/R value in YCbCr/RGB Mode */
    uint16_t     MeanR;
    uint16_t     MeanG;
    uint16_t     MeanB;
} CamAwbMeasResult_t;

typedef struct CamAwbMeasConfig_s
{
    uint8_t MaxY;           /**< YCbCr Mode: only pixels values Y <= ucMaxY contribute to WB measurement (set to 0 to disable this feature) */
                            /**< RGB Mode  : unused */
    uint8_t RefCr_MaxR;     /**< YCbCr Mode: Cr reference value */
                            /**< RGB Mode  : only pixels values R < MaxR contribute to WB measurement */
    uint8_t MinY_MaxG;      /**< YCbCr Mode: only pixels values Y >= ucMinY contribute to WB measurement */
                            /**< RGB Mode  : only pixels values G < MaxG contribute to WB measurement */
    uint8_t RefCb_MaxB;     /**< YCbCr Mode: Cb reference value */
                            /**< RGB Mode  : only pixels values B < MaxB contribute to WB measurement */
    uint8_t MaxCSum;        /**< YCbCr Mode: chrominance sum maximum value, only consider pixels with Cb+Cr smaller than threshold for WB measurements */
                            /**< RGB Mode  : unused */
    uint8_t MinC;           /**< YCbCr Mode: chrominance minimum value, only consider pixels with Cb/Cr each greater than threshold value for WB measurements */
                            /**< RGB Mode  : unused */
} CamAwbMeasConfig_t;

typedef struct CamWbGainsOverG_s
{
    float GainROverG;                           /**< (Gain-Red / Gain-Green) */
    float GainBOverG;                           /**< (Gain-Blue / Gain-Green) */
} CamWbGainsOverG_t;

typedef struct CamAwbWpGet_s
{
	CamAwbMeasResult_t measResult;
	CamAwbMeasConfig_t measConfig;
	CamWbGainsOverG_t  wbGainOver;
	CamWbGainsOverG_t  wbClipGainOver;
	float rgProj;
	float regionSize;
}CamAwbWpGet_t;

/*****************************************************************************/
/**
 *          IsiLine_t
 *
 * @brief   Contains parameters for a straight line in Hesse normal form in
 *          Rg/Bg colorspace
 *
 */
/*****************************************************************************/
typedef struct CamCenterLine_s
{
    float f_N0_Rg;                                  /**< Rg component of normal vector */
    float f_N0_Bg;                                  /**< Bg component of normal vector */
    float f_d;                                      /**< Distance of normal vector     */
} CamCenterLine_t;

typedef struct CamAwbCurve_s
{
	Cam1x1FloatMatrix_t    *pKFactor;
	CamCenterLine_t        centerLine; /**< center-line in Rg/Rb-layer */
	CamAwbClipParm_t       gainClipCurve;
	CamAwbGlobalFadeParm_t globalFadeParam;
}CamAwbCurve_t;


/*****************************************************************************/
/**
 * @brief   Global AWB IIR Filter
 */
/*****************************************************************************/
typedef struct CamCalibIIR_s
{
    float       fIIRDampCoefAdd;                        /**< incrementer of damping coefficient */
    float       fIIRDampCoefSub;                        /**< decrementer of damping coefficient */
    float       fIIRDampFilterThreshold;                /**< threshold for incrementing or decrementing of damping coefficient */

    float       fIIRDampingCoefMin;                     /**< minmuim value of damping coefficient */
    float       fIIRDampingCoefMax;                     /**< maximum value of damping coefficient */
    float       fIIRDampingCoefInit;                    /**< initial value of damping coefficient */

    uint16_t    IIRFilterSize;                          /**< number of filter items */
    float       fIIRFilterInitValue;                    /**< initial value of the filter items */
} CamCalibIIR_t;


/*****************************************************************************/
/**
 * @brief  AwbVersion_t
 */
/*****************************************************************************/
typedef enum CAM_AwbVersion_e
{
  CAM_AWB_VERSION_INVALID 		= 0,        /* invalid */
  CAM_AWB_VERSION_10  			= 1,        /*illuminatant estmation by GMM */
  CAM_AWB_VERSION_11  			= 2,  		/*illuminatant estmation by minimun distance method, using direct white point conditon  */
  CAM_AWB_VERSION_MAX
} CAM_AwbVersion_t;


/*****************************************************************************/
/**
 * @brief   Global AWBV11 calibration structure
 */
/*****************************************************************************/
typedef struct CamCalibAwb_V11_Global_s {
  void*                    p_next;                    /**< for adding to a list */

  CamAwbProfileName_t     name;                       /**< profile name */
  CamResolutionName_t     resolution;                 /**< resolution link */

  //Cam3x1FloatMatrix_t     SVDMeanValue;
  //Cam3x2FloatMatrix_t     PCAMatrix;

  CamCenterLine_t         CenterLine;
  Cam1x1FloatMatrix_t     KFactor;

  uint16_t                AwbClipEnable;
  CamAwbClipParm_t        AwbClipParam;               /**< clipping parameter in Rg/Bg space */
  CamAwbGlobalFadeParm_t  AwbGlobalFadeParm;
  CamAwb_V11_Fade2Parm_t  AwbFade2Parm;

  float                   fRgProjIndoorMin;
  float                   fRgProjOutdoorMin;
  float                   fRgProjMax;
  float                   fRgProjMaxSky;

  uint16_t                fRgProjYellowLimitEnable;   //oyyf
  float                   fRgProjALimit;    //oyyf
  float                   fRgProjAWeight;   //oyyf
  float                   fRgProjYellowLimit;   //oyyf
  uint16_t                fRgProjIllToCwfEnable;    //oyyf
  float                   fRgProjIllToCwf;    //oyyf
  float                   fRgProjIllToCwfWeight;  //oyyf


  CamIlluminationName_t   outdoor_clipping_profile;

  float                   fRegionSize;
  float                   fRegionSizeInc;
  float                   fRegionSizeDec;
  float                   awbMeasWinWidthScale;
  float                   awbMeasWinHeightScale;
  float 			      fExppriorOutdoorSwitchOff;


  CamCalibIIR_t           IIR;
} CamCalibAwb_V11_Global_t;


/*****************************************************************************/
/**
 * @brief   Global AWB_V10 calibration structure
 */
/*****************************************************************************/
typedef struct CamCalibAwb_V10_Global_s {
  void*                    p_next;                    /**< for adding to a list */

  CamAwbProfileName_t     name;                       /**< profile name */
  CamResolutionName_t     resolution;                 /**< resolution link */

  Cam3x1FloatMatrix_t     SVDMeanValue;
  Cam3x2FloatMatrix_t     PCAMatrix;

  CamCenterLine_t         CenterLine;
  Cam1x1FloatMatrix_t     KFactor;

  //uint16_t                AwbClipEnable;
  CamAwbClipParm_t        AwbClipParam;               /**< clipping parameter in Rg/Bg space */
  CamAwbGlobalFadeParm_t  AwbGlobalFadeParm;
  CamAwb_V10_Fade2Parm_t  AwbFade2Parm;

  float                   fRgProjIndoorMin;
  float                   fRgProjOutdoorMin;
  float                   fRgProjMax;
  float                   fRgProjMaxSky;

  uint16_t                fRgProjYellowLimitEnable;   //oyyf
  float                   fRgProjALimit;    //oyyf
  float                   fRgProjAWeight;   //oyyf
  float                   fRgProjYellowLimit;   //oyyf
  uint16_t                fRgProjIllToCwfEnable;    //oyyf
  float                   fRgProjIllToCwf;    //oyyf
  float                   fRgProjIllToCwfWeight;  //oyyf

  CamIlluminationName_t   outdoor_clipping_profile;

  float                   fRegionSize;
  float                   fRegionSizeInc;
  float                   fRegionSizeDec;
  float                   awbMeasWinWidthScale;
  float                   awbMeasWinHeightScale;
  float 			      fExppriorOutdoorSwitchOff;

  CamCalibIIR_t           IIR;
} CamCalibAwb_V10_Global_t;

/*****************************************************************************/
/**
 * @brief   AWB Version11 para
 */
/*****************************************************************************/
typedef struct CamAwbPara_V11_s{
	List						awb_global; 	/**< list of supported awb_globals */
	List						illumination;	/**< list of supported illuminations */

}CamAwbPara_V11_t;


/*****************************************************************************/
/**
 * @brief   AWB Version11 para
 */
/*****************************************************************************/
typedef struct CamAwbPara_V10_s{
	List						awb_global; 	/**< list of supported awb_globals */
	List						illumination;	/**< list of supported illuminations */

}CamAwbPara_V10_t;


/*****************************************************************************/
/**
 * @brief   AWB profile
 */
/*****************************************************************************/
typedef struct CamCalibAwbPara_s{
	CAM_AwbVersion_t valid_version;
	CamAwbPara_V10_t Para_V10;
	CamAwbPara_V11_t Para_V11;
}CamCalibAwbPara_t;


/*****************************************************************************/
/**
 * @brief   ECM scheme
 */
/*****************************************************************************/
typedef struct CamEcmScheme_s
{
    void                    *p_next;                    /**< for adding to a list */

    CamEcmSchemeName_t      name;                       /**< scheme name */

    float                   OffsetT0Fac;                /**< start flicker avoidance above OffsetT0Fac * Tflicker integration time */
    float                   SlopeA0;                    /**< slope of gain */
} CamEcmScheme_t;



/*****************************************************************************/
/**
 * @brief   ECM profile
 */
/*****************************************************************************/
typedef struct CamEcmProfile_s
{
    void                    *p_next;                    /**< for adding to a list */

    CamEcmProfileName_t     name;                       /**< profile name => serves as resolution link as well */

    List                    ecm_scheme;                 /**< list of ECM schemes; at least one item is expected */
} CamEcmProfile_t;

/*****************************************************************************/
/**
 * @brief   Matrix coefficients
 *
 *          | 0 | 1 |  2 |3| 4 |  5 |
 *
 * @note    Coefficients are represented as float numbers
 */
/*****************************************************************************/
typedef struct Cam6x1FloatMatrix
{
	float fCoeff[6];
} Cam6x1FloatMatrix_t;

typedef struct CamAECGridWeight_s
{
	uint8_t   *pWeight;
	uint16_t  ArraySize;
}CamAECGridWeight_t;

/*****************************************************************************/
/**
 * @brief   Global AEC calibration structure
 */
/*****************************************************************************/
typedef struct CamCalibAecGlobal_s
{
    float                   SetPoint;                   /**< set point to hit by the ae control system */
    float                   ClmTolerance;
    float                   DampOverStill;              /**< damping coefficient for still image mode */
    float                   DampUnderStill;             /**< damping coefficient for still image mode */
    float                   DampOverVideo;              /**< damping coefficient for video mode */
    float                   DampUnderVideo;             /**< damping coefficient for video mode */
    float                   AfpsMaxGain;
    CamAECGridWeight_t 		GridWeights;//cxf
    float					EcmDotEnable;
    Cam6x1FloatMatrix_t     EcmTimeDot;
    Cam6x1FloatMatrix_t     EcmGainDot;
    float                   MeasuringWinWidthScale;//cxf
    float                   MeasuringWinHeightScale;//cxf
} CamCalibAecGlobal_t;

/******************************************************************************/
/**
 * @brief   Enumeration type for x scaling of the gamma curve 
 *
 * @note    This structure needs to be converted to driver structure
 *
 *****************************************************************************/
typedef enum CamEngineGammaOutXScale_e
{
    CAM_ENGINE_GAMMAOUT_XSCALE_INVALID  = 0,    /**< lower border (only for an internal evaluation) */
    CAM_ENGINE_GAMMAOUT_XSCALE_LOG      = 1,    /**< logarithmic segmentation from 0 to 4095 
                                                     (64,64,64,64,128,128,128,128,256,256,256,512,512,512,512,512) */
    CAM_ENGINE_GAMMAOUT_XSCALE_EQU      = 2,    /**< equidistant segmentation from 0 to 4095
                                                     (256, 256, ... ) */
    CAM_ENGINE_GAMMAOUT_XSCALE_MAX              /**< upper border (only for an internal evaluation) */
} CamEngineGammaOutXScale_t;

 /******************************************************************************/
 /**
 * @brief   This macro defines the number of elements in a gamma-curve.
 *
 *****************************************************************************/
#ifdef RK_ISP_V12
#define CAMERIC_ISP_GAMMA_CURVE_SIZE        34
#else
#define CAMERIC_ISP_GAMMA_CURVE_SIZE        17
#endif
/* @endcond */


/******************************************************************************/
/**
 * @brief   Structure to configure the gamma curve.
 *
 * @note    This structure needs to be converted to driver structure
 *
 *****************************************************************************/
typedef struct CamEngineGammaOutCurve_s
{
    CamEngineGammaOutXScale_t   xScale;
    uint16_t                    GammaY[CAMERIC_ISP_GAMMA_CURVE_SIZE];
} CamEngineGammaOutCurve_t;

typedef struct CamGoc_s
{
	uint8_t  mode;
    uint16_t gamma_y[34];
} CamGoc_t;


/*****************************************************************************/
/**
 * @brief   Gamma calibration structure
 */
/*****************************************************************************/
typedef struct CamCalibGammaOut_s
{
	CamEngineGammaOutCurve_t Curve;

} CamCalibGammaOut_t;


/*****************************************************************************/
/**
 * @brief   System data structure.
 */
/*****************************************************************************/
typedef struct CamCalibSystemData_s
{
    bool_t                  AfpsDefault;
    uint8_t                 OutputGrayMode;
} CamCalibSystemData_t;


typedef struct CamCalibWdrMaxGainLevelCurve_s {
  uint16_t nSize;
  uint16_t filter_enable;
  float*  pfSensorGain_level;
  float*  pfMaxGain_level;
} CamCalibWdrMaxGainLevelCurve_t;

typedef struct CamCalibWdrGlobal_s {
  uint16_t                   Enabled;
  uint16_t                   Mode;
  uint16_t                   LocalCurve[33];
  uint16_t                   GlobalCurve[33];
  uint16_t                   wdr_noiseratio;
  uint16_t                   wdr_bestlight;
  uint32_t                   wdr_gain_off1;
  uint16_t                   wdr_pym_cc;
  uint8_t                    wdr_epsilon;
  uint8_t                    wdr_lvl_en;
  uint8_t                    wdr_flt_sel;
  uint8_t                    wdr_gain_max_clip_enable;
  uint8_t                    wdr_gain_max_value;
  uint8_t                    wdr_bavg_clip;
  uint8_t                    wdr_nonl_segm;
  uint8_t                    wdr_nonl_open;
  uint8_t                    wdr_nonl_mode1;
  uint32_t                   wdr_coe0;
  uint32_t                   wdr_coe1;
  uint32_t                   wdr_coe2;
  uint32_t                   wdr_coe_off;

  CamCalibWdrMaxGainLevelCurve_t  wdr_MaxGain_Level_curve;
} CamCalibWdrGlobal_t;

#ifdef __cplusplus
}
#endif

/* @} cam_types */

#endif /* __CAM_TYPES_H__ */

