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
 * @file cam_info.h
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
 * @defgroup cam_cb   Common Camera Type Definitions
 * @{
 *
 */
 #include <ebase/types.h>
 #include <time.h>
#ifndef __CAM_INFO_H__
#define __CAM_INFO_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct CamWindow_s
{
    uint16_t						hOffset;
    uint16_t						vOffset;
    uint16_t						width;
    uint16_t						height;
} CamWindow_t;

#define CAM_HIST_GRID_ITEMS         25  /**< number of grid sub windows */
typedef uint8_t CamHistWeights_t[CAM_HIST_GRID_ITEMS];
#define CAM_HIST_NUM_BINS           16  /**< number of bins */
typedef uint32_t CamHistBins_t[CAM_HIST_NUM_BINS];

typedef struct CamDeviceContext_s
{
	float 							gain;
	float 							time;
	uint32_t 						pos;
} CamDeviceContext_t;

typedef bool (*getDeviceInfoCb_t)(void *user, CamDeviceContext_t *DeviceContext);
typedef struct CamSensorCb_s
{
    getDeviceInfoCb_t       getDviceInfoCb;      /**< Notification callback */
    void*                  pUserContext;        /**< Pointer to user context to pass to callback */
} CamSensorCb_t;

#define CAM_AEC_HIST_NUM_BINS           16  /**< number of bins */
typedef uint32_t CamAecHistBins_t[CAM_AEC_HIST_NUM_BINS];
#define CAM_EXP_GRID_ITEMS          25  /**< number of grid items (see @ref CamerIcMeanLuma_t) */
typedef uint8_t CamMeanLuma_t[CAM_EXP_GRID_ITEMS];

typedef enum CamHistMode_e
{
    CAM_HIST_MODE_INVALID       = 0,    /**< lower border (only for an internal evaluation) */
    CAM_HIST_MODE_RGB_COMBINED  = 1,    /**< RGB combined histogram */
    CAM_HIST_MODE_R             = 2,    /**< R histogram */
    CAM_HIST_MODE_G             = 3,    /**< G histogram */
    CAM_HIST_MODE_B             = 4,    /**< B histogram */
    CAM_HIST_MODE_Y             = 5,    /**< luminance histogram */
    CAM_HIST_MODE_MAX,     				/**< upper border (only for an internal evaluation) */
} CamHistMode_t;

typedef enum CamExpMeasuringMode_e
{
    CAM_EXP_MEASURING_MODE_INVALID    = 0,    /**< invalid histogram measuring mode   */
    CAM_EXP_MEASURING_MODE_1          = 1,    /**< Y = (R + G + B) x (85/256)         */
    CAM_EXP_MEASURING_MODE_2          = 2,    /**< Y = 16 + 0.25R + 0.5G + 0.1094B    */
    CAM_EXP_MEASURING_MODE_MAX,
} CamExpMeasuringMode_t;

typedef struct CamTimestampContext_e
{
	struct timeval					vstart_tv;
	struct timeval					framein_tv;
} CamTimestampContext_t;


typedef struct CamAfmMeasuringResult_s
{
    uint32_t						SharpnessA;         /**< sharpness of window A */
    uint32_t						SharpnessB;         /**< sharpness of window B */
    uint32_t						SharpnessC;         /**< sharpness of window C */

    uint32_t						LuminanceA;         /**< luminance of window A */
    uint32_t						LuminanceB;         /**< luminance of window B */
    uint32_t						LuminanceC;         /**< luminance of window C */

    uint32_t						PixelCntA;
    uint32_t						PixelCntB;
    uint32_t						PixelCntC;

    CamWindow_t						WindowA;
    CamWindow_t						WindowB;
    CamWindow_t						WindowC;
} CamAfmMeasuringResult_t;

typedef struct CamHistContext_s
{
    bool_t						enabled;			/**< measuring enabled */
    CamHistMode_t					mode;				/**< histogram mode */
    uint16_t						StepSize;			/**< stepsize calculated from measuirng window */

    CamWindow_t						Window;				/**< measuring window */
    CamWindow_t						Grid;				/**< measuring Grid */
    CamHistWeights_t					Weights;			/**< measuring Weights */
    CamHistBins_t					Bins;				/**< measured Bins */
} CamHistContext_t;

typedef struct CamExpContext_s
{
    bool_t						enabled;			/**< measuring enabled */
    bool_t						autostop;			/**< stop measuring after a complete frame */
    CamExpMeasuringMode_t				mode;				/**< measuring mode */

    CamWindow_t						Window;				/**< measuring window */
    CamWindow_t						Grid;				/**< measuring window */
    CamMeanLuma_t					Luma;				/**< currently measured luma values */
} CamExpContext_t;

 typedef struct CamAfmContext_s
{
	bool_t							enabled;		/**< measuring enabled */

	CamWindow_t						WindowA;		/**< measuring window A */
	uint32_t						PixelCntA;
	bool_t							EnabledWdwA;
	CamWindow_t						WindowB;		/**< measuring window B */
	uint32_t						PixelCntB;
	bool_t							EnabledWdwB;
	CamWindow_t						WindowC;		/**< measuring window C */
	uint32_t						PixelCntC;
	bool_t							EnabledWdwC;

	uint32_t						Threshold;
	uint32_t						lum_shift;
	uint32_t						afm_shift;
	uint32_t						MaxPixelCnt;

	CamAfmMeasuringResult_t					MeasResult;
} CamAfmContext_t;


typedef struct Camispinfo_s {
	CamTimestampContext_t				timestamp;
	CamHistContext_t				CamHist;
	CamExpContext_t					CamExp;
	CamAfmContext_t					CamAfm;
	CamDeviceContext_t				pDeviceInfo;
}Camispinfo_t;

#ifdef __cplusplus
}
#endif

/* @} cam_info */

#endif /* __CAM_INFO_H__ */

