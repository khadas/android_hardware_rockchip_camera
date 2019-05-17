
/******************************************************************************
 *
 * Copyright 2019, Fuzhou Rockchip Electronics Co.Ltd. All rights reserved.
 * No part of this work may be reproduced, modified, distributed, transmitted,
 * transcribed, or translated into any language or computer format, in any form
 * or by any means without written permission of:
 * Fuzhou Rockchip Electronics Co.Ltd .
 *
 *
 *****************************************************************************/
/**
 * @file    uvc_hal_types.h
 *
 *****************************************************************************/
#ifndef _UVC_HAL_TYPES_H_
#define _UVC_HAL_TYPES_H_

#define _VERSION_(a,b,c) (((a)<<16)+((b)<<8)+(c))

#define UVC_HAL_VERSION           _VERSION_(1, 0x0, 0)

enum UvcCmd{
    CMD_REBOOT = 1,
    CMD_SET_CAPS,
    CMD_GET_CAPS,
    CMD_SET_BLS,
    CMD_GET_BLS,
    CMD_SET_LSC,
    CMD_GET_LSC,
    CMD_SET_CCM,
    CMD_GET_CCM,
    CMD_SET_AWB,
    CMD_GET_AWB,
    CMD_SET_AWB_WP,
    CMD_GET_AWB_WP,
    CMD_SET_AWB_CURV,
    CMD_GET_AWB_CURV,
    CMD_SET_AWB_REFGAIN,
    CMD_GET_AWB_REFGAIN,
    CMD_SET_GOC,
    CMD_GET_GOC,
    CMD_SET_CPROC,
    CMD_GET_CPROC,
    CMD_SET_DPF,
    CMD_GET_DPF,
    CMD_SET_FLT,
    CMD_GET_FLT,
    CMD_GET_SYSINFO,
    CMD_GET_SENSOR_INFO,
    CMD_GET_PROTOCOL_VER,
    CMD_SET_EXPOSURE,
    CMD_SET_MIRROR,
    CMD_SET_SENSOR_REG,
    CMD_GET_SENSOR_REG
};

enum ISP_UVC_CMD_TYPE_e{
    CMD_TYPE_SYNC = 0xF,
    CMD_TYPE_ASYNC = 0x1F
};

typedef int (*vpu_encode_jpeg_init_t)(int width,int height,int quant);
typedef void (*uvc_set_run_state_t)(bool state);
typedef void (*vpu_encode_jpeg_done_t)();
typedef bool (*uvc_get_run_state_t)();
typedef unsigned int (*uvc_get_fcc_t)();
typedef void (*uvc_get_resolution_t)(int* width, int* height);
typedef void (*uvc_buffer_write_t)(void* extra_data,
                                        size_t extra_size,
                                        void* data,
                                        size_t size,
                                        unsigned int fcc);
typedef	int (*vpu_encode_jpeg_doing_t)(
                                void* srcbuf,
                                int src_fd,
                                size_t src_size);
typedef void (*vpu_encode_jpeg_set_encbuf_t)(int fd, void *viraddr, unsigned long phyaddr, unsigned int size);
typedef void (*vpu_encode_jpeg_get_encbuf_t)(unsigned char** jpeg_out, unsigned int *jpeg_len);
typedef bool (*uvc_buffer_write_enable_t)();
typedef int (*uvc_main_proc_t)();
typedef void (*uvc_getMsg_t)(void *pMsg);
typedef void (*uvc_sem_signal_t)();
typedef unsigned int (*uvc_getVersion_t)();


typedef struct uvc_vpu_ops_s {
    vpu_encode_jpeg_init_t encode_init;
    vpu_encode_jpeg_done_t encode_deinit;
    vpu_encode_jpeg_doing_t encode_process;
    vpu_encode_jpeg_set_encbuf_t encode_set_buf;
    vpu_encode_jpeg_get_encbuf_t encode_get_buf;
}uvc_vpu_ops_t;

typedef struct uvc_proc_ops_s {
    uvc_set_run_state_t set_state;
    uvc_get_run_state_t get_state;
    uvc_get_fcc_t get_fcc;
    uvc_get_resolution_t get_res;
    uvc_buffer_write_t transfer_data;
    uvc_buffer_write_enable_t transfer_data_enable;
    uvc_main_proc_t  uvc_main_proc;
    uvc_getMsg_t     uvc_getMessage;
    uvc_sem_signal_t uvc_signal;
    uvc_getVersion_t uvc_getVersion;
}uvc_proc_ops_t;

enum HAL_BLS_MODE {
  HAL_BLS_MODE_FIXED = 0,
  HAL_BLS_MODE_AUTO = 1
};

enum HAL_BLS_WINCFG {
  HAL_BLS_WINCFG_OFF = 0,
  HAL_BLS_WINCFG_WIN1 = 1,
  HAL_BLS_WINCFG_WIN2 = 2,
  HAL_BLS_WINCFG_WIN1_2 = 3
};

typedef struct HAL_Bls_Win {
  uint16_t h_offs;
  uint16_t v_offs;
  uint16_t width;
  uint16_t height;
} __attribute__((packed)) HAL_Bls_Win_t;

struct HAL_ISP_bls_cfg_s {
  enum HAL_BLS_MODE mode;
  enum HAL_BLS_WINCFG win_cfg;
  HAL_Bls_Win_t win1;
  HAL_Bls_Win_t win2;
  uint8_t samples;
  uint16_t fixed_red;
  uint16_t fixed_greenR;
  uint16_t fixed_greenB;
  uint16_t fixed_blue;
}__attribute__((packed));

#define HAL_ISP_LSC_NAME_LEN         25
#define HAL_ISP_LSC_SIZE_TBL_LEN     8
#define HAL_ISP_LSC_MATRIX_COLOR_NUM 4
#define HAL_ISP_LSC_MATRIX_TBL_LEN   289

struct HAL_ISP_Lsc_Profile_s {
  int8_t    LscName[HAL_ISP_LSC_NAME_LEN];

  uint16_t  LscSectors;
  uint16_t  LscNo;
  uint16_t  LscXo;
  uint16_t  LscYo;

  uint16_t  LscXSizeTbl[HAL_ISP_LSC_SIZE_TBL_LEN];
  uint16_t  LscYSizeTbl[HAL_ISP_LSC_SIZE_TBL_LEN];

  uint16_t  LscMatrix[HAL_ISP_LSC_MATRIX_COLOR_NUM][HAL_ISP_LSC_MATRIX_TBL_LEN];
}__attribute__((packed));

struct HAL_ISP_Lsc_Query_s {
  int8_t    LscNameUp[HAL_ISP_LSC_NAME_LEN];
  int8_t    LscNameDn[HAL_ISP_LSC_NAME_LEN];
}__attribute__((packed));

#define HAL_ISP_ILL_NAME_LEN    20
struct HAL_ISP_AWB_CCM_SET_s {
  int8_t ill_name[HAL_ISP_ILL_NAME_LEN];
#if 0
  float coeff00;
  float coeff01;
  float coeff02;
  float coeff10;
  float coeff11;
  float coeff12;
  float coeff20;
  float coeff21;
  float coeff22;
#else
  float coeff[9];
#endif

  float ct_offset_r;
  float ct_offset_g;
  float ct_offset_b;
}__attribute__((packed));

struct HAL_ISP_AWB_CCM_GET_s {
  int8_t name_up[HAL_ISP_ILL_NAME_LEN];
  int8_t name_dn[HAL_ISP_ILL_NAME_LEN];
  #if 0
  float coeff00;
  float coeff01;
  float coeff02;
  float coeff10;
  float coeff11;
  float coeff12;
  float coeff20;
  float coeff21;
  float coeff22;
  #else
  float coeff[9];
  #endif
  float ct_offset_r;
  float ct_offset_g;
  float ct_offset_b;
}__attribute__((packed));

struct HAL_ISP_AWB_s {
  float r_gain;
  float gr_gain;
  float gb_gain;
  float b_gain;
  uint8_t lock_ill;
  char ill_name[HAL_ISP_ILL_NAME_LEN];
}__attribute__((packed));

#define HAL_ISP_AWBFADE2PARM_LEN  6
struct HAL_ISP_AWB_White_Point_Set_s {
  uint16_t win_h_offs;
  uint16_t win_v_offs;
  uint16_t win_width;
  uint16_t win_height;
  uint8_t awb_mode;
  #if 1//awb_v11
  float afFade[HAL_ISP_AWBFADE2PARM_LEN];
  float afmaxCSum_br[HAL_ISP_AWBFADE2PARM_LEN];
  float afmaxCSum_sr[HAL_ISP_AWBFADE2PARM_LEN];
  float afminC_br[HAL_ISP_AWBFADE2PARM_LEN];
  float afMaxY_br[HAL_ISP_AWBFADE2PARM_LEN];
  float afMinY_br[HAL_ISP_AWBFADE2PARM_LEN];
  float afminC_sr[HAL_ISP_AWBFADE2PARM_LEN];
  float afMaxY_sr[HAL_ISP_AWBFADE2PARM_LEN];
  float afMinY_sr[HAL_ISP_AWBFADE2PARM_LEN];
  float afRefCb[HAL_ISP_AWBFADE2PARM_LEN];
  float afRefCr[HAL_ISP_AWBFADE2PARM_LEN];
#else//awb_v10
    float afFade[HAL_ISP_AWBFADE2PARM_LEN];
    float afCbMinRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afCrMinRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afMaxCSumRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afCbMinRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afCrMinRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afMaxCSumRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afMinCRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afMinCRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afMaxYRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afMaxYRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afMinYMaxGRegionMax[HAL_ISP_AWBFADE2PARM_LEN];
    float afMinYMaxGRegionMin[HAL_ISP_AWBFADE2PARM_LEN];
    float afRefCb[HAL_ISP_AWBFADE2PARM_LEN];
    float afRefCr[HAL_ISP_AWBFADE2PARM_LEN];
#endif
  float fRgProjIndoorMin;
  float fRgProjOutdoorMin;
  float fRgProjMax;
  float fRgProjMaxSky;
  float fRgProjALimit;
  float fRgProjAWeight;
  float fRgProjYellowLimitEnable;
  float fRgProjYellowLimit;
  float fRgProjIllToCwfEnable;
  float fRgProjIllToCwf;
  float fRgProjIllToCwfWeight;
  float fRegionSize;
  float fRegionSizeInc;
  float fRegionSizeDec;

  uint32_t cnt;
  uint8_t mean_y;
  uint8_t mean_cb;
  uint8_t mean_cr;
  uint16_t mean_r;
  uint16_t mean_b;
  uint16_t mean_g;
}__attribute__((packed));


struct HAL_ISP_AWB_White_Point_Get_s {
  uint16_t win_h_offs;
  uint16_t win_v_offs;
  uint16_t win_width;
  uint16_t win_height;
  uint8_t awb_mode;
  uint32_t cnt;
  uint8_t mean_y;
  uint8_t mean_cb;
  uint8_t mean_cr;
  uint16_t mean_r;
  uint16_t mean_b;
  uint16_t mean_g;

  uint8_t RefCr;
  uint8_t RefCb;
  uint8_t MinY;
  uint8_t MaxY;
  uint8_t MinC;
  uint8_t MaxCSum;

  float RgProjection;
  float RegionSize;
  float Rg_clipped;
  float Rg_unclipped;
  float Bg_clipped;
  float Bg_unclipped;
}__attribute__((packed));


#define HAL_ISP_CURVE_NAME_LEN    20
#define HAL_ISP_AWBCLIPPARM_LEN   16
struct HAL_ISP_AWB_Curve_s {
  float f_N0_Rg;
  float f_N0_Bg;
  float f_d;
  float Kfactor;

  float afRg1[HAL_ISP_AWBCLIPPARM_LEN];
  float afMaxDist1[HAL_ISP_AWBCLIPPARM_LEN];
  float afRg2[HAL_ISP_AWBCLIPPARM_LEN];
  float afMaxDist2[HAL_ISP_AWBCLIPPARM_LEN];
  float afGlobalFade1[HAL_ISP_AWBCLIPPARM_LEN];
  float afGlobalGainDistance1[HAL_ISP_AWBCLIPPARM_LEN];
  float afGlobalFade2[HAL_ISP_AWBCLIPPARM_LEN];
  float afGlobalGainDistance2[HAL_ISP_AWBCLIPPARM_LEN];
}__attribute__((packed));

struct HAL_ISP_AWB_RefGain_s {
  int8_t ill_name[HAL_ISP_ILL_NAME_LEN];
  float refRGain;
  float refGrGain;
  float refGbGain;
  float refBGain;
}__attribute__((packed));

#define HAL_ISP_GOC_SCENE_NAME_LEN    20
//#ifdef RKISP_V12
#define HAL_ISP_GOC_GAMMA_NUM    34
//#else
//#define HAL_ISP_GOC_GAMMA_NUM    17
//#endif
enum HAL_ISP_GOC_WDR_STATUS {
  HAL_ISP_GOC_NORMAL,
  HAL_ISP_GOC_WDR_ON
};

enum HAL_ISP_GOC_CFG_MODE {
  HAL_ISP_GOC_CFG_MODE_LOGARITHMIC = 1,
  HAL_ISP_GOC_CFG_MODE_EQUIDISTANT
};

struct HAL_ISP_GOC_s {
  int8_t scene_name[HAL_ISP_GOC_SCENE_NAME_LEN];
  enum HAL_ISP_GOC_WDR_STATUS wdr_status;
  enum HAL_ISP_GOC_CFG_MODE cfg_mode;
  uint16_t gamma_y[HAL_ISP_GOC_GAMMA_NUM];
}__attribute__((packed));

enum HAL_ISP_CPROC_MODE {
  HAL_ISP_CPROC_PREVIEW,
  HAL_ISP_CPROC_CAPTURE,
  HAL_ISP_CPROC_VIDEO
};

struct HAL_ISP_CPROC_s {
  enum HAL_ISP_CPROC_MODE mode;
  float cproc_contrast;
  float cproc_hue;
  float cproc_saturation;
  int8_t cproc_brightness;
}__attribute__((packed));


#define HAL_ISP_ADPF_DPF_NAME_LEN  20
#define HAL_ISP_ADPF_DPF_NLL_COEFF_LEN  17
struct HAL_ISP_ADPF_DPF_s {
  int8_t  dpf_name[HAL_ISP_ADPF_DPF_NAME_LEN];
  uint8_t dpf_enable;
  uint8_t nll_segment;
  uint16_t nll_coeff[HAL_ISP_ADPF_DPF_NLL_COEFF_LEN];
  uint16_t sigma_green;
  uint16_t sigma_redblue;
  float gradient;
  float offset;
  float fRed;
  float fGreenR;
  float fGreenB;
  float fBlue;
}__attribute__((packed));

#define HAL_ISP_FLT_CURVE_NUM 5
#define HAL_ISP_FLT_NAME_LEN  20
struct HAL_ISP_FLT_Denoise_Curve_s {
  uint8_t denoise_gain[HAL_ISP_FLT_CURVE_NUM];
  uint8_t denoise_level[HAL_ISP_FLT_CURVE_NUM];
}__attribute__((packed));

struct HAL_ISP_FLT_Sharp_Curve_s {
  uint8_t sharp_gain[HAL_ISP_FLT_CURVE_NUM];
  uint8_t sharp_level[HAL_ISP_FLT_CURVE_NUM];
}__attribute__((packed));

struct HAL_ISP_FLT_Level_Conf_s {
  uint8_t grn_stage1;
  uint8_t chr_h_mode;
  uint8_t chr_v_mode;
  #if 1
  uint32_t thresh_bl0;
  uint32_t thresh_bl1;
  uint32_t thresh_sh0;
  uint32_t thresh_sh1;
  uint32_t fac_sh1;
  uint32_t fac_sh0;
  uint32_t fac_mid;
  uint32_t fac_bl0;
  uint32_t fac_bl1;
  #else
  float thresh_bl0;
  float thresh_bl1;
  float thresh_sh0;
  float thresh_sh1;
  float fac_sh1;
  float fac_sh0;
  float fac_mid;
  float fac_bl0;
  float fac_bl1;
  #endif
}__attribute__((packed));

struct HAL_ISP_FLT_Set_s {
  int8_t  filter_name[HAL_ISP_FLT_NAME_LEN];
  uint8_t scene_mode;
  uint8_t filter_enable;
  struct HAL_ISP_FLT_Denoise_Curve_s denoise;
  struct HAL_ISP_FLT_Sharp_Curve_s sharp;
  uint8_t level_conf_enable;
  uint8_t level;
  struct HAL_ISP_FLT_Level_Conf_s level_conf;
}__attribute__((packed));

struct HAL_ISP_FLT_Get_ParamIn_s {
  uint8_t scene;
  uint8_t level;
}__attribute__((packed));

struct HAL_ISP_FLT_Get_s {
  int8_t  filter_name[HAL_ISP_FLT_NAME_LEN];
  uint8_t filter_enable;
  struct HAL_ISP_FLT_Denoise_Curve_s denoise;
  struct HAL_ISP_FLT_Sharp_Curve_s sharp;
  uint8_t level_conf_enable;
  uint8_t is_level_exit;
  struct HAL_ISP_FLT_Level_Conf_s level_conf;
}__attribute__((packed));

#define HAL_ISP_STORE_PATH_LEN         32
enum HAL_ISP_CAP_FORMAT {
  HAL_ISP_FMT_YUV420 = 0x18,
  HAL_ISP_FMT_YUV422 = 0x1E,
  HAL_ISP_FMT_RAW10 = 0x2B,
  HAL_ISP_FMT_RAW12 = 0x2C
};

enum HAL_ISP_CAP_RESULT {
  HAL_ISP_CAP_FINISH,
  HAL_ISP_CAP_RUNNING
};

enum HAL_ISP_AE_MODE {
  HAL_ISP_AE_MANUAL,
  HAL_ISP_AE_AUTO
};

struct HAL_ISP_Cap_Req_s {
  uint8_t cap_id;
  int8_t  store_path[HAL_ISP_STORE_PATH_LEN];
  enum HAL_ISP_CAP_FORMAT cap_format;
  uint8_t cap_num;
  uint16_t cap_height;
  uint16_t cap_width;
  enum HAL_ISP_AE_MODE ae_mode;
  uint8_t exp_time_h;
  uint8_t exp_time_l;
  uint8_t exp_gain_h;
  uint8_t exp_gain_l;
  uint16_t af_code;
}__attribute__((packed));

struct HAL_ISP_Cap_Result_s {
  uint8_t cap_id;
  enum HAL_ISP_CAP_RESULT result;
}__attribute__((packed));


#define HAL_ISP_SYS_INFO_LEN         32
#define HAL_ISP_SENSOR_RESOLUTION_NUM         8

struct HAL_ISP_Sensor_Reso_s {
  uint16_t width;
  uint16_t height;
}__attribute__((packed));

struct HAL_ISP_OTP_Info_s {
  uint8_t awb_otp:1;
  uint8_t lsc_otp:1;
}__attribute__((packed));

struct HAL_ISP_Sys_Info_s {
  int8_t  platform[HAL_ISP_SYS_INFO_LEN];
  int8_t  sensor[HAL_ISP_SYS_INFO_LEN];
  int8_t  module[HAL_ISP_SYS_INFO_LEN];
  int8_t  lens[HAL_ISP_SYS_INFO_LEN];
  int8_t  iq_name[HAL_ISP_SYS_INFO_LEN*2];
  uint8_t otp_info;
  uint8_t max_exp_time_h;
  uint8_t max_exp_time_l;
  uint8_t max_exp_gain_h;
  uint8_t max_exp_gain_l;
  uint8_t reso_num;
  struct HAL_ISP_Sensor_Reso_s reso[HAL_ISP_SENSOR_RESOLUTION_NUM];
  uint8_t sensor_fmt;
}__attribute__((packed));

struct HAL_ISP_Sensor_Mirror_s {
  uint8_t horizontal_mirror:1;
  uint8_t vertical_mirror:1;
}__attribute__((packed));

struct HAL_ISP_Sensor_Info_s {
  uint8_t exp_time_h;
  uint8_t exp_time_l;
  uint8_t exp_gain_h;
  uint8_t exp_gain_l;
  uint8_t mirror_info;
  uint16_t frame_length_lines;
  uint16_t line_length_pck;
  uint32_t vt_pix_clk_freq_hz;
  uint8_t binning;
  uint8_t black_white_mode;
}__attribute__((packed));

struct HAL_ISP_Sensor_Exposure_s {
  enum HAL_ISP_AE_MODE ae_mode;
  uint8_t exp_time_h;
  uint8_t exp_time_l;
  uint8_t exp_gain_h;
  uint8_t exp_gain_l;
}__attribute__((packed));

struct HAL_ISP_Sensor_Reg_s {
  uint8_t reg_addr_len;
  uint16_t reg_addr;
  uint8_t reg_data_len;
  uint16_t reg_data;
}__attribute__((packed));

#define HAL_ISP_IQ_PATH_LEN    32
struct HAL_ISP_Reboot_Req_s {
  uint8_t reboot;
  int8_t  iq_path[HAL_ISP_IQ_PATH_LEN];
}__attribute__((packed));

struct HAL_ISP_Protocol_Ver_s {
    uint8_t major_ver;
    uint8_t minor_ver;
    uint32_t magicCode;
}__attribute__((packed));

#endif
