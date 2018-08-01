# Copyright (C) 201 Fuzhou Rockchip Electronics Co., Ltd. All rights reserved.
# BY DOWNLOADING, INSTALLING, COPYING, SAVING OR OTHERWISE USING THIS SOFTWARE,
# YOU ACKNOWLEDGE THAT YOU AGREE THE SOFTWARE RECEIVED FORM ROCKCHIP IS PROVIDED
# TO YOU ON AN "AS IS" BASIS and ROCKCHP DISCLAIMS ANY AND ALL WARRANTIES AND
# REPRESENTATIONS WITH RESPECT TO SUCH FILE, WHETHER EXPRESS, IMPLIED, STATUTORY
# OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE,
# NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR
# A PARTICULAR PURPOSE. 
# Rockchip hereby grants to you a limited, non-exclusive, non-sublicensable and 
# non-transferable license (a) to install, save and use the Software; (b) to copy 
# and distribute the Software in binary code format only. 
# Except as expressively authorized by Rockchip in writing, you may NOT: (a) distribute 
# the Software in source code; (b) distribute on a standalone basis but you may distribute 
# the Software in conjunction with platforms incorporating Rockchip integrated circuits;
# (c) modify the Software in whole or part;(d) decompile, reverse-engineer, dissemble,
# or attempt to derive any source code from the Software;(e) remove or obscure any copyright,
# patent, or trademark statement or notices contained in the Software.

LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=\
	source/OV5640_PARREL.c\
	source/OV5640_tables.c\
    source/OV5640_af_firmware.c	

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include_priv\
	$(LOCAL_PATH)/../../include\
	$(LOCAL_PATH)/../../include_priv\
	$(LOCAL_PATH)/../../../include\
	
	

LOCAL_CFLAGS := -Wall -Wextra -std=c99   -Wformat-nonliteral -g -O0 -DDEBUG -pedantic
LOCAL_CFLAGS += -DLINUX  -DMIPI_USE_CAMERIC -DHAL_MOCKUP -DCAM_ENGINE_DRAW_DOM_ONLY -D_FILE_OFFSET_BITS=64 -DHAS_STDINT_H
#LOCAL_STATIC_LIBRARIES := libisp_ebase libisp_oslayer libisp_common libisp_hal libisp_cameric_reg_drv libisp_cameric_drv libisp_isi
LOCAL_SHARED_LIBRARIES := libutils libcutils libion

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 7.0)))
ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3326)
MY_ISP_LIB_NAME := lib_rkisp12_api
else
MY_ISP_LIB_NAME := lib_rkisp1_api
endif
else
MY_ISP_LIB_NAME := libisp_silicomimageisp_api
endif

LOCAL_SHARED_LIBRARIES += \
	$(MY_ISP_LIB_NAME)

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 8.0)))
	LOCAL_PROPRIETARY_MODULE := true
endif
LOCAL_MODULE:= libisp_isi_drv_OV5640

#LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH := hw
endif

LOCAL_MODULE_TAGS:= optional
include $(BUILD_SHARED_LIBRARY)

