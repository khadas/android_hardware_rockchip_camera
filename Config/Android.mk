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
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3326)
LOCAL_MODULE := lib_rkisp12_api
else
LOCAL_MODULE := lib_rkisp1_api
endif

ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 7.0)))
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_7x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_7x_64bit.so
else
ifeq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 6.0)))
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_6x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_6x_64bit.so
else
LOCAL_SRC_FILES_arm := $(LOCAL_MODULE)_5x_32bit.so
LOCAL_SRC_FILES_arm64 := $(LOCAL_MODULE)_5x_64bit.so
endif
endif

ifneq ($(filter rk3366 rk3399 rk3328, $(strip $(TARGET_BOARD_PLATFORM))), )
#include $(CLEAR_VARS)
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH :=
endif
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3368)
#include $(CLEAR_VARS)
ifneq (1,$(strip $(shell expr $(PLATFORM_VERSION) \>= 5.0)))
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)
else
ifneq ($(strip $(TARGET_2ND_ARCH)), )
LOCAL_MULTILIB := both
endif
LOCAL_MODULE_RELATIVE_PATH :=
endif
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3288)
#include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifneq ($(filter rk322x rk312x rk3126c rk3128 px3se, $(strip $(TARGET_BOARD_PLATFORM))), )
#include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3036)
#include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)),rk3188)
#include $(CLEAR_VARS)
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_STEM := $(LOCAL_MODULE)
LOCAL_MODULE_SUFFIX := .so
include $(BUILD_PREBUILT)
endif

