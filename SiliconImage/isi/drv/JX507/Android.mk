#
# RockChip Camera HAL 
#
LOCAL_PATH:= $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SRC_FILES:=\
	source/JX507_MIPI.c\
	source/JX507_tables.c\
	

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/include_priv\
	$(LOCAL_PATH)/../../include\
	$(LOCAL_PATH)/../../include_priv\
	$(LOCAL_PATH)/../../../include\
	
	

LOCAL_CFLAGS := -Wall -Wextra -std=c99  -Wformat-nonliteral -g -O0 -DDEBUG -pedantic
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
LOCAL_MODULE:= libisp_isi_drv_JX507

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

