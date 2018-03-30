#
# RockChip Camera HAL 
#
LOCAL_PATH:= $(call my-dir)

#$(info my-dir=  $(call my-dir) )
#include $(all-subdir-makefiles)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:=\
	isi.c\
	isisup.c\
	

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH)/../include\
	$(LOCAL_PATH)/../include_priv\
	$(LOCAL_PATH)/../../include\


LOCAL_CFLAGS := -Wall -Wextra -std=c99   -Wformat-nonliteral -g -O0 -DDEBUG -pedantic
LOCAL_CFLAGS += -DLINUX  -DMIPI_USE_CAMERIC -DHAL_MOCKUP -DCAM_ENGINE_DRAW_DOM_ONLY -D_FILE_OFFSET_BITS=64 -DHAS_STDINT_H
#LOCAL_STATIC_LIBRARIES := libisp_ebase libisp_oslayer libisp_common libisp_hal libisp_cameric_reg_drv libisp_cameric_drv 
#LOCAL_WHOLE_STATIC_LIBRARIES := libisp_ebase libisp_common libisp_hal libisp_cameric_reg_drv libisp_cameric_drv
#full_path := $(shell pwd)
#LOCAL_LDFLAGS := $(full_path)/out/target/product/rk30sdk/obj/STATIC_LIBRARIES/libisp_hal_intermediates/libisp_hal.a
#LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

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

LOCAL_MODULE:= libisp_isi

LOCAL_MODULE_TAGS:= optional
#include $(BUILD_SHARED_LIBRARY)
include $(BUILD_STATIC_LIBRARY)

