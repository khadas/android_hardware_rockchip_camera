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
ifeq ($(filter box vr stbvr, $(strip $(TARGET_BOARD_PLATFORM_PRODUCT))), )

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3399)
PRODUCT_PACKAGES += \
    libisp_isi_drv_OV2659 \
	libisp_isi_drv_OV8825 \
	libisp_isi_drv_OV8820 \
	libisp_isi_drv_OV8858 \
	libisp_isi_drv_OV8865 \
	libisp_isi_drv_GS8604 \
	libisp_isi_drv_OV5648 \
	libisp_isi_drv_OV5640 \
	libisp_isi_drv_OV13850 \
	libisp_isi_drv_IMX214 \
	libisp_isi_drv_HM2057 \
	libisp_isi_drv_HM5040 \
	libisp_isi_drv_SP2518 \
	libisp_isi_drv_GC0308 \
	libisp_isi_drv_GC2035 \
	libisp_isi_drv_GC2155 \
	libisp_isi_drv_GC2355 \
	libisp_isi_drv_NT99252 \
	libisp_isi_drv_OV2680 \
	libisp_isi_drv_OV5645 \
	libisp_isi_drv_TC358749XBG \
	libisp_isi_drv_OV2685
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3366)
PRODUCT_PACKAGES += \
    libisp_isi_drv_OV2659 \
	libisp_isi_drv_OV8825 \
	libisp_isi_drv_OV8820 \
	libisp_isi_drv_OV8858 \
	libisp_isi_drv_GS8604 \
	libisp_isi_drv_OV5648 \
	libisp_isi_drv_OV5640 \
	libisp_isi_drv_OV13850 \
	libisp_isi_drv_IMX214 \
	libisp_isi_drv_HM2057 \
	libisp_isi_drv_HM5040 \
	libisp_isi_drv_SP2518 \
	libisp_isi_drv_GC0308 \
	libisp_isi_drv_GC2035 \
	libisp_isi_drv_GC2155 \
	libisp_isi_drv_NT99252 \
	libisp_isi_drv_OV2680 \
	libisp_isi_drv_OV5645 \
	libisp_isi_drv_TC358749XBG \
	libisp_isi_drv_OV2685
endif


ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3368)
PRODUCT_PACKAGES += \
    libisp_isi_drv_OV2659 \
	libisp_isi_drv_OV8825 \
	libisp_isi_drv_OV8820 \
	libisp_isi_drv_OV8858 \
	libisp_isi_drv_GS8604 \
	libisp_isi_drv_OV5648 \
	libisp_isi_drv_OV5640 \
	libisp_isi_drv_OV13850 \
	libisp_isi_drv_IMX214 \
	libisp_isi_drv_HM2057 \
	libisp_isi_drv_HM5040 \
	libisp_isi_drv_SP2518 \
	libisp_isi_drv_GC0308 \
	libisp_isi_drv_GC2035 \
	libisp_isi_drv_GC2155 \
	libisp_isi_drv_NT99252 \
	libisp_isi_drv_OV2680 \
	libisp_isi_drv_OV5645 \
	libisp_isi_drv_TC358749XBG \
	libisp_isi_drv_OV2685
endif

ifeq ($(strip $(TARGET_BOARD_PLATFORM)), rk3288)
	
PRODUCT_PACKAGES += \
    libisp_isi_drv_OV2659 \
	libisp_isi_drv_OV8825 \
	libisp_isi_drv_OV8820 \
	libisp_isi_drv_OV8858 \
	libisp_isi_drv_GS8604 \
	libisp_isi_drv_OV5648 \
	libisp_isi_drv_OV5640 \
	libisp_isi_drv_OV13850 \
	libisp_isi_drv_IMX214 \
	libisp_isi_drv_HM2057 \
	libisp_isi_drv_HM5040 \
	libisp_isi_drv_SP2518 \
	libisp_isi_drv_GC0308 \
	libisp_isi_drv_GC2035 \
	libisp_isi_drv_GC2155 \
	libisp_isi_drv_NT99252 \
	libisp_isi_drv_OV2680 \
	libisp_isi_drv_OV5645 \
	libisp_isi_drv_TC358749XBG \
	libisp_isi_drv_OV2685
endif

endif
