/******************************************************************************
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd. All rights reserved.
 * BY DOWNLOADING, INSTALLING, COPYING, SAVING OR OTHERWISE USING THIS SOFTWARE,
 * YOU ACKNOWLEDGE THAT YOU AGREE THE SOFTWARE RECEIVED FROM ROCKCHIP IS PROVIDED
 * TO YOU ON AN "AS IS" BASIS and ROCKCHIP DISCLAIMS ANY AND ALL WARRANTIES AND
 * REPRESENTATIONS WITH RESPECT TO SUCH FILE, WHETHER EXPRESS, IMPLIED, STATUTORY
 * OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY IMPLIED WARRANTIES OF TITLE,
 * NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY QUALITY, ACCURACY OR FITNESS FOR
 * A PARTICULAR PURPOSE. 
 * Rockchip hereby grants to you a limited, non-exclusive, non-sublicensable and 
 * non-transferable license (a) to install, save and use the Software; (b) to copy 
 * and distribute the Software in binary code format only. 
 * Except as expressively authorized by Rockchip in writing, you may NOT: (a) distribute 
 * the Software in source code; (b) distribute on a standalone basis but you may distribute 
 * the Software in conjunction with platforms incorporating Rockchip integrated circuits;
 * (c) modify the Software in whole or part;(d) decompile, reverse-engineer, dissemble,
 * or attempt to derive any source code from the Software;(e) remove or obscure any copyright,
 * patent, or trademark statement or notices contained in the Software.
 *
 *****************************************************************************/
#ifndef __CAMERA_MEM__H__
#define __CAMERA_MEM__H__
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <unistd.h>
#ifdef __cplusplus
extern "C"
{
#endif

enum cam_mem_type_e {
	CAM_MEM_TYPE_INVALID,
	CAM_MEM_TYPE_ION,
	CAM_MEM_TYPE_IONDMA,
	CAM_MEM_TYPE_GRALLOC,
};

enum cam_mem_flag_e {
	CAM_MEM_FLAG_HW_WRITE	= 0x1,
	CAM_MEM_FLAG_HW_READ	= 0x2,
	CAM_MEM_FLAG_SW_WRITE	= 0x4,
	CAM_MEM_FLAG_SW_READ	= 0x8,
};

#define DEFAULT_MEM_FLAG (CAM_MEM_FLAG_HW_WRITE|CAM_MEM_FLAG_HW_READ|\
                          CAM_MEM_FLAG_SW_WRITE|CAM_MEM_FLAG_SW_READ)

typedef struct cam_mem_handle_s {
	enum cam_mem_type_e mem_type;
	int iommu_enabled;
	int phy_continuos;
	int camsys_fd;
	unsigned int flag;
	void* priv;

}cam_mem_handle_t;


typedef struct cam_mem_info_s {
	cam_mem_handle_t* handlle;
	unsigned long vir_addr;
    unsigned long phy_addr;
	unsigned long mmu_addr;
	int iommu_maped;
    size_t size;
	int fd;
	void* priv;
}cam_mem_info_t;


typedef struct cam_mem_ops_s {
	//init
	cam_mem_handle_t* (*init)(int iommu_enabled,unsigned int mem_flag,int phy_continuos);
	//alloc
	cam_mem_info_t* (*alloc)(cam_mem_handle_t* handle,size_t size);
	//free
	int (*free)(cam_mem_handle_t* handle,cam_mem_info_t* mem);
	int (*iommu_map)(cam_mem_handle_t* handle,cam_mem_info_t* mem);
	int (*iommu_unmap)(cam_mem_handle_t* handle,cam_mem_info_t* mem);
	//flush cache
	int (*flush_cache)(cam_mem_handle_t* handle,cam_mem_info_t* mem);
	//deinit
	int (*deInit)(cam_mem_handle_t* handle);
}cam_mem_ops_t;


cam_mem_ops_t* get_cam_ops(enum cam_mem_type_e mem_type);


#ifdef __cplusplus
}
#endif

#endif

