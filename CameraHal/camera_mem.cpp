/******************************************************************************
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd. All rights reserved.
 * BY DOWNLOADING, INSTALLING, COPYING, SAVING OR OTHERWISE USING THIS SOFTWARE,
 * YOU ACKNOWLEDGE THAT YOU AGREE THE SOFTWARE RECEIVED FORM ROCKCHIP IS PROVIDED
 * TO YOU ON AN "AS IS" BASIS and ROCKCHP DISCLAIMS ANY AND ALL WARRANTIES AND
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
#include "camera_mem.h"

struct cam_mem_ops_des_s{
	const char* name;
	enum cam_mem_type_e mem_type;
	cam_mem_ops_t* ops;
};

//extern cam_mem_ops_t g_rk_ion_mem_ops;
//extern cam_mem_ops_t g_rk_ionDma_mem_ops;
extern cam_mem_ops_t g_rk_gralloc_mem_ops;

static struct cam_mem_ops_des_s cam_mem_ops_array[] = {
	{"ion",CAM_MEM_TYPE_ION,NULL},
	{"ionDma",CAM_MEM_TYPE_IONDMA,NULL},
	{"gralloc",CAM_MEM_TYPE_GRALLOC,&g_rk_gralloc_mem_ops},
};

cam_mem_ops_t* get_cam_ops(enum cam_mem_type_e mem_type)
{
	int ops_index = -1;
	switch(mem_type) {

		case CAM_MEM_TYPE_ION:
			ops_index = 0;
			break;
		case CAM_MEM_TYPE_IONDMA:
			ops_index = 1;
			break;
		case CAM_MEM_TYPE_GRALLOC:
			ops_index = 2;
			break;
		default:
			ops_index = -1;
	}

	if (ops_index != -1)
		return cam_mem_ops_array[ops_index].ops;
	else
		return NULL;
}

