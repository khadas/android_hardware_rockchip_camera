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
#include "CameraHal.h"

namespace android{
CameraDeinterlace* mCameraDeinterlace;

CameraDeinterlace::CameraDeinterlace()
{


}
CameraDeinterlace::~CameraDeinterlace()
{

}
bool CameraDeinterlace::is_interlace_resolution(void){
	bool ret = false;
	char *p;
    char res_prop[PROPERTY_VALUE_MAX];
	property_get("sys.hdmiin.resolution",res_prop, "false");

	p = strstr(res_prop, "I");
	#if (CONFIG_EVEN_ODD_MERGE == 0x01) 
	ret = (NULL != p);
	#else
	ret = false;
	#endif
	return ret;
}

int CameraDeinterlace::odd_even_field_merge(mmerge_interface_t* merge_para ) 
{
	int ret = 0;
	mrga_interface_t* rga_para;
	bool is_interlace_resolution = true;
	is_interlace_resolution = CameraDeinterlace::is_interlace_resolution();

        rga_para=(mrga_interface_t *)malloc(sizeof(mrga_interface_t));
        if(!rga_para){
                LOGE("error,malloc failed!");
                goto failed;
        }
	rga_para->src = merge_para->src;
	//rga_para->offset_x = merge_para->offset_x;
	rga_para->offset_y = 0;
	rga_para->src_width = merge_para->src_width;
	rga_para->src_height = merge_para->src_height;
	rga_para->src_vir_width = merge_para->src_width;
	rga_para->src_vir_height = merge_para->src_height;
	//rga_para->dst = merge_para->dst;
	rga_para->dst_width = merge_para->dst_width;
	//rga_para->dst_height = merge_para->dst_height;
	//rga_para->dst_vir_width = merge_para->dst_vir_width;
	rga_para->dst_vir_height = merge_para->dst_height;
	rga_para->zoom_val = merge_para->zoom_val;
	rga_para->mirror = merge_para->mirror;
	rga_para->isNeedCrop = merge_para->isNeedCrop;
	rga_para->isDstNV21 = merge_para->isDstNV21;
	rga_para->is_viraddr_valid = merge_para->is_viraddr_valid;
	
#if (CONFIG_EVEN_ODD_MERGE == 0x01)
	if(!is_interlace_resolution) {
		rga_para->offset_x = 0;
		rga_para->dst = merge_para->dst;
		rga_para->dst_height = merge_para->dst_height;
		rga_para->dst_vir_width = merge_para->dst_width;
	} else {
		rga_para->dst_height = merge_para->dst_height/2;
		rga_para->dst_vir_width = merge_para->dst_width*2;
		if(!merge_para->is_even_field) {
			rga_para->offset_x = 0;
			rga_para->dst = merge_para->dst;
		} else {
			#if defined(RK_DRM_GRALLOC)
			rga_para->offset_x = merge_para->dst_width;
			rga_para->dst = merge_para->dst;
			#else
			rga_para->offset_x = 0;
			rga_para->dst = merge_para->dst + merge_para->dst_width;
			#endif
		}
	}
#else
	rga_para->offset_x = 0;
	rga_para->dst = merge_para->dst;
	rga_para->dst_height = merge_para->dst_height;
	rga_para->dst_vir_width = merge_para->dst_width;
#endif

	rga_nv12_scale_crop(rga_para);
        free(rga_para);

	return ret;
failed:
	return -1;
}


}
