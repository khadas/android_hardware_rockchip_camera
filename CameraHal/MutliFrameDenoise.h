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
#ifndef MY_JNI_MYJNI_H
#define MY_JNI_MYJNI_H



#include <jni.h>
#include "string.h"
#include "assert.h"
#include <system/window.h>
#include <hardware/gralloc.h>

enum OUTPUT_FMT {
	OUTPUT_FMT_CV_MAT,
	OUTPUT_FMT_YUV,
	OUTPUT_FMT_RGBA,
	OUTPUT_NONE
};

struct cv_fimc_buffer {
	void	*start;
	int share_fd;
	size_t	length;
	int stride;
	size_t	bytesused;
	buffer_handle_t handle;
};

class PerspectiveAdd;

class MutliFrameDenoise {
	public:
		bool initialized;
		MutliFrameDenoise();
		~MutliFrameDenoise();
		void initOpenGLES(alloc_device_t *m_alloc_dev, int width, int height);
		void updateImageData(struct cv_fimc_buffer *m_buffers_capture);
		void setFrames(int frameNum = 6);
		long processing(long* targetAddr,float _iso = 2,int mode = OUTPUT_NONE);
		int getResult(long targetAddr);
		void destroy();
	private:
		void getImageUnderDir(char *path, char *suffix);
	private:
		float mfdISO;
		int mFrameNum;
		PerspectiveAdd * g_APUnit;

};
#endif //MY_JNI_MYJNI_H
