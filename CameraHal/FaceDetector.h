/******************************************************************************
 *
 * Copyright (C) 2018 Fuzhou Rockchip Electronics Co., Ltd.
 * Modification based on code covered by the License (the "License").
 * You may not use this software except in compliance with the License.
 * THIS SOFTWARE IS PROVIDED TO YOU ON AN "AS IS" BASIS and ROCKCHIP DISCLAIMS 
 * ANY AND ALL WARRANTIES AND REPRESENTATIONS WITH RESPECT TO SUCH SOFTWARE, 
 * WHETHER EXPRESS,IMPLIED, STATUTORY OR OTHERWISE, INCLUDING WITHOUT LIMITATION,
 * ANY IMPLIED WARRANTIES OF TITLE, NON-INFRINGEMENT, MERCHANTABILITY, SATISFACTROY
 * QUALITY, ACCURACY OR FITNESS FOR A PARTICULAR PURPOSE. 
 * Rockchip shall not be liable to make any corrections to this software or to 
 * provide any support or assistance with respect to it.
 *
 *****************************************************************************/

#ifndef __FACE_DETECTOR_H_
#define __FACE_DETECTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

// Detector Type
enum {
    DETECTOR_OPENCV = 0,
    DETECTOR_OPENCL,
};

// Image Format
enum {
    IMAGE_RGBA8888 = 0,
    IMAGE_GRAYSCALE,
    IMAGE_YUV420SP,
    IMAGE_YUV420P,
};

// Image Orientation
enum {
    ROTATION_0 = 0,
    ROTATION_90,
    ROTATION_180,
    ROTATION_270,
};

struct RectFace {
    int x;
    int y;
    int width;
    int height;
};

#if 0
/*
  @type: Detector Type.
  @width: Initialize Image Width.
  @height: Initialize Image Height.
  @format: Image Format.
  @threshold: Detector Threshold, default = 10.0f.
*/
void FaceDetector_start(int type, int width, int height, int format, float threshold);

void FaceDetector_stop();

/*
  @src: Image Data.
  @orientation: Image Orientation.
  @faces: Output Find Face Rects.
  @num: Output Find Face Number.
*/
int FaceDetector_findFaces(void* src, int orientation, int isDrawRect, struct RectFace** faces, int* num);
#endif

#ifdef __cplusplus
}
#endif

#endif
