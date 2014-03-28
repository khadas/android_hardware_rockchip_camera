#include <stdio.h>
#include <stdlib.h> 
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <utils/Log.h>
#include <linux/videodev2.h> 
#include <binder/IPCThreadState.h>


#include <camera/CameraParameters.h>

#if defined(TARGET_RK30) && (defined(TARGET_BOARD_PLATFORM_RK30XX) || (defined(TARGET_BOARD_PLATFORM_RK2928)))
#include "../libgralloc_ump/gralloc_priv.h"
#if (CONFIG_CAMERA_INVALIDATE_RGA==0)
#include <hardware/rga.h>
#endif
#elif defined(TARGET_RK30) && defined(TARGET_BOARD_PLATFORM_RK30XXB)
#include <hardware/hal_public.h>
#include <hardware/rga.h>
#elif defined(TARGET_RK29)
#include "../libgralloc/gralloc_priv.h"
#endif

#include "../jpeghw/release/encode_release/rk29-ipp.h"

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

#ifdef ALOGD
#define LOGD      ALOGD
#endif
#ifdef ALOGV
#define LOGV      ALOGV
#endif
#ifdef ALOGE
#define LOGE      ALOGE
#endif
#ifdef ALOGI
#define LOGI      ALOGI
#endif

#define CAMERA_DISPLAY_FORMAT_NV12       "nv12"

static char cameraCallProcess[30];
extern "C" int getCallingPid() {
    return android::IPCThreadState::self()->getCallingPid();
}

extern "C" char* getCallingProcess()
{
    int fp = -1;
	cameraCallProcess[0] = 0x00; 
	sprintf(cameraCallProcess,"/proc/%d/cmdline",getCallingPid());

	fp = open(cameraCallProcess, O_RDONLY);

	if (fp < 0) {
		memset(cameraCallProcess,0x00,sizeof(cameraCallProcess));
		LOGE("Obtain calling process info failed");
	} 
	else {
		memset(cameraCallProcess,0x00,sizeof(cameraCallProcess));
		read(fp, cameraCallProcess, 29);
		close(fp);
		fp = -1;
		LOGD("Calling process is: %s",cameraCallProcess);
	}
    return cameraCallProcess;

}

extern "C" int cameraPixFmt2HalPixFmt(const char *fmt)
{
    int hal_pixel_format=HAL_PIXEL_FORMAT_YCrCb_NV12;
    
    if (strcmp(fmt,android::CameraParameters::PIXEL_FORMAT_RGB565) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_RGB_565;        
    } else if (strcmp(fmt,android::CameraParameters::PIXEL_FORMAT_YUV420SP) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_420_SP;
    } else if (strcmp(fmt,CAMERA_DISPLAY_FORMAT_NV12) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCrCb_NV12;
    } else if (strcmp(fmt,android::CameraParameters::PIXEL_FORMAT_YUV422SP) == 0) {
        hal_pixel_format = HAL_PIXEL_FORMAT_YCbCr_422_SP;
    } else {
        hal_pixel_format = -EINVAL;
        LOGE("%s(%d): pixel format %s is unknow!",__FUNCTION__,__LINE__,fmt);        
    }

    return hal_pixel_format;
}
extern "C" void arm__scale_crop_nv12torgb565(int srcW, int srcH,int dstW,int dstH, char *srcbuf, short int *dstbuf)
{
#if 0
    int line, col;
    int uv_src_w = srcW/2;
    int uv_src_h = srcH/2;
    int src_w = srcW,src_h = srcH;
    int dst_w = dstW,dst_h = dstH;
    int ratio,cropW,cropH,left_offset,top_offset;
    char* src,*psY,*psUV,*dst,*pdUV,*pdY;
	long zoomindstxIntInv,zoomindstyIntInv;
	long sX,sY,sY_UV;
    
	long yCoeff00_y,yCoeff01_y,xCoeff00_y,xCoeff01_y;
	long yCoeff00_uv,yCoeff01_uv,xCoeff00_uv,xCoeff01_uv;
	long r0,r1,a,b,c,d,ry,ru,rv;

    int y, u, v, yy, vr, ug, vg, ub,r,g,b1;

    

	src = psY = (unsigned char*)(srcbuf)+top_offset*src_w+left_offset;
	//psUV = psY +src_w*src_h+top_offset*src_w/2+left_offset;
	psUV = (unsigned char*)(srcbuf) +src_w*src_h+top_offset*src_w/2+left_offset;

	

	
	dst = pdY = (unsigned char*)dstbuf; 
	pdUV = pdY + dst_w*dst_h;

    	//need crop ?
	if((src_w*100/src_h) != (dst_w*100/dst_h)){
		ratio = ((src_w*100/dst_w) >= (src_h*100/dst_h))?(src_h*100/dst_h):(src_w*100/dst_w);
		cropW = ratio*dst_w/100;
		cropH = ratio*dst_h/100;
		
		left_offset=((src_w-cropW)>>1) & (~0x01);
		top_offset=((src_h-cropH)>>1) & (~0x01);
	}else{
		cropW = src_w;
		cropH = src_h;
		top_offset=0;
		left_offset=0;
	}

	zoomindstxIntInv = ((unsigned long)(cropW)<<16)/dstW + 1;
	zoomindstyIntInv = ((unsigned long)(cropH)<<16)/dstH + 1;

    for (line = 0; line < dstH; line++) {
		yCoeff00_y = (line*zoomindstyIntInv)&0xffff;
		yCoeff01_y = 0xffff - yCoeff00_y; 
		sY = (line*zoomindstyIntInv >> 16);
		sY = (sY >= srcH - 1)? (srcH - 2) : sY; 
        
        sY_UV = = ((line/2)*zoomindstyIntInv >> 16);
	    sY_UV = (sY_UV >= uv_src_h - 1)? (uv_src_h - 2) : sY_UV; 
        for (col = 0; col < dstW; col++) {


            //get y
			xCoeff00_y = (col*zoomindstxIntInv)&0xffff;
			xCoeff01_y = 0xffff - xCoeff00_y;	
			sX = (col*zoomindstxIntInv >> 16);
			sX = (sX >= srcW -1)?(srcW- 2) : sX;
			a = psY[sY*srcW + sX];
			b = psY[sY*srcW + sX + 1];
			c = psY[(sY+1)*srcW + sX];
			d = psY[(sY+1)*srcW + sX + 1];

			r0 = (a * xCoeff01_y + b * xCoeff00_y)>>16 ;
			r1 = (c * xCoeff01_y + d * xCoeff00_y)>>16 ;
			ry = (r0 * yCoeff01_y + r1 * yCoeff00_y)>>16;

            //get u & v
			xCoeff00_uv = ((col/2)*zoomindstxIntInv)&0xffff;
			xCoeff01_uv = 0xffff - xCoeff00_uv;	
			sX = ((col/2)*zoomindstxIntInv >> 16);
			sX = (sX >= uv_src_w -1)?(uv_src_w- 2) : sX;
			//U
			a = psUV[(sY*uv_src_w + sX)*2];
			b = psUV[(sY*uv_src_w + sX + 1)*2];
			c = psUV[((sY+1)*uv_src_w + sX)*2];
			d = psUV[((sY+1)*uv_src_w + sX + 1)*2];

			r0 = (a * xCoeff01_uv + b * xCoeff00_uv)>>16 ;
			r1 = (c * xCoeff01_uv + d * xCoeff00_uv)>>16 ;
			ru = (r0 * yCoeff01_uv + r1 * yCoeff00_uv)>>16;

            //v

			a = psUV[(sY*uv_src_w + sX)*2 + 1];
			b = psUV[(sY*uv_src_w + sX + 1)*2 + 1];
			c = psUV[((sY+1)*uv_src_w + sX)*2 + 1];
			d = psUV[((sY+1)*uv_src_w + sX + 1)*2 + 1];

			r0 = (a * xCoeff01_uv + b * xCoeff00_uv)>>16 ;
			r1 = (c * xCoeff01_uv + d * xCoeff00_uv)>>16 ;
			rv = (r0 * yCoeff01_uv + r1 * yCoeff00_uv)>>16;



            yy = ry << 8;
            u =  ru - 128;
            ug = 88 * u;
            ub = 454 * u;
            v =  rv - 128;
            vg = 183 * v;
            vr = 359 * v;

            r = (yy +      vr) >> 8;
            g = (yy - ug - vg) >> 8;
            b = (yy + ub     ) >> 8;
            if (r < 0)   r = 0;
            if (r > 255) r = 255;
            if (g < 0)   g = 0;
            if (g > 255) g = 255;
            if (b < 0)   b = 0;
            if (b > 255) b = 255;

            
        }
    }
#endif
}


extern "C" void arm_nv12torgb565(int width, int height, char *src, short int *dst,int dstbuf_w)
{
    int line, col;
    int y, u, v, yy, vr, ug, vg, ub;
    int r, g, b;
    char *py, *pu, *pv;    

    py = src;
    pu = py + (width * height);
    pv = pu + 1;
    y = *py++;
    yy = y << 8;
    u = *pu - 128;
    ug = 88 * u;
    ub = 454 * u;
    v = *pv - 128;
    vg = 183 * v;
    vr = 359 * v;
    
    for (line = 0; line < height; line++) {
        for (col = 0; col < width; col++) {
            r = (yy +      vr) >> 8;
            g = (yy - ug - vg) >> 8;
            b = (yy + ub     ) >> 8;
            if (r < 0)   r = 0;
            if (r > 255) r = 255;
            if (g < 0)   g = 0;
            if (g > 255) g = 255;
            if (b < 0)   b = 0;
            if (b > 255) b = 255;
            
            *dst++ = (((__u16)r>>3)<<11) | (((__u16)g>>2)<<5) | (((__u16)b>>3)<<0);

            y = *py++;
            yy = y << 8;
            if (col & 1) {
                pu += 2;
                pv = pu+1;
                u = *pu - 128;
                ug =   88 * u;
                ub = 454 * u;
                v = *pv - 128;
                vg = 183 * v;
                vr = 359 * v;
            }
        }
        dst += dstbuf_w - width;
        if ((line & 1) == 0) { 
            //even line: rewind
            pu -= width;
            pv = pu+1;
        }
    }
}




extern "C" int rga_nv12torgb565(int src_width, int src_height, char *src, short int *dst, int dstbuf_width,int dst_width,int dst_height)
{
    int rgafd = -1,ret = -1;

    //LOGD("rga_nv12torgb565: (%dx%d)->(%dx%d),src_buf = 0x%x,dst_buf = 0x%x",src_width,src_height,dst_width,dst_height,src,dst);

#ifdef TARGET_RK30

    if((rgafd = open("/dev/rga",O_RDWR)) < 0) {
    	LOGE("%s(%d):open rga device failed!!",__FUNCTION__,__LINE__);
        ret = -1;
    	return ret;
	}

#if (CONFIG_CAMERA_INVALIDATE_RGA==0)
    struct rga_req  Rga_Request;
    int err = 0;
    
    memset(&Rga_Request,0x0,sizeof(Rga_Request));

	unsigned char *psY, *psUV;
	int srcW,srcH,cropW,cropH;
	int ratio = 0;
	int top_offset=0,left_offset=0;
//need crop ?
	if((src_width*100/src_height) != (dst_width*100/dst_height)){
		ratio = ((src_width*100/dst_width) >= (src_height*100/dst_height))?(src_height*100/dst_height):(src_width*100/dst_width);
		cropW = ratio*dst_width/100;
		cropH = ratio*dst_height/100;
		
		left_offset=((src_width-cropW)>>1) & (~0x01);
		top_offset=((src_height-cropH)>>1) & (~0x01);
	}else{
		cropW = src_width;
		cropH = src_height;
		top_offset=0;
		left_offset=0;
	}

	psY = (unsigned char*)(src)+top_offset*src_width+left_offset;
	psUV = (unsigned char*)(src) +src_width*src_height+top_offset*src_width/2+left_offset;
	
	Rga_Request.src.yrgb_addr =  (int)psY;
    Rga_Request.src.uv_addr  = (int)psUV;
    Rga_Request.src.v_addr   =  Rga_Request.src.uv_addr;
    Rga_Request.src.vir_w =  src_width;
    Rga_Request.src.vir_h = src_height;
    Rga_Request.src.format = RK_FORMAT_YCbCr_420_SP;
    Rga_Request.src.act_w = cropW;
    Rga_Request.src.act_h = cropH;
    Rga_Request.src.x_offset = 0;
    Rga_Request.src.y_offset = 0;

    Rga_Request.dst.yrgb_addr = (int)dst;
    Rga_Request.dst.uv_addr  = 0;
    Rga_Request.dst.v_addr   = 0;
    Rga_Request.dst.vir_w = dstbuf_width;
    Rga_Request.dst.vir_h = dst_height;
    Rga_Request.dst.format = RK_FORMAT_RGB_565;
    Rga_Request.clip.xmin = 0;
    Rga_Request.clip.xmax = dst_width - 1;
    Rga_Request.clip.ymin = 0;
    Rga_Request.clip.ymax = dst_height - 1;
    Rga_Request.dst.act_w = dst_width;
    Rga_Request.dst.act_h = dst_height;
    Rga_Request.dst.x_offset = 0;
    Rga_Request.dst.y_offset = 0;
    Rga_Request.mmu_info.mmu_en    = 1;
    Rga_Request.mmu_info.mmu_flag  = ((2 & 0x3) << 4) | 1;
    Rga_Request.alpha_rop_flag |= (1 << 5);             /* ddl@rock-chips.com: v0.4.3 */

	if((src_width!=dst_width) || ( src_height!=dst_height)){
		Rga_Request.sina = 0;
		Rga_Request.cosa = 0x10000;
		Rga_Request.rotate_mode = 1;
		Rga_Request.scale_mode = 1;
	}else{
		Rga_Request.sina = 0;
		Rga_Request.cosa = 0;
		Rga_Request.rotate_mode = 0;
		Rga_Request.scale_mode = 0;
	}

    if(ioctl(rgafd, RGA_BLIT_SYNC, &Rga_Request) != 0) {
        LOGE("%s(%d):  RGA_BLIT_ASYNC Failed", __FUNCTION__, __LINE__);
        err = -1;
    }    
    return err;
#else
    LOGE("%s(%d): RGA is invalidate!",__FUNCTION__, __LINE__);
    return 0;
#endif
#else
    LOGE("%s(%d): rk29 havn't RGA device in chips!!",__FUNCTION__, __LINE__);
    return -1;
#endif
}


extern "C"  int arm_camera_yuv420_scale_arm(int v4l2_fmt_src, int v4l2_fmt_dst, 
									char *srcbuf, char *dstbuf,int src_w, int src_h,int dst_w, int dst_h,bool mirror)
{
	unsigned char *psY,*pdY,*psUV,*pdUV; 
	unsigned char *src,*dst;
	int srcW,srcH,cropW,cropH,dstW,dstH;
	long zoomindstxIntInv,zoomindstyIntInv;
	long x,y;
	long yCoeff00,yCoeff01,xCoeff00,xCoeff01;
	long sX,sY;
	long r0,r1,a,b,c,d;
	int ret = 0;
	bool nv21DstFmt = false;
	int ratio = 0;
	int top_offset=0,left_offset=0;
	if((v4l2_fmt_src != V4L2_PIX_FMT_NV12) ||
		((v4l2_fmt_dst != V4L2_PIX_FMT_NV12) && (v4l2_fmt_dst != V4L2_PIX_FMT_NV21) )){
		LOGE("%s:%d,not suppport this format ",__FUNCTION__,__LINE__);
		return -1;
	}

	if ((v4l2_fmt_dst == V4L2_PIX_FMT_NV21)){
		nv21DstFmt = true;
		
	}

	//need crop ?
	if((src_w*100/src_h) != (dst_w*100/dst_h)){
		ratio = ((src_w*100/dst_w) >= (src_h*100/dst_h))?(src_h*100/dst_h):(src_w*100/dst_w);
		cropW = ratio*dst_w/100;
		cropH = ratio*dst_h/100;
		
		left_offset=((src_w-cropW)>>1) & (~0x01);
		top_offset=((src_h-cropH)>>1) & (~0x01);
	}else{
		cropW = src_w;
		cropH = src_h;
		top_offset=0;
		left_offset=0;
	}

	src = psY = (unsigned char*)(srcbuf)+top_offset*src_w+left_offset;
	//psUV = psY +src_w*src_h+top_offset*src_w/2+left_offset;
	psUV = (unsigned char*)(srcbuf) +src_w*src_h+top_offset*src_w/2+left_offset;

	
	srcW =src_w;
	srcH = src_h;
//	cropW = src_w;
//	cropH = src_h;

	
	dst = pdY = (unsigned char*)dstbuf; 
	pdUV = pdY + dst_w*dst_h;
	dstW = dst_w;
	dstH = dst_h;

	zoomindstxIntInv = ((unsigned long)(cropW)<<16)/dstW + 1;
	zoomindstyIntInv = ((unsigned long)(cropH)<<16)/dstH + 1;
	//y
	//for(y = 0; y<dstH - 1 ; y++ ) {	
	for(y = 0; y<dstH; y++ ) {	 
		yCoeff00 = (y*zoomindstyIntInv)&0xffff;
		yCoeff01 = 0xffff - yCoeff00; 
		sY = (y*zoomindstyIntInv >> 16);
		sY = (sY >= srcH - 1)? (srcH - 2) : sY; 	 
		for(x = 0; x<dstW; x++ ) {
			xCoeff00 = (x*zoomindstxIntInv)&0xffff;
			xCoeff01 = 0xffff - xCoeff00;	
			sX = (x*zoomindstxIntInv >> 16);
			sX = (sX >= srcW -1)?(srcW- 2) : sX;
			a = psY[sY*srcW + sX];
			b = psY[sY*srcW + sX + 1];
			c = psY[(sY+1)*srcW + sX];
			d = psY[(sY+1)*srcW + sX + 1];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;
			
			if(mirror)
				pdY[dstW -1 - x] = r0;
			else
				pdY[x] = r0;
		}
		pdY += dstW;
	}

	dstW /= 2;
	dstH /= 2;
	srcW /= 2;
	srcH /= 2;

	//UV
	//for(y = 0; y<dstH - 1 ; y++ ) {
	for(y = 0; y<dstH; y++ ) {
		yCoeff00 = (y*zoomindstyIntInv)&0xffff;
		yCoeff01 = 0xffff - yCoeff00; 
		sY = (y*zoomindstyIntInv >> 16);
		sY = (sY >= srcH -1)? (srcH - 2) : sY;		
		for(x = 0; x<dstW; x++ ) {
			xCoeff00 = (x*zoomindstxIntInv)&0xffff;
			xCoeff01 = 0xffff - xCoeff00;	
			sX = (x*zoomindstxIntInv >> 16);
			sX = (sX >= srcW -1)?(srcW- 2) : sX;
			//U
			a = psUV[(sY*srcW + sX)*2];
			b = psUV[(sY*srcW + sX + 1)*2];
			c = psUV[((sY+1)*srcW + sX)*2];
			d = psUV[((sY+1)*srcW + sX + 1)*2];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;
		
			if(mirror && nv21DstFmt)
				pdUV[dstW*2-1- (x*2)] = r0;
			else if(mirror)
				pdUV[dstW*2-1-(x*2+1)] = r0;
			else if(nv21DstFmt)
				pdUV[x*2 + 1] = r0;
			else
				pdUV[x*2] = r0;
			//V
			a = psUV[(sY*srcW + sX)*2 + 1];
			b = psUV[(sY*srcW + sX + 1)*2 + 1];
			c = psUV[((sY+1)*srcW + sX)*2 + 1];
			d = psUV[((sY+1)*srcW + sX + 1)*2 + 1];

			r0 = (a * xCoeff01 + b * xCoeff00)>>16 ;
			r1 = (c * xCoeff01 + d * xCoeff00)>>16 ;
			r0 = (r0 * yCoeff01 + r1 * yCoeff00)>>16;

			if(mirror && nv21DstFmt)
				pdUV[dstW*2-1- (x*2+1) ] = r0;
			else if(mirror)
				pdUV[dstW*2-1-(x*2)] = r0;
			else if(nv21DstFmt)
				pdUV[x*2] = r0;
			else
				pdUV[x*2 + 1] = r0;
		}
		pdUV += dstW*2;
	}
	return 0;
}	

extern "C" int rk_camera_zoom_ipp(int v4l2_fmt_src, int srcbuf, int src_w, int src_h,int dstbuf,int zoom_value)
{
	int vipdata_base;

	struct rk29_ipp_req ipp_req;
	int src_y_offset,src_uv_offset,dst_y_offset,dst_uv_offset,src_y_size,dst_y_size;
	int scale_w_times =0,scale_h_times = 0,w,h;
	int ret = 0;
    int ippFd = -1;
    int ratio = 0;
    int top_offset=0,left_offset=0;
    int cropW,cropH;


    if((ippFd = open("/dev/rk29-ipp",O_RDWR)) < 0) {
    	LOGE("%s(%d):open rga device failed!!",__FUNCTION__,__LINE__);
        ret = -1;
    	goto do_ipp_err;
	}

    /*
    *ddl@rock-chips.com: 
    * IPP Dest image resolution is 2047x1088, so scale operation break up some times
    */
    if ((src_w > 0x7f0) || (src_h > 0x430)) {
        scale_w_times = ((src_w/0x7f0)>(src_h/0x430))?(src_w/0x7f0):(src_h/0x430); 
        scale_h_times = scale_w_times;
        scale_w_times++;
        scale_h_times++;
    } else {
        scale_w_times = 1;
        scale_h_times = 1;
    }
    memset(&ipp_req, 0, sizeof(struct rk29_ipp_req));


    //compute zoom 
	cropW = (src_w*100/zoom_value)& (~0x03);
	cropH = (src_h*100/zoom_value)& (~0x03);
	left_offset=MAX((((src_w-cropW)>>1)-1),0);
	top_offset=MAX((((src_h-cropH)>>1)-1),0);
    left_offset &= ~0x01; 
    top_offset &=~0x01;

    ipp_req.timeout = 3000;
    ipp_req.flag = IPP_ROT_0; 
    ipp_req.store_clip_mode =1;
    ipp_req.src0.w = cropW/scale_w_times;
    ipp_req.src0.h = cropH/scale_h_times;
    ipp_req.src_vir_w = src_w;
    ipp_req.src0.fmt = IPP_Y_CBCR_H2V2;
    ipp_req.dst0.w = src_w/scale_w_times;
    ipp_req.dst0.h = src_h/scale_h_times;
    ipp_req.dst_vir_w = src_w;   
    ipp_req.dst0.fmt = IPP_Y_CBCR_H2V2;
    vipdata_base = srcbuf;
    src_y_size = src_w*src_h;
    dst_y_size = src_w*src_h;

    for (h=0; h<scale_h_times; h++) {
        for (w=0; w<scale_w_times; w++) {
            int ipp_times = 3;
            src_y_offset = (top_offset + h*cropH/scale_h_times)* src_w 
                        + left_offset + w*cropW/scale_w_times;
		    src_uv_offset = (top_offset + h*cropH/scale_h_times)* src_w/2
                        + left_offset + w*cropW/scale_w_times;

            dst_y_offset = src_w*src_h*h/scale_h_times + src_w*w/scale_w_times;
            dst_uv_offset = src_w*src_h*h/scale_h_times/2 + src_w*w/scale_w_times;

    		ipp_req.src0.YrgbMst = vipdata_base + src_y_offset;
    		ipp_req.src0.CbrMst = vipdata_base + src_y_size + src_uv_offset;
    		ipp_req.dst0.YrgbMst = dstbuf + dst_y_offset;
    		ipp_req.dst0.CbrMst = dstbuf + dst_y_size + dst_uv_offset;
    		while(ipp_times-- > 0) {
                if (ioctl(ippFd,IPP_BLIT_SYNC,&ipp_req)){
                    LOGE("ipp do erro,do again,ipp_times = %d!\n",ipp_times);
                 } else {
                    break;
                 }
            }
            if (ipp_times <= 0) {
                ret = -1;
    			goto do_ipp_err;
    		}
        }
    }

do_ipp_err:
    if(ippFd > 0)
       close(ippFd);
    
	return ret;    
}

extern "C" int rk_camera_yuv_scale_crop_ipp(int v4l2_fmt_src, int v4l2_fmt_dst, 
			int srcbuf, int dstbuf,int src_w, int src_h,int dst_w, int dst_h,bool rotation_180)
{
	int vipdata_base;

	struct rk29_ipp_req ipp_req;
	int src_y_offset,src_uv_offset,dst_y_offset,dst_uv_offset,src_y_size,dst_y_size;
	int scale_w_times =0,scale_h_times = 0,w,h;
	int ret = 0;
    int ippFd = -1;
    int ratio = 0;
    int top_offset=0,left_offset=0;
    int cropW,cropH;


    if((ippFd = open("/dev/rk29-ipp",O_RDWR)) < 0) {
    	LOGE("%s(%d):open rga device failed!!",__FUNCTION__,__LINE__);
        ret = -1;
    	goto do_ipp_err;
	}


    /*
    *ddl@rock-chips.com: 
    * IPP Dest image resolution is 2047x1088, so scale operation break up some times
    */
    if ((dst_w > 0x7f0) || (dst_h > 0x430)) {
        scale_w_times = ((dst_w/0x7f0)>(dst_h/0x430))?(dst_w/0x7f0):(dst_h/0x430); 
        scale_h_times = scale_w_times;
        scale_w_times++;
        scale_h_times++;
    } else {
        scale_w_times = 1;
        scale_h_times = 1;
    }
    memset(&ipp_req, 0, sizeof(struct rk29_ipp_req));

    	//need crop ?
	if((src_w*100/src_h) != (dst_w*100/dst_h)){
		ratio = ((src_w*100/dst_w) >= (src_h*100/dst_h))?(src_h*100/dst_h):(src_w*100/dst_w);
		cropW = (ratio*dst_w/100)& (~0x03);
		cropH = (ratio*dst_h/100)& (~0x03);
		
		left_offset=MAX((((src_w-cropW)>>1)-1),0);
		top_offset=MAX((((src_h-cropH)>>1)-1),0);
        left_offset &= ~0x01; 
        top_offset &=~0x01;
	}else{
		cropW = src_w;
		cropH = src_h;
		top_offset=0;
		left_offset=0;
	}
#if 1
    if((src_w == 2592) && (src_h == 1944) && (dst_w == 2592) && (dst_h == 1458)){
        scale_w_times= 2;
        scale_h_times = 3;
		cropW = dst_w;
		cropH = dst_h;
		
		left_offset=0;
		top_offset=242;
    }
#endif
    ipp_req.timeout = 3000;
    if(rotation_180)
        ipp_req.flag = IPP_ROT_180; 
    else
        ipp_req.flag = IPP_ROT_0; 
    ipp_req.store_clip_mode =1;
    ipp_req.src0.w = cropW/scale_w_times;
    ipp_req.src0.h = cropH/scale_h_times;
    ipp_req.src_vir_w = src_w;
    if(v4l2_fmt_src == V4L2_PIX_FMT_NV12)
        ipp_req.src0.fmt = IPP_Y_CBCR_H2V2;
    else if(v4l2_fmt_src == V4L2_PIX_FMT_NV21)
        ipp_req.src0.fmt = IPP_Y_CBCR_H2V1;
    ipp_req.dst0.w = dst_w/scale_w_times;
    ipp_req.dst0.h = dst_h/scale_h_times;
    ipp_req.dst_vir_w = dst_w;   
    if(v4l2_fmt_dst == V4L2_PIX_FMT_NV12)
        ipp_req.dst0.fmt = IPP_Y_CBCR_H2V2;
    else if(v4l2_fmt_dst == V4L2_PIX_FMT_NV21)
        ipp_req.dst0.fmt = IPP_Y_CBCR_H2V1;
    vipdata_base = srcbuf;
    src_y_size = src_w*src_h;
    dst_y_size = dst_w*dst_h;
    for (h=0; h<scale_h_times; h++) {
        for (w=0; w<scale_w_times; w++) {
            int ipp_times = 3;
            src_y_offset = (top_offset + h*cropH/scale_h_times)* src_w 
                        + left_offset + w*cropW/scale_w_times;
		    src_uv_offset = (top_offset + h*cropH/scale_h_times)* src_w/2
                        + left_offset + w*cropW/scale_w_times;

            if(rotation_180){
                dst_y_offset = dst_w*dst_h*(scale_h_times-1-h)/scale_h_times + dst_w*(scale_w_times-1-w)/scale_w_times;
                dst_uv_offset = dst_w*dst_h*(scale_h_times-1-h)/scale_h_times/2 + dst_w*(scale_w_times-1-w)/scale_w_times;
            }
            else{
                dst_y_offset = dst_w*dst_h*h/scale_h_times + dst_w*w/scale_w_times;
                dst_uv_offset = dst_w*dst_h*h/scale_h_times/2 + dst_w*w/scale_w_times;
            }

    		ipp_req.src0.YrgbMst = vipdata_base + src_y_offset;
    		ipp_req.src0.CbrMst = vipdata_base + src_y_size + src_uv_offset;
    		ipp_req.dst0.YrgbMst = dstbuf + dst_y_offset;
    		ipp_req.dst0.CbrMst = dstbuf + dst_y_size + dst_uv_offset;
    		while(ipp_times-- > 0) {
                if (ioctl(ippFd,IPP_BLIT_SYNC,&ipp_req)){
                    LOGE("ipp do erro,do again,ipp_times = %d!\n",ipp_times);
                 } else {
                    break;
                 }
            }
            if (ipp_times <= 0) {
                ret = -1;
    			goto do_ipp_err;
    		}
        }
    }

do_ipp_err:
    if(ippFd > 0)
       close(ippFd);
	return ret;    
}

extern "C"  int YData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w)
{
    int i;

    for (i=0; i<(w>>2); i++) {
        *pdst = ((*psrc>>24)&0x000000ff) | ((*psrc>>8)&0x0000ff00)
                | ((*psrc<<8)&0x00ff0000) | ((*psrc<<24)&0xff000000);
        psrc++;
        pdst--;
    }

    return 0;
}
extern "C"  int UVData_Mirror_Line(int v4l2_fmt_src, int *psrc, int *pdst, int w)
{
    int i;

    for (i=0; i<(w>>2); i++) {
        *pdst = ((*psrc>>16)&0x0000ffff) | ((*psrc<<16)&0xffff0000);                
        psrc++;
        pdst--;
    }

    return 0;
}
extern "C"  int YuvData_Mirror_Flip(int v4l2_fmt_src, char *pdata, char *pline_tmp, int w, int h)
{
    int *pdata_tmp = NULL;
    int *ptop, *pbottom;
    int err = 0,i,j;

    pdata_tmp = (int*)pline_tmp;
    
    // Y mirror and flip
    ptop = (int*)pdata;
    pbottom = (int*)(pdata+w*(h-1));    
    for (j=0; j<(h>>1); j++) {
        YData_Mirror_Line(v4l2_fmt_src, ptop, pdata_tmp+((w>>2)-1),w);
        YData_Mirror_Line(v4l2_fmt_src, pbottom, ptop+((w>>2)-1), w);
        memcpy(pbottom, pdata_tmp, w);
        ptop += (w>>2);
        pbottom -= (w>>2);
    }
    // UV mirror and flip
    ptop = (int*)(pdata+w*h);
    pbottom = (int*)(pdata+w*(h*3/2-1));    
    for (j=0; j<(h>>2); j++) {
        UVData_Mirror_Line(v4l2_fmt_src, ptop, pdata_tmp+((w>>2)-1),w);
        UVData_Mirror_Line(v4l2_fmt_src, pbottom, ptop+((w>>2)-1), w);
        memcpy(pbottom, pdata_tmp, w);
        ptop += (w>>2);
        pbottom -= (w>>2);
    }
YuvData_Mirror_Flip_end:
    return err;
}
extern "C" int YUV420_rotate(const unsigned char* srcy, int src_stride,  unsigned char* srcuv,
                   unsigned char* dsty, int dst_stride, unsigned char* dstuv,
                   int width, int height,int rotate_angle){
   int i = 0,j = 0;
	// 90 , y plane
  if(rotate_angle == 90){
      srcy += src_stride * (height - 1);
  	  srcuv += src_stride * ((height >> 1)- 1); 
  	  src_stride = -src_stride;
	}else if(rotate_angle == 270){
      dsty += dst_stride * (width - 1);
      dstuv += dst_stride * ((width>>1) - 1);
	  dst_stride = -dst_stride;
  }

  for (i = 0; i < width; ++i)
    for (j = 0; j < height; ++j)
      *(dsty+i * dst_stride + j) = *(srcy+j * src_stride + i); 
  
  //uv 
  unsigned char av_u0,av_v0;
  for (i = 0; i < width; i += 2)
    for (j = 0; j < (height>>1); ++j) {
		av_u0 = *(srcuv+i + (j * src_stride));
		av_v0 = *(srcuv+i + (j * src_stride)+1);
      *(dstuv+((j<<1) + ((i >> 1) * dst_stride)))= av_u0;
      *(dstuv+((j<<1) + ((i >> 1) * dst_stride)+1)) = av_v0;
	}
   
  return 0;
 }
