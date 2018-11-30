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
#ifndef ANDROID_HARDWARE_CAMERA_ISP_HARDWARE_H
#define ANDROID_HARDWARE_CAMERA_ISP_HARDWARE_H

//usb camera adapter
#include "CameraHal.h"
#include "cam_api/camdevice.h"
#include "cam_api/halholder.h"
#include "oslayer/oslayer.h"
#include <string>
#include <utils/KeyedVector.h>
#include "CameraGLAdapter.h"
#define LOCK3A_TIMEOUT_US		2000 * 1000//3a lock time
#define AECCNT					6// > 2*max(ExpCnt,MeanCnt,AwbCnt) fps

namespace android{
typedef enum
{
    FLASH_STATE_IDLE_STATUS,
    FLASH_STATE_WAIT_3ALOCK,
    FLASH_STATE_LOCK_TIMEOUT,
    FLASH_STATE_FLASH_TORCH
} Flash_State_t;

typedef struct unflashPara_s
{
	float settime;
	float newtime;
	float mintime;
	float maxtime;
	float timestep;

	float setgain;
	float newgain;
	float mingain;
	float maxgain;
	float gainstep;
}unflashPara_t;

typedef struct flashStatus_s
{
	bool                isflash;
	Flash_State_t       old_state;
	Flash_State_t       current_state;
	unsigned int        skip_frames;
	unsigned long long  flash_time;
	bool                capture_ready;
	unflashPara_t       unflash;
}flashStatus_t;


typedef struct awbStatus{
    bool enabled;
    CamEngineAwbMode_t mode;
    uint32_t idx;
    CamEngineAwbRgProj_t RgProj;
    bool  damping;
    bool  manual_mode;
}awbStatus_s;
typedef struct manExpConfig{
	float minus_level_3;
	float minus_level_2;
	float minus_level_1;
	float level_0;
	float plus_level_1;
	float plus_level_2;
	float plus_level_3;
	float clmtolerance;
}manExpConfig_s;

typedef struct uvnrprocess{
	bool enable;
}uvnrprocess_s;

typedef struct mfdprocess{
	bool enable;
	bool buffer_full;
	int	 process_frames;//3<= mfd_frames <= 6
	int  frame_cnt;
}mfdprocess_s;

typedef struct CaptureOption_s{
    bool enable;
	uint8_t capture_id;
	//char path[64];
	uint8_t format;
	uint8_t number;
	uint16_t width;
	uint16_t height;
	uint8_t ae_mode;
	float integrationTime;
	float gain;
}CaptureOption_t;


class CameraIspAdapter;

typedef void (*uvc_setDevice)(CameraIspAdapter *adp, CamDevice *dev);
typedef int (*vpu_encode_jpeg_init)(int width,int height,int quant);
typedef void (*uvc_set_run_state)(bool state);
typedef void (*vpu_encode_jpeg_done)();
typedef bool (*uvc_get_run_state)();
typedef unsigned int (*uvc_get_fcc)();
typedef void (*uvc_get_resolution)(int* width, int* height);
typedef void (*uvc_buffer_write)(void* extra_data,
                      size_t extra_size,
                      void* data,
                      size_t size,
                      unsigned int fcc);
typedef	int (*vpu_encode_jpeg_doing)(
								   void* srcbuf,
								   int src_fd,
								   size_t src_size);
typedef void (*vpu_encode_jpeg_set_encbuf)(int fd, void *viraddr, unsigned long phyaddr, unsigned int size);
typedef void (*vpu_encode_jpeg_get_encbuf)(unsigned char** jpeg_out, unsigned int *jpeg_len);
typedef bool (*uvc_buffer_write_enable)();
typedef bool (*uvc_gadget_main)();

typedef struct uvc_cam_ops_s{
	vpu_encode_jpeg_init encode_init;
	vpu_encode_jpeg_doing encode_process;
	vpu_encode_jpeg_done encode_deinit;
	vpu_encode_jpeg_set_encbuf encode_set_buf;
	vpu_encode_jpeg_get_encbuf encode_get_buf;
	uvc_setDevice set_device;
	uvc_set_run_state set_state;
	uvc_get_run_state get_state;
	uvc_get_fcc get_fcc;
	uvc_get_resolution get_res;
	uvc_buffer_write transfer_data;
	uvc_buffer_write_enable transfer_data_enable;
	uvc_gadget_main uvc_thread_func;
}uvc_cam_ops_t;

extern void* mLibUvcApp;
extern uvc_cam_ops_t uvc_cam_ops;

class CameraIspTunning;
class CameraIspAdapter: public CameraAdapter,public BufferCb
{
public:
	static int preview_frame_inval;
    static int DEFAULTPREVIEWWIDTH;
    static int DEFAULTPREVIEWHEIGHT;
    CameraIspAdapter(int cameraId);
    virtual ~CameraIspAdapter();
    virtual status_t startPreview(int preview_w,int preview_h,int w, int h, int fmt,bool is_capture);
    status_t startPreviewEx(int preview_w,int preview_h,int w, int h, int fmt, CamEngineBestSensorResReq_t resReq);
    virtual status_t stopPreview();
    virtual int setParameters(const CameraParameters &params_set,bool &isRestartValue);
    virtual void initDefaultParameters(int camFd);
    virtual status_t autoFocus();
    virtual status_t cancelAutoFocus();
    virtual int getCurPreviewState(int *drv_w,int *drv_h);
    virtual int selectPreferedDrvSize(int *width,int * height,bool is_capture);
    void AfpsResChangeCb();
    virtual void bufferCb( MediaBuffer_t* pMediaBuffer );

    virtual void setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h,int zoom_value);

	virtual void dump(int cameraId);
    virtual void getCameraParamInfo(cameraparam_info_s &paraminfo);
	virtual bool getFlashStatus();
	virtual void getSensorMaxRes(unsigned int &max_w, unsigned int &max_h);
	virtual int faceNotify(struct RectFace* faces, int* num);
    void captureFrame();
	bool tuningThreadIsRunning();
    CaptureOption_t capOption;
    bool mISPTunningRun;
    MessageQueue mISPUvcQ;

private:
    //talk to driver
    virtual int cameraCreate(int cameraId);
    virtual int cameraDestroy();
    virtual int adapterReturnFrame(long index,int cmd);


    //for isp
    void setScenarioMode(CamEngineModeType_t newScenarioMode);

    void setSensorItf(int newSensorItf);
    void enableAfps( bool enable = false );

    void loadSensor(int cameraId =-1 );
    void loadCalibData(const char* fileName = NULL);
    void openImage( const char* fileName = NULL);

    bool connectCamera();
    void disconnectCamera();

    int start();
    int pause();
    int stop();

    int afListenerThread(void);
    int cameraConfig(const CameraParameters &tmpparams,bool isInit,bool &isRestartValue);
    bool isLowIllumin(const float lumaThreshold);
    void paraReConfig(void);
    void flashProcess(void);
    void flashCtrl(void);
    void flashSettle(CamEngineFlashMode_t mode);
    bool isNeedToEnableFlash();
	void setMwb(const char *white_balance);
	void setMwb_Temp(uint32_t colortemperature);
	void setMe(const char *exposure);

	uvnrprocess uvnr;
	mfdprocess mfd;

protected:
    CamDevice       *m_camDevice;
    HalHolder     *m_halHolder;
    KeyedVector<void *, void *> mFrameInfoArray;
    Mutex  mFrameArrayLock;     
    void clearFrameArray();
	mutable Mutex mLock;

    std::string mSensorDriverFile[3];
    int mSensorItfCur;
    bool mFlashStatus;
	bool mTorchStatus;
	CtxCbResChange_t mCtxCbResChange;
    bool mAfChk;
    class CameraAfThread :public Thread
    {
        //deque 到帧后根据需要分发给DisplayAdapter类及EventNotifier类。
        CameraIspAdapter* mCameraAdapter;
    public:
        CameraAfThread(CameraIspAdapter* adapter)
            : Thread(false), mCameraAdapter(adapter) { }

        virtual bool threadLoop() {
            mCameraAdapter->afListenerThread();

            return false;
        }
    };
    
    CamEngineAfEvtQue_t  mAfListenerQue; 
    sp<CameraAfThread>   mAfListenerThread;

    enum ISP_TUNNING_THREAD_CMD_e{
       ISP_TUNNING_CMD_START,
       ISP_TUNNING_CMD_EXIT,
       ISP_TUNNING_CMD_PROCESS_FRAME
    };

    class CamISPTunningThread :public Thread
    {
        //deque 到帧后根据需要分发给DisplayAdapter类及EventNotifier类。
        CameraIspAdapter* mCameraAdapter;
    public:
        CamISPTunningThread(CameraIspAdapter* adapter)
            : Thread(false), mCameraAdapter(adapter) { }

        virtual bool threadLoop() {
            mCameraAdapter->ispTunningThread();

            return false;
        }
    };

    int ispTunningThread(void);
    MessageQueue* mISPTunningQ;
    sp<CamISPTunningThread>   mISPTunningThread;
    int mISPOutputFmt;

    bool mIsSendToTunningTh;    
    enum ISP_UVC_THREAD_CMD_e{
       ISP_UVC_CMD_START,
       ISP_UVC_CMD_EXIT,
       ISP_UVC_CMD_CAPTURE,
       ISP_UVC_CMD_REBOOT,
       ISP_UVC_CMD_PROCESS_FRAME
    };
    class CamISPUvcThread :public Thread
    {
        CameraIspAdapter* mCameraAdapter;
    public:
        CamISPUvcThread(CameraIspAdapter* adapter)
            : Thread(false), mCameraAdapter(adapter) { }

        virtual bool threadLoop() {
            mCameraAdapter->ispUvcThread();

            return false;
        }
    };
	int ispUvcThread(void);

    //BufferProvider* mUvcBuf;
    sp<CamISPUvcThread>   mISPUvcThread;
    class CamISPUvcProcessThread :public Thread
    {
        CameraIspAdapter* mCameraAdapter;
    public:
        CamISPUvcProcessThread(CameraIspAdapter* adapter)
            : Thread(false), mCameraAdapter(adapter) { }

        virtual bool threadLoop() {
        	if(uvc_cam_ops.uvc_thread_func != NULL)
            	uvc_cam_ops.uvc_thread_func();

            return false;
        }
    };
	sp<CamISPUvcProcessThread>   mISPUvcProcessThread;
	bool mIsSendToUvcTh;
	bool mUvcThreaRunning;
    int mDispFrameLeak;
    int mVideoEncFrameLeak;
    int mPreviewCBFrameLeak;
    int mPicEncFrameLeak;
private:
    flashStatus_t curFlashStatus;
    awbStatus curAwbStatus;
    CameraIspTunning* mIspTunningTask;
    manExpConfig_s manExpConfig;
	UVNRAdapter* mUVNRAdapter;
    MFNRAdapter* mMFNRAdapter;
	bool mUVNRAvailable, mMFNRAvailable;
    
};

class CameraIspSOCAdapter: public CameraIspAdapter
{
public:

    CameraIspSOCAdapter(int cameraId);
    virtual ~CameraIspSOCAdapter();
#if 0
    virtual int setParameters(const CameraParameters &params_set);
    virtual void initDefaultParameters()
    {
        mParameters.set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES, "10,15,30");  

    };
   virtual status_t autoFocus();
#endif
    virtual void setupPreview(int width_sensor,int height_sensor,int preview_w,int preview_h,int zoom_value);
    virtual void bufferCb( MediaBuffer_t* pMediaBuffer );

private:
    bool    mIs10bit0To0;

};



}
#endif
