#include "CameraIspAdapter.h"
#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "CameraGLAdapter"
#endif

#ifdef TARGET_RK3368
#include <hardware/img_gralloc_public.h>
typedef IMG_native_handle_t GRAPHIC_BUFFER_NATIVE_HANDLE;
#else
#ifdef RK_DRM_GRALLOC
#include <gralloc_drm_priv.h>
#include <gralloc_buffer_priv.h>
#include <cutils/ashmem.h>
#include <sys/mman.h>
typedef gralloc_drm_handle_t GRAPHIC_BUFFER_NATIVE_HANDLE;
#else
#include <gralloc_priv.h>
typedef private_handle_t GRAPHIC_BUFFER_NATIVE_HANDLE;
#endif
#endif

#if (defined(__arm64__) || defined(__aarch64__))
#define UVNR_LIB_PATH "/system/lib64/libuvnr.so"
#define MFNR_LIB_PATH "/system/lib64/libmfnr.so"
#else
#define UVNR_LIB_PATH "/system/lib/libuvnr.so"
#define MFNR_LIB_PATH "/system/lib/libmfnr.so"
#endif


const char*
UVNR_FUNC_NAMES[] = {
    "UVNR_init",
    "UVNR_setCallback",
    "UVNR_update",
    "UVNR_process",
    "UVNR_sync",
    "UVNR_deinit"};

enum UVNR_FUNC_INDEX {
    UVNR_init,
    UVNR_setCallback,
    UVNR_update,
    UVNR_process,
    UVNR_sync,
    UVNR_deinit
};

const char*
MFNR_FUNC_NAMES[] = {
    "MFNR_init",
    "MFNR_setCallback",
    "MFNR_setFrames",
    "MFNR_update",
    "MFNR_process",
    "MFNR_sync",
    "MFNR_deinit"};

enum MFNR_FUNC_INDEX {
    MFNR_init,
    MFNR_setCallback,
    MFNR_setFrames,
    MFNR_update,
    MFNR_process,
    MFNR_sync,
    MFNR_deinit
};

bool checkLinkStatus(void* handle, const char* func) {
    if (handle == NULL) {
        ALOGE("dlsym %s not found", func);
        const char *errmsg;
        if ((errmsg = dlerror()) != NULL) {
            ALOGE("dlsym %s fail errmsg: %s", func, errmsg);
        }
        return false;
    }
    
    ALOGD("dlsym %s success", func);
    return true;
}

buffer_handle_t bufferHandleAlloc(native_handle_bundle *bundle) {
	int bpp = 0;
	int iPlanes = 0;
	int size = 0;
	int byte_stride = 0;
	int pixel_stride = 0;

	if (bundle->format == HAL_PIXEL_FORMAT_RGBA_8888) {
		bpp = 4;
		iPlanes = 1;
		size = bundle->w * bundle->h * 4;
		byte_stride = bundle->w * 4;
		pixel_stride = byte_stride / 4;
	} else if (bundle->format == HAL_PIXEL_FORMAT_YCrCb_NV12) {
		bpp = 1;
		iPlanes = 2;
		size = bundle->w * bundle->h * 3 / 2;
		byte_stride = bundle->w;
		pixel_stride = byte_stride;
	}

#ifdef TARGET_RK3368
    int i = 0;
    int iNumInvalidFds = MAX_SUB_ALLOCS - 1;

    GRAPHIC_BUFFER_NATIVE_HANDLE *psNativeHandle = (GRAPHIC_BUFFER_NATIVE_HANDLE *)
    native_handle_create(IMG_NATIVE_HANDLE_NUMFDS  - iNumInvalidFds,
						 IMG_NATIVE_HANDLE_NUMINTS + iNumInvalidFds);

    if (!psNativeHandle) {
    	ALOGD("%s: Failed to allocate buffer handle", __func__);
    	return NULL;
    }

	psNativeHandle->iNumSubAllocs = IMG_NATIVE_HANDLE_NUMFDS - iNumInvalidFds;
	psNativeHandle->ui64Stamp = bundle->ui64Stamp;

	for (i = 0; i < psNativeHandle->iNumSubAllocs; i++)
		psNativeHandle->fd[i] = bundle->shareFd;
	for (; i < MAX_SUB_ALLOCS; i++)
		psNativeHandle->fd[i] = -1;

    psNativeHandle->usage = bundle->usage;//GRALLOC_USAGE_HW_TEXTURE;
    psNativeHandle->width = psNativeHandle->iWidth	= bundle->w;
    psNativeHandle->height = psNativeHandle->iHeight = bundle->h;
    psNativeHandle->format = psNativeHandle->iFormat = bundle->format;
    psNativeHandle->stride = pixel_stride;
    psNativeHandle->uiBpp = bpp;
    psNativeHandle->iPlanes = iPlanes;

    psNativeHandle->type = 1;
    psNativeHandle->share_fd = psNativeHandle->fd[0];
    psNativeHandle->offset = 0;
    psNativeHandle->size = size;
    psNativeHandle->pvBase = NULL;

    for (i = 0; i < iPlanes; ++i)
   	{
   	    psNativeHandle->aiStride[i] = pixel_stride;
   	   	psNativeHandle->aiVStride[i] = bundle->h;
   	   	psNativeHandle->aulPlaneOffset[i] = 0;
   	}
   	for (i = iPlanes; i < MAX_SUB_ALLOCS; ++i)
   	{
   	    psNativeHandle->aiStride[i] = 0;
   	   	psNativeHandle->aiVStride[i] = 0;
   	   	psNativeHandle->aulPlaneOffset[i] = 0;
   	}

    ALOGD("--------native handle:");
    ALOGD("        handle base info: version: %d, numInts: %d, numFds: %d",
        psNativeHandle->base.version, psNativeHandle->base.numInts, psNativeHandle->base.numFds);
    ALOGD("        handle pvrUI64Stamp: %ld usage: 0x%.8x", psNativeHandle->ui64Stamp, psNativeHandle->usage);
    return &psNativeHandle->base;
#else
#ifdef RK_DRM_GRALLOC
		GRAPHIC_BUFFER_NATIVE_HANDLE *hnd = new GRAPHIC_BUFFER_NATIVE_HANDLE;
		if (!hnd)
			return NULL;
		memset(hnd, 0, sizeof(GRAPHIC_BUFFER_NATIVE_HANDLE));
	
		hnd->prime_fd = (bundle->shareFd);
	
		hnd->base.version = sizeof(hnd->base);
		hnd->base.numInts = GRALLOC_DRM_HANDLE_NUM_INTS;
		hnd->base.numFds = GRALLOC_DRM_HANDLE_NUM_FDS;
	
		hnd->magic = GRALLOC_DRM_HANDLE_MAGIC;
		hnd->width = bundle->w;
		hnd->height = bundle->h;
		hnd->offset = 0;
		hnd->stride = bundle->w;
		hnd->byte_stride = bundle->w*4;
		hnd->format = bundle->format;
		hnd->usage = bundle->usage;

		ALOGD("--------native handle:");
        ALOGD("        handle base info: version: %d, numInts: %d, numFds: %d, magic: %x",
            hnd->base.version, hnd->base.numInts, hnd->base.numFds, hnd->magic);
        ALOGD("        fd=%d, yuv=%d",
            hnd->prime_fd,hnd->yuv_info);
		ALOGD("        ref=%d, owner=%d, YUV=%d",
			hnd->ref, hnd->data_owner, MALI_YUV_BT709_WIDE);
	
		return &hnd->base;
#else

	GRAPHIC_BUFFER_NATIVE_HANDLE *hnd = new GRAPHIC_BUFFER_NATIVE_HANDLE( private_handle_t::PRIV_FLAGS_USES_ION, usage, w*h*4, mBufferAddr,
													  private_handle_t::LOCK_STATE_MAPPED );
	hnd->share_fd = shareFd;
	hnd->ion_hnd = ionHandle;
	hnd->type = 1;

    int private_usage = usage & (GRALLOC_USAGE_PRIVATE_0 |
                                 GRALLOC_USAGE_PRIVATE_1);
    switch (private_usage)
    {
        case 0:
            hnd->yuv_info = MALI_YUV_BT601_NARROW;
            break;
        case GRALLOC_USAGE_PRIVATE_1:
            hnd->yuv_info = MALI_YUV_BT601_WIDE;
            break;
        case GRALLOC_USAGE_PRIVATE_0:
            hnd->yuv_info = MALI_YUV_BT709_NARROW;
            break;
        case (GRALLOC_USAGE_PRIVATE_0 | GRALLOC_USAGE_PRIVATE_1):
            hnd->yuv_info = MALI_YUV_BT709_WIDE;
            break;
    }

    hnd->req_format = format;
    hnd->byte_stride = byte_stride;
    hnd->internal_format = format;
    hnd->video_width = 0;
    hnd->video_height = 0;
    hnd->format = format;
    hnd->width = w;
    hnd->height = h;
    hnd->stride = pixel_stride;

	return hnd;
#endif
#endif
}

sp<GraphicBuffer> allocateGraphicBuffer(native_handle_bundle *bundle) {
    sp<GraphicBuffer> graphicBuffer;
#ifdef ANDROID_7_X
    if (bundle->nativeHandle != NULL) {
        graphicBuffer = new GraphicBuffer(bundle->w, bundle->h, bundle->format,
                    bundle->usage, bundle->w, (native_handle_t*)bundle->nativeHandle, false);
    } else {
        if (bundle->shareFd == -1) {
            graphicBuffer = new GraphicBuffer(bundle->w, bundle->h, bundle->format,
                        bundle->usage);
        } else {
            buffer_handle_t handle = bufferHandleAlloc(bundle);
            graphicBuffer = new GraphicBuffer(bundle->w, bundle->h, bundle->format,
                bundle->usage, bundle->w, (native_handle_t*)handle, false);
        }
    }
#endif
    return graphicBuffer;
}

UVNRAdapter::UVNRAdapter():
	mCheckInitialized(false),	
	mCheckAvailable(false),
    mISO(2),
    mUVNREngine(NULL),
    mGPUCommandThreadState(STA_GPUCMD_IDLE),
    m_buffers_capture(NULL)
{
    memset(&mUVNRFunc, 0, sizeof(struct UVNRFunc));
}

UVNRAdapter::~UVNRAdapter() {
    if (mGPUCommandThread != NULL) {
           sendBlockedMsg(CMD_GPU_PROCESS_DEINIT);
        mGPUCommandThread->requestExitAndWait();
        mGPUCommandThread.clear();
        mGPUCommandThread = NULL;
    }

    if (m_buffers_capture != NULL){
        delete m_buffers_capture;
        m_buffers_capture = NULL;
    }
}

void UVNRAdapter::setCameraDevice(CamDevice *camDevice) {
    mCamDevice = camDevice;
}

bool UVNRAdapter::initEnv() {
	mUVNRFunc.mUVNRHandle = dlopen(UVNR_LIB_PATH, RTLD_NOW);
    if (mUVNRFunc.mUVNRHandle == NULL) {
        ALOGE("%s(%d): open libuvnr.so fail",__FUNCTION__,__LINE__);
        const char *errmsg;
        if ((errmsg = dlerror()) != NULL) {
            ALOGE("dlopen libuvnr.so fail errmsg: %s", errmsg);
        }
        return false;
    } else {
        ALOGD("%s(%d): open libuvnr.so success",__FUNCTION__,__LINE__);
        mUVNRFunc.mUVNRInitFunc =
            (UVNR_init_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_init]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRInitFunc, UVNR_FUNC_NAMES[UVNR_init]);
        mUVNRFunc.mUVNRCallbackFunc =
            (UVNR_callback_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_setCallback]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRCallbackFunc, UVNR_FUNC_NAMES[UVNR_setCallback]);
        mUVNRFunc.mUVNRUpdateFunc =
            (UVNR_update_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_update]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRUpdateFunc, UVNR_FUNC_NAMES[UVNR_update]);
        mUVNRFunc.mUVNRProcFunc =
            (UVNR_proc_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_process]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRProcFunc, UVNR_FUNC_NAMES[UVNR_process]);
        mUVNRFunc.mUVNRSyncFunc =
            (UVNR_sync_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_sync]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRSyncFunc, UVNR_FUNC_NAMES[UVNR_sync]);
        mUVNRFunc.mUVNRDeinitFunc =
            (UVNR_deinit_func)dlsym(mUVNRFunc.mUVNRHandle, UVNR_FUNC_NAMES[UVNR_deinit]);
        checkLinkStatus((void*)mUVNRFunc.mUVNRDeinitFunc, UVNR_FUNC_NAMES[UVNR_deinit]);
		if (mUVNRFunc.mUVNRInitFunc == NULL
                || mUVNRFunc.mUVNRCallbackFunc == NULL
				|| mUVNRFunc.mUVNRUpdateFunc == NULL				
				|| mUVNRFunc.mUVNRProcFunc == NULL
				|| mUVNRFunc.mUVNRSyncFunc == NULL
				|| mUVNRFunc.mUVNRDeinitFunc == NULL) {
			ALOGD("dlsym UVNR funcs failed!");
			return false;
		}
    }

	m_buffers_capture = new cv_fimc_buffer();
	memset(m_buffers_capture,0x0,sizeof(cv_fimc_buffer));
	mGPUCommandThread = new GPUCommandThread(this);
	mGPUCommandThreadState = STA_GPUCMD_IDLE;
	mGPUCommandThread->run("GPUCommandThread",ANDROID_PRIORITY_DISPLAY);
    ALOGD("init environment success!");

	return true;
}

bool UVNRAdapter::checkAvailable() {
	return mCheckAvailable;
}

void UVNRAdapter::setDimension(int width, int height) {
	mGpuFBOWidth = width;
	mGpuFBOHeight = height;
}

void UVNRAdapter::wrapData(void* graphic_buffer, void* start, int share_fd, int length, buffer_handle_t handle) {
	m_buffers_capture->start = (void *)start;
	m_buffers_capture->share_fd = share_fd;
	m_buffers_capture->length = length;
	m_buffers_capture->handle = handle;

#ifdef TARGET_RK3368
    if (graphic_buffer != NULL) {
       ANativeWindowBuffer* nativeWindowBuffer = (ANativeWindowBuffer* )((GraphicBuffer*)graphic_buffer)->getNativeBuffer();
       GRAPHIC_BUFFER_NATIVE_HANDLE *psNativeHandle = (GRAPHIC_BUFFER_NATIVE_HANDLE*)nativeWindowBuffer->handle;
       m_buffers_capture->ui64Stamp = psNativeHandle->ui64Stamp;
    }
#endif
}

void UVNRAdapter::gpuCommandThread()
{
    Message_cam msg;
    float uvnr_set_ratio;
    float uvnr_set_distances;
    char  uvnr_set_enable;
    float uvnr_ISO[3];
    float uvnr_ratio[3];
    float uvnr_distances[3];

    while (mGPUCommandThreadState != STA_GPUCMD_STOP) {
    gpu_receive_cmd:
        if (gpuCmdThreadCommandQ.isEmpty() == false ) {
            gpuCmdThreadCommandQ.get(&msg);

            //ALOGD("isp-msg,command thread receive message: %d", msg.command);
            switch (msg.command)
            {
                case CMD_GPU_PROCESS_INIT:
                {
                    //ALOGD("check init, w-h: %d-%d, tid: %d", mGpuFBOWidth, mGpuFBOHeight, gettid());
                    if( mUVNRFunc.mUVNRInitFunc(mUVNREngine, mGpuFBOWidth, mGpuFBOHeight, 8) < 0) {
                        mCheckInitialized = false;
                    } else {
                        mCheckInitialized = true;
                    }
                    mUVNRFunc.mUVNRCallbackFunc(mUVNREngine, allocateGraphicBuffer);

                    mGPUCommandThreadState = STA_GPUCMD_RUNNING;
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();
                    break;
                }
                case CMD_GPU_PROCESS_UPDATE:
                {
                    mUVNRFunc.mUVNRUpdateFunc(mUVNREngine, m_buffers_capture);
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    break;
                }
                case CMD_GPU_PROCESS_RENDER:
                {
                    if (mCamDevice != NULL) {
    					mCamDevice->getUvnrPara(&uvnr_set_enable, uvnr_ISO, uvnr_ratio,uvnr_distances);
                        mCamDevice->getGain(mISO);
                    }

					if(uvnr_set_enable == true) {
						if(mISO > uvnr_ISO[2]) {
							uvnr_set_ratio = uvnr_ratio[2];
							uvnr_set_distances = uvnr_distances[2];
						}
						else if(mISO > uvnr_ISO[1]){
							uvnr_set_ratio = uvnr_ratio[1];
							uvnr_set_distances = uvnr_distances[1];
						}
						else {
							uvnr_set_ratio = uvnr_ratio[0];
							uvnr_set_distances = uvnr_distances[0];
						}
					} else {
						uvnr_set_ratio = 15;
						uvnr_set_distances = 5;
					}


                    if (mUVNREngine == NULL) {
                        ALOGD("uvnr CMD_GPU_PROCESS_RENDER engine is null");
                    }

                    mUVNRFunc.mUVNRProcFunc(mUVNREngine, NULL, OUTPUT_NONE, uvnr_set_ratio, uvnr_set_distances);

                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    break;
                }
                case CMD_GPU_PROCESS_GETRESULT:
                {
					mUVNRFunc.mUVNRSyncFunc(mUVNREngine, (long)m_buffers_capture->start);
					if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();
                    break;
                }
                case CMD_GPU_PROCESS_DEINIT:
                {
                    mUVNRFunc.mUVNRDeinitFunc(mUVNREngine);
					mCheckInitialized = false;

                    mGPUCommandThreadState = STA_GPUCMD_STOP;

                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    continue;
                }
            }
        }

        mGpuOPLock.lock();
        if (gpuCmdThreadCommandQ.isEmpty() == false ) {
            mGpuOPLock.unlock();
            goto gpu_receive_cmd;
        }

        mGpuOPCond.wait(mGpuOPLock);
        mGpuOPLock.unlock();

        goto gpu_receive_cmd;
    }
}

void UVNRAdapter::sendBlockedMsg(int message) {
    //ALOGD("isp-msg, receive message: %d", message);
    Message_cam msg;
    Semaphore sem;

    mGpuOPLock.lock();

    msg.command = message;
    sem.Create();
    msg.arg1 = (void*)(&sem);
    gpuCmdThreadCommandQ.put(&msg);
    mGpuOPCond.signal();
    mGpuOPLock.unlock();
    if(msg.arg1){
        sem.Wait();
    }

}

//MFNR
MFNRAdapter::MFNRAdapter():
	mCheckInitialized(false),	
	mCheckAvailable(false),
    mMFNREngine(NULL),
    mFrameCount(0),
    mMFDCommandThreadState(STA_GPUCMD_IDLE),
    m_buffers_capture(NULL)
{
    memset(&mMFNRFunc, 0, sizeof(struct MFNRFunc));
}

MFNRAdapter::~MFNRAdapter() {
    if (mMFDCommandThread != NULL) {
           sendBlockedMsg(CMD_GPU_PROCESS_DEINIT);
        mMFDCommandThread->requestExitAndWait();
        mMFDCommandThread.clear();
        mMFDCommandThread = NULL;
    }

    if (m_buffers_capture != NULL){
        delete m_buffers_capture;
        m_buffers_capture = NULL;
    }
}

void MFNRAdapter::setCameraDevice(CamDevice *camDevice) {
    mCamDevice = camDevice;
}

bool MFNRAdapter::initEnv() {
	//return false;
	mMFNRFunc.mMFNRHandle = dlopen(MFNR_LIB_PATH, RTLD_NOW);
    if (mMFNRFunc.mMFNRHandle == NULL) {
        ALOGE("%s(%d): open libmfnr.so fail",__FUNCTION__,__LINE__);
        const char *errmsg;
        if ((errmsg = dlerror()) != NULL) {
            ALOGE("dlopen libmfnr.so fail errmsg: %s", errmsg);
        }
        return false;
    } else {
        ALOGD("%s(%d): open libmfnr.so success",__FUNCTION__,__LINE__);
        mMFNRFunc.mMFNRInitFunc =
            (MFNR_init_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_init]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRInitFunc, MFNR_FUNC_NAMES[MFNR_init]);
        mMFNRFunc.mMFNRCallbackFunc =
            (MFNR_callback_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_setCallback]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRCallbackFunc, MFNR_FUNC_NAMES[MFNR_setCallback]);
        mMFNRFunc.mMFNRFrameFunc =
            (MFNR_frame_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_setFrames]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRFrameFunc, MFNR_FUNC_NAMES[MFNR_setFrames]);
        mMFNRFunc.mMFNRUpdateFunc =
            (MFNR_update_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_update]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRUpdateFunc, MFNR_FUNC_NAMES[MFNR_update]);
        mMFNRFunc.mMFNRProcFunc =
            (MFNR_proc_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_process]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRProcFunc, MFNR_FUNC_NAMES[MFNR_process]);
        mMFNRFunc.mMFNRSyncFunc =
            (MFNR_sync_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_sync]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRSyncFunc, MFNR_FUNC_NAMES[MFNR_sync]);
        mMFNRFunc.mMFNRDeinitFunc =
            (MFNR_deinit_func)dlsym(mMFNRFunc.mMFNRHandle, MFNR_FUNC_NAMES[MFNR_deinit]);
        checkLinkStatus((void*)mMFNRFunc.mMFNRDeinitFunc, MFNR_FUNC_NAMES[MFNR_deinit]);
		if (mMFNRFunc.mMFNRInitFunc == NULL
                || mMFNRFunc.mMFNRCallbackFunc == NULL
                || mMFNRFunc.mMFNRFrameFunc == NULL
				|| mMFNRFunc.mMFNRUpdateFunc == NULL				
				|| mMFNRFunc.mMFNRProcFunc == NULL
				|| mMFNRFunc.mMFNRSyncFunc == NULL
				|| mMFNRFunc.mMFNRDeinitFunc == NULL) {
			ALOGD("dlsym MFNR funcs failed!");
			return false;
		}
    }

	m_buffers_capture = new cv_fimc_buffer();
	memset(m_buffers_capture,0x0,sizeof(cv_fimc_buffer));
	mMFDCommandThread = new MFDCommandThread(this);
	mMFDCommandThreadState = STA_GPUCMD_IDLE;
	mMFDCommandThread->run("MFDCommandThread",ANDROID_PRIORITY_DISPLAY);
    ALOGD("init environment success!");

	return true;
}

bool MFNRAdapter::checkAvailable() {
	return mCheckAvailable;
}

void MFNRAdapter::setDimension(int width, int height) {
	mMfdFBOWidth = width;
	mMfdFBOHeight = height;
}

int MFNRAdapter::getFrameCount() {
    return mFrameCount;
}

void MFNRAdapter::wrapData(void* graphic_buffer, void* start, int share_fd, int length, buffer_handle_t handle) {
	m_buffers_capture->start = (void *)start;
	m_buffers_capture->share_fd = share_fd;
	m_buffers_capture->length = length;
	m_buffers_capture->handle = handle;

#ifdef TARGET_RK3368
    if (graphic_buffer != NULL) {
       ANativeWindowBuffer* nativeWindowBuffer = (ANativeWindowBuffer* )((GraphicBuffer*)graphic_buffer)->getNativeBuffer();
       GRAPHIC_BUFFER_NATIVE_HANDLE *psNativeHandle = (GRAPHIC_BUFFER_NATIVE_HANDLE*)nativeWindowBuffer->handle;
       m_buffers_capture->ui64Stamp = psNativeHandle->ui64Stamp;
    }
#endif
}

void MFNRAdapter::mfdCommandThread()
{
    Message_cam msg;
    float mfdISO;
	char mfd_set_enable;
	float get_mfdISO[3];
	float mfdFrames[3];
    while (mMFDCommandThreadState != STA_GPUCMD_STOP) {
    gpu_receive_cmd:
        if (mfdCmdThreadCommandQ.isEmpty() == false ) {
            mfdCmdThreadCommandQ.get(&msg);

            //ALOGD("isp-msg,command thread receive message: %d", msg.command);
            switch (msg.command)
            {
                case CMD_GPU_PROCESS_INIT:
                {
                    //ALOGD("check init, w-h: %d-%d, tid: %d", width, height, gettid());
                    //if (!mCheckInitialized)
                    {
                        mMFNRFunc.mMFNRInitFunc(mMFNREngine, mMfdFBOWidth, mMfdFBOHeight);
                        mCheckInitialized = true;
                    }

                    mMFNRFunc.mMFNRCallbackFunc(mMFNREngine, allocateGraphicBuffer);

                    mMFDCommandThreadState = STA_GPUCMD_RUNNING;
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();
                    break;
                }
                case CMD_GPU_PROCESS_UPDATE:
                {
                    if (mCheckInitialized) {
                        mMFNRFunc.mMFNRUpdateFunc(mMFNREngine, m_buffers_capture);
                    }
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    break;
                }
                case CMD_GPU_PROCESS_RENDER:
                {
                    if (mCheckInitialized) {
                        mCamDevice->getGain(mfdISO);
                        mMFNRFunc.mMFNRProcFunc(mMFNREngine, NULL, mfdISO, OUTPUT_NONE);
                        LOGD("CameraIspAdapter::mfdCommandThread mfdISO = %f",mfdISO);
                    }
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    break;
                }
                case CMD_GPU_PROCESS_SETFRAMES:
                {
                    if (mCamDevice != NULL) {
    					mCamDevice->getGain(mfdISO);
    					mCamDevice->getMfdGain(&mfd_set_enable, get_mfdISO, mfdFrames);
                    }

					if(mfd_set_enable == 1) {
						if(mfdISO > get_mfdISO[2]) {
							mFrameCount = mfdFrames[2];
						}
						else if(mfdISO > get_mfdISO[1]){
							mFrameCount = mfdFrames[1];
						}
						else {
							mFrameCount = mfdFrames[0];
						}
					} else {
						mFrameCount = 2;
					}

                    LOGD("SETFRAMES(%d), enable: %d, iso: %f\n" \
                                    "mfdISO = [%f-%f-%f]\n" \
                                    "mfdFrames = [%f-%f-%f]",
                                    mFrameCount, mfd_set_enable, mfdISO,
                                    get_mfdISO[0], get_mfdISO[1], get_mfdISO[2],
                                    mfdFrames[0], mfdFrames[1], mfdFrames[2]);

					mMFNRFunc.mMFNRFrameFunc(mMFNREngine, mFrameCount);
                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();
                    break;
                }
                case CMD_GPU_PROCESS_GETRESULT:
                {
					mMFNRFunc.mMFNRSyncFunc(mMFNREngine, (long)m_buffers_capture->start);
					if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();
                    break;
                }
                case CMD_GPU_PROCESS_DEINIT:
                {
                    if (mCheckInitialized) {
                        mMFNRFunc.mMFNRDeinitFunc(mMFNREngine);
                        mCheckInitialized = false;
                    }

                    mMFDCommandThreadState = STA_GPUCMD_STOP;

                    if(msg.arg1)
                        ((Semaphore*)msg.arg1)->Signal();

                    continue;
                }
            }
        }

        mMfdOPLock.lock();
        if (mfdCmdThreadCommandQ.isEmpty() == false ) {
            mMfdOPLock.unlock();
            goto gpu_receive_cmd;
        }

        mMfdOPCond.wait(mMfdOPLock);
        mMfdOPLock.unlock();

        goto gpu_receive_cmd;
    }
}

void MFNRAdapter::sendBlockedMsg(int message) {
    //ALOGD("isp-msg, receive message: %d", message);
    Message_cam msg;
    Semaphore sem;

    mMfdOPLock.lock();
    msg.command = message;
    sem.Create();
    msg.arg1 = (void*)(&sem);
    mfdCmdThreadCommandQ.put(&msg);
    mMfdOPCond.signal();
    mMfdOPLock.unlock();
    if(msg.arg1){
        sem.Wait();
    }

}

