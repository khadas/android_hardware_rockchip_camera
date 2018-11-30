#include "cam_api/camdevice.h"
#ifndef CAMERA_GL_ADAPTER_H
#define CAMERA_GL_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _native_handle_bundle {
    uint32_t w;
    uint32_t h;
    android::PixelFormat format;
    int shareFd;
    int ionHandle;
    buffer_handle_t nativeHandle;
    uint32_t usage;
    uint64_t ui64Stamp;
} native_handle_bundle;

enum OUTPUT_FMT {
    OUTPUT_FMT_CV_MAT,
    OUTPUT_FMT_YUV,
    OUTPUT_FMT_RGBA,
    OUTPUT_NONE
};

struct cv_fimc_buffer {
    void    *start;
    int share_fd;
    size_t  length;
    int stride;
    size_t  bytesused;
    buffer_handle_t handle;
    uint64_t ui64Stamp;
};

enum GPU_COMMAND_STATUS{
	STA_GPUCMD_IDLE,
	STA_GPUCMD_RUNNING,
	STA_GPUCMD_STOP,
};

enum GPU_PROCESS_COMMANDS {
    // Commands
    CMD_GPU_PROCESS_INIT,
    CMD_GPU_PROCESS_UPDATE,
    CMD_GPU_PROCESS_RENDER,
    CMD_GPU_PROCESS_GETRESULT,
    CMD_GPU_PROCESS_SETFRAMES,
    CMD_GPU_PROCESS_DEINIT
};

typedef sp<GraphicBuffer> (*BufferHandleAllocType)(native_handle_bundle *bundle);

typedef int (*UVNR_init_func)(void* &engine, int width, int height, int buf_cnt);
typedef void (*UVNR_callback_func)(void* engine, BufferHandleAllocType allocHandle);
typedef int (*UVNR_update_func)(void* engine, struct cv_fimc_buffer *m_buffers_capture);
typedef int (*UVNR_proc_func)(void* engine, long* targetAddr, int mode, int ratio, float distance);
typedef int (*UVNR_sync_func)(void* engine, long targetAddr);
typedef void (*UVNR_deinit_func)(void* &engine);

struct UVNRFunc{
    void* mUVNRHandle;
    UVNR_init_func  mUVNRInitFunc;
    UVNR_callback_func mUVNRCallbackFunc;
	UVNR_update_func mUVNRUpdateFunc;
    UVNR_proc_func   mUVNRProcFunc;
	UVNR_sync_func mUVNRSyncFunc;
    UVNR_deinit_func mUVNRDeinitFunc;
};


class UVNRAdapter {
public:
	UVNRAdapter();
	~UVNRAdapter();
public:
	bool mCheckInitialized;
    void setCameraDevice(CamDevice *camDevice);
	bool initEnv();
	bool checkAvailable();
	void setDimension(int width, int height);
	void wrapData(void* graphic_buffer, void* start, int share_fd, int length, buffer_handle_t handle);
	void sendBlockedMsg(int message);
	void gpuCommandThread();
private:
	class GPUCommandThread : public Thread {
		UVNRAdapter* mUVNRAdapter;
	public:
		GPUCommandThread(UVNRAdapter* adapter)
			: Thread(false),mUVNRAdapter(adapter){}

		virtual bool threadLoop() {
			mUVNRAdapter->gpuCommandThread();
			return false;
		}
	};

	UVNRFunc mUVNRFunc;

	bool mCheckAvailable;
	float mISO;
	void* mUVNREngine;
	Mutex mGpuOPLock;
	Condition mGpuOPCond;	
	int mGPUCommandThreadState;
	int mGpuFBOWidth, mGpuFBOHeight;	
	struct cv_fimc_buffer* m_buffers_capture;
	sp<GPUCommandThread> mGPUCommandThread;
	MessageQueue gpuCmdThreadCommandQ;
    CamDevice *mCamDevice;
};

typedef int (*MFNR_init_func)(void* &engine, int width, int height);
typedef void (*MFNR_callback_func)(void* engine, BufferHandleAllocType allocHandle);
typedef void (*MFNR_frame_func)(void* engine, int frameNum);
typedef int (*MFNR_update_func)(void* engine, struct cv_fimc_buffer *m_buffers_capture);
typedef int (*MFNR_proc_func)(void* engine, long* targetAddr, float iso,int mode);
typedef int (*MFNR_sync_func)(void* engine, long targetAddr);
typedef void (*MFNR_deinit_func)(void* &engine);

struct MFNRFunc{
    void* mMFNRHandle;
    MFNR_init_func  mMFNRInitFunc;
    MFNR_callback_func mMFNRCallbackFunc;
    MFNR_frame_func mMFNRFrameFunc;
	MFNR_update_func mMFNRUpdateFunc;
    MFNR_proc_func   mMFNRProcFunc;
	MFNR_sync_func mMFNRSyncFunc;
    MFNR_deinit_func mMFNRDeinitFunc;
};

class MFNRAdapter {
public:
	MFNRAdapter();
	~MFNRAdapter();
public:
	bool mCheckInitialized;
    void setCameraDevice(CamDevice *camDevice);
	bool initEnv();
	bool checkAvailable();
	void setDimension(int width, int height);
    int getFrameCount();
	void wrapData(void* graphic_buffer, void* start, int share_fd, int length, buffer_handle_t handle);
	void sendBlockedMsg(int message);
	void mfdCommandThread();
private:
	class MFDCommandThread : public Thread {
		MFNRAdapter* mMFNRAdapter;
	public:
		MFDCommandThread(MFNRAdapter* adapter)
			: Thread(false),mMFNRAdapter(adapter){}

		virtual bool threadLoop() {
			mMFNRAdapter->mfdCommandThread();
			return false;
		}
	};

	MFNRFunc mMFNRFunc;

	void* mMFNREngine;    
	bool mCheckAvailable;
	int mMfdFBOWidth, mMfdFBOHeight;	
	int mFrameCount;
	struct cv_fimc_buffer* m_buffers_capture;

	Mutex mMfdOPLock;
	Condition mMfdOPCond;	
	int mMFDCommandThreadState;

	sp<MFDCommandThread> mMFDCommandThread;
	MessageQueue mfdCmdThreadCommandQ;
    CamDevice *mCamDevice;
};

#ifdef __cplusplus
}
#endif

#endif

