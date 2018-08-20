/*
 * Copyright (C) 2017 Intel Corporation.
 * Copyright (c) 2017, Fuzhou Rockchip Electronics Co., Ltd
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PSL_RKISP1_WORKERS_INPUTFRAMEWORKER_H_
#define PSL_RKISP1_WORKERS_INPUTFRAMEWORKER_H_

#include "IDeviceWorker.h"
#include "NodeTypes.h"
#include "tasks/ICaptureEventSource.h"
#include "PostProcessPipeline.h"
namespace android {
namespace camera2 {

class InputFrameWorker: public IDeviceWorker, public ICaptureEventSource, public IPostProcessListener
{
public:
    InputFrameWorker(int cameraId,
                     camera3_stream_t* stream, std::vector<camera3_stream_t*>& outStreams,
                     size_t pipelineDepth);
    virtual ~InputFrameWorker();

    virtual status_t configure(std::shared_ptr<GraphConfig> &config);
    status_t prepareRun(std::shared_ptr<DeviceMessage> msg);
    status_t run();
    status_t postRun();
    status_t stopWorker();
    status_t startWorker();
    std::shared_ptr<V4L2VideoNode> getNode() const { return nullptr; }
    status_t notifyNewFrame(const std::shared_ptr<PostProcBuffer>& buf,
                            const std::shared_ptr<ProcUnitSettings>& settings,
                            int err);
    virtual status_t asyncPollDone(std::shared_ptr<DeviceMessage> msg, bool polled) {
        mMsg = msg;
        return OK;
    }

private:
    std::shared_ptr<CameraBuffer> findInputBuffer(Camera3Request* request,
                                             camera3_stream_t* stream);
    std::vector<std::shared_ptr<CameraBuffer>> findOutputBuffers(Camera3Request* request);
    status_t prepareBuffer(std::shared_ptr<CameraBuffer>& buffer);
    void returnBuffers();

private:
    std::vector<std::shared_ptr<CameraBuffer>> mProcessingInputBufs;
    camera3_stream_t* mStream; /* InputFrameWorker doesn't own mStream */
    std::vector<camera3_stream_t*> mOutputStreams; /* InputFrameWorker doesn't own mStream */
    bool mNeedPostProcess;
    size_t mPipelineDepth;

    std::unique_ptr<PostProcessPipeLine> mPostPipeline;
};

} /* namespace camera2 */
} /* namespace android */

#endif /* PSL_RKISP1_WORKERS_INPUTFRAMEWORKER_H_ */