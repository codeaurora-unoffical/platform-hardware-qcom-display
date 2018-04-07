/*
* Copyright (c) 2018, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <sys/resource.h>
#include <sys/prctl.h>
#include <utils/constants.h>
#include <utils/String16.h>
#include <utils/debug.h>
#include <gralloc_priv.h>
#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"

#define __CLASS__ "HWCSideband"

namespace sdm {

SidebandStreamBuf::~SidebandStreamBuf(void) {
  if (mHandle) {
    mHandle->releaseBuffer(mIdx);
    delete mSBHandle;
  }
}

HWCSidebandStream::HWCSidebandStream(HWCSidebandStreamSession *session, buffer_handle_t handle)
  :mSession(session) {

  mNativeHandle = native_handle_clone(handle);

  int enableBackpressure = 0;
  if (Debug::Get()->GetProperty("sdm.debug.sideband.backpressure", &enableBackpressure) == 0)
    enableBackpressure_ = enableBackpressure;

  android::SidebandStreamHandle * sidebandHandle = SidebandStreamLoader::GetSidebandStreamHandle();
  if (sidebandHandle && sidebandHandle->mHandleConsumer) {
    sb_nativeHandle_ = sidebandHandle->mHandleConsumer(mNativeHandle);
    if (sb_nativeHandle_) {
      if (pthread_create(&sideband_thread_, NULL, &SidebandStreamThread, this) < 0) {
        DLOGE("Failed to start thread %p\n", sb_nativeHandle_);
      }
    } else {
      DLOGE("Failed to create SidebandStream consumer");
    }
  } else {
    DLOGE("Failed to load SidebandStreamHandler");
  }
}

HWCSidebandStream::~HWCSidebandStream() {
  mSidebandLock_.Lock();
  sideband_thread_exit_ = true;
  mSidebandLock_.Signal();
  mSidebandLock_.Unlock();
  pthread_join(sideband_thread_, NULL);
  mStreamBuf_ = nullptr;
  if (sb_nativeHandle_)
    delete sb_nativeHandle_;
  native_handle_close(mNativeHandle);
  native_handle_delete(mNativeHandle);
}

int32_t HWCSidebandStream::AddLayer(hwc2_layer_t layer, hwc2_display_t display) {
  SCOPE_LOCK(mSidebandLock_);
  if (!mLayers.count(layer)) {
    mLayers[layer] = display;
    displayMask_ |= (1 << display);
    mDisplays.insert(display);
  }
  return 0;
}

int32_t HWCSidebandStream::RemoveLayer(hwc2_layer_t layer) {
  SCOPE_LOCK(mSidebandLock_);
  auto iter = mLayers.find(layer);
  if (iter != mLayers.end()) {
    if (pendingMask_ | (1 << iter->second)) {
      pendingMask_ &= ~(1 << iter->second);
      if (!pendingMask_)
        mSidebandLock_.Signal();
    }
    mLayers.erase(iter);
    displayMask_ = 0;
    mDisplays.clear();
    if (mLayers.size() == 0) {
      return HWC2_ERROR_NO_RESOURCES;
    }
    for (auto & layer : mLayers) {
      displayMask_ |= (1 << layer.second);
      mDisplays.insert(layer.second);
    }
  }
  return 0;
}

int32_t HWCSidebandStream::SetBuffer(android::sp<SidebandStreamBuf> buf) {
  SCOPE_LOCK(mSidebandLock_);
  if (pendingMask_ && !sideband_thread_exit_) {
    mSidebandLock_.Wait();
  }
  if (enableBackpressure_) {
    pendingMask_ = displayMask_;
  }
  mStreamBuf_ = buf;
  return 0;
}

android::sp<SidebandStreamBuf> HWCSidebandStream::GetBuffer(void) {
  SCOPE_LOCK(mSidebandLock_);
  return mStreamBuf_;
}

int32_t HWCSidebandStream::PostDisplay(hwc2_display_t displayId) {
  SCOPE_LOCK(mSidebandLock_);
  if (pendingMask_ | (1 << displayId)) {
    pendingMask_ &= ~(1 << displayId);
    if (!pendingMask_)
      mSidebandLock_.Signal();
  }
  return 0;
}

void *HWCSidebandStream::SidebandStreamThread(void *context) {
  if (context) {
    return reinterpret_cast<HWCSidebandStream *>(context)->SidebandThreadHandler();
  }

  return NULL;
}

void *HWCSidebandStream::SidebandThreadHandler(void) {
  setpriority(PRIO_PROCESS, 0, HAL_PRIORITY_URGENT_DISPLAY);
  android::status_t result = android::NO_ERROR;
  int width, height, buf_idx, buf_fd, format;
  unsigned int size;

  width = sb_nativeHandle_->getBufferWidth();
  height = sb_nativeHandle_->getBufferHeight();
  size = (unsigned int)sb_nativeHandle_->getBufferSize();

  while (!sideband_thread_exit_) {
    result = sb_nativeHandle_->acquireBuffer(&buf_idx, 50);
    if (result==android::NO_ERROR ){
      android::sp<SidebandStreamBuf> ptr = new SidebandStreamBuf;
      buf_fd = sb_nativeHandle_->getBufferFd(buf_idx);
      format = sb_nativeHandle_->getColorFormat();

#if USE_GRALLOC1
      ptr->mSBHandle = new private_handle_t(buf_fd,
                            -1,
                            0,
                            width,   //buffer width
                            height,  //buffer height
                            width,   //image width
                            height,  //image height
                            format,
                            BUFFER_TYPE_VIDEO,
                            size,
                            GRALLOC1_PRODUCER_USAGE_NONE,
                            GRALLOC1_CONSUMER_USAGE_NONE);
#else
      ptr->mSBHandle = new private_handle_t(buf_fd,
                            size,
                            0,
                            BUFFER_TYPE_VIDEO,
                            format,
                            width,
                            height,
                            -1,
                            0,
                            0,
                            width,
                            height);
#endif
      ptr->mHandle = sb_nativeHandle_;
      ptr->mIdx = buf_idx;

      SetBuffer(ptr);

      if (sideband_thread_exit_)
        break;

      mSession->UpdateSidebandStream(this);
    }
  }

  return NULL;
}

android::SidebandStreamHandle * SidebandStreamLoader::handle_inst_ = NULL;

android::SidebandStreamHandle * SidebandStreamLoader::GetSidebandStreamHandle(void) {
  if (!handle_inst_) {
    handle_inst_ = new android::SidebandStreamHandle;
    handle_inst_->init();
  }
  return handle_inst_;
}

int32_t HWCSidebandStreamSession::Init(HWCSession *session) {
  hwc_session = session;
  return 0;
}

void HWCSidebandStreamSession::StartPresentation(hwc2_display_t display) {
  present_start = true;
}

void HWCSidebandStreamSession::StopPresentation(hwc2_display_t display) {
  for (auto & sbstream : mSidebandStreamList) {
    sbstream.second->PostDisplay(display);
  }

  clock_gettime(CLOCK_MONOTONIC, &present_timestamp_);
  present_start = false;
}

int32_t HWCSidebandStreamSession::SetLayerSidebandStream(hwc2_display_t display,
                                  hwc2_layer_t layer, buffer_handle_t stream) {
  int32_t stream_id;
  HWCSidebandStream *stm;

  if (android::SidebandHandleBase::validate(stream))
    return HWC2_ERROR_NOT_VALIDATED;

  const android::SidebandHandleBase * sb_nativeHandle_ = static_cast<const android::SidebandHandleBase *>(stream);
  stream_id = sb_nativeHandle_->getSidebandHandleId();

  if (mSidebandStreamList.count(stream_id) == 0) {
    //No active sideband thread for this stream, create one.
    stm = new HWCSidebandStream(this, stream);
    mSidebandStreamList[stream_id] = stm;
  } else {
    //sideband thread exist for this stream, Get the thread handle.
    stm = mSidebandStreamList[stream_id];
  }

  //SF will always clone stream for sideband, delete them here to avoid memory/fd leak
  native_handle_t* native_handle = const_cast<native_handle_t*>(stream);
  native_handle_close(native_handle);
  native_handle_delete(native_handle);

  //try to add new layer as a listner to this sideband stream
  stm->AddLayer(layer, display);

  //pickup most recent sideband stream buffer
  android::sp<SidebandStreamBuf> buf = stm->GetBuffer();
  if (buf != nullptr) {
    HWCSession::CallLayerFunction(hwc_session, display, layer, &HWCLayer::SetLayerSidebandStream, buf);
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSidebandStreamSession::DestroyLayer(hwc2_display_t display, hwc2_layer_t layer) {
  std::vector<int32_t> removeList;
  for (auto & sbstream : mSidebandStreamList) {
    if (sbstream.second->RemoveLayer(layer) == HWC2_ERROR_NO_RESOURCES) {
      removeList.push_back(sbstream.first);
    }
  }

  for (auto stream_id : removeList) {
    auto sbstream = mSidebandStreamList[stream_id];
    mSidebandStreamList.erase(stream_id);
    delete sbstream;
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSidebandStreamSession::UpdateSidebandStream(HWCSidebandStream * stm) {
  int vsync_span = hwc_session->GetVsyncPeriod(0);

  SCOPE_LOCK(hwc_session->locker_);

  // if there are more than one streams, go back to surfaceflinger
  if (mSidebandStreamList.size() > 1) {
    for (auto & display : stm->mDisplays) {
      hwc_session->Refresh(display);
    }
    return 0;
  }

  // if present start, return immediately
  if (present_start)
    return 0;

  // check most recent present timestamp, if larger than vsync period, display right away
  struct timespec tv;
  clock_gettime(CLOCK_MONOTONIC, &tv);
  int64_t span = (tv.tv_sec - present_timestamp_.tv_sec) * 1000000000LL +
                (tv.tv_nsec - present_timestamp_.tv_nsec);
  if (span <= vsync_span) {
    hwc_session->locker_.WaitFinite((int)(vsync_span - span) / 1000000);
    if (present_start)
      return 0;
  }

  for (auto & pair : stm->mLayers) {
    hwc2_layer_t layer = pair.first;
    hwc2_display_t display = pair.second;
    HWCSession::CallLayerFunction(hwc_session, display, layer, &HWCLayer::SetLayerSidebandStream, stm->GetBuffer());
  }

  for (auto & display : stm->mDisplays) {
    uint32_t num_types, num_requests;
    auto status = hwc_session->hwc_display_[display]->Validate(&num_types, &num_requests);
    if (status != HWC2::Error::None) {
      goto next;
    }

    int32_t retire_fence;
    status = hwc_session->hwc_display_[display]->Present(&retire_fence);
    if (status != HWC2::Error::None) {
      goto next;
    }
    if (retire_fence >= 0)
      ::close(retire_fence);

    uint32_t num_layer;
    hwc_session->hwc_display_[display]->GetReleaseFences(&num_layer, NULL, NULL);
    if (num_layer) {
      std::vector<hwc2_layer_t> layers(num_layer);
      std::vector<int32_t> fences(num_layer);
      hwc_session->hwc_display_[display]->GetReleaseFences(&num_layer, layers.data(), fences.data());
      for (uint32_t i = 0; i < num_layer; i++) {
        if (fences[i] >= 0)
          ::close(fences[i]);
      }
    }

next:
    stm->PostDisplay(display);
  }

  return 0;
}

}  // namespace sdm
