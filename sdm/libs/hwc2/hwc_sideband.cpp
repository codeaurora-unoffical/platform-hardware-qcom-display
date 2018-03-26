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

HWCSidebandStream::HWCSidebandStream(hwc2_device_t *device, buffer_handle_t handle)
  :mDevice(device) {

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
    if (mLayers.size() == 0) {
      return HWC2_ERROR_NO_RESOURCES;
    }
    for (auto & layer : mLayers) {
      displayMask_ |= (1 << layer.second);
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
  HWCSession *hwc_session = static_cast<HWCSession *>(mDevice);

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

      for (auto & layer : mLayers) {
        hwc_session->Refresh(layer.second);
      }
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

}  // namespace sdm
