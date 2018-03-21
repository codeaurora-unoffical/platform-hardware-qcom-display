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
  sideband_thread_exit_ = true;
  pthread_join(sideband_thread_, NULL);
  mStreamBuf_ = nullptr;
  if (sb_nativeHandle_)
    delete sb_nativeHandle_;
  native_handle_close(mNativeHandle);
  native_handle_delete(mNativeHandle);
}

int32_t HWCSidebandStream::AddLayer(hwc2_layer_t layer, hwc2_display_t display) {
  SCOPE_LOCK(mSidebandLock_);
  mLayers[layer] = display;
  return 0;
}

int32_t HWCSidebandStream::RemoveLayer(hwc2_layer_t layer) {
  SCOPE_LOCK(mSidebandLock_);
  mLayers.erase(layer);

  if (mLayers.size() == 0) {
    return HWC2_ERROR_NO_RESOURCES;
  }

  return 0;
}

bool HWCSidebandStream::HasLayer(hwc2_layer_t layer) {
  SCOPE_LOCK(mSidebandLock_);
  return mLayers.count(layer) > 0;
}

int32_t HWCSidebandStream::SetBuffer(android::sp<SidebandStreamBuf> buf) {
  SCOPE_LOCK(mSidebandLock_);
  mStreamBuf_ = buf;
  return 0;
}

android::sp<SidebandStreamBuf> HWCSidebandStream::GetBuffer(void) {
  SCOPE_LOCK(mSidebandLock_);
  return mStreamBuf_;
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

      std::bitset<32> bit_mask_display_refresh = 0;
      for(auto & layer : mLayers) {
        bit_mask_display_refresh[(size_t)layer.second] = 1;
      }
      for (uint32_t i = HWC_DISPLAY_PRIMARY; i < kOrderMax + MAX_VIRTUAL_DISPLAY_NUM; i++) {
        if (bit_mask_display_refresh[i]) {
          hwc_session->Refresh(i);
        }
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
