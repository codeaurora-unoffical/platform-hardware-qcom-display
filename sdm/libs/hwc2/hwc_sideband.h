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

#ifndef __HWC_SIDEBAND_H__
#define __HWC_SIDEBAND_H__

#include <hardware/hwcomposer2.h>
#include <utils/locker.h>
#include <utils/StrongPointer.h>
#include <utils/LightRefBase.h>
#include <SidebandHandle.h>
#include <map>

namespace sdm {

  struct SidebandStreamBuf : public android::LightRefBase<SidebandStreamBuf> {
    private_handle_t *mSBHandle = nullptr;
    android::SidebandHandle *mHandle = nullptr;
    int32_t mIdx = 0;
    ~SidebandStreamBuf(void);
  };

  class HWCSidebandStream {
   public:
    int32_t AddLayer(hwc2_layer_t layer, hwc2_display_t display);
    int32_t RemoveLayer(hwc2_layer_t layer);
    bool HasLayer(hwc2_layer_t layer);
    int32_t SetBuffer(android::sp<SidebandStreamBuf> buf);
    android::sp<SidebandStreamBuf> GetBuffer(void);
    static void *SidebandStreamThread(void *context);
    void *SidebandThreadHandler();
    HWCSidebandStream(hwc2_device_t *device, buffer_handle_t handle);
    ~HWCSidebandStream();

   private:
    std::map<hwc2_layer_t, hwc2_display_t> mLayers;
    Locker mSidebandLock_;
    hwc2_device_t *mDevice = NULL;
    native_handle_t *mNativeHandle = NULL;
    android::SidebandHandle *sb_nativeHandle_ = NULL;
    android::sp<SidebandStreamBuf> mStreamBuf_;
    pthread_t sideband_thread_ = {};
    bool sideband_thread_exit_ = false;
  };

}  // namespace sdm

#endif  // __HWC_SESSION_H__
