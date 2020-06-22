/*
* Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
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
#include <SidebandStreamHandle.h>
#include <unordered_map>
#include <set>

namespace sdm {

#define CSC_HUE_TAG             1 << 1
#define CSC_BRIGHTNESS_TAG      1 << 2
#define CSC_CONTRAST_TAG        1 << 3
#define CSC_SATURATION_TAG      1 << 4
#define CSC_COLOR_TONE_TAG      1 << 5

struct color_data_pack {
  uint32_t flags;
  float hue;
  float saturation;
  float tone_cb;
  float tone_cr;
  float contrast;
  float brightness;
  bool color_dirty;
};

/**
 * struct csc_mat:    csc matrix structure
 * @ctm_coeff:        New main matrix coefficients
 * @post_biast:        New post-bias offset values
 */
  struct csc_mat {
    int64_t ctm_coeff[CSC_MATRIX_COEFF_SIZE];
    uint32_t post_bias[CSC_BIAS_SIZE];
  };

  struct SidebandHandlePtr : public android::LightRefBase<SidebandHandlePtr> {
    android::SidebandHandleBase *mHandle = nullptr;
    explicit SidebandHandlePtr(android::SidebandHandleBase *base) : mHandle(base) {}
    ~SidebandHandlePtr(void) { delete mHandle; }
    android::SidebandHandleBase *get() const { return mHandle; }
  };

  struct SidebandStreamBuf : public android::LightRefBase<SidebandStreamBuf> {
    private_handle_t *mSBHandle = nullptr;
    android::sp<SidebandHandlePtr> mHandle;
    int32_t mIdx = 0;
    ~SidebandStreamBuf(void);
  };

  class HWCSession;
  class HWCSidebandStreamSession;

  class HWCSidebandStream {
   public:
    int32_t AddLayer(hwc2_layer_t layer, hwc2_display_t display);
    int32_t RemoveLayer(hwc2_layer_t layer);
    int32_t SetBuffer(android::sp<SidebandStreamBuf> buf);
    android::sp<SidebandStreamBuf> GetBuffer(void);
    int32_t CheckBuffer(void);
    int32_t PostDisplay(hwc2_display_t display);
    int32_t CalcCscInputData(color_data_pack *color_data_pack_, csc_mat *new_mat);
    void MatrixMultiplication(float *pArray1, int pArrary1_row_num,
                              int pArray1_col_num, float *pArray2,
                              int pArray2_col_num, float *pDestArray);
    HWCSidebandStream(HWCSidebandStreamSession *session, buffer_handle_t handle);
    ~HWCSidebandStream();

   private:
    static void *SidebandStreamThread(void *context);
    void *SidebandThreadHandler();

   private:
    friend class HWCSidebandStreamSession;
    std::unordered_map<hwc2_layer_t, hwc2_display_t> mLayers;
    Locker mSidebandLock_;
    HWCSidebandStreamSession *mSession = NULL;
    native_handle_t *mNativeHandle = NULL;
    android::sp<SidebandHandlePtr> sb_nativeHandle_;
    android::sp<SidebandStreamBuf> mStreamBuf_;
    std::set<hwc2_display_t> mDisplays;
    uint32_t displayMask_ = 0;
    uint32_t displayValidateMask_ = 0;
    uint32_t pendingMask_ = 0;
    bool enableBackpressure_ = true;
    pthread_t sideband_thread_ = {};
    bool sideband_thread_exit_ = false;
    bool new_bufffer_ = false;
    color_data_pack color_data_pack_ = {};
    int32_t retire_fences_ = -1;
  };

  class SidebandStreamLoader {
   private:
    static android::SidebandStreamHandle * handle_inst_;
    explicit SidebandStreamLoader(void){}
   public:
    static android::SidebandStreamHandle * GetSidebandStreamHandle(void);
  };

  class HWCSidebandStreamSession {
   public:
    HWCSidebandStreamSession() {};
    ~HWCSidebandStreamSession(void) {};
    int32_t Init(HWCSession *session);
    int32_t SetLayerSidebandStream(hwc2_display_t display, hwc2_layer_t layer,
                    buffer_handle_t stream);
    void StartPresentation(hwc2_display_t display);
    void StopPresentation(hwc2_display_t display);
    int32_t DestroyLayer(hwc2_display_t display, hwc2_layer_t layer);
    int32_t UpdateSidebandStream(HWCSidebandStream * stream);

   private:
    int32_t UpdateSidebandStreamLocked(HWCSidebandStream * stm,
                                    std::vector<hwc2_display_t> *refresh_list);
    HWCSession * hwc_session = NULL;
    std::map<int32_t, HWCSidebandStream*> mSidebandStreamList;
    uint32_t present_start = 0;
  };

}  // namespace sdm

#endif  // __HWC_SESSION_H__
