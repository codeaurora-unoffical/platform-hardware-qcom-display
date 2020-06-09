/*
* Copyright (c) 2018-2019, 2020 The Linux Foundation. All rights reserved.
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
#include <sync/sync.h>
#include "hwc_buffer_sync_handler.h"
#include "hwc_session.h"

#define __CLASS__ "HWCSideband"

namespace sdm {

SidebandStreamBuf::~SidebandStreamBuf(void) {
  if (mHandle != nullptr) {
    mHandle->get()->releaseBuffer(mIdx);
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
    sb_nativeHandle_ = new SidebandHandlePtr(sidebandHandle->mHandleConsumer(mNativeHandle));
    if (sb_nativeHandle_ != nullptr) {
      if (pthread_create(&sideband_thread_, NULL, &SidebandStreamThread, this) < 0) {
        DLOGE("Failed to start thread %p\n", sb_nativeHandle_->get());
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
  sb_nativeHandle_ = nullptr;
  native_handle_close(mNativeHandle);
  native_handle_delete(mNativeHandle);
  if (retire_fences_ >= 0)
    ::close(retire_fences_);
}

int32_t HWCSidebandStream::AddLayer(hwc2_layer_t layer, hwc2_display_t display) {
  SCOPE_LOCK(mSidebandLock_);
  if (!mLayers.count(layer)) {
    mLayers[layer] = display;
    displayMask_ |= (1 << display);
    displayValidateMask_ |= (1 << display);
    mDisplays.insert(display);
    return 1;
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
    displayValidateMask_ = 0;
    displayMask_ = 0;
    mDisplays.clear();
    if (mLayers.size() == 0) {
      return HWC2_ERROR_NO_RESOURCES;
    }
    for (auto & layer : mLayers) {
      displayValidateMask_ |= (1 << layer.second);
      displayMask_ |= (1 << layer.second);
      mDisplays.insert(layer.second);
    }
  }
  return 0;
}

int32_t HWCSidebandStream::SetBuffer(android::sp<SidebandStreamBuf> buf) {
  SCOPE_LOCK(mSidebandLock_);
  pendingMask_ = displayMask_;
  mStreamBuf_ = buf;
  new_bufffer_ = true;
  return 0;
}

android::sp<SidebandStreamBuf> HWCSidebandStream::GetBuffer(void) {
  SCOPE_LOCK(mSidebandLock_);
  new_bufffer_ = false;
  return mStreamBuf_;
}

int32_t HWCSidebandStream::CheckBuffer(void) {
  SCOPE_LOCK(mSidebandLock_);
  if (new_bufffer_)
    return HWC2_ERROR_NOT_VALIDATED;
  if (enableBackpressure_ && pendingMask_ && !sideband_thread_exit_) {
    mSidebandLock_.WaitFinite(100);
  }
  return 0;
}

int32_t HWCSidebandStream::PostDisplay(hwc2_display_t displayId) {
  SCOPE_LOCK(mSidebandLock_);
  if ((pendingMask_ | (1 << displayId)) && !new_bufffer_) {
    pendingMask_ &= ~(1 << displayId);
    if (!pendingMask_)
      mSidebandLock_.Signal();
  }
  return 0;
}

void HWCSidebandStream::MatrixMultiplication(float *pArray1, int pArrary1_row_num,
                                             int pArray1_col_num, float *pArray2,
                                             int pArray2_col_num, float *pDestArray) {
  int i, j, k;

  for (i = 0; i < pArrary1_row_num; i++) {
    for (j = 0; j < pArray2_col_num; j++) {
        for (k = 0; k < pArray1_col_num; k++)
          pDestArray[pArray2_col_num * i + j] +=
            pArray1[pArray1_col_num * i + k] * pArray2[pArray2_col_num * k + j];
    }
  }
}

/*
 * Color space conversion formulas
 *
 * The default YUV2RGB transformation CSC matrix is:
 * YUV2RGB = [1.1644, 0.0000, 1.596,
 *            1.1644, -0.3918, -0.813,
 *            1.1644, 2.0172, 0.0000,]
 *
 * 1) For the general input case, Hue(H), Contrast(C), Brightness(B), Saturation(S),
 *    the calculated CSC coeffcients is:
 *    YUV2RGB *[C, 0,        0,
 *              0, S*cos(H), S*sin(H),
 *              0, -S*sin(H), S*cos(H),].
 *
 * 2) For the input case like color tone(Cb, Cr), Contrast(C), Brightness(B), Saturation(S),
 *    the calculated CSC coeffcients is:
 *    YUV2RGB *[C, 0,    0,
 *              0, S*Cb, 0,
 *              0, 0,    S*Cr,].
 *
 * 3) The offset of output pre basis matrix(3*1 matrix) is from:
 *    [B - 128 *(C - 1), 0, 0,].
 */
int32_t HWCSidebandStream::CalcCscInputData(color_data_pack *color_data_pack_, csc_mat *csc_mat) {
  float default_yuv2rgb_matrix[CSC_MATRIX_COEFF_SIZE] = {(float)1.1644, (float)0.0000, (float)1.596,
                                                       (float)1.1644, (float)-0.3918, (float)-0.813,
                                                       (float)1.1644, (float)2.0172, (float)0.0000,};
  float coeff_adj_matrix[CSC_MATRIX_COEFF_SIZE] = {(float)1.0, (float)0.0, (float)0.0,
                                                   (float)0.0, (float)1.0, (float)0.0,
                                                   (float)0.0, (float)0.0, (float)1.0};
  float new_coeff_matrix[CSC_MATRIX_COEFF_SIZE] = {(float(0.0))};
  float new_post_bias_matrix[CSC_BIAS_SIZE] = {(float)0.0};
  float bias_adj_matrix[CSC_BIAS_SIZE] = {(float)0.0};
  int32_t i = 0, temp;

  SCOPE_LOCK(mSidebandLock_);

  DLOGI("CalcCscInputData color_data_pack %f, %f, %f, %f, %f",
             color_data_pack_->saturation, color_data_pack_->tone_cb, color_data_pack_->tone_cr,
             color_data_pack_->contrast, color_data_pack_->brightness);

  if ((color_data_pack_->flags & CSC_HUE_TAG) && (color_data_pack_->flags & CSC_COLOR_TONE_TAG)) {
    DLOGE("Hue and Color tone setting can't co-exist.");
    return HWC2_ERROR_NOT_VALIDATED;
  }

  /* Update coeff adj matrix */
  coeff_adj_matrix[0] = color_data_pack_->contrast;

  /* Hanle Hue case and Color Tone case respectively. */
  if (color_data_pack_->flags & CSC_HUE_TAG) {
    coeff_adj_matrix[4] = color_data_pack_->saturation * cos(color_data_pack_->hue);
    coeff_adj_matrix[5] = color_data_pack_->saturation * sin(color_data_pack_->hue);
    coeff_adj_matrix[7] = -color_data_pack_->saturation * sin(color_data_pack_->hue);
    coeff_adj_matrix[8] = color_data_pack_->saturation * cos(color_data_pack_->hue);
  } else if (color_data_pack_->flags & CSC_COLOR_TONE_TAG) {
    coeff_adj_matrix[4] = color_data_pack_->saturation * color_data_pack_->tone_cb;
    coeff_adj_matrix[8] = color_data_pack_->saturation * color_data_pack_->tone_cr;
  }

  /* Update output bias offset matrix */
  bias_adj_matrix[0] = color_data_pack_->brightness -
                                          128 * (color_data_pack_->contrast - 1);

  MatrixMultiplication(default_yuv2rgb_matrix, 3, 3, bias_adj_matrix, 1,
                       new_post_bias_matrix);
  for (i = 0; i < CSC_BIAS_SIZE; i++) {
    temp = (int32_t)(new_post_bias_matrix[i] + 0.5); //round
    csc_mat->post_bias[i] = (uint32_t)temp;
  }

  /* final coeff matrix = default_coeff_matrix * coeff_adj_matrix */
  MatrixMultiplication(default_yuv2rgb_matrix, 3, 3, coeff_adj_matrix, 3,
                       new_coeff_matrix);

  /* round by multiply 65536 */
  for (i = 0; i < CSC_MATRIX_COEFF_SIZE; i++) {
    csc_mat->ctm_coeff[i] = static_cast<int64_t>((new_coeff_matrix[i] * 65536.0));
  }

  return HWC2_ERROR_NONE;
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

  width = sb_nativeHandle_->get()->getBufferWidth();
  height = sb_nativeHandle_->get()->getBufferHeight();
  size = (unsigned int)sb_nativeHandle_->get()->getBufferSize();

  while (!sideband_thread_exit_) {
    if (CheckBuffer() == HWC2_ERROR_NOT_VALIDATED) {
      mSession->UpdateSidebandStream(this);
      continue;
    }

    /* Process input chroma data */
    color_data_pack_.color_dirty = false;
    android::color_data_t color_data;
    if(sb_nativeHandle_->get()->getColorData(&color_data) != SETTING_NO_DATA_CHANGE){
      DLOGI("getColorData %d, %f, %f, %f, %f, %f, %f", color_data.flags, color_data.hue,
                                                       color_data.saturation,
                                                       color_data.tone_cb,
                                                       color_data.tone_cr,
                                                       color_data.contrast,
                                                       color_data.brightness);

      color_data_pack_.color_dirty = true;
      color_data_pack_.flags = color_data.flags;
      color_data_pack_.saturation = (float)color_data.saturation;
      if (color_data.flags & CSC_HUE_TAG)
        color_data_pack_.hue = (float)color_data.hue;

      if (color_data.flags & CSC_COLOR_TONE_TAG) {
        color_data_pack_.tone_cb = (float)color_data.tone_cb;
        color_data_pack_.tone_cr = (float)color_data.tone_cr;
      }

      color_data_pack_.contrast = (float)color_data.contrast;
      color_data_pack_.brightness = (float)color_data.brightness;
    }

    result = sb_nativeHandle_->get()->acquireBuffer(&buf_idx, 50);
    if (result==android::NO_ERROR ){
      android::sp<SidebandStreamBuf> ptr = new SidebandStreamBuf;
      buf_fd = sb_nativeHandle_->get()->getBufferFd(buf_idx);
      format = sb_nativeHandle_->get()->getColorFormat();

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
  present_start |= (1 << (int)display);
}

void HWCSidebandStreamSession::StopPresentation(hwc2_display_t display) {
  for (auto & sbstream : mSidebandStreamList) {
    sbstream.second->PostDisplay(display);
  }
  present_start &= ~(1 << (int)display);
}

int32_t HWCSidebandStreamSession::SetLayerSidebandStream(hwc2_display_t display,
                                  hwc2_layer_t layer, buffer_handle_t stream) {
  int32_t stream_id;
  HWCSidebandStream *stm;
  int new_layer;

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
  new_layer = stm->AddLayer(layer, display);

  //pickup most recent sideband stream buffer
  android::sp<SidebandStreamBuf> buf = stm->GetBuffer();
  if (buf != nullptr) {
    HWCSession::CallLayerFunction(hwc_session, display, layer, &HWCLayer::SetLayerSidebandStream, buf);
  }

  //destroy previous stream attached to the same layer
  if (new_layer) {
    std::vector<std::pair<int32_t, HWCSidebandStream*>> removeList;
    for (auto & sbstream : mSidebandStreamList) {
      if (sbstream.second != stm &&
          sbstream.second->RemoveLayer(layer) == HWC2_ERROR_NO_RESOURCES) {
        removeList.push_back(sbstream);
      }
    }

    // remove stream from list in locked context
    for (auto & sbstream : removeList) {
      mSidebandStreamList.erase(sbstream.first);
    }

    // delete stream from unlocked context
    hwc_session->locker_.Unlock();
    for (auto & sbstream : removeList) {
      delete sbstream.second;
    }
    hwc_session->locker_.Lock();
  }

  return HWC2_ERROR_NONE;
}

int32_t HWCSidebandStreamSession::DestroyLayer(hwc2_display_t display, hwc2_layer_t layer) {
  std::vector<std::pair<int32_t, HWCSidebandStream*>> removeList;
  for (auto & sbstream : mSidebandStreamList) {
    if (sbstream.second->RemoveLayer(layer) == HWC2_ERROR_NO_RESOURCES) {
      removeList.push_back(sbstream);
    }
  }

  // remove stream from list in locked context
  for (auto & sbstream : removeList) {
    mSidebandStreamList.erase(sbstream.first);
  }

  // delete stream from unlocked context
  hwc_session->locker_.Unlock();
  for (auto & sbstream : removeList) {
    delete sbstream.second;
  }
  hwc_session->locker_.Lock();

  return HWC2_ERROR_NONE;
}

int32_t HWCSidebandStreamSession::UpdateSidebandStream(HWCSidebandStream * stm) {
  color_data_pack color_data_pack_;
  csc_mat csc_mat;

  /* wait previous fence */
  if (stm->retire_fences_ >= 0) {
    int err = sync_wait(stm->retire_fences_, 1000);
    if (err < 0) {
      DLOGE("retire_fence_ error %d", err);
    }
    ::close(stm->retire_fences_);
    stm->retire_fences_ = -1;
  }

  SCOPE_LOCK(hwc_session->locker_);
  auto status = HWC2::Error::None;
  uint32_t validateMask = stm->displayValidateMask_;
  uint32_t pendingMask = stm->pendingMask_ & ~present_start;

  memcpy(&color_data_pack_, &stm->color_data_pack_, sizeof(color_data_pack_));

  /* only calculate when color data is changed */
  if (color_data_pack_.color_dirty) {
    stm->CalcCscInputData(&color_data_pack_, &csc_mat);

    for (auto & pair : stm->mLayers) {
      hwc2_layer_t layer = pair.first;
      hwc2_display_t display = pair.second;
      hwc_session->hwc_display_[display]->SetLayerCscData(layer,
                                                          csc_mat.ctm_coeff,
                                                          CSC_MATRIX_COEFF_SIZE,
                                                          csc_mat.post_bias,
                                                          CSC_BIAS_SIZE);
    }
  }

  for (auto & pair : stm->mLayers) {
    hwc2_layer_t layer = pair.first;
    hwc2_display_t display = pair.second;
    HWCSession::CallLayerFunction(hwc_session, display, layer, &HWCLayer::SetLayerSidebandStream, stm->GetBuffer());
    stm->displayValidateMask_ &= ~(1 << display );
  }

  if (validateMask) {
    for (auto & display : stm->mDisplays) {
      hwc_session->Refresh(display);
    }
    return 0;
  }

  for (auto & display : stm->mDisplays) {
    /* only update pending displays */
    if (!(pendingMask & (1 << display)))
      continue;

    if (hwc_session->hwc_display_[display]->GetGeometryChanges())
      continue;

    int32_t retire_fence;
    status = hwc_session->hwc_display_[display]->SidebandStreamPresent(&retire_fence);
    if (status != HWC2::Error::None) {
      hwc_session->Refresh(display);
      continue;
    }

    if (retire_fence >= 0) {
      if (stm->retire_fences_ < 0) {
        stm->retire_fences_ = retire_fence;
      } else {
        ::close(retire_fence);
      }
    }

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

    stm->PostDisplay(display);
  }

  return 0;
}

}  // namespace sdm
