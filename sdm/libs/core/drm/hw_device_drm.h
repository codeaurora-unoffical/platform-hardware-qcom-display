/*
* Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
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

#ifndef __HW_DEVICE_DRM_H__
#define __HW_DEVICE_DRM_H__

#include <drm_interface.h>
#include <errno.h>
#include <pthread.h>
#include <xf86drmMode.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "hw_interface.h"
#include "hw_scale_drm.h"

#define IOCTL_LOGE(ioctl, type) \
  DLOGE("ioctl %s, device = %d errno = %d, desc = %s", #ioctl, type, errno, strerror(errno))

#define UI_FBID_LIMIT 3
#define VIDEO_FBID_LIMIT 16
#define OFFLINE_ROTATOR_FBID_LIMIT 2

namespace sdm {
class HWInfoInterface;

class HWDeviceDRM : public HWInterface {
 public:
  explicit HWDeviceDRM(DisplayOrder order, BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                       HWInfoInterface *hw_info_intf);
  virtual ~HWDeviceDRM() {}
  virtual DisplayError Init();
  virtual DisplayError Deinit();

 protected:
  // From HWInterface
  virtual DisplayError GetActiveConfig(uint32_t *active_config);
  virtual DisplayError GetNumDisplayAttributes(uint32_t *count);
  virtual DisplayError GetDisplayAttributes(uint32_t index,
                                            HWDisplayAttributes *display_attributes);
  virtual DisplayError GetHWPanelInfo(HWPanelInfo *panel_info);
  virtual DisplayError SetDisplayAttributes(uint32_t index);
  virtual DisplayError SetDisplayAttributes(const HWDisplayAttributes &display_attributes);
  virtual DisplayError GetConfigIndex(char *mode, uint32_t *index);
  virtual DisplayError PowerOn();
  virtual DisplayError PowerOff();
  virtual DisplayError Doze();
  virtual DisplayError DozeSuspend();
  virtual DisplayError Standby();
  virtual DisplayError Validate(HWLayers *hw_layers);
  virtual DisplayError Commit(HWLayers *hw_layers);
  virtual DisplayError Flush();
  virtual DisplayError GetPPFeaturesVersion(PPFeatureVersion *vers);
  virtual DisplayError SetPPFeatures(PPFeaturesConfig *feature_list);
  // This API is no longer supported, expectation is to call the correct API on HWEvents
  virtual DisplayError SetVSyncState(bool enable);
  virtual void SetPageFlipState(bool enable, void *user_data);
  virtual void SetIdleTimeoutMs(uint32_t timeout_ms);
  virtual DisplayError SetDisplayMode(const HWDisplayMode hw_display_mode);
  virtual DisplayError SetRefreshRate(uint32_t refresh_rate);
  virtual DisplayError SetPanelBrightness(int level);
  virtual DisplayError CachePanelBrightness(int level);
  virtual DisplayError GetHWScanInfo(HWScanInfo *scan_info);
  virtual DisplayError GetVideoFormat(uint32_t config_index, uint32_t *video_format);
  virtual DisplayError GetMaxCEAFormat(uint32_t *max_cea_format);
  virtual DisplayError SetCursorPosition(HWLayers *hw_layers, int x, int y);
  virtual DisplayError OnMinHdcpEncryptionLevelChange(uint32_t min_enc_level);
  virtual DisplayError GetPanelBrightness(int *level);
  virtual DisplayError SetAutoRefresh(bool enable) { return kErrorNone; }
  virtual DisplayError SetS3DMode(HWS3DMode s3d_mode);
  virtual DisplayError SetScaleLutConfig(HWScaleLutInfo *lut_info);
  virtual DisplayError SetMixerAttributes(const HWMixerAttributes &mixer_attributes);
  virtual DisplayError GetMixerAttributes(HWMixerAttributes *mixer_attributes);
  virtual DisplayError EnablePllUpdate(int32_t enable) { return kErrorNotSupported; };
  virtual DisplayError UpdateDisplayPll(int32_t ppm) { return kErrorNotSupported; };

  enum {
    kHWEventVSync,
    kHWEventBlank,
  };

  static const int kMaxStringLength = 1024;
  static const int kNumPhysicalDisplays = 2;
  static const int kMaxSysfsCommandLength = 12;
  static constexpr const char *kBrightnessNode =
    "/sys/class/backlight/panel0-backlight/brightness";

  DisplayError SetFormat(const LayerBufferFormat &source, uint32_t *target);
  DisplayError SetStride(HWDeviceType device_type, LayerBufferFormat format, uint32_t width,
                         uint32_t *target);
  DisplayError PopulateDisplayAttributes();
  void PopulateHWPanelInfo();
  void GetHWDisplayPortAndMode();
  void GetHWPanelMaxBrightness();
  void ResetDisplayParams();
  bool EnableHotPlugDetection(int enable);
  virtual void UpdateMixerAttributes();
  void InitializeConfigs();
  void SetBlending(const LayerBlending &source, sde_drm::DRMBlendType *target);
  void SetSrcConfig(const LayerBuffer &input_buffer, uint32_t *config);
  void SelectCscType(const LayerBuffer &input_buffer, sde_drm::DRMCscType *type);
  void SetRect(const LayerRect &source, sde_drm::DRMRect *target);
  DisplayError DefaultCommit(HWLayers *hw_layers);
  DisplayError AtomicCommit(HWLayers *hw_layers);
  void SetupAtomic(HWLayers *hw_layers, bool validate);
  void SetUsrCscConfig(LayerBuffer &input_buffer, sde_drm_csc_v1 *csc_data);

  class Registry {
   public:
    explicit Registry(BufferAllocator *buffer_allocator);
    // Call at the end of draw cycle to clear the next slot for business
    void UnregisterNext();
    // Call on display disconnect to release all gem handles and fb_ids
    void Clear();
    // Finds an fb_id corresponding to an fd in current map

    void Register(HWLayers *hw_layers);
    // Create the fd_id for the given buffer.
    int CreateFbId(LayerBuffer *buffer, uint32_t *fb_id);
    // Find handle_id in the layer map. Else create fb_id and add <handle_id,fb_id> in map.
    void MapBufferToFbId(Layer* layer, LayerBuffer* buffer);
    // Find fb_id for given handle_id in the layer map.
    uint32_t GetFbId(Layer *layer, uint64_t handle_id);


   private:
    static const int kCycleDelay = 2;  // N cycle delay before destroy
    // fd to fb_id map. fd is used as key only for a single draw cycle between
    // prepare and commit. It should not be used for caching in future due to fd recycling
    std::unordered_map<int, uint32_t> hashmap_[kCycleDelay] {};
    int current_index_ = 0;
    bool disable_fbid_cache_ = false;
    std::unordered_map<uint64_t, std::shared_ptr<LayerBufferObject>> output_buffer_map_ {};
    BufferAllocator *buffer_allocator_ = {};
    uint8_t fbid_cache_limit_ = UI_FBID_LIMIT;
  };

  HWResourceInfo hw_resource_ = {};
  HWPanelInfo hw_panel_info_ = {};
  HWInfoInterface *hw_info_intf_ = {};
  BufferSyncHandler *buffer_sync_handler_ = {};
  DisplayOrder display_order_ = {};
  HWDeviceType device_type_ = {};
  const char *device_name_ = {};
  bool synchronous_commit_ = false;
  bool enable_pflip_event_ = false;
  void *pflip_user_data_ = {};
  HWDisplayAttributes display_attributes_ = {};
  int64_t release_fence_ = -1;
  int64_t retire_fence_ = -1;
  HWMixerAttributes mixer_attributes_ = {};
  sde_drm::DRMManagerInterface *drm_mgr_intf_ = {};
  sde_drm::DRMAtomicReqInterface *drm_atomic_intf_ = {};
  sde_drm::DRMDisplayToken token_ = {};
  drmModeModeInfo current_mode_ = {};
  bool default_mode_ = false;
  sde_drm::DRMConnectorInfo connector_info_ = {};
  std::string interface_str_ = "DSI";
  HWScaleDRM *hw_scale_ = {};
  Registry registry_;
};

}  // namespace sdm

#endif  // __HW_DEVICE_DRM_H__
