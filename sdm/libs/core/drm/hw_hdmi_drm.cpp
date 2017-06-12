/*
* Copyright (c) 2015 - 2017, The Linux Foundation. All rights reserved.
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

#include <drm_lib_loader.h>
#include <drm_master.h>
#include <drm_res_mgr.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <utils/debug.h>
#include <utils/sys.h>
#include <utils/formats.h>

#include <vector>
#include <map>
#include <utility>

#include "hw_hdmi_drm.h"

#define __CLASS__ "HWHDMIDRM"

#define  DRM_BIT_RGB 20
#define  DRM_BIT_YUV 21

using drm_utils::DRMMaster;
using drm_utils::DRMResMgr;
using drm_utils::DRMLibLoader;
using drm_utils::DRMBuffer;
using sde_drm::GetDRMManager;
using sde_drm::DestroyDRMManager;
using sde_drm::DRMDisplayType;
using sde_drm::DRMDisplayToken;
using sde_drm::DRMConnectorInfo;
using sde_drm::DRMPPFeatureInfo;
using sde_drm::DRMOps;
using sde_drm::DRMTopology;

namespace sdm {

HWHDMIDRM::HWHDMIDRM(BufferSyncHandler *buffer_sync_handler, BufferAllocator *buffer_allocator,
                     HWInfoInterface *hw_info_intf)
  : HWDeviceDRM(buffer_sync_handler, buffer_allocator, hw_info_intf),
  active_config_index_(0) {
  HWDeviceDRM::device_type_ = kDeviceHDMI;
  HWDeviceDRM::device_name_ = "HDMI Display Device";
}

// TODO(user) : split function in base class and avoid code duplicacy
// by using base implementation for this basic stuff
DisplayError HWHDMIDRM::Init() {
  DisplayError error = kErrorNone;

  default_mode_ = (DRMLibLoader::GetInstance()->IsLoaded() == false);

  if (!default_mode_) {
    DRMMaster *drm_master = {};
    int dev_fd = -1;
    DRMMaster::GetInstance(&drm_master);
    drm_master->GetHandle(&dev_fd);
    DRMLibLoader::GetInstance()->FuncGetDRMManager()(dev_fd, &drm_mgr_intf_);
    if (drm_mgr_intf_->RegisterDisplay(DRMDisplayType::TV, &token_)) {
      DLOGE("RegisterDisplay failed");
      return kErrorResources;
    }

    drm_mgr_intf_->CreateAtomicReq(token_, &drm_atomic_intf_);
    drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
    InitializeConfigs();
   }
  hw_info_intf_->GetHWResourceInfo(&hw_resource_);
  // TODO(user): In future, remove has_qseed3 member, add version and pass version to constructor
  if (hw_resource_.has_qseed3) {
    hw_scale_ = new HWScaleDRM(HWScaleDRM::Version::V2);
  }

  if (error != kErrorNone) {
    return error;
  }

  return error;
}

DisplayError HWHDMIDRM::GetNumDisplayAttributes(uint32_t *count) {
  *count = connector_info_.num_modes;
  if (*count <= 0) {
    return kErrorHardware;
  }

  return kErrorNone;
}

DisplayError HWHDMIDRM::GetActiveConfig(uint32_t *active_config_index) {
  *active_config_index = active_config_index_;
  return kErrorNone;
}

DisplayError HWHDMIDRM::GetDisplayAttributes(uint32_t index,
                                            HWDisplayAttributes *display_attributes) {
  *display_attributes = display_attributes_;
  drmModeModeInfo mode = {};
  uint32_t mm_width = 0;
  uint32_t mm_height = 0;
  DRMTopology topology = DRMTopology::SINGLE_LM;

  if (default_mode_) {
    DRMResMgr *res_mgr = nullptr;
    int ret = DRMResMgr::GetInstance(&res_mgr);
    if (ret < 0) {
      DLOGE("Failed to acquire DRMResMgr instance");
      return kErrorResources;
    }

    res_mgr->GetMode(&mode);
    res_mgr->GetDisplayDimInMM(&mm_width, &mm_height);
  } else {
    mode = connector_info_.modes[index];
    mm_width = mode.hdisplay;
    mm_height = mode.vdisplay;
    topology = connector_info_.topology;
  }

  display_attributes-> x_pixels = mode.hdisplay;
  display_attributes->y_pixels = mode.vdisplay;
  display_attributes->fps = mode.vrefresh;
  display_attributes->vsync_period_ns = UINT32(1000000000L / display_attributes->fps);

  /*
              Active                 Front           Sync           Back
              Region                 Porch                          Porch
     <-----------------------><----------------><-------------><-------------->
     <----- [hv]display ----->
     <------------- [hv]sync_start ------------>
     <--------------------- [hv]sync_end --------------------->
     <-------------------------------- [hv]total ----------------------------->
   */

  display_attributes->v_front_porch = mode.vsync_start - mode.vdisplay;
  display_attributes->v_pulse_width = mode.vsync_end - mode.vsync_start;
  display_attributes->v_back_porch = mode.vtotal - mode.vsync_end;
  display_attributes->v_total = mode.vtotal;

  display_attributes->h_total = mode.htotal;
  uint32_t h_blanking = mode.htotal - mode.hdisplay;
  display_attributes->is_device_split =
      (topology == DRMTopology::DUAL_LM || topology == DRMTopology::DUAL_LM_MERGE);
  display_attributes->h_total += display_attributes->is_device_split ? h_blanking : 0;

  display_attributes->x_dpi = (FLOAT(mode.hdisplay) * 25.4f) / FLOAT(mm_width);
  display_attributes->y_dpi = (FLOAT(mode.vdisplay) * 25.4f) / FLOAT(mm_height);

  return kErrorNone;
}

DisplayError HWHDMIDRM::SetDisplayAttributes(uint32_t index) {
  // TODO check if index will start from 0? then >=
  if (index >= connector_info_.num_modes) {
    return kErrorNotSupported;
  }

  active_config_index_ = index;
  // Get the display attributes for current active config index
  GetDisplayAttributes(active_config_index_, &display_attributes_);

  frame_rate_ = display_attributes_.fps;
  current_mode_ = connector_info_.modes[index];

  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_MODE, token_.crtc_id, &connector_info_.modes[index]);
  drm_atomic_intf_->Perform(DRMOps::CRTC_SET_OUTPUT_FENCE_OFFSET, token_.crtc_id, 1);

  // TODO(user): Enable this and remove the one in SetupAtomic() onces underruns are fixed
  // drm_atomic_intf_->Perform(DRMOps::CRTC_SET_ACTIVE, token_.crtc_id, 1);
  // Commit to setup pipeline with mode, which then tells us the topology etc
  if (drm_atomic_intf_->Commit(true /* synchronous */)) {
    DLOGE("Setting up CRTC %d, Connector %d for %s failed", token_.crtc_id, token_.conn_id,
        device_name_);
  return kErrorResources;
  }

  // Reload connector info for updated info after 1st commit
  drm_mgr_intf_->GetConnectorInfo(token_.conn_id, &connector_info_);
  DLOGI("Setup CRTC %d, Connector %d for %s", token_.crtc_id, token_.conn_id, device_name_);

  PopulateDisplayAttributes();
  PopulateHWPanelInfo();
  UpdateMixerAttributes();
  return kErrorNone;
}

DisplayError HWHDMIDRM::GetConfigIndex(char *mode, uint32_t *index) {

  uint32_t width = 0, height = 0, fps = 0, format = 0;
  std::string str(mode);

  //mode should be in width:height:fps:format
  //TODO: it is not fully robust, User needs to provide in above format only
  if(str.length()!=0) {
    width = UINT32(stoi(str));
    height = UINT32(stoi(str.substr(str.find(':') + 1)));
    std::string str3 = str.substr(str.find(':') + 1);
    fps = UINT32(stoi(str3.substr(str3.find(':')  + 1)));
    std::string str4 = str3.substr(str3.find(':') + 1);
    format = UINT32(stoi(str4.substr(str4.find(':') + 1)));
  }
  for (size_t idex = 0; idex < connector_info_.num_modes; idex ++) {
    if ((height == connector_info_.modes[idex].vdisplay) &&
        (width == connector_info_.modes[idex].hdisplay) &&
        (fps == connector_info_.modes[idex].vrefresh)) {

      if((format>>1)&(connector_info_.modes[idex].flags >> DRM_BIT_YUV))
        *index = UINT32(idex);

      if(format & (connector_info_.modes[idex].flags >> DRM_BIT_RGB))
        *index = UINT32(idex);
	}
  }
  return kErrorNone;
}

DisplayError HWHDMIDRM::Validate(HWLayers *hw_layers) {
  HWDeviceDRM::ResetDisplayParams();

  return HWDeviceDRM::Validate(hw_layers);
}

DisplayError HWHDMIDRM::Commit(HWLayers *hw_layers) {
  return HWDeviceDRM::Commit(hw_layers);
}

}  // namespace sdm

