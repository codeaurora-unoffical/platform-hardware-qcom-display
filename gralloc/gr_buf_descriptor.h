/*
 * Copyright (c) 2016-2018, The Linux Foundation. All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of The Linux Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef __GR_BUF_DESCRIPTOR_H__
#define __GR_BUF_DESCRIPTOR_H__

#include <hardware/gralloc1.h>
#include <atomic>

namespace gralloc {
class BufferDescriptor {
 public:
  BufferDescriptor() : id_(next_id_++) {}

  BufferDescriptor(int w, int h, int f)
      : width_(w),
        height_(h),
        format_(f),
        usage_(GRALLOC1_CONSUMER_USAGE_NONE),
        id_(next_id_++) {}

  BufferDescriptor(int w, int h, int f, 
                   uint64_t usage)
      : width_(w),
        height_(h),
        format_(f),
        usage_(usage),
        id_(next_id_++) {}

  void SetUsage(uint64_t usage) { usage_ |= usage; }

  void SetDimensions(int w, int h) {
    width_ = w;
    height_ = h;
  }

  void SetColorFormat(int format) { format_ = format; }

  void SetLayerCount(uint32_t layer_count) { layer_count_ = layer_count; }

  uint64_t GetUsage() const { return usage_; }

  int GetWidth() const { return width_; }

  int GetHeight() const { return height_; }

  int GetFormat() const { return format_; }

  uint32_t GetLayerCount() const { return layer_count_; }

  uint64_t GetId() const { return id_; }

 private:
  int width_ = -1;
  int height_ = -1;
  int format_ = -1;
  uint32_t layer_count_ = 1;
  uint64_t usage_ = 0;
  const uint64_t id_ = 0;
  static std::atomic<gralloc1_buffer_descriptor_t> next_id_;
};
};      // namespace gralloc
#endif  // __GR_BUF_DESCRIPTOR_H__
