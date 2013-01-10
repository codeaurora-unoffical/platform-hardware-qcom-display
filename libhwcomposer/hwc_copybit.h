/*
 * Copyright (C) 2010 The Android Open Source Project
 * Copyright (C) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * Not a Contribution, Apache license notifications and license are retained
 * for attribution purposes only.
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
#ifndef HWC_COPYBIT_H
#define HWC_COPYBIT_H
#include "hwc_utils.h"

#define LIKELY( exp )       (__builtin_expect( (exp) != 0, true  ))
#define UNLIKELY( exp )     (__builtin_expect( (exp) != 0, false ))

namespace qhwc {

class CopyBit {
public:
    CopyBit();
    ~CopyBit();
    // API to get copybit engine(non static)
    struct copybit_device_t *getCopyBitDevice();
    //Sets up members and prepares copybit if conditions are met
    bool prepare(hwc_context_t *ctx, hwc_display_contents_1_t *list,
                                                                   int dpy);
    //Draws layer if the layer is set for copybit in prepare
    bool draw(hwc_context_t *ctx, hwc_display_contents_1_t *list,
                                                        int dpy, int* fd);
    // Helper functions for copybit composition
    int  drawLayerUsingCopybit(hwc_context_t *dev, hwc_layer_1_t *layer,
                                       private_handle_t *renderBuffer, int dpy);

    // trigger the copybit composition
    int finish();
    // resets the values
    void reset();
private:
    // holds the copybit device
    struct copybit_device_t *mEngine;
    bool canUseCopybitForYUV (hwc_context_t *ctx);
    bool canUseCopybitForRGB (hwc_context_t *ctx,
                                     hwc_display_contents_1_t *list, int dpy);
    bool validateParams (hwc_context_t *ctx,
                                const hwc_display_contents_1_t *list);
    //Flags if this feature is on.
    bool mIsModeOn;
    // flag that indicates whether CopyBit composition is enabled for this cycle
    bool mCopyBitDraw;

    unsigned int getRGBRenderingArea
                            (const hwc_display_contents_1_t *list);

    void getLayerResolution(const hwc_layer_1_t* layer,
                                   unsigned int &width, unsigned int& height);
};

}; //namespace qhwc

#endif //HWC_COPYBIT_H
