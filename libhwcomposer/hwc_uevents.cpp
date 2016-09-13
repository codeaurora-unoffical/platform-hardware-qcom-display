
/*
 * Copyright (C) 2010 The Android Open Source Project
 * Copyright (C) 2012, 2015, The Linux Foundation. All rights reserved.
 *
 * Not a Contribution, Apache license notifications and license are
 * retained for attribution purposes only.

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
#define UEVENT_DEBUG 0
#include <hardware_legacy/uevent.h>
#include <utils/Log.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <string.h>
#include <stdlib.h>
#include "hwc_utils.h"
#include "hwc_fbupdate.h"
#include "hwc_mdpcomp.h"
#include "hwc_copybit.h"
#include "comptype.h"
#include "external.h"
#include "virtual.h"
#include "mdp_version.h"
using namespace overlay;
namespace qhwc {
#define HWC_UEVENT_SWITCH_STR  "change@/devices/virtual/switch/"
#define HWC_UEVENT_THREAD_NAME "hwcUeventThread"
#define REVERSE_CAMERA_ACK "/sys/class/switch/reverse/ack"

/* External Display states */
enum {
    EXTERNAL_OFFLINE = 0,
    EXTERNAL_ONLINE,
    EXTERNAL_PAUSE,
    EXTERNAL_RESUME
};

/* Parse uevent data for devices which we are interested */
static int getConnectedDisplay(const char* strUdata)
{
    if(strcasestr("change@/devices/virtual/switch/hdmi", strUdata))
        return HWC_DISPLAY_SECONDARY;
    if(strcasestr("change@/devices/virtual/switch/wfd", strUdata))
        return HWC_DISPLAY_VIRTUAL;
    return -1;
}

static bool getPanelResetStatus(hwc_context_t* ctx, const char* strUdata, int len)
{
    const char* iter_str = strUdata;
    if (strcasestr("change@/devices/virtual/graphics/fb0", strUdata)) {
        while(((iter_str - strUdata) <= len) && (*iter_str)) {
            char* pstr = strstr(iter_str, "PANEL_ALIVE=0");
            if (pstr != NULL) {
                ALOGE("%s: got change event in fb0 with PANEL_ALIVE=0",
                                                           __FUNCTION__);
                ctx->mPanelResetStatus = true;
                return true;
            }
            iter_str += strlen(iter_str)+1;
        }
    }
    return false;
}

/* Parse uevent data for action requested for the display */
static int getConnectedState(const char* strUdata, int len)
{
    const char* iter_str = strUdata;
    while(((iter_str - strUdata) <= len) && (*iter_str)) {
        char* pstr = strstr(iter_str, "SWITCH_STATE=");
        if (pstr != NULL) {
            return (atoi(pstr + strlen("SWITCH_STATE=")));
        }
        iter_str += strlen(iter_str)+1;
    }
    return -1;
}

static void signalReverseCameraToStart() {
    // Open a switch node to send notification to reverse camera .
    int fd = open(REVERSE_CAMERA_ACK, O_RDWR, 0);
    if (fd < 0) {
        ALOGE ("%s:not able to open %s node %s",
                __FUNCTION__, REVERSE_CAMERA_ACK, strerror(errno));
        return;
    }
    // write to the node to send ack for reverse camera
    ssize_t len = pwrite(fd, "1", strlen("1"), 0);
    if (UNLIKELY(len < 0)) {
        ALOGE ("%s: Unable to write reverse camera ack node : %s",
                __FUNCTION__, strerror(errno));
    }
    close(fd);
    return;
}

static void handleReverseCameraState(hwc_context_t* ctx,
                                           const eReverseCameraState state) {
    ctx->mDrawLock.lock();
    int dpy = HWC_DISPLAY_PRIMARY;
    switch(state) {
        case REVERSE_CAMERA_OFF:
            ctx->dpyAttr[dpy].isPause = false;
            ctx->proc->invalidate(ctx->proc);
            break;
        case REVERSE_CAMERA_ON:
            // Change the primary display state to pause not to configure
            // the overlay from next draw cycle and trigger new draw cycle
            ctx->dpyAttr[dpy].isPause = true;
            ctx->proc->invalidate(ctx->proc);
            //Wait for draw thread to signal the completion of overlay unset
            ctx->mDrawLock.wait();
            // Call display commit to release the layer's fence and the pipes
            //assoiciated with the layers
            if(!Overlay::displayCommit(ctx->dpyAttr[dpy].fd, 1)) {
                ALOGE("%s: display commit fail for primary!", __FUNCTION__);
            }
            // Acknowledge the reverse camera driver to acquire the pipes
            signalReverseCameraToStart();
            break;
        default:
            ALOGE("%s: Invalid reverse camera state %d", __FUNCTION__, state);
            break;
    }
    ctx->mDrawLock.unlock();
}

/* Parse uevent data for action requested for the display */
static bool getReverseCameraState(const char* strUdata,
                                       int len,
                                       eReverseCameraState& reverseCameraState)
{
    const char* iter_str = strUdata;
    if (strcasestr("change@/devices/virtual/switch/reverse", strUdata)) {
        while(((iter_str - strUdata) <= len) && (*iter_str)) {
            char* pstr = strstr(iter_str, "SWITCH_STATE=");
            if (pstr != NULL) {
                reverseCameraState =
                    (eReverseCameraState)(atoi(pstr + strlen("SWITCH_STATE=")));
                ALOGE_IF(UEVENT_DEBUG, "%s: reverse camera switch state %d",
                    __FUNCTION__, reverseCameraState);
                // send an update to the screen, so that when user switches from
                // rear view camera to UI, the UI will be shown immediately.
                return true;
            }
            iter_str += strlen(iter_str)+1;
        }
    }
    return false;
}

static void handleMdpArbEvent(hwc_context_t* ctx,
                              const mdp_arb_notification_event event,
                              int *fbIdx,
                              int numFbIdx,
                              int eventState)
{
    ctx->mDrawLock.lock();
    int dpy = HWC_DISPLAY_PRIMARY;
    int ret = 0;
    int i = 0;
    bool invalidate = false, waitDrawLock = false, needCommit = false;

    if (ctx->mKpiLog.MdpArb) {
        char kpiLog[24] = {0};
        snprintf(kpiLog, sizeof(kpiLog), "hwc arb[%d:%d] in",
            event, eventState);
        place_marker(kpiLog);
    }
    for (i = 0; i < numFbIdx; i++) {
        if (fbIdx[i] < 0) {
            continue;
        }
        for (dpy = 0; dpy < HWC_NUM_DISPLAY_TYPES; dpy++) {
            if (ctx->dpyAttr[dpy].fb_idx == fbIdx[i]) {
                break;
            }
        }
        if (dpy == HWC_NUM_DISPLAY_TYPES) {
            ALOGE("%s can't find i=%d fd_idx=%d", __FUNCTION__, i, fbIdx[i]);
            ctx->mDrawLock.unlock();
            return;
        }
        ALOGD_IF(isDebug(), "%s event=%d,dpy=%d,state=%d,pause=%d,optimize=%d",
            __FUNCTION__, event, dpy, eventState, ctx->dpyAttr[dpy].isPause,
            ctx->dpyAttr[dpy].inOptimizeMode);
        switch(event) {
            case MDP_ARB_NOTIFICATION_UP:
                if (ctx->dpyAttr[dpy].isPause) {
                    ctx->dpyAttr[dpy].isPause = false;
                    invalidate = true;
                }
                if ((eventState == 0) && (ctx->dpyAttr[dpy].inOptimizeMode)) {
                    // Clear optimize mode when there is event change
                    ctx->dpyAttr[dpy].inOptimizeMode = false;
                    invalidate = true;
                }
                if ((eventState == 1) && (!ctx->dpyAttr[dpy].inOptimizeMode)) {
                    ctx->dpyAttr[dpy].inOptimizeMode = true;
                    invalidate = true;
                    waitDrawLock= true;
                    needCommit = true;
                }
                break;
            case MDP_ARB_NOTIFICATION_DOWN:
                if (!ctx->dpyAttr[dpy].isPause) {
                    // Change the primary display state to pause not to
                    // configure the overlay from next draw cycle and trigger
                    // new draw cycle
                    ctx->dpyAttr[dpy].isPause = true;
                    if (ctx->dpyAttr[dpy].inOptimizeMode) {
                        // Clear optimize mode when there is event change
                        ctx->dpyAttr[dpy].inOptimizeMode = false;
                    }
                    invalidate = true;
                    waitDrawLock= true;
                    needCommit = true;
                }
                break;
            case MDP_ARB_NOTIFICATION_OPTIMIZE:
                if (!ctx->dpyAttr[dpy].inOptimizeMode) {
                    ctx->dpyAttr[dpy].inOptimizeMode = true;
                    invalidate = true;
                    waitDrawLock= true;
                    needCommit = true;
                }
                break;
            default:
                ALOGE("%s: Invalid reverse camera state %d", __FUNCTION__,
                    event);
                break;
        }
    }

    if (invalidate) {
        ctx->proc->invalidate(ctx->proc);
    }
    if (waitDrawLock) {
        // Wait for draw thread to signal the completion of overlay unset
        ctx->mDrawLock.wait();
    }
    if (needCommit) {
        // Call display commit to release the layer's fence and the
        // pipes assoiciated with the layers
        if(!Overlay::displayCommit(ctx->dpyAttr[dpy].arb_fd, 1)) {
            ALOGE("%s,%d: display commit fail for dpy=%d!",
                __FUNCTION__, __LINE__, dpy);
        }
    }
    for (i = 0; i < numFbIdx; i++) {
        if (fbIdx[i] < 0) {
            continue;
        }
        for (dpy = 0; dpy < HWC_NUM_DISPLAY_TYPES; dpy++) {
            if (ctx->dpyAttr[dpy].fb_idx == fbIdx[i]) {
                break;
            }
        }
        if (dpy == HWC_NUM_DISPLAY_TYPES) {
            ALOGE("%s invalid fb index=%d", __FUNCTION__, fbIdx[i]);
            continue;
        }
        // Acknowledge the MDP arb
        ret = ioctl(ctx->dpyAttr[dpy].arb_fd, MSMFB_ARB_ACKNOWLEDGE, &event);
        if (ret) {
            ALOGE("%s mdp arb ack fails=%d", __FUNCTION__, ret);
        } else {
             ALOGD_IF(isDebug(), "%s set mdp arb ack dpy=%d, event=%d",
                __FUNCTION__, dpy, event);
        }
    }
    if (ctx->mKpiLog.MdpArb) {
        char kpiLog[24] = {0};
        snprintf(kpiLog, sizeof(kpiLog), "hwc arb[%d:%d] out",
            event, eventState);
        place_marker(kpiLog);
    }
    ctx->mDrawLock.unlock();
}

/* Parse uevent data for action requested for the display */
static bool getMdpArbNotification(const char* strUdata,
                                  int len,
                                  mdp_arb_notification_event& event,
                                  int *fbIdx,
                                  int numFbIdx,
                                  int& eventState)
{
    const char* iter_str = strUdata;
    char iter_temp[128];
    char *p = NULL;
    bool found = false;
    char *token = NULL, *last = NULL;
    const char *delimit = ", ";
    int idx = 0, l = 0, c = 0;
    char* pstr = NULL;
    int i = 0, j = 0;

    if (strcasestr("change@/devices/virtual/mdp_arb/mdp_arb", strUdata)) {
        while(((iter_str - strUdata) <= len)) {
            memset(iter_temp, 0x00, sizeof(iter_temp));
            strlcpy(iter_temp, iter_str, 128);
            if ((strstr(iter_temp, "hwc")) != NULL) {
                if ((pstr = strstr(iter_temp, "optimize=")) != NULL) {
                    event = MDP_ARB_NOTIFICATION_OPTIMIZE;
                    p = pstr + strlen("optimize=");
                } else if ((pstr = strstr(iter_temp, "down=")) != NULL) {
                    event = MDP_ARB_NOTIFICATION_DOWN;
                    p = pstr + strlen("down=");
                } else if ((pstr = strstr(iter_temp, "up=")) != NULL) {
                    event = MDP_ARB_NOTIFICATION_UP;
                    p = pstr + strlen("up=");
                } else {
                    ALOGE("%s can't find notification string, u=%s",
                        __FUNCTION__, iter_temp);
                    continue;
                }
                l = strlen(p);
                token = strtok_r(p, delimit, &last);
                i = 0;
                c = 0;
                while((NULL != token) && (c < l)) {
                    if (!strncmp(token, "hwc", strlen("hwc"))) {
                        fbIdx[idx] = i;
                        idx++;
                        if (idx > numFbIdx) {
                            ALOGI("%s idx=%d is bigger than numFbIdx=%d",
                                __FUNCTION__, idx, numFbIdx);
                            break;
                        }
                    }
                    c += strlen(token);
                    token = strtok_r(NULL, delimit, &last);
                    i++;
                }
                found = true;
            }
            if ((pstr = strstr(iter_temp, "fb_idx=")) != NULL) {
                p = pstr + strlen("fb_idx=");
                l = strlen(p);
                token = strtok_r(p, delimit, &last);
                i = 0;
                j = 0;
                c = 0;
                while((token) && (j < idx) && (c < l)) {
                    if (i == fbIdx[j]) {
                        if (token) {
                            fbIdx[j] = atoi(token);
                        } else {
                            ALOGI("%s token is NULL, iter_str=%s,i=%d,j=%d,"\
                                "idx=%d", __FUNCTION__, iter_str, i, j, idx);
                        }
                        j++;
                    }
                    c += strlen(token);
                    token = strtok_r(NULL, delimit, &last);
                    i++;
                }
            }
            if ((pstr = strstr(iter_temp, "state=")) != NULL) {
                p = pstr + strlen("state=");
                eventState = atoi(p);
            }
            iter_str += strlen(iter_str)+1;
        }
    }
    return found;
}

static void handle_uevent(hwc_context_t* ctx, const char* udata, int len)
{
    // do not handle uevent of hdmi or wfd, if automotive mode is on
    if(ctx->mAutomotiveModeOn) {
        if (ctx->mMDPArbSuppport) {
            mdp_arb_notification_event event;
            int fb_idx[HWC_NUM_DISPLAY_TYPES] = {-1, -1, -1, -1};
            int state = -1;
            bool update = getMdpArbNotification(udata, len, event, fb_idx,
                            HWC_NUM_DISPLAY_TYPES, state);
            if (update)
                handleMdpArbEvent(ctx, event, fb_idx, HWC_NUM_DISPLAY_TYPES,
                    state);
        } else {
            eReverseCameraState reverseCameraState;
            bool state_update = getReverseCameraState(udata, len,
                                                      reverseCameraState);
            // Handle the state transition, if there is any state update
            if(state_update)
                handleReverseCameraState(ctx, reverseCameraState);
        }
       if (!ctx->mHPDEnabled)
           return;
    }

    bool bpanelReset = getPanelResetStatus(ctx, udata, len);
    if (bpanelReset) {
        ctx->proc->invalidate(ctx->proc);
        return;
    }

    int dpy = getConnectedDisplay(udata);
    if(dpy < 0) {
        ALOGD_IF(UEVENT_DEBUG, "%s: Not disp Event ", __FUNCTION__);
        return;
    }

    // If hdmi is primary, skip uevent
    if (dpy == HWC_DISPLAY_PRIMARY)
        return;

    int switch_state = getConnectedState(udata, len);

    ALOGE_IF(UEVENT_DEBUG,"%s: uevent recieved: %s switch state: %d",
             __FUNCTION__,udata, switch_state);

    switch(switch_state) {
    case EXTERNAL_OFFLINE:
        {
            /* Display not connected */
            if(!ctx->dpyAttr[dpy].connected){
                ALOGE_IF(UEVENT_DEBUG,"%s: Ignoring EXTERNAL_OFFLINE event"
                         "for display: %d", __FUNCTION__, dpy);
                break;
            }

            ctx->mDrawLock.lock();
            clearObject(ctx, dpy);
            ctx->dpyAttr[dpy].connected = false;
            ctx->dpyAttr[dpy].isActive = false;

            /* We need to send hotplug to SF only when we are disconnecting
             * (1) HDMI OR (2) proprietary WFD session */
            if(dpy == HWC_DISPLAY_SECONDARY ||
                    ctx->mVirtualonExtActive) {
                ALOGE_IF(UEVENT_DEBUG,"%s:Sending EXTERNAL OFFLINE hotplug"
                        "event", __FUNCTION__);
                ctx->proc->hotplug(ctx->proc, HWC_DISPLAY_SECONDARY,
                        EXTERNAL_OFFLINE);
                ctx->mVirtualonExtActive = false;
            }
            ctx->proc->invalidate(ctx->proc);
            ctx->mDrawLock.wait();
            // At this point all the pipes used by External have been
            // marked as UNSET.

            // Perform commit to unstage the pipes.
            if (!Overlay::displayCommit(ctx->dpyAttr[dpy].fd)) {
                ALOGE("%s: display commit fail! for %d dpy",
                        __FUNCTION__, dpy);
            }

            if(dpy == HWC_DISPLAY_SECONDARY) {
                ctx->mBasePipeSetup[dpy] = false;
                freeBasePipe(ctx, dpy);
                ctx->mSecondaryDisplay->teardown();
            } else {
                ctx->mVirtualDisplay->teardown();
            }
            ctx->mDrawLock.unlock();

            break;
        }
    case EXTERNAL_ONLINE:
        {
            /* Display already connected */
            if(ctx->dpyAttr[dpy].connected) {
                ALOGE_IF(UEVENT_DEBUG,"%s: Ignoring EXTERNAL_ONLINE event"
                         "for display: %d", __FUNCTION__, dpy);
                break;
            }

            //Force composition to give up resources like pipes and
            //close fb. For example if assertive display is going on,
            //fb2 could be open, thus connecting Layer Mixer#0 to
            //WriteBack module. If HDMI attempts to open fb1, the driver
            //will try to attach Layer Mixer#0 to HDMI INT, which will
            //fail, since Layer Mixer#0 is still connected to WriteBack.
            //This block will force composition to close fb2 in above
            //example.
            ctx->mDrawLock.lock();
            ctx->dpyAttr[dpy].isConfiguring = true;
            ctx->proc->invalidate(ctx->proc);

            ctx->mDrawLock.wait();
            ctx->mDrawLock.unlock();
            if(dpy == HWC_DISPLAY_SECONDARY) {
                if(ctx->dpyAttr[HWC_DISPLAY_VIRTUAL].connected) {
                    ALOGD_IF(UEVENT_DEBUG,"Received HDMI connection request"
                             "when WFD is active");

                    ctx->mDrawLock.lock();
                    clearObject(ctx, HWC_DISPLAY_VIRTUAL);
                    clearObject(ctx, HWC_DISPLAY_VIRTUAL);
                    ctx->dpyAttr[HWC_DISPLAY_VIRTUAL].connected = false;
                    ctx->dpyAttr[HWC_DISPLAY_VIRTUAL].isActive = false;

                    /* Need to send hotplug only when connected WFD in
                     * proprietary path */
                    if(ctx->mVirtualonExtActive) {
                        ALOGE_IF(UEVENT_DEBUG,"%s: Sending EXTERNAL OFFLINE"
                                "hotplug event", __FUNCTION__);
                        ctx->proc->hotplug(ctx->proc, HWC_DISPLAY_SECONDARY,
                                EXTERNAL_OFFLINE);
                        ctx->mVirtualonExtActive = false;
                    }
                    ctx->proc->invalidate(ctx->proc);

                    ctx->mDrawLock.wait();
                    // At this point all the pipes used by WFD(Virtual) have been
                    // marked as UNSET.
                    // Perform commit to unstage the pipes.
                    if (!Overlay::displayCommit(ctx->dpyAttr[HWC_DISPLAY_VIRTUAL].fd)) {
                        ALOGE("%s: display commit fail! for %d dpy",
                                __FUNCTION__, HWC_DISPLAY_VIRTUAL);
                    }
                    ctx->mDrawLock.unlock();

                    ctx->mVirtualDisplay->teardown();
                }
                ctx->mSecondaryDisplay->configure();
            } else {
                {
                    Locker::Autolock _l(ctx->mDrawLock);
                    /* TRUE only when we are on proprietary WFD session */
                    ctx->mVirtualonExtActive = true;
                    char property[PROPERTY_VALUE_MAX];
                    if((property_get("persist.sys.wfd.virtual",
                                                  property, NULL) > 0) &&
                       (!strncmp(property, "1", PROPERTY_VALUE_MAX ) ||
                       (!strncasecmp(property,"true", PROPERTY_VALUE_MAX )))) {
                        // This means we are on Google's WFD session
                        ctx->mVirtualonExtActive = false;
                    }
                }
                ctx->mVirtualDisplay->configure();
            }

            Locker::Autolock _l(ctx->mDrawLock);
            setupObject(ctx, dpy);
            ctx->dpyAttr[dpy].isPause = false;
            ctx->dpyAttr[dpy].connected = true;
            ctx->dpyAttr[dpy].isConfiguring = true;

            if(dpy == HWC_DISPLAY_SECONDARY ||
               ctx->mVirtualonExtActive) {
                /* External display is HDMI or non-hybrid WFD solution */
                ALOGE_IF(UEVENT_DEBUG, "%s: Sending EXTERNAL_OFFLINE ONLINE"
                         "hotplug event", __FUNCTION__);
                ctx->proc->hotplug(ctx->proc,HWC_DISPLAY_SECONDARY,
                                   EXTERNAL_ONLINE);
            } else {
                /* We wont be getting unblank for VIRTUAL DISPLAY and its
                 * always guaranteed from WFD stack that CONNECT uevent for
                 * VIRTUAL DISPLAY will be triggered before creating
                 * surface for the same. */
                ctx->dpyAttr[HWC_DISPLAY_VIRTUAL].isActive = true;
            }
            break;
        }
        case EXTERNAL_PAUSE:
            {   // pause case

                 ALOGD("%s Received Pause event",__FUNCTION__);
                 /* Display already in pause */
                 if(ctx->dpyAttr[dpy].isPause) {
                    ALOGE_IF(UEVENT_DEBUG,"%s: Ignoring EXTERNAL_PAUSE event"
                             "for display: %d", __FUNCTION__, dpy);
                    break;
                 }

                 ctx->mDrawLock.lock();
                 ctx->dpyAttr[dpy].isActive = true;
                 ctx->dpyAttr[dpy].isPause = true;
                 ctx->proc->invalidate(ctx->proc);

                 ctx->mDrawLock.wait();
                 // At this point all the pipes used by External have been
                 // marked as UNSET.
                 // Perform commit to unstage the pipes.
                 if (!Overlay::displayCommit(ctx->dpyAttr[dpy].fd)) {
                     ALOGE("%s: display commit fail! for %d dpy",
                             __FUNCTION__, dpy);
                 }
                 ctx->mDrawLock.unlock();

                 break;
            }
        case EXTERNAL_RESUME:
            {  // resume case

                ALOGD("%s Received resume event",__FUNCTION__);

                /* Display already is resumed */
                if(not ctx->dpyAttr[dpy].isPause) {
                    ALOGE_IF(UEVENT_DEBUG,"%s: Ignoring EXTERNAL_RESUME event"
                             "for display: %d", __FUNCTION__, dpy);
                    break;
                }

                //Treat Resume as Online event
                //Since external didnt have any pipes, force primary to give up
                //its pipes; we don't allow inter-mixer pipe transfers.

                ctx->mDrawLock.lock();
                ctx->dpyAttr[dpy].isConfiguring = true;
                ctx->dpyAttr[dpy].isActive = true;
                ctx->proc->invalidate(ctx->proc);

                ctx->mDrawLock.wait();
                //At this point external has all the pipes it would need.
                ctx->dpyAttr[dpy].isPause = false;
                ctx->proc->invalidate(ctx->proc);
                ctx->mDrawLock.unlock();

                break;
            }
    default:
        {
            ALOGE("%s: Invalid state to swtich:%d", __FUNCTION__, switch_state);
            break;
        }
    }
}

static void *uevent_loop(void *param)
{
    int len = 0;
    static char udata[PAGE_SIZE];
    hwc_context_t * ctx = reinterpret_cast<hwc_context_t *>(param);
    char thread_name[64] = HWC_UEVENT_THREAD_NAME;
    prctl(PR_SET_NAME, (unsigned long) &thread_name, 0, 0, 0);
    setpriority(PRIO_PROCESS, 0, HAL_PRIORITY_URGENT_DISPLAY);
    if(!uevent_init()) {
        ALOGE("%s: failed to init uevent ",__FUNCTION__);
        return NULL;
    }

    // Need to check current event state before processing the first uevent
    // in order to catch up any missing uevent
    if (checkMdpArbitratorEvent(ctx) < 0) {
        ALOGE("%s: failed to check MDP arbitrator event!!", __FUNCTION__);
    }

    while(1) {
        len = uevent_next_event(udata, sizeof(udata) - 2);
        handle_uevent(ctx, udata, len);
    }

    return NULL;
}

void init_uevent_thread(hwc_context_t* ctx)
{
    pthread_t uevent_thread;
    int ret;

    ALOGI("Initializing UEVENT Thread");
    ret = pthread_create(&uevent_thread, NULL, uevent_loop, (void*) ctx);
    if (ret) {
        ALOGE("%s: failed to create %s: %s", __FUNCTION__,
            HWC_UEVENT_THREAD_NAME, strerror(ret));
    }
}

}; //namespace
