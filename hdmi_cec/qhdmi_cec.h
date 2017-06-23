/*
* Copyright (c) 2017 The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation. nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
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
#ifndef QHDMI_CEC_H
#define QHDMI_CEC_H

#include <hardware/hdmi_cec.h>
#include <utils/RefBase.h>
#include <sys/poll.h>
#include <sys/resource.h>
#include <sys/prctl.h>
#include <malloc.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/queue.h>
#include <linux/netlink.h>
#include <thread>
#include <vector>
#include <mutex>

namespace qhdmicec {

#define SYSFS_BASE  "/sys/devices/virtual/graphics/fb"
#define MAX_PATH_LENGTH  128
#define MAX_STRING_LENGTH 1024
#define UEVENT_SWITCH_HDMI "change@/devices/virtual/switch/hdmi"
#define FB_PATH "/sys/devices/virtual/graphics/fb"


struct cec_callback_t {
    // Function in HDMI service to call back on CEC messages
    event_callback_t callback_func;
    // This stores the object to pass back to the framework
    void* callback_arg;

};

struct cec_context_t {
    hdmi_cec_device_t device;    // Device for HW module
    cec_callback_t callback;     // Struct storing callback object
    bool enabled;
    bool arc_enabled;
    bool system_control;         // If true, HAL/driver handle CEC messages
    int fb_num;                  // Framebuffer node for HDMI
    char fb_sysfs_path[MAX_PATH_LENGTH];
    hdmi_port_info *port_info;   // HDMI port info

    // Logical address is stored in an array, the index of the array is the
    // logical address and the value in the index shows whether it is set or not
    int logical_address[CEC_ADDR_BROADCAST];
    int version;
    uint32_t vendor_id;

    std::vector<pollfd> poll_fds_;              // poll fds for cec message monitor and exit signal
                                                // on cec message monitor thread
    int exit_fd_;
    bool cec_exit_thread_;
    bool hdmi_plugin_monitor_exit = true;       // hdmi plugin monitor thread will exit on true
    std::mutex hdmi_plugin_monitor_exit_mutex;  // hdmi plugin monitor variable mutex
    std::thread *hdmi_plugin_monitor = NULL;    // hdmi plugin monitor thread variable
    int uevent_fd = -1;                         // uevent file descriptor used to monitor hdmi uevents

    const int kThreadPriorityUrgent = -9;
};

void cec_receive_message(cec_context_t *ctx, char *msg, ssize_t len);
void cec_hdmi_hotplug(cec_context_t *ctx, int connected);

}; //namespace
#endif /* end of include guard: QHDMI_CEC_H */
