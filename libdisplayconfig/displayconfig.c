/*
* Copyright (c) 2018 The Linux Foundation. All rights reserved.
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

#include "displayconfig.h"

// ----------------------------------------------------------------------------
// Functions for linking dynamically
// ----------------------------------------------------------------------------
int minHdcpEncryptionLevelChanged(int display_id, const uint32_t min_enc_level)
{
    const char sysfs_node[256] = "/sys/devices/virtual/hdcp/msm_hdcp/min_level_change";
    int sysfs_node_fd, ret;
    char enc_level[32] = { 0 };
    size_t wr_cnt;

    sysfs_node_fd = open(sysfs_node, O_WRONLY);
    if (sysfs_node_fd < 1) {
        fprintf(stderr, "can't open file %s: %d, %s\n",
            sysfs_node, errno, strerror(errno));
        return errno;
    }

    snprintf(enc_level, sizeof(enc_level), "%d", min_enc_level);
    wr_cnt = strlen(enc_level);

    ret = write(sysfs_node_fd, (const void *)enc_level, wr_cnt);
    if (ret != wr_cnt) {
        if (ret == -1) {
            fprintf(stderr, "write failed, %d, %s\n", errno, strerror(errno));
	    return errno;
        } else {
            fprintf(stderr, "%s: expected write count=%d, actual=%d\n",
                 __func__, wr_cnt, ret);
            return -EINVAL;
        }
    }

    ret = close(sysfs_node_fd);
    if (ret) {
        fprintf(stderr, "can't close file: %s, %d, %s\n", sysfs_node, errno, strerror(errno));
        return ret;
    }

    return 0;
}

