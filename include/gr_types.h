/*
* Copyright (c) 2020, The Linux Foundation. All rights reserved.
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

#ifndef __GR_TYPE_H__
#define __GR_TYPE_H__
#include <inttypes.h>
/**
 * Buffer usage definitions.
 */
enum BufferUsage : uint64_t {
    /** bit 0-3 is an enum */
    CPU_READ_MASK   = 0xfULL,
    /** buffer is never read by CPU */
    CPU_READ_NEVER  = 0,
    /** buffer is rarely read by CPU */
    CPU_READ_RARELY = 2,
    /** buffer is often read by CPU */
    CPU_READ_OFTEN  = 3,

    /** bit 4-7 is an enum */
    CPU_WRITE_MASK   = 0xfULL << 4,
    /** buffer is never written by CPU */
    CPU_WRITE_NEVER  = 0 << 4,
    /** buffer is rarely written by CPU */
    CPU_WRITE_RARELY = 2 << 4,
    /** buffer is often written by CPU */
    CPU_WRITE_OFTEN  = 3 << 4,

    /** buffer is used as a GPU texture */
    GPU_TEXTURE       = 1ULL << 8,

    /** buffer is used as a GPU render target */
    GPU_RENDER_TARGET = 1ULL << 9,

    /** bit 10 must be zero */

    /** buffer is used as a composer HAL overlay layer */
    COMPOSER_OVERLAY  = 1ULL << 11,
    /** buffer is used as a composer HAL client target */
    COMPOSER_CLIENT_TARGET = 1ULL << 12,

    /** bit 13 must be zero */

    /**
     * Buffer is allocated with hardware-level protection against copying the
     * contents (or information derived from the contents) into unprotected
     * memory.
     */
    PROTECTED         = 1ULL << 14,

    /** buffer is used as a hwcomposer HAL cursor layer */
    COMPOSER_CURSOR   = 1ULL << 15,

    /** buffer is used as a video encoder input */
    VIDEO_ENCODER     = 1ULL << 16,

    /** buffer is used as a camera HAL output */
    CAMERA_OUTPUT     = 1ULL << 17,

    /** buffer is used as a camera HAL input */
    CAMERA_INPUT      = 1ULL << 18,

    /** bit 19 must be zero */

    /** buffer is used as a renderscript allocation */
    RENDERSCRIPT      = 1ULL << 20,

    /** bit 21 must be zero */

    /** buffer is used as a video decoder output */
    VIDEO_DECODER     = 1ULL << 22,

    /** buffer is used as a sensor direct report output */
    SENSOR_DIRECT_DATA = 1ULL << 23,

    /**
     * buffer is used as as an OpenGL shader storage or uniform
     * buffer object
     */
    GPU_DATA_BUFFER   = 1ULL << 24,
	/** buffer is used as a cube map texture */
	GPU_CUBE_MAP          = 1ULL << 25,
	
	/** buffer contains a complete mipmap hierarchy */
	GPU_MIPMAP_COMPLETE   = 1ULL << 26,
	
	/* bits 27 and 32-47 must be zero and are reserved for future versions */

    /** bits 25-27 must be zero and are reserved for future versions */
    /** bits 28-31 are reserved for vendor extensions */
    VENDOR_MASK       = 0xfULL << 28,

    /** bits 32-47 must be zero and are reserved for future versions */
    /** bits 48-63 are reserved for vendor extensions */
    VENDOR_MASK_HI    = 0xffffULL << 48,
};


//typedef int32_t Transform;
//typedef int32_t Dataspace;
//typedef int32_t ColorMode;
//typedef int32_t ColorTransform;
//typedef int32_t Hdr;
#endif
