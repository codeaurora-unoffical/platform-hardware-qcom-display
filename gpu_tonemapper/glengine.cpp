/*
 * Copyright (c) 2016, The Linux Foundation. All rights reserved.
 * Not a Contribution.
 *
 * Copyright 2015 The Android Open Source Project
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

#include "glengine.h"
#include <utils/Log.h>
#include "engine.h"

void checkGlError(const char *, int);
void checkEglError(const char *, int);

static EGLDisplay eglDisplay;
static EGLContext eglContext;
static EGLSurface eglSurface;

static bool isEngineInitialized = false;
static int refCount = 0;

//-----------------------------------------------------------------------------
// Make Current
void engine_bind()
//-----------------------------------------------------------------------------
{
  EGL(eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext));
}

//-----------------------------------------------------------------------------
// initialize GL
//
bool engine_initialize()
//-----------------------------------------------------------------------------
{
  if (isEngineInitialized) {
    refCount++;
    return true;
  }

  EGLBoolean result = false;

  // display
  eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  EGL(eglBindAPI(EGL_OPENGL_ES_API));

  // initialize
  EGL(eglInitialize(eglDisplay, 0, 0));

  // config
  EGLConfig eglConfig;
  EGLint eglConfigAttribList[] = {EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
                                  EGL_RED_SIZE,     8,
                                  EGL_GREEN_SIZE,   8,
                                  EGL_BLUE_SIZE,    8,
                                  EGL_ALPHA_SIZE,   8,
                                  EGL_NONE};
  int numConfig = 0;
  EGL(eglChooseConfig(eglDisplay, eglConfigAttribList, &eglConfig, 1, &numConfig));

  // context
  EGLint eglContextAttribList[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
  eglContext = eglCreateContext(eglDisplay, eglConfig, NULL, eglContextAttribList);

  // surface
  EGLint eglSurfaceAttribList[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE, EGL_NONE};
  eglSurface = eglCreatePbufferSurface(eglDisplay, eglConfig, eglSurfaceAttribList);

  result = (EGL_TRUE == eglMakeCurrent(eglDisplay, eglSurface, eglSurface, eglContext));

  isEngineInitialized = result;
  if (isEngineInitialized)
    refCount = 1;

  ALOGI("In %s result = %d context = %p", __FUNCTION__, result, (void *)eglContext);

  return result;
}

//-----------------------------------------------------------------------------
// Shutdown.
void engine_shutdown()
//-----------------------------------------------------------------------------
{
  if (!isEngineInitialized)
    return;

  if (refCount > 1) {
    refCount--;
    return;
  }

  EGL(eglMakeCurrent(eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
  EGL(eglDestroySurface(eglDisplay, eglSurface));
  EGL(eglDestroyContext(eglDisplay, eglContext));
  EGL(eglTerminate(eglDisplay));
  eglDisplay = EGL_NO_DISPLAY;
  eglContext = EGL_NO_CONTEXT;
  eglSurface = EGL_NO_SURFACE;
  isEngineInitialized = false;
  refCount = 0;
}

//-----------------------------------------------------------------------------
void engine_deleteInputBuffer(unsigned int id)
//-----------------------------------------------------------------------------
{
  if (id != 0) {
    GL(glDeleteTextures(1, &id));
  }
}

//-----------------------------------------------------------------------------
void engine_deleteProgram(unsigned int id)
//-----------------------------------------------------------------------------
{
  if (id != 0) {
    GL(glDeleteProgram(id));
  }
}

//-----------------------------------------------------------------------------
unsigned int engine_load3DTexture(void *colorMapData, int sz, int format)
//-----------------------------------------------------------------------------
{
  GLuint texture = 0;
  GL(glGenTextures(1, &texture));
  GL(glBindTexture(GL_TEXTURE_3D, texture));
  GL(glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
  GL(glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
  GL(glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE));
  GL(glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
  GL(glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

  GL(glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB10_A2, sz, sz, sz, 0, GL_RGBA,
                  GL_UNSIGNED_INT_2_10_10_10_REV, colorMapData));

  return texture;
}
//-----------------------------------------------------------------------------
unsigned int engine_load1DTexture(void *data, int sz, int format)
//-----------------------------------------------------------------------------
{
  GLuint texture = 0;
  if ((data != 0) && (sz != 0)) {
    GL(glGenTextures(1, &texture));
    GL(glBindTexture(GL_TEXTURE_2D, texture));
    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GL(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    GL(glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB10_A2, sz, 1, 0, GL_RGBA,
                    GL_UNSIGNED_INT_2_10_10_10_REV, data));
  }
  return texture;
}

//-----------------------------------------------------------------------------
void dumpShaderLog(int shader)
//-----------------------------------------------------------------------------
{
  int success;
  GLchar infoLog[512];
  GL(glGetShaderiv(shader, GL_COMPILE_STATUS, &success));
  if (!success) {
    glGetShaderInfoLog(shader, 512, NULL, infoLog);
    ALOGI("Shader Failed to compile: %s\n", infoLog);
  }
}

//-----------------------------------------------------------------------------
GLuint engine_loadProgram(int vertexEntries, const char **vertex, int fragmentEntries,
                          const char **fragment)
//-----------------------------------------------------------------------------
{
  GLuint progId = glCreateProgram();

  int vertId = glCreateShader(GL_VERTEX_SHADER);
  int fragId = glCreateShader(GL_FRAGMENT_SHADER);

  GL(glShaderSource(vertId, vertexEntries, vertex, 0));
  GL(glCompileShader(vertId));
  dumpShaderLog(vertId);

  GL(glShaderSource(fragId, fragmentEntries, fragment, 0));
  GL(glCompileShader(fragId));
  dumpShaderLog(fragId);

  GL(glAttachShader(progId, vertId));
  GL(glAttachShader(progId, fragId));

  GL(glLinkProgram(progId));

  GL(glDetachShader(progId, vertId));
  GL(glDetachShader(progId, fragId));

  GL(glDeleteShader(vertId));
  GL(glDeleteShader(fragId));

  return progId;
}

//-----------------------------------------------------------------------------
void WaitOnNativeFence(int fd)
//-----------------------------------------------------------------------------
{
  if (fd != -1) {
    EGLint attribs[] = {EGL_SYNC_NATIVE_FENCE_FD_ANDROID, fd, EGL_NONE};

    EGLSyncKHR sync = eglCreateSyncKHR(eglDisplay, EGL_SYNC_NATIVE_FENCE_ANDROID, attribs);

    if (sync == EGL_NO_SYNC_KHR) {
      ALOGE("%s - Failed to Create sync from source fd", __FUNCTION__);
    } else {
      // the gpu will wait for this sync - not this cpu thread.
      EGL(eglWaitSyncKHR(eglDisplay, sync, 0));
      EGL(eglDestroySyncKHR(eglDisplay, sync));
    }
  }
}

//-----------------------------------------------------------------------------
int CreateNativeFence()
//-----------------------------------------------------------------------------
{
  int fd = -1;

  EGLSyncKHR sync = eglCreateSyncKHR(eglDisplay, EGL_SYNC_NATIVE_FENCE_ANDROID, NULL);
  GL(glFlush());
  if (sync == EGL_NO_SYNC_KHR) {
    ALOGE("%s - Failed to Create Native Fence sync", __FUNCTION__);
  } else {
    fd = eglDupNativeFenceFDANDROID(eglDisplay, sync);
    if (fd == EGL_NO_NATIVE_FENCE_FD_ANDROID) {
      ALOGE("%s - Failed to dup sync", __FUNCTION__);
    }
    EGL(eglDestroySyncKHR(eglDisplay, sync));
  }

  return fd;
}

//-----------------------------------------------------------------------------
void engine_setDestination(int id, int x, int y, int w, int h)
//-----------------------------------------------------------------------------
{
  GL(glBindFramebuffer(GL_FRAMEBUFFER, id));
  GL(glViewport(x, y, w, h));
}

//-----------------------------------------------------------------------------
void engine_setProgram(int id)
//-----------------------------------------------------------------------------
{
  GL(glUseProgram(id));
}

//-----------------------------------------------------------------------------
void engine_set2DInputBuffer(int binding, unsigned int id)
//-----------------------------------------------------------------------------
{
  GL(glActiveTexture(GL_TEXTURE0 + binding));
  GL(glBindTexture(GL_TEXTURE_2D, id));
}

//-----------------------------------------------------------------------------
void engine_set3DInputBuffer(int binding, unsigned int id)
//-----------------------------------------------------------------------------
{
  GL(glActiveTexture(GL_TEXTURE0 + binding));
  GL(glBindTexture(GL_TEXTURE_3D, id));
}

//-----------------------------------------------------------------------------
void engine_setExternalInputBuffer(int binding, unsigned int id)
//-----------------------------------------------------------------------------
{
  GL(glActiveTexture(GL_TEXTURE0 + binding));
  GL(glBindTexture(0x8D65, id));
}

//-----------------------------------------------------------------------------
int engine_blit(int srcFenceFd)
//-----------------------------------------------------------------------------
{
  int fd = -1;
  WaitOnNativeFence(srcFenceFd);
  float fullscreen_vertices[]{0.0f, 2.0f, 0.0f, 0.0f, 2.0f, 0.0f};
  GL(glEnableVertexAttribArray(0));
  GL(glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, fullscreen_vertices));
  GL(glDrawArrays(GL_TRIANGLES, 0, 3));
  fd = CreateNativeFence();
  GL(glFlush());
  return fd;
}

//-----------------------------------------------------------------------------
void checkGlError(const char *file, int line)
//-----------------------------------------------------------------------------
{
  for (GLint error = glGetError(); error; error = glGetError()) {
    char *pError;
    switch (error) {
      case GL_NO_ERROR:
        pError = (char *)"GL_NO_ERROR";
        break;
      case GL_INVALID_ENUM:
        pError = (char *)"GL_INVALID_ENUM";
        break;
      case GL_INVALID_VALUE:
        pError = (char *)"GL_INVALID_VALUE";
        break;
      case GL_INVALID_OPERATION:
        pError = (char *)"GL_INVALID_OPERATION";
        break;
      case GL_OUT_OF_MEMORY:
        pError = (char *)"GL_OUT_OF_MEMORY";
        break;
      case GL_INVALID_FRAMEBUFFER_OPERATION:
        pError = (char *)"GL_INVALID_FRAMEBUFFER_OPERATION";
        break;

      default:
        ALOGE("glError (0x%x) %s:%d\n", error, file, line);
        return;
    }

    ALOGE("glError (%s) %s:%d\n", pError, file, line);
    return;
  }
  return;
}

//-----------------------------------------------------------------------------
void checkEglError(const char *file, int line)
//-----------------------------------------------------------------------------
{
  for (int i = 0; i < 5; i++) {
    const EGLint error = eglGetError();
    if (error == EGL_SUCCESS) {
      break;
    }

    char *pError;
    switch (error) {
      case EGL_SUCCESS:
        pError = (char *)"EGL_SUCCESS";
        break;
      case EGL_NOT_INITIALIZED:
        pError = (char *)"EGL_NOT_INITIALIZED";
        break;
      case EGL_BAD_ACCESS:
        pError = (char *)"EGL_BAD_ACCESS";
        break;
      case EGL_BAD_ALLOC:
        pError = (char *)"EGL_BAD_ALLOC";
        break;
      case EGL_BAD_ATTRIBUTE:
        pError = (char *)"EGL_BAD_ATTRIBUTE";
        break;
      case EGL_BAD_CONTEXT:
        pError = (char *)"EGL_BAD_CONTEXT";
        break;
      case EGL_BAD_CONFIG:
        pError = (char *)"EGL_BAD_CONFIG";
        break;
      case EGL_BAD_CURRENT_SURFACE:
        pError = (char *)"EGL_BAD_CURRENT_SURFACE";
        break;
      case EGL_BAD_DISPLAY:
        pError = (char *)"EGL_BAD_DISPLAY";
        break;
      case EGL_BAD_SURFACE:
        pError = (char *)"EGL_BAD_SURFACE";
        break;
      case EGL_BAD_MATCH:
        pError = (char *)"EGL_BAD_MATCH";
        break;
      case EGL_BAD_PARAMETER:
        pError = (char *)"EGL_BAD_PARAMETER";
        break;
      case EGL_BAD_NATIVE_PIXMAP:
        pError = (char *)"EGL_BAD_NATIVE_PIXMAP";
        break;
      case EGL_BAD_NATIVE_WINDOW:
        pError = (char *)"EGL_BAD_NATIVE_WINDOW";
        break;
      case EGL_CONTEXT_LOST:
        pError = (char *)"EGL_CONTEXT_LOST";
        break;
      default:
        ALOGE("eglError (0x%x) %s:%d\n", error, file, line);
        return;
    }
    ALOGE("eglError (%s) %s:%d\n", pError, file, line);
    return;
  }
  return;
}
