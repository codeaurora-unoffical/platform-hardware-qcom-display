LOCAL_PATH := $(call my-dir)
include $(LOCAL_PATH)/../common.mk
include $(CLEAR_VARS)

LOCAL_SRC_FILES := hdmi_cec_unit_test.cpp
LOCAL_MODULE := hdmi_cec_unit_test
LOCAL_SHARED_LIBRARIES += \
    libhardware \
    liblog \
    libutils \

LOCAL_MODULE := hdmi_cec_unit_test
LOCAL_MODULE_TAGS := tests

include $(BUILD_NATIVE_TEST)
