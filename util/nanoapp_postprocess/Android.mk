LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := libc

LOCAL_SRC_FILES := \
        postprocess.c

LOCAL_MODULE := nanoapp_postprocess

LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)
