LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES:= nanoapp_cmd.c

LOCAL_C_INCLUDES := $(LOCAL_PATH)/../../firmware/inc

LOCAL_MODULE:= nanoapp_cmd

LOCAL_MODULE_TAGS:= optional
LOCAL_MODULE_OWNER := google
LOCAL_PROPRIETARY_MODULE := true

LOCAL_SHARED_LIBRARIES := \
	libutils \

include $(BUILD_EXECUTABLE)
