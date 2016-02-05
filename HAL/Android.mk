LOCAL_PATH := $(call my-dir)

# HAL module implemenation stored in
# hw/<CONTEXT_HUB_MODULE_ID>.<ro.hardware>.so
include $(CLEAR_VARS)

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MULTILIB := both
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_SRC_FILES := nanohubhal.c system_comms.c
LOCAL_MODULE_OWNER := google
LOCAL_PROPRIETARY_MODULE := true

# Include target-specific files.

ifneq ($(filter bullhead, $(TARGET_DEVICE)),)
LOCAL_SRC_FILES += nanohubhal_bullhead.c
endif

ifneq ($(filter angler, $(TARGET_DEVICE)),)
LOCAL_SRC_FILES += nanohubhal_angler.c
endif



LOCAL_MODULE := context_hub.$(TARGET_DEVICE)
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)
