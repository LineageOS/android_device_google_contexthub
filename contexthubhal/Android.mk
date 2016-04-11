LOCAL_PATH := $(call my-dir)

SUPPORTED_DEVICES := \
	bullhead \
	angler \
	marlin \
	sailfish

ifneq ($(filter $(SUPPORTED_DEVICES),$(TARGET_DEVICE)),)

# HAL module implemenation stored in
# hw/<CONTEXT_HUB_MODULE_ID>.<ro.hardware>.so
include $(CLEAR_VARS)

LOCAL_MODULE_RELATIVE_PATH := hw
LOCAL_MULTILIB := both
LOCAL_SHARED_LIBRARIES := liblog libcutils
LOCAL_SRC_FILES := nanohubhal.c system_comms.c
LOCAL_CFLAGS := -Wall -Werror -Wextra
LOCAL_MODULE_OWNER := google

# Include target-specific files.
LOCAL_SRC_FILES += nanohubhal_$(TARGET_DEVICE).c

LOCAL_MODULE := context_hub.$(TARGET_DEVICE)
LOCAL_MODULE_TAGS := optional
include $(BUILD_SHARED_LIBRARY)

endif
