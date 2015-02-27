LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_SHARED_LIBRARIES := libcutils libc

LOCAL_LDFLAGS += -Wl,--hash-style=sysv

LOCAL_SRC_FILES := \
        flash.c \
        i2c.c \
        stm32_bl.c \
        stm32f4_crc.c

LOCAL_MODULE := stm32_flash

LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)
