# Copyright (C) 2015 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

LOCAL_PATH := $(call my-dir)

MY_CFLAGS := -Wall -Werror -Wextra

ifneq ($(TARGET_USES_CHINOOK_SENSORHUB),true)

ifneq ($(filter bullhead angler marlin sailfish,$(TARGET_DEVICE)),)

include $(CLEAR_VARS)

LOCAL_MODULE := sensors.$(TARGET_DEVICE)

LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := google

LOCAL_CFLAGS += $(MY_CFLAGS)

LOCAL_C_INCLUDES +=                     \
	device/google/contexthub/firmware/inc \
	device/google/contexthub/util/common

LOCAL_SRC_FILES :=                      \
	sensors.cpp

LOCAL_SHARED_LIBRARIES :=               \
	libcutils                       \
	libhubconnection                \
	libstagefright_foundation       \
	libutils

include $(BUILD_SHARED_LIBRARY)

################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := activity_recognition.$(TARGET_DEVICE)

LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_CFLAGS += $(MY_CFLAGS)

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := google

LOCAL_C_INCLUDES +=                     \
	device/google/contexthub/firmware/inc \
	device/google/contexthub/util/common

LOCAL_SRC_FILES :=                      \
	activity.cpp

LOCAL_SHARED_LIBRARIES :=               \
	libcutils                       \
	libhubconnection                \
	liblog                          \
	libstagefright_foundation       \
	libutils

include $(BUILD_SHARED_LIBRARY)

################################################################################

include $(CLEAR_VARS)

LOCAL_MODULE := libhubconnection

LOCAL_CFLAGS += $(MY_CFLAGS)
LOCAL_C_INCLUDES +=                     \
	device/google/contexthub/firmware/inc \
	device/google/contexthub/util/common

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_OWNER := google

LOCAL_SRC_FILES :=                          \
	hubconnection.cpp                   \
	../util/common/file.cpp       \
	../util/common/JSONObject.cpp \
	../util/common/ring.cpp

LOCAL_SHARED_LIBRARIES :=                   \
	libcutils                           \
	liblog                              \
	libstagefright_foundation           \
	libutils

include $(BUILD_SHARED_LIBRARY)

endif

endif
