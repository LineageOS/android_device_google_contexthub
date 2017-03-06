/*
 * Copyright (C) 2017 The Android Open Source Project
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

#include "directchannel.h"

#include <cutils/ashmem.h>
#include <hardware/gralloc.h>
#include <hardware/sensors.h>
#include <utils/Log.h>

#include <sys/mman.h>

namespace android {

bool DirectChannelBase::isValid() {
    return mBuffer != nullptr;
}

int DirectChannelBase::getError() {
    return mError;
}

void DirectChannelBase::write(const sensors_event_t * ev) {
    if (isValid()) {
        mBuffer->write(ev, 1);
    }
}

AshmemDirectChannel::AshmemDirectChannel(const struct sensors_direct_mem_t *mem) : mAshmemFd(0) {
    mAshmemFd = mem->handle->data[0];

    if (!::ashmem_valid(mAshmemFd)) {
        mError = BAD_VALUE;
        return;
    }

    if ((size_t)::ashmem_get_size_region(mAshmemFd) != mem->size) {
        mError = BAD_VALUE;
        return;
    }

    mSize = mem->size;

    mBase = ::mmap(NULL, mem->size, PROT_WRITE, MAP_SHARED, mAshmemFd, 0);
    if (mBase == nullptr) {
        mError = NO_MEMORY;
        return;
    }

    mBuffer = std::unique_ptr<LockfreeBuffer>(new LockfreeBuffer(mBase, mSize));
    if (!mBuffer) {
        mError = NO_MEMORY;
    }
}

AshmemDirectChannel::~AshmemDirectChannel() {
    if (mBase) {
        mBuffer = nullptr;
        ::munmap(mBase, mSize);
        mBase = nullptr;
    }
    ::close(mAshmemFd);
}

ANDROID_SINGLETON_STATIC_INSTANCE(GrallocHalWrapper);

GrallocHalWrapper::GrallocHalWrapper() {
    status_t err = ::hw_get_module(GRALLOC_HARDWARE_MODULE_ID,
                                 (hw_module_t const**)&mGrallocModule);
    ALOGE_IF(err, "couldn't load %s module (%s)",
             GRALLOC_HARDWARE_MODULE_ID, strerror(-err));

    if (mGrallocModule != nullptr) {
        err = ::gralloc_open(&mGrallocModule->common, &mAllocDevice);
        ALOGE_IF(err, "cannot open alloc device (%s)", strerror(-err));
        if (mAllocDevice != nullptr) {
            err = NO_ERROR;
        }
    }
    mError = err;
}

GrallocHalWrapper::~GrallocHalWrapper() {
    if (mAllocDevice != nullptr) {
        ::gralloc_close(mAllocDevice);
    }
}

int GrallocHalWrapper::registerBuffer(const native_handle_t *handle) {
    if (mGrallocModule == nullptr) {
        return NO_INIT;
    }
    return mGrallocModule->registerBuffer(mGrallocModule, handle);
}

int GrallocHalWrapper::unregisterBuffer(const native_handle_t *handle) {
    if (mGrallocModule == nullptr) {
        return NO_INIT;
    }
    return mGrallocModule->unregisterBuffer(mGrallocModule, handle);
}

int GrallocHalWrapper::lock(const native_handle_t *handle,
                           int usage, int l, int t, int w, int h, void **vaddr) {
    if (mGrallocModule == nullptr) {
        return NO_INIT;
    }
    return mGrallocModule->lock(mGrallocModule, handle, usage, l, t, w, h, vaddr);
}

int GrallocHalWrapper::unlock(const native_handle_t *handle) {
    if (mGrallocModule == nullptr) {
        return NO_INIT;
    }
    return mGrallocModule->unlock(mGrallocModule, handle);
}

GrallocDirectChannel::GrallocDirectChannel(const struct sensors_direct_mem_t *mem)
        : mNativeHandle(nullptr) {
    if (mem->handle == nullptr) {
        ALOGE("mem->handle == nullptr");
        mError = BAD_VALUE;
        return;
    }

    mNativeHandle = ::native_handle_clone(mem->handle);
    if (mNativeHandle == nullptr) {
        ALOGE("clone mem->handle failed...");
        mError = NO_MEMORY;
        return;
    }

    mError = GrallocHalWrapper::getInstance().registerBuffer(mNativeHandle);
    if (mError != NO_ERROR) {
        ALOGE("registerBuffer failed");
        return;
    }

    mError = GrallocHalWrapper::getInstance().lock(mNativeHandle,
            GRALLOC_USAGE_SW_WRITE_OFTEN, 0, 0, mem->size, 1, &mBase);
    if (mError != NO_ERROR) {
        ALOGE("lock buffer failed");
        return;
    }

    if (mBase == nullptr) {
        ALOGE("lock buffer => nullptr");
        mError = NO_MEMORY;
        return;
    }

    mSize = mem->size;
    mBuffer = std::make_unique<LockfreeBuffer>(mBase, mSize);
    if (!mBuffer) {
        mError = NO_MEMORY;
        return;
    }

    mError = NO_ERROR;
}

GrallocDirectChannel::~GrallocDirectChannel() {
    if (mNativeHandle != nullptr) {
        if (mBase) {
            mBuffer = nullptr;
            GrallocHalWrapper::getInstance().unlock(mNativeHandle);
            mBase = nullptr;
        }
        GrallocHalWrapper::getInstance().unregisterBuffer(mNativeHandle);
        ::native_handle_close(mNativeHandle);
        ::native_handle_delete(mNativeHandle);
        mNativeHandle = nullptr;
    }
}

} // namespace android
