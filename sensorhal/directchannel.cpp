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
    }
    ::close(mAshmemFd);
}

} // namespace android
