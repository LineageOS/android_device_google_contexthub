/*
 * Copyright (C) 2016 The Android Open Source Project
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

#ifndef DIRECTCHANNEL_H_
#define DIRECTCHANNEL_H_

#include "ring.h"
#include <hardware/sensors.h>
#include <memory>

namespace android {

class DirectChannelBase {
public:
    DirectChannelBase() : mError(NO_ERROR) { }
    virtual ~DirectChannelBase() {}

    bool isValid();
    int getError();
    void write(const sensors_event_t * ev);

protected:
    int mError;
    std::unique_ptr<LockfreeBuffer> mBuffer;

    size_t mSize;
    void* mBase;
};

class AshmemDirectChannel : public DirectChannelBase {
public:
    AshmemDirectChannel(const struct sensors_direct_mem_t *mem);
    virtual ~AshmemDirectChannel();
private:
    int mAshmemFd;
};

} // namespace android

#endif  // DIRECTCHANNEL_H_
