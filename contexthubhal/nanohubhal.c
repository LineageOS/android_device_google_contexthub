/*
 * Copyright (c) 2016, Google. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define LOG_TAG "NanohubHAL"
#include <hardware/context_hub.h>
#include <hardware/hardware.h>
#include "nanohub_perdevice.h"
#include "system_comms.h"
#include "nanohubhal.h"
#include <utils/Log.h>
#include <sys/inotify.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>



#define NANOHUB_LOCK_DIR        "/data/system/nanohub_lock"
#define NANOHUB_LOCK_FILE       NANOHUB_LOCK_DIR "/lock"
#define NANOHUB_LOCK_DIR_PERMS  (S_IRUSR | S_IWUSR | S_IXUSR)



static pthread_mutex_t mLock = PTHREAD_MUTEX_INITIALIZER;
static context_hub_callback *mMsgCbkFunc = NULL;
static int mThreadClosingPipe[2], mFd; // [0] is read end
static void * mMsgCbkData = NULL;
static pthread_t mWorkerThread;



static int rwrite(int fd, const void *buf, int len)
{
    int ret;

    do {
        ret = write(fd, buf, len);
    } while (ret < 0 && errno == EINTR);

    if (ret != len)
        return errno ? -errno : -EIO;

    return 0;
}

static int rread(int fd, void *buf, int len)
{
    int ret;

    do {
        ret = read(fd, buf, len);
    } while (ret < 0 && errno == EINTR);

    return ret;
}

static bool hal_hub_init_inotify(struct pollfd *pfd) {
    bool success = false;

    mkdir(NANOHUB_LOCK_DIR, NANOHUB_LOCK_DIR_PERMS);
    pfd->fd = inotify_init1(IN_NONBLOCK);
    if (pfd->fd < 0) {
        ALOGE("Couldn't initialize inotify: %s", strerror(errno));
    } else if (inotify_add_watch(inotifyFd, NANOHUB_LOCK_DIR, IN_CREATE | IN_DELETE) < 0) {
        ALOGE("Couldn't add inotify watch: %s", strerror(errno));
        close(pfd->fd);
    } else {
        pfd->events = POLLIN;
        success = true;
    }

    return success;
}

static void hal_hub_discard_inotify_evt(int inotifyFd) {
    char buf[sizeof(struct inotify_event) + NAME_MAX + 1];
    int ret = read(inotifyFd, buf, sizeof(buf));
    ALOGD("Discarded %d bytes of inotify data", ret);
}

static void hal_hub_wait_on_dev_lock(struct pollfd *pfd) {
    // While the lock file exists, poll on the inotify fd (with timeout)
    while (access(NANOHUB_LOCK_FILE, F_OK) == 0) {
        ALOGW("Nanohub is locked; blocking read thread");
        int ret = poll(pfd, 1, 5000);
        if (pfd->revents & POLLIN) {
            hal_hub_discard_inotify_evt();
        }
    }
}


int hal_hub_do_send_msg(const void *name, uint8_t len, const void *data) //assumes message is not malformed or invalid
{
    uint8_t buf[EVENT_ID_LEN + APP_NAME_LEN + 1 + MAX_RX_PACKET];
    uint32_t event_id = APP_FROM_HOST_EVENT_ID;

    memcpy(buf + 0, &event_id, EVENT_ID_LEN);
    memcpy(buf + EVENT_ID_LEN, name, APP_NAME_LEN);
    buf[EVENT_ID_LEN + APP_NAME_LEN] = len;
    memcpy(buf + EVENT_ID_LEN + APP_NAME_LEN + 1, data, len);

    return rwrite(mFd, buf, EVENT_ID_LEN + APP_NAME_LEN + 1 + len);
}

void hal_hub_call_rx_msg_f(const void *name, uint32_t typ, uint32_t len, const void *data)
{
    struct hub_app_name_t appName = { .app_name_len = APP_NAME_LEN, .app_name = name, };
    struct hub_message_t msg = { .app = &appName, .message_type = typ, .message_len = len, .message = data, };

    mMsgCbkFunc(0, &msg, mMsgCbkData);
}

static void* hal_hub_thread(void *unused) //all nanoapp message_type vals are "0" for this hal. apps can send type themselves in payload
{
    uint8_t buffer[EVENT_ID_LEN + APP_NAME_LEN + 1 + MAX_RX_PACKET];
    uint8_t *header = &buffer[EVENT_ID_LEN];
    uint8_t *packetData = &buffer[EVENT_ID_LEN + APP_NAME_LEN + 1];
    const int idxNanohub = 0, idxClosePipe = 1;
    struct pollfd myFds[3] = {
        [idxNanohub] = { .fd = mFd, .events = POLLIN, },
        [idxClosePipe] = { .fd = mThreadClosingPipe[0], .events = POLLIN, },
    };
    int idxInotify = -1;
    int numPollFds = 2;
    int ret;

    if (hal_hub_init_inotify(&myFds[numPollFds])) {
        idxInotify = numPollFds++;
    }

    (void)unused;
    while (1) {
        ret = poll(myFds, numPollFds, -1);
        if (ret <= 0) {
            ALOGD("poll is being weird");
            continue;
        }

        if (idxInotify >= 0 && myFds[idxInotify].revents & POLLIN) {
            hal_hub_discard_inotify_evt(myFds[idxInotify].fd);
            hal_hub_wait_on_dev_lock(&myFds[idxInotify]);
        }

        if (myFds[idxNanohub].revents & POLLIN) { // we have data

            uint32_t len;

            ret = rread(mFd, buffer, sizeof(buffer));
            if (ret <= 0) {
                ALOGE("read 1 fails");
                break;
            }

            len = header[APP_NAME_LEN];

            if (len > MAX_RX_PACKET) {
                ALOGE("malformed packet");
                break;
            }

            if (ret != EVENT_ID_LEN + APP_NAME_LEN + 1 + len) {
                ALOGE("read 2 fails");
                break;
            }

            ret = system_comms_handle_rx(header, len, packetData);
            if (!ret)
                hal_hub_call_rx_msg_f(header, 0, len, packetData);
        }

        if (myFds[idxClosePipe].revents & POLLIN) { // we have been asked to die
            ALOGD("thread exiting");
            break;
        }
    }

    close(mFd);
    return NULL;
}

static int hal_hub_open(void)
{
    int ret = 0;

    mFd = open(get_devnode_path(), O_RDWR);
    if (mFd < 0) {
        ALOGE("cannot find hub devnode '%s'", get_devnode_path());
        ret = -errno;
        goto fail_open;
    }

    if (pipe(mThreadClosingPipe)) {
        ALOGE("failed to create signal pipe");
        ret = -errno;
        goto fail_pipe;
    }

    if (pthread_create(&mWorkerThread, NULL, hal_hub_thread, NULL)) {
        ALOGE("failed to spawn worker thread");
        ret = -errno;
        goto fail_thread;
    }

    return 0;

fail_thread:
    close(mThreadClosingPipe[0]);
    close(mThreadClosingPipe[1]);

fail_pipe:
    close(mFd);

fail_open:
    return ret;
}

static int hal_hub_close(void)
{
    char zero = 0;

    //signal
    while(write(mThreadClosingPipe[1], &zero, 1) != 1);

    //wait
    (void)pthread_join(mWorkerThread, NULL);

    //cleanup
    close(mThreadClosingPipe[0]);
    close(mThreadClosingPipe[1]);

    return 0;
}

static int hal_hub_subscribe_messages(uint32_t hub_id, context_hub_callback *cbk, void *cookie)
{
    int ret = 0;

    if (hub_id)
        return -ENODEV;

    pthread_mutex_lock(&mLock);
    if (!mMsgCbkFunc && !cbk) { //we're off and staying off - do nothing

        ALOGD("staying off");
    }
    else if (cbk && mMsgCbkFunc) { //new callback but staying on

        ALOGD("staying on");
    }
    else if (mMsgCbkFunc) {     //we were on but turning off

        ALOGD("turning off");

        ret = hal_hub_close();
    }
    else if (cbk) {             //we're turning on

        ALOGD("turning on");
        ret = hal_hub_open();
    }

    mMsgCbkFunc = cbk;
    mMsgCbkData = cookie;

    pthread_mutex_unlock(&mLock);

    return ret;
}

static int hal_hub_send_message(uint32_t hub_id, const struct hub_message_t *msg)
{
    int ret = 0;

    if (hub_id)
        return -ENODEV;

    pthread_mutex_lock(&mLock);

    if (!mMsgCbkFunc) {

        ALOGW("refusing to send a message when nobody around to get a reply!");
        ret = -EIO;
    }
    else {

        if (!msg || !msg->app || !msg->app->app_name || msg->app->app_name_len != APP_NAME_LEN || !msg->message) {
            ALOGW("not sending invalid message 1");
            ret = -EINVAL;
        }

        if (!memcmp(get_hub_info()->os_app_name->app_name, msg->app->app_name, APP_NAME_LEN)) { //messages to the "system" app are special - hal handles them
            ret = system_comms_handle_tx(msg);
        }
        else if (msg->message_type || msg->message_len > MAX_RX_PACKET) {
            ALOGW("not sending invalid message 2");
            ret = -EINVAL;
        }
        else {
            ret = hal_hub_do_send_msg(msg->app->app_name, msg->message_len, msg->message);
        }
    }

    pthread_mutex_unlock(&mLock);

    return ret;
}

static int hal_get_hubs(struct context_hub_module_t* module, const struct context_hub_t ** list)
{
    (void)module;

    *list = get_hub_info();

    return 1; /* we have one hub */
}

struct context_hub_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = CONTEXT_HUB_DEVICE_API_VERSION_1_0,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = CONTEXT_HUB_MODULE_ID,
        .name = "Bullhead nanohub HAL",
        .author = "Google",
    },

    .get_hubs = hal_get_hubs,
    .subscribe_messages = hal_hub_subscribe_messages,
    .send_message = hal_hub_send_message,
};

