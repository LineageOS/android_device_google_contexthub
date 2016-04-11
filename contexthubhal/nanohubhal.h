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


#ifndef _NANOHUB_HAL_H_
#define _NANOHUB_HAL_H_

#include <hardware/context_hub.h>

#define NANOAPP_VENDOR_GOOGLE NANOAPP_VENDOR("Googl")

//as per protocol
#define MAX_RX_PACKET           128
#define APP_FROM_HOST_EVENT_ID  0x000000F8

struct nano_message_hdr {
    uint32_t event_id;
    struct hub_app_name_t app_name;
    uint8_t len;
} __attribute__((packed));

struct nano_message {
    struct nano_message_hdr hdr;
    uint8_t data[MAX_RX_PACKET];
} __attribute__((packed));

int hal_hub_do_send_msg(const struct hub_app_name_t *name, uint32_t len, const void *data);
void hal_hub_call_rx_msg_f(const struct hub_app_name_t *name, uint32_t type, uint32_t len, const void *data);


#endif

