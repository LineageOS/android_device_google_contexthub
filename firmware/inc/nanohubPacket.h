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

#ifndef __NANOHUBPACKET_H
#define __NANOHUBPACKET_H

/**
 * Formats and constants related to nanohub packets.  This header is intended
 * to be shared between the host Linux kernel and the nanohub implementation.
 */

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <hostIntf.h>
#include <stdint.h>

typedef uint16_t __le16;
typedef uint16_t __be16;
typedef uint32_t __le32;
typedef uint32_t __be32;
typedef uint64_t __le64;
typedef uint64_t __be64;
#endif

struct NanohubPacket {
    uint8_t sync;
    __le32 seq;
    __le32 reason;
    uint8_t len;
    uint8_t data[0];
} __attribute__((packed));

struct NanohubPacketFooter {
    __le32 crc;
} __attribute__((packed));

static inline struct NanohubPacketFooter *nanohubGetPacketFooter(struct NanohubPacket *packet)
{
    return (struct NanohubPacketFooter *)(packet->data + packet->len);
}

#define NANOHUB_PACKET_SIZE(len) \
    (sizeof(struct NanohubPacket) + (len) + sizeof(struct NanohubPacketFooter))

#define NANOHUB_PACKET_PAYLOAD_MAX    255
#define NANOHUB_PACKET_SIZE_MAX       NANOHUB_PACKET_SIZE(NANOHUB_PACKET_PAYLOAD_MAX)
#define NANOHUB_SENSOR_DATA_MAX       240

#define NANOHUB_SYNC_BYTE             0x31

#define NANOHUB_PREAMBLE_BYTE         0xFF
#define NANOHUB_ACK_PREAMBLE_LEN      16
#define NANOHUB_PAYLOAD_PREAMBLE_LEN  512
#define NANOHUB_RSA_KEY_CHUNK_LEN     64

#define NANOHUB_INT_BOOT_COMPLETE     0
#define NANOHUB_INT_WAKE_COMPLETE     0
#define NANOHUB_INT_WAKEUP            1
#define NANOHUB_INT_NONWAKEUP         2
#define NANOHUB_INT_CMD_WAIT          3

#define NANOHUB_REASON_ACK                    0x00000000
#define NANOHUB_REASON_NAK                    0x00000001
#define NANOHUB_REASON_NAK_BUSY               0x00000002

/**
 * INFORMATIONAL
 */

#define NANOHUB_REASON_GET_OS_HW_VERSIONS     0x00001000

struct NanohubOsHwVersionsRequest {
} __attribute__((packed));

struct NanohubOsHwVersionsResponse {
    __le16 hwType;
    __le16 hwVer;
    __le16 blVer;
    __le16 osVer;
    __le32 variantVer;
} __attribute__((packed));

#define NANOHUB_REASON_GET_APP_VERSIONS       0x00001001

struct NanohubAppVersionsRequest {
    __le64 appId;
} __attribute__((packed));

struct NanohubAppVersionsResponse {
    __le32 appVer;
} __attribute__((packed));

#define NANOHUB_REASON_QUERY_APP_INFO         0x00001002

struct NanohubAppInfoRequest {
    __le32 appIdx;
} __attribute__((packed));

struct NanohubAppInfoResponse {
    __le64 appId;
    __le32 appVer;
    __le32 appSize;
} __attribute__((packed));

#define NANOHUB_REASON_START_FIRMWARE_UPLOAD  0x00001040

struct NanohubStartFirmwareUploadRequest {
    __le32 size;
    __le32 crc;
    uint8_t type;
} __attribute__((packed));

struct NanohubStartFirmwareUploadResponse {
    uint8_t accepted;
} __attribute__((packed));

#define NANOHUB_REASON_FIRMWARE_CHUNK         0x00001041

struct NanohubFirmwareChunkRequest {
    __le32 offset;
    uint8_t data[NANOHUB_PACKET_PAYLOAD_MAX-sizeof(__le32)];
} __attribute__((packed));

enum NanohubFirmwareChunkReply {
    NANOHUB_FIRMWARE_CHUNK_REPLY_ACCEPTED = 0,
    NANOHUB_FIRMWARE_CHUNK_REPLY_WAIT,
    NANOHUB_FIRMWARE_CHUNK_REPLY_RESEND,
    NANOHUB_FIRMWARE_CHUNK_REPLY_RESTART,
    NANOHUB_FIRMWARE_CHUNK_REPLY_CANCEL,
    NANOHUB_FIRMWARE_CHUNK_REPLY_CANCEL_NO_RETRY,
};

struct NanohubFirmwareChunkResponse {
    uint8_t chunkReply;
} __attribute__ ((packed));

#define NANOHUB_REASON_FINISH_FIRMWARE_UPLOAD 0x00001042

struct NanohubFinishFirmwareUploadRequest {
} __attribute__ ((packed));

enum NanohubFirmwareUploadReply {
    NANOHUB_FIRMWARE_UPLOAD_SUCCESS = 0,
    NANOHUB_FIRMWARE_UPLOAD_PROCESSING,
    NANOHUB_FIRMWARE_UPLOAD_WAITING_FOR_DATA,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_KEY_NOT_FOUND,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_HEADER_ERROR,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_TOO_MUCH_DATA,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_TOO_LITTLE_DATA,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_SIG_VERIFY_FAIL,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_SIG_DECODE_FAIL,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_SIG_ROOT_UNKNOWN,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_MEMORY_ERROR,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_INVALID_DATA,
    NANOHUB_FIRMWARE_UPLOAD_APP_SEC_BAD,
};

struct NanohubFinishFirmwareUploadResponse {
   uint8_t uploadReply;
} __attribute__ ((packed));

#define NANOHUB_REASON_GET_INTERRUPT          0x00001080

struct NanohubGetInterruptRequest {
    uint32_t clear[HOSTINTF_MAX_INTERRUPTS/(32*sizeof(uint8_t))];
} __attribute__((packed));

struct NanohubGetInterruptResponse {
    uint32_t interrupts[HOSTINTF_MAX_INTERRUPTS/(32*sizeof(uint8_t))];
} __attribute__ ((packed));

#define NANOHUB_REASON_MASK_INTERRUPT         0x00001081

struct NanohubMaskInterruptRequest {
    uint8_t interrupt;
} __attribute__((packed));

struct NanohubMaskInterruptResponse {
    uint8_t accepted;
} __attribute__ ((packed));

#define NANOHUB_REASON_UNMASK_INTERRUPT       0x00001082

struct NanohubUnmaskInterruptRequest {
    uint8_t interrupt;
} __attribute__((packed));

struct NanohubUnmaskInterruptResponse {
    uint8_t accepted;
} __attribute__ ((packed));

#define NANOHUB_REASON_READ_EVENT             0x00001090

struct NanohubReadEventRequest {
    __le64 apBootTime;
} __attribute__((packed));

struct NanohubReadEventResponse {
    __le32 evtType;
    uint8_t evtData[NANOHUB_PACKET_PAYLOAD_MAX - sizeof(__le32)];
} __attribute__ ((packed));

#define NANOHUB_REASON_WRITE_EVENT            0x00001091

struct NanohubWriteEventRequest {
    __le32 evtType;
    uint8_t evtData[NANOHUB_PACKET_PAYLOAD_MAX - sizeof(__le32)];
} __attribute__((packed));

struct NanohubWriteEventResponse {
    uint8_t accepted;
} __attribute__ ((packed));

struct NanohubHalHdr {
    uint64_t appId;
    uint8_t len;
    uint8_t msg;
} __attribute__ ((packed));

#define NANOHUB_HAL_EXT_APPS_ON     0
#define NANOHUB_HAL_EXT_APPS_OFF    1
#define NANOHUB_HAL_EXT_APP_DELETE  2
#define NANOHUB_HAL_QUERY_MEMINFO   3
#define NANOHUB_HAL_QUERY_APPS      4

struct NanohubHalQueryAppsRx {
    __le32 idx;
} __attribute__ ((packed));

struct NanohubHalQueryAppsTx {
    struct NanohubHalHdr hdr;
    __le64 appId;
    __le32 version;
    __le32 flashUse;
    __le32 ramUse;
} __attribute__ ((packed));

#define NANOHUB_HAL_QUERY_RSA_KEYS  5

struct NanohubHalQueryRsaKeysRx {
    __le32 offset;
} __attribute__ ((packed));

struct NanohubHalQueryRsaKeysTx {
    struct NanohubHalHdr hdr;
    uint8_t data[];
} __attribute__ ((packed));

#define NANOHUB_HAL_START_UPLOAD    6

struct NanohubHalStartUploadRx {
    uint8_t isOs;
    __le32 length;
} __attribute__ ((packed));

struct NanohubHalStartUploadTx {
    struct NanohubHalHdr hdr;
    uint8_t success;
} __attribute__ ((packed));

#define NANOHUB_HAL_CONT_UPLOAD     7

struct NanohubHalContUploadRx {
    __le32 offset;
    uint8_t data[];
} __attribute__ ((packed));

struct NanohubHalContUploadTx {
    struct NanohubHalHdr hdr;
    uint8_t success;
} __attribute__ ((packed));

#define NANOHUB_HAL_FINISH_UPLOAD   8

struct NanohubHalFinishUploadTx {
    struct NanohubHalHdr hdr;
    uint8_t success;
} __attribute__ ((packed));

#define NANOHUB_HAL_REBOOT          9

struct NanohubHalRebootTx {
    struct NanohubHalHdr hdr;
    uint8_t success;
} __attribute__ ((packed));

#endif /* __NANOHUBPACKET_H */
