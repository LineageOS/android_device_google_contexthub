#ifndef __NANOHUBPACKET_H
#define __NANOHUBPACKET_H

/**
 * Formats and constants related to nanohub packets.  This header is intended
 * to be shared between the host Linux kernel and the nanohub implementation.
 */

#ifdef __KERNEL__
#include <linux/types.h>
#else
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

#define NANOHUB_SYNC_BYTE             0x31

#define NANOHUB_PREAMBLE_BYTE         0xFF
#define NANOHUB_ACK_PREAMBLE_LEN      16
#define NANOHUB_PAYLOAD_PREAMBLE_LEN  512

#define NANOHUB_INT_BOOT_COMPLETE     0
#define NANOHUB_INT_WAKE_COMPLETE     0
#define NANOHUB_INT_WAKEUP            1
#define NANOHUB_INT_NONWAKEUP         2
#define NANOHUB_INT_CMD_WAIT          3

#define NANOHUB_REASON_ACK                    0x00000000
#define NANOHUB_REASON_NAK                    0x00000001

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
} __attribute__((packed));

#define NANOHUB_REASON_GET_APP_VERSIONS       0x00001001

struct NanohubAppVersionsRequest {
    __le64 appId;
} __attribute__((packed));

struct NanohubAppVersionsResponse {
    __le16 version;
} __attribute__((packed));

#define NANOHUB_REASON_START_FIRMWARE_UPLOAD  0x00001040

struct NanohubStartFirmwareUploadRequest {
    __le32 size;
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

#define NANOHUB_REASON_GET_INTERRUPT          0x00001080

struct NanohubGetInterruptRequest {
} __attribute__((packed));

struct NanohubGetInterruptResponse {
    uint8_t interrupts[MAX_INTERRUPTS/(8*sizeof(uint8_t))];
} __attribute__ ((packed));

#define NANOHUB_REASON_READ_EVENT             0x00001090

struct NanohubReadEventRequest {
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

#endif /* __NANOHUBPACKET_H */
