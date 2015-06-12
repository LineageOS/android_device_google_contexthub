#include <inttypes.h>
#include <stdint.h>
#include <sys/endian.h>

#include <hostIntf_priv.h>
#include <nanohubCommand.h>
#include <nanohubPacket.h>
#include <seos.h>
#include <util.h>

#define NANOHUB_COMMAND(_reason, _handler, _minReqType, _maxReqType) \
        { .reason = _reason, .handler = _handler, \
          .minDataLen = sizeof(_minReqType), .maxDataLen = sizeof(_maxReqType) }

static size_t hostIntfGetOsHwVersion(void *rx, uint8_t, void *tx);

const struct NanohubCommand gBuiltinCommands[] = {
        NANOHUB_COMMAND(NANOHUB_REASON_GET_OS_HW_VERSIONS,
                hostIntfGetOsHwVersion,
                struct NanohubOsHwVersionsRequest,
                struct NanohubOsHwVersionsRequest),
};

static size_t hostIntfGetOsHwVersion(void *rx, uint8_t rx_len, void *tx)
{
    struct NanohubOsHwVersionsResponse *resp = tx;
    resp->hwType = htole16(platHwType());
    resp->hwVer = htole16(platHwVer());
    resp->blVer = htole16(platBlVer());
    resp->osVer = htole16(OS_VER);

    return sizeof(*resp);
}

const struct NanohubCommand *nanohubFindCommand(uint32_t packetReason)
{
    size_t i;

    for (i = 0; i < ARRAY_SIZE(gBuiltinCommands); i++) {
        const struct NanohubCommand *cmd = &gBuiltinCommands[i];
        if (cmd->reason == packetReason)
            return cmd;
    }
    return NULL;
}
