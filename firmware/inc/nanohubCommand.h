#ifndef __NANOHUBCOMMAND_H
#define __NANOHUBCOMMAND_H

struct NanohubCommand {
    uint32_t reason;
    size_t (*handler)(void *, uint8_t, void *, uint64_t);
    uint8_t minDataLen;
    uint8_t maxDataLen;
};

const struct NanohubCommand *nanohubFindCommand(uint32_t packetReason);

#endif /* __NANOHUBCOMMAND_H */
