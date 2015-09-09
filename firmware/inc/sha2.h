#ifndef _SHA2_H_
#define _SHA2_H_

//this is neither the fastest nor the smallest. but it is simple and matches the spec. cool.

#include <stdint.h>

#define SHA2_BLOCK_SIZE         64 //in bytes
#define SHA2_WORDS_STATE_SIZE   64 //in words
struct Sha2state {
    uint32_t h[8];
    uint64_t msgLen;
    union {
        uint32_t w[SHA2_WORDS_STATE_SIZE];
        uint8_t b[SHA2_BLOCK_SIZE];
    };
    uint8_t bufBytesUsed;
};


void sha2init(struct Sha2state *state);
void sha2processBytes(struct Sha2state *state, const void *bytes, uint32_t numBytes);
const uint32_t* sha2finish(struct Sha2state *state); //returned hash pointer is only valid as long as "state" is!




#endif

