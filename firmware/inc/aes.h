#ifndef _AES_H_
#define _AES_H_
#include <stdint.h>

struct AesContext {
    uint32_t K[64];
};

struct AesSetupTempWorksSpace { //unsed temporarily for aesInitForDecr() only, not used after, need not be live
    struct AesContext tmpCtx;
};

#define AES_KEY_WORDS     8
#define AES_BLOCK_WORDS   4

void aesInitForEncr(struct AesContext *ctx, const uint32_t *k);
void aesInitForDecr(struct AesContext *ctx, struct AesSetupTempWorksSpace *tmpSpace, const uint32_t *k);
void aesEncr(struct AesContext *ctx, const uint32_t *src, uint32_t *dst); //encrypts 4 words
void aesDecr(struct AesContext *ctx, const uint32_t *src, uint32_t *dst); //deencrypts 4 words




#endif

