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

//basic AES block ops
void aesInitForEncr(struct AesContext *ctx, const uint32_t *k);
void aesInitForDecr(struct AesContext *ctx, struct AesSetupTempWorksSpace *tmpSpace, const uint32_t *k);
void aesEncr(struct AesContext *ctx, const uint32_t *src, uint32_t *dst); //encrypts AES_BLOCK_WORDS words
void aesDecr(struct AesContext *ctx, const uint32_t *src, uint32_t *dst); //deencrypts AES_BLOCK_WORDS words

//AES-CBC
struct AesCbcContext {
    struct AesContext aes;
    uint32_t iv[AES_BLOCK_WORDS];
};

void aesCbcInitForEncr(struct AesCbcContext *ctx, const uint32_t *k, const uint32_t *iv);
void aesCbcInitForDecr(struct AesCbcContext *ctx, const uint32_t *k, const uint32_t *iv);
void aesCbcEncr(struct AesCbcContext *ctx, const uint32_t *src, uint32_t *dst); //encrypts AES_BLOCK_WORDS words
void aesCbcDecr(struct AesCbcContext *ctx, const uint32_t *src, uint32_t *dst); //encrypts AES_BLOCK_WORDS words


#endif

